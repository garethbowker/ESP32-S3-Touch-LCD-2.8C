//! Bounce-buffer mode for the DPI panel.
//!
//! ## Why this exists
//!
//! Direct PSRAM→DPI DMA on the ESP32-S3 produces diagonal drift
//! artefacts at *any* PCLK we tested (18, 12, 8 MHz). Halving PCLK
//! *doubled* the visible stripe density, which proves the drift
//! trigger is at a fixed time rate (PSRAM refresh cycles) and isn't
//! a pure bandwidth issue. The only fix is to keep DMA out of PSRAM.
//!
//! ## How it works
//!
//! Two small DRAM "bounce" buffers, each holding [`N_LINES_PER_HALF`]
//! scanlines. A *frame-sized* descriptor ring of
//! [`HALVES_PER_FRAME`] × [`DESC_PER_HALF`] entries alternates between
//! the two halves (even-indexed halves → A, odd → B). The ring is
//! closed (last descriptor's `next` points back to the first), so DMA
//! loops continuously and — critically — each loop covers exactly one
//! LCD frame's worth of bytes, so the panel's vsync stays locked to
//! the ring position with no rolling.
//!
//! [`HALVES_PER_FRAME`] must be **even** for the alternating A/B
//! assignment to repeat cleanly across frame boundaries — otherwise
//! the last half of frame N and the first half of frame N+1 would
//! both land in the same bounce buffer, breaking the
//! "EOF#N → refill bounce[N%2]" pattern.
//!
//! `suc_eof = 1` is set on the last descriptor of each half so the
//! DMA controller fires an EOF interrupt at every half boundary
//! ([`HALVES_PER_FRAME`] times per frame). The ISR memcpy-refills the
//! just-consumed half from the master framebuffer in PSRAM, which is
//! observed through the CPU cache naturally. DMA itself never
//! touches PSRAM, so no drift.
//!
//! At 12 MHz PCLK with 16-line halves, the ISR fires roughly every
//! 640 µs and the refill memcpy is ~150 µs of CPU time, so other
//! tasks get >75 % of the budget.
//!
//! ## Hardware-derived half index (not a software counter)
//!
//! The ISR reads `dma.ch(0).out_eof_des_addr()` — a GDMA register that
//! latches the address of the descriptor that most-recently generated
//! an EOF — and derives "which half just finished" from that. We do
//! **not** rely on a software EOF counter to decide which physical
//! bounce buffer to refill. If two EOFs ever coalesce because the ISR
//! was held off (long critical section, log burst, etc.), a counter
//! drifts permanently from reality: bounce-half assignment swaps, the
//! image jumps by [`N_LINES_PER_HALF`] lines, and *stays* shifted
//! until the next coalesce event. The hardware register is always
//! up-to-date with the most recent EOF, so the worst we lose to a
//! coalesce is one half's refill for a single frame.
//!
//! Reference: `esp_lcd_panel_rgb.c` in ESP-IDF.

#![allow(dead_code)] // helpers used at setup time only

use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};

use esp_hal::dma::{
    BurstConfig, DmaDescriptor, DmaTxBuffer, Owner, Preparation, TransferDirection,
};
use esp_hal::interrupt::{self, Priority};
use esp_hal::peripherals::Interrupt;

use crate::framebuffer::{self, WIDTH};

/// How many scanlines each bounce buffer holds. Chosen so that
/// [`HALVES_PER_FRAME`] comes out even (see module docs).
pub const N_LINES_PER_HALF: usize = 16;

/// Bytes per bounce-buffer half.
pub const HALF_BYTES: usize = N_LINES_PER_HALF * WIDTH * 2;

/// Max DMA chunk size (must be ≤ 4095 on ESP32-S3). 4032 is
/// 64-byte-aligned which matches GDMA's preferred burst size.
const CHUNK: usize = 4032;

/// Descriptors per half (last descriptor may be shorter than CHUNK).
const DESC_PER_HALF: usize = HALF_BYTES.div_ceil(CHUNK);
/// Halves per frame.
pub const HALVES_PER_FRAME: usize = framebuffer::BYTES / HALF_BYTES;

/// Fixed offset added to every chunk index, in *halves*. Compensates
/// for the deterministic startup race between DMA and the LCD_CAM
/// peripheral inside `Dpi::send`. Measured empirically on the
/// Waveshare 2.8C, stable boot-to-boot at 12 MHz PCLK. Adjust if PCLK
/// or panel timing changes meaningfully.
///
/// Empirically: 20 → image 1/3 down; 10 → 2/3 down; 0 → top.
const STARTUP_OFFSET_HALVES: usize = 0;

/// Sub-half pixel-level shift added on top of `STARTUP_OFFSET_HALVES`,
/// in *scanlines* (i.e. one unit = one row of the framebuffer).
/// Positive values shift the image *down* on the panel. Used to dial
/// in the last few pixels that the half-granularity offset can't
/// reach.
const STARTUP_OFFSET_LINES: usize = 6;
/// Byte count for `STARTUP_OFFSET_LINES`.
const STARTUP_OFFSET_BYTES: usize = STARTUP_OFFSET_LINES * WIDTH * 2;
/// Total descriptors in the frame-sized ring.
const N_DESC: usize = DESC_PER_HALF * HALVES_PER_FRAME;

// Compile-time invariants for the alternating-bounce algorithm to work.
const _: () = assert!(framebuffer::BYTES % HALF_BYTES == 0);
const _: () = assert!(HALVES_PER_FRAME % 2 == 0);

#[repr(align(64))]
struct BounceHalf([u8; HALF_BYTES]);

static mut BOUNCE_A: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut BOUNCE_B: BounceHalf = BounceHalf([0; HALF_BYTES]);
static mut DESCRIPTORS: [DmaDescriptor; N_DESC] = [DmaDescriptor::EMPTY; N_DESC];

/// Pointer to the PSRAM master framebuffer the ISR refills from.
///
/// Set initially by [`BounceRing::new`] before the DMA transfer starts.
/// Re-pointed by the EOF ISR when a pending flip is consumed. Read by
/// [`eof_isr`] on every fire.
static PSRAM_FB_PTR: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());

/// New front-buffer pointer queued by [`request_flip`], to be applied
/// by the ISR at the precise moment that makes the next frame fully
/// tear-free.
///
/// That moment is the EOF for the half whose refill writes chunk 0 of
/// the next frame — i.e. when `next_chunk == 0` in [`eof_isr`]. By
/// changing `PSRAM_FB_PTR` *immediately before* that refill, both
/// look-ahead halves (chunk 0 and chunk 1 of the next frame) are
/// filled from the new front, so when DMA wraps the next frame is
/// entirely new content from the very first scanline.
///
/// `null` means "no flip pending".
static PENDING_SWAP: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());

/// Number of EOF interrupts fired since transfer start. Diagnostics-only
/// — the ISR no longer uses this to choose a bounce half or compute
/// the next chunk index. See the module docs for why; both are now
/// derived from the GDMA's `out_eof_des_addr` register on every fire.
static EOF_COUNT: AtomicU32 = AtomicU32::new(0);

/// Custom `DmaTxBuffer` impl wrapping the bounce-buffer descriptor
/// ring. Modelled after [`esp_hal::dma::DmaLoopBuf`] but with a
/// multi-descriptor closed ring instead of a single self-referencing
/// descriptor.
pub struct BounceRing {
    head: *mut DmaDescriptor,
}

impl BounceRing {
    /// Build the descriptor ring, prime both bounce halves with the
    /// first two chunks of the master framebuffer, and return a handle
    /// usable with `Dpi::send`.
    ///
    /// # Safety
    ///
    /// - `psram_fb_ptr` must remain valid for [`framebuffer::BYTES`]
    ///   bytes for as long as the returned `BounceRing` is in use.
    /// - Must be called at most once per boot. The module-level
    ///   statics (`BOUNCE_*`, `DESCRIPTORS`, `PSRAM_FB_PTR`,
    ///   `EOF_COUNT`) are reset by this function but the design
    ///   assumes a single setup.
    pub unsafe fn new(psram_fb_ptr: *const u8) -> Self { unsafe {
        let descs_base: *mut DmaDescriptor = core::ptr::addr_of_mut!(DESCRIPTORS) as *mut _;
        let buf_a: *mut u8 = core::ptr::addr_of_mut!(BOUNCE_A) as *mut u8;
        let buf_b: *mut u8 = core::ptr::addr_of_mut!(BOUNCE_B) as *mut u8;

        for half in 0..HALVES_PER_FRAME {
            let buf_base = if half % 2 == 0 { buf_a } else { buf_b };
            let desc_offset = half * DESC_PER_HALF;

            for i in 0..DESC_PER_HALF {
                let byte_start = i * CHUNK;
                let byte_end = ((i + 1) * CHUNK).min(HALF_BYTES);
                let chunk_len = byte_end - byte_start;

                let d = descs_base.add(desc_offset + i);
                (*d).set_size(chunk_len);
                (*d).set_length(chunk_len);
                (*d).buffer = buf_base.add(byte_start);
                (*d).set_owner(Owner::Dma);

                // suc_eof on the last descriptor of each half — that's
                // what triggers the EOF interrupt.
                let is_last_in_half = i == DESC_PER_HALF - 1;
                (*d).set_suc_eof(is_last_in_half);

                // Linkage: next desc in the same half, then on to the
                // next half. The very last descriptor wraps back to
                // descriptor 0 to close the ring — DMA loops every
                // frame's worth of bytes, perfectly aligned with the
                // LCD's frame timing.
                let next_global =
                    if is_last_in_half && half == HALVES_PER_FRAME - 1 {
                        0
                    } else {
                        desc_offset + i + 1
                    };
                (*d).next = descs_base.add(next_global);
            }
        }

        // Initial fill, offset so the first chunks DMA hands to the
        // LCD line up with the LCD's frame position 0.
        let init_a_chunk = STARTUP_OFFSET_HALVES % HALVES_PER_FRAME;
        let init_b_chunk = (STARTUP_OFFSET_HALVES + 1) % HALVES_PER_FRAME;
        psram_copy_wrapping(psram_fb_ptr, init_a_chunk, buf_a);
        psram_copy_wrapping(psram_fb_ptr, init_b_chunk, buf_b);

        PSRAM_FB_PTR.store(psram_fb_ptr as *mut u8, Ordering::Release);
        EOF_COUNT.store(0, Ordering::Relaxed);

        Self { head: descs_base }
    }}

    /// `(half-bytes, descriptors-per-half, halves-per-frame)`, for
    /// boot-time logging.
    pub fn dimensions() -> (usize, usize, usize) {
        (HALF_BYTES, DESC_PER_HALF, HALVES_PER_FRAME)
    }

    /// Total EOF interrupts fired since transfer start. For
    /// diagnostics — should be ≈1290/s at 12 MHz PCLK with 16-line
    /// halves.
    pub fn isr_fires() -> u32 {
        EOF_COUNT.load(Ordering::Relaxed)
    }
}

unsafe impl DmaTxBuffer for BounceRing {
    type View = Self;

    fn prepare(&mut self) -> Preparation {
        Preparation {
            start: self.head,
            // DMA reads DRAM bounce buffers only; never PSRAM.
            accesses_psram: false,
            direction: TransferDirection::Out,
            burst_transfer: BurstConfig::default(),
            // Owner bit is set to DMA on every descriptor at setup
            // time and not maintained by hardware in this mode.
            check_owner: Some(false),
            auto_write_back: false,
        }
    }

    fn into_view(self) -> Self::View {
        self
    }

    fn from_view(view: Self::View) -> Self {
        view
    }
}

/// Queue a front-buffer swap. The ISR will apply it at the precise
/// EOF where doing so yields a tear-free next frame (see
/// [`PENDING_SWAP`]).
///
/// If a previous request hasn't been consumed yet, it's silently
/// overwritten — only the most recent target ever takes effect.
/// Callers that need to know when their flip has actually been
/// applied should poll [`is_flip_pending`] until it returns `false`.
pub fn request_flip(new_front: *mut u8) {
    PENDING_SWAP.store(new_front, Ordering::Release);
}

/// True if a [`request_flip`] is queued and hasn't been consumed by
/// the ISR yet. Used by [`crate::Framebuffer::flip`] to spin-wait
/// until the swap is in effect, so the caller can safely start
/// writing to the new back buffer (which was the old front).
pub fn is_flip_pending() -> bool {
    !PENDING_SWAP.load(Ordering::Acquire).is_null()
}

/// Install the EOF ISR and enable the GDMA channel-0 OUT_EOF interrupt.
///
/// # Safety
///
/// Touches the GDMA channel-0 `out_int` registers and the global
/// interrupt vector for `DMA_OUT_CH0`. Must be called from outside any
/// interrupt context, and only after the DPI transfer has been
/// started so that PSRAM writes from the framebuffer are visible.
pub unsafe fn enable_eof_interrupt() { unsafe {
    interrupt::bind_interrupt(Interrupt::DMA_OUT_CH0, eof_isr_trampoline);
    interrupt::enable(Interrupt::DMA_OUT_CH0, Priority::Priority2)
        .expect("DMA_OUT_CH0 enable");

    let dma = &*esp_hal::peripherals::DMA::PTR;
    dma.ch(0)
        .out_int()
        .ena()
        .modify(|_, w| w.out_eof().set_bit());
}}

unsafe extern "C" fn eof_isr_trampoline() {
    eof_isr();
}

/// EOF interrupt body. Fires once per [`HALF_BYTES`] of DMA progress
/// (roughly every 640 µs at 12 MHz PCLK / 16-line halves).
fn eof_isr() {
    // SAFETY: GDMA register block is a fixed-address singleton.
    let dma = unsafe { &*esp_hal::peripherals::DMA::PTR };

    // Confirm OUT_EOF on channel 0 actually fired.
    if !dma.ch(0).out_int().st().read().out_eof().bit_is_set() {
        return;
    }

    // Hardware truth: which descriptor most-recently generated an EOF.
    let eof_des_addr = dma.ch(0).out_eof_des_addr().read().bits() as usize;
    let descs_base = core::ptr::addr_of!(DESCRIPTORS) as usize;
    let desc_size = core::mem::size_of::<DmaDescriptor>();
    let ring_bytes = N_DESC * desc_size;

    // Defensive: if the peripheral reports an address outside our ring
    // (shouldn't happen in steady state), bail rather than dereference
    // a wild pointer.
    if eof_des_addr < descs_base || eof_des_addr >= descs_base + ring_bytes {
        EOF_COUNT.fetch_add(1, Ordering::Relaxed);
        dma.ch(0)
            .out_int()
            .clr()
            .write(|w| w.out_eof().clear_bit_by_one());
        return;
    }

    let desc_idx = (eof_des_addr - descs_base) / desc_size;
    let just_finished_half = desc_idx / DESC_PER_HALF;

    // Which DRAM half just finished, given the alternating A/B layout?
    //   even-indexed halves → BOUNCE_A
    //   odd-indexed halves  → BOUNCE_B
    let dest: *mut u8 = if just_finished_half % 2 == 0 {
        core::ptr::addr_of_mut!(BOUNCE_A) as *mut u8
    } else {
        core::ptr::addr_of_mut!(BOUNCE_B) as *mut u8
    };

    // The chunk we need to load is the one DMA will see the NEXT time
    // it reads this physical buffer — i.e. two halves from now in the
    // ring. mod HALVES_PER_FRAME because the descriptor ring wraps.
    let next_chunk =
        (just_finished_half + 2 + STARTUP_OFFSET_HALVES) % HALVES_PER_FRAME;

    // Tear-free flip point: the refill that produces chunk 0 of the
    // next frame is the first opportunity to bring new-buffer content
    // into the bounce ring. If we update PSRAM_FB_PTR right before
    // doing that refill, this ISR (chunk 0) and the next ISR
    // (chunk 1) both pull from the new front, so when DMA wraps to
    // start the next frame both look-ahead halves are new and the
    // transition is seamless.
    //
    // Done unconditionally inside the ISR rather than in a separate
    // function so it's atomic with respect to other EOFs.
    if next_chunk == 0 {
        let pending = PENDING_SWAP.swap(core::ptr::null_mut(), Ordering::AcqRel);
        if !pending.is_null() {
            PSRAM_FB_PTR.store(pending, Ordering::Release);
        }
    }

    // SAFETY: PSRAM_FB_PTR is set before the EOF interrupt is enabled.
    let psram_base = PSRAM_FB_PTR.load(Ordering::Acquire);
    unsafe {
        psram_copy_wrapping(psram_base, next_chunk, dest);
    }

    EOF_COUNT.fetch_add(1, Ordering::Relaxed);

    // Acknowledge so the controller can refire next half.
    dma.ch(0)
        .out_int()
        .clr()
        .write(|w| w.out_eof().clear_bit_by_one());
}

/// Copy one bounce-half's worth of pixels from PSRAM into `dest`,
/// applying [`STARTUP_OFFSET_BYTES`] of sub-half line shift and
/// handling the framebuffer-end wrap-around.
///
/// # Safety
///
/// `psram_base` must point to at least [`framebuffer::BYTES`] bytes
/// of valid PSRAM; `dest` must own at least [`HALF_BYTES`] writable
/// bytes; both must not alias.
#[inline]
unsafe fn psram_copy_wrapping(psram_base: *const u8, chunk_idx: usize, dest: *mut u8) { unsafe {
    let natural = chunk_idx * HALF_BYTES;
    let start = (natural + framebuffer::BYTES - STARTUP_OFFSET_BYTES) % framebuffer::BYTES;
    let end = start + HALF_BYTES;

    if end <= framebuffer::BYTES {
        core::ptr::copy_nonoverlapping(psram_base.add(start), dest, HALF_BYTES);
    } else {
        let first = framebuffer::BYTES - start;
        let second = HALF_BYTES - first;
        core::ptr::copy_nonoverlapping(psram_base.add(start), dest, first);
        core::ptr::copy_nonoverlapping(psram_base, dest.add(first), second);
    }
}}
