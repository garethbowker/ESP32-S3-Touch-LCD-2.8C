//! PSRAM-backed RGB565 framebuffer for the DPI panel — double-buffered.
//!
//! Backing the framebuffer with PSRAM rather than DRAM is the only way
//! to fit a 480×480×2 B = 460 800 B buffer on the ESP32-S3 (twice that
//! for the double-buffer pair). Internal DRAM is ~512 KB total and
//! most of that is needed for stack, .bss, and any heap the user's UI
//! toolkit allocates against.
//!
//! ## Mode: PSRAM master + DRAM bounce buffers
//!
//! Single-FB-in-PSRAM mode (DMA reads PSRAM directly) was tried at
//! 18, 12, and 8 MHz PCLK; all produced diagonal "drift" stripes.
//! Halving PCLK doubled the stripe density — proving the drift trigger
//! is at a fixed time rate (PSRAM refresh cycles) and not a bandwidth
//! problem. ESP-IDF's documented fix is **bounce buffer mode**:
//!
//! 1. CPU writes into the current back PSRAM framebuffer.
//! 2. Two small DRAM bounce halves sit between PSRAM and the DPI DMA
//!    engine. The EOF ISR memcpy-refills each half from the current
//!    *front* buffer through the CPU cache.
//!
//! Net effect: DMA only ever reads DRAM, drift is gone, and we don't
//! need explicit cache-flush instructions on every CPU pixel write —
//! the cache writeback in [`Framebuffer::write_row`] is enough.
//!
//! ## Double-buffering
//!
//! Two complete framebuffers live back-to-back in PSRAM. The CPU
//! writes into whichever is currently "back"; the bounce-buffer ISR
//! reads from whichever is currently "front". [`Framebuffer::flip`]
//! atomically swaps the roles and memcpy-syncs the new back from the
//! new front, so the user can keep doing delta updates against the
//! state that's currently on the panel.
//!
//! The flip itself is atomic at the pointer level, but because the
//! bounce-buffer DMA looks ~one half-period ahead, the transition
//! from old to new content spans roughly one frame (~23 ms at 12 MHz
//! PCLK). Tearing within a single frame is eliminated.
//!
//! ## Aliasing model
//!
//! [`Framebuffer`] holds raw pointers to the same PSRAM bytes that
//! [`super::bounce_buffer`]'s EOF ISR reads via
//! `core::ptr::copy_nonoverlapping`. The CPU only ever writes to the
//! back buffer and the ISR only ever reads from the front buffer —
//! they never touch the same buffer concurrently, so no shared `&mut`
//! is ever formed.

use core::ops::Range;
use core::sync::atomic::{AtomicBool, Ordering};

/// Logical width of the visible framebuffer, in pixels.
pub const WIDTH: usize = 480;
/// Logical height of the visible framebuffer, in pixels.
pub const HEIGHT: usize = 480;
/// Bytes in one framebuffer (RGB565 = 2 bytes per pixel).
pub const BYTES: usize = WIDTH * HEIGHT * 2;
/// Total PSRAM bytes required (two buffers, back-to-back).
pub const PSRAM_BYTES_REQUIRED: usize = BYTES * 2;

/// CPU-side write handle to the PSRAM framebuffer pair.
///
/// Constructed by [`super::init`]; one per board. Writes go to the
/// back buffer; [`Self::flip`] makes them visible.
pub struct Framebuffer {
    a: *mut u8,
    b: *mut u8,
    /// `true` means buffer A is currently the back (CPU-writable) one.
    /// `false` means B is back. Atomic so [`Self::flip`] is safe to
    /// call from any context.
    back_is_a: AtomicBool,
}

unsafe impl Send for Framebuffer {}
unsafe impl Sync for Framebuffer {}

impl Framebuffer {
    /// Carve a pair of framebuffers off the start of the PSRAM region.
    ///
    /// Returns `Some((initial_front_ptr, fb))` if the region is at
    /// least [`PSRAM_BYTES_REQUIRED`] bytes. `initial_front_ptr` is
    /// what the bounce-buffer ring should be primed with — the EOF
    /// ISR's source pointer is later updated by [`Self::flip`].
    ///
    /// Both buffers are zeroed (black) before return, so the initial
    /// frame on the panel is solid black rather than uninitialised
    /// PSRAM.
    ///
    /// # Safety
    ///
    /// `psram_ptr` must point to at least [`PSRAM_BYTES_REQUIRED`]
    /// bytes of valid, CPU-mapped, DMA-capable PSRAM, and the caller
    /// must guarantee nothing else accesses that range for the
    /// lifetime of the returned values.
    pub unsafe fn split_from_psram(
        psram_ptr: *mut u8,
        psram_size: usize,
    ) -> Option<(*mut u8, Self)> { unsafe {
        if psram_size < PSRAM_BYTES_REQUIRED {
            return None;
        }
        let a = psram_ptr;
        let b = psram_ptr.add(BYTES);

        // Initial state: A is front (ISR reads), B is back (CPU writes).
        let fb = Self {
            a,
            b,
            back_is_a: AtomicBool::new(false),
        };

        // Zero both buffers so the first frame on the panel is black
        // rather than uninitialised PSRAM.
        fb.fill_buffer(a, 0x0000);
        fb.fill_buffer(b, 0x0000);

        Some((a, fb))
    }}

    fn back_ptr(&self) -> *mut u8 {
        if self.back_is_a.load(Ordering::Acquire) {
            self.a
        } else {
            self.b
        }
    }

    /// Write a span of `range.len()` RGB565 pixels into row `y` of the
    /// back buffer, then flush those bytes from the D-cache back to
    /// physical PSRAM.
    ///
    /// The flush is load-bearing: empirically the cache lines aren't
    /// reliably written back on eviction, so when this buffer later
    /// becomes the front and the EOF ISR reads from it, it would see
    /// stale / uninitialised PSRAM otherwise.
    ///
    /// Bounds-checked; out-of-range coordinates silently no-op rather
    /// than panic, so a UI toolkit's `LineBufferProvider`-style API
    /// can pass renderer-supplied ranges without risking a crash on a
    /// partial line outside the visible area.
    pub fn write_row(&self, y: usize, range: Range<usize>, pixels: &[u16]) {
        if y >= HEIGHT {
            return;
        }
        let end = range.end.min(WIDTH);
        if range.start >= end {
            return;
        }
        let n = (end - range.start).min(pixels.len());
        unsafe {
            let dst = (self.back_ptr() as *mut u16).add(y * WIDTH + range.start);
            for i in 0..n {
                dst.add(i).write_volatile(pixels[i]);
            }
            cache_writeback(dst as u32, (n * core::mem::size_of::<u16>()) as u32);
        }
    }

    /// Fill the back buffer with a single RGB565 colour.
    pub fn fill(&self, colour: u16) {
        self.fill_buffer(self.back_ptr(), colour);
    }

    /// Paint a horizontal solid-colour run on row `y`, columns
    /// `x_range`, with a single cache flush at the end.
    ///
    /// Equivalent to [`Self::write_row`] with a uniform pixel slice,
    /// but skips the slice-indexing overhead and avoids allocating /
    /// stacking a temporary buffer at the call site.
    pub fn draw_row_solid(&self, y: usize, x_range: Range<usize>, colour: u16) {
        if y >= HEIGHT {
            return;
        }
        let end = x_range.end.min(WIDTH);
        if x_range.start >= end {
            return;
        }
        let n = end - x_range.start;
        unsafe {
            let dst = (self.back_ptr() as *mut u16).add(y * WIDTH + x_range.start);
            for i in 0..n {
                dst.add(i).write_volatile(colour);
            }
            cache_writeback(dst as u32, (n * core::mem::size_of::<u16>()) as u32);
        }
    }

    /// Paint a vertical solid-colour run at column `x`, rows
    /// `y_range`, with **one** cache flush covering the whole column
    /// at the end.
    ///
    /// Doing this with [`Self::write_row`] would mean 480 separate
    /// `cache_writeback` calls; the ROM `Cache_Suspend_DCache_Autoload`
    /// / `Resume` pair around each one has enough fixed overhead that
    /// 480 calls measures slower than a single call walking a ~460 KB
    /// range. (Tried both empirically.)
    ///
    /// The flush range covers all bytes from the first to the last
    /// written pixel — the ROM `Cache_WriteBack_Addr` walks tags in
    /// that range and only writes back lines that are actually dirty.
    pub fn draw_column(&self, x: usize, y_range: Range<usize>, colour: u16) {
        if x >= WIDTH {
            return;
        }
        let end = y_range.end.min(HEIGHT);
        if y_range.start >= end {
            return;
        }
        unsafe {
            let base = self.back_ptr() as *mut u16;
            for y in y_range.start..end {
                base.add(y * WIDTH + x).write_volatile(colour);
            }
            // Flush range: from first written pixel to one past the
            // last. Includes the (WIDTH-1) clean pixels per row in
            // between, but those tags get walked-and-skipped quickly.
            let start_addr = base.add(y_range.start * WIDTH + x) as u32;
            let end_addr = base.add((end - 1) * WIDTH + x).add(1) as u32;
            cache_writeback(start_addr, end_addr - start_addr);
        }
    }

    fn fill_buffer(&self, base: *mut u8, colour: u16) {
        unsafe {
            let lo = (colour & 0xFF) as u8;
            let hi = (colour >> 8) as u8;
            if lo == hi {
                // Uniform-byte fast path: black (0x0000), white (0xFFFF),
                // and other byte-symmetric colours hit a hardware-optimised
                // `memset` rather than the per-pixel write loop. About 2×
                // faster on this SoC because the compiler emits Xtensa
                // block stores instead of a one-u16-at-a-time loop, and the
                // cache lines are dirtied in straight sequential order.
                core::ptr::write_bytes(base, lo, BYTES);
            } else {
                let dst = base as *mut u16;
                for i in 0..(WIDTH * HEIGHT) {
                    core::ptr::write(dst.add(i), colour);
                }
            }
            cache_writeback(base as u32, BYTES as u32);
        }
    }

    /// Tear-free swap of front and back buffers.
    ///
    /// Queues the swap and blocks until the bounce-buffer ISR applies
    /// it at the precise EOF that makes the next frame fully new
    /// content (no scanline shows half-old / half-new, no top-of-frame
    /// stale-content band). Average wait is ~half a frame (~11 ms at
    /// 12 MHz PCLK); worst case ~one frame (~23 ms).
    ///
    /// On return:
    /// - the bounce-buffer ISR is now reading the new front,
    /// - subsequent CPU writes via [`Self::write_row`] etc. go to the
    ///   new back (= the previous front), which the ISR is no longer
    ///   reading and is therefore safe to mutate.
    ///
    /// The new back is **not** synced from the new front — its
    /// contents are whatever the caller's previous frame left in it.
    /// Callers doing delta updates against panel state should track
    /// per-buffer state themselves; callers doing full redraws each
    /// frame can ignore this.
    pub fn flip(&self) {
        let was_back_a = self.back_is_a.load(Ordering::Acquire);
        let new_front = if was_back_a { self.a } else { self.b };

        // Wait for any previously queued (but not yet consumed) flip
        // to be applied. Without this we could overwrite that
        // pending pointer with our own and the caller of the earlier
        // flip() would falsely conclude theirs took effect.
        while crate::bounce_buffer::is_flip_pending() {
            core::hint::spin_loop();
        }

        crate::bounce_buffer::request_flip(new_front);

        // Wait until the ISR applies the swap at the next end-of-frame
        // alignment point. After this returns, the ISR is reading from
        // `new_front` and the previous front (= our new back) is free
        // for the caller to mutate.
        while crate::bounce_buffer::is_flip_pending() {
            core::hint::spin_loop();
        }

        self.back_is_a.store(!was_back_a, Ordering::Release);
    }

    /// Queue a swap and return immediately, without waiting for it to
    /// take effect.
    ///
    /// **Caveat:** until the ISR consumes the queued swap (which can
    /// be up to ~one frame later), the panel is still reading from
    /// the old front *and* the caller's perceived "new back" is in
    /// fact that same old front. Writing to the new back during this
    /// window can race the bounce-buffer ISR's reads, putting torn
    /// pixels on the panel.
    ///
    /// Safe to use only if the caller doesn't write to the back
    /// buffer between this call and the next [`Self::flip`].
    pub fn flip_no_sync(&self) {
        let was_back_a = self.back_is_a.load(Ordering::Acquire);
        let new_front = if was_back_a { self.a } else { self.b };
        crate::bounce_buffer::request_flip(new_front);
        self.back_is_a.store(!was_back_a, Ordering::Release);
    }
}

/// Flush the L1 D-cache lines covering `[addr, addr + size)` back to
/// the underlying PSRAM. Wraps the ESP32-S3 ROM helpers — esp-hal
/// has a thin pub-internal wrapper for the same code
/// (`soc::esp32s3::cache_writeback_addr`) but doesn't re-export it.
///
/// The `Cache_Suspend_DCache_Autoload` dance is from the ROM driver:
/// it stops the cache controller from pulling new lines in while
/// `Cache_WriteBack_Addr` walks the tags, so the writeback can't
/// race with an autoload and end up flushing freshly-pulled-in stale
/// data.
///
/// # Safety
///
/// `addr` must be in a PSRAM-mapped address range and `size` must be
/// within bounds of that mapping; otherwise the ROM helper can flush
/// adjacent unrelated cache lines.
#[inline]
unsafe fn cache_writeback(addr: u32, size: u32) { unsafe {
    unsafe extern "C" {
        fn rom_Cache_WriteBack_Addr(addr: u32, size: u32);
        fn Cache_Suspend_DCache_Autoload() -> u32;
        fn Cache_Resume_DCache_Autoload(value: u32);
    }
    let autoload = Cache_Suspend_DCache_Autoload();
    rom_Cache_WriteBack_Addr(addr, size);
    Cache_Resume_DCache_Autoload(autoload);
}}
