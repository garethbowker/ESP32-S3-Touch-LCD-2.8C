//! Touch demo: black background, draw white crosshairs tracking the
//! current finger position, briefly flash the panel white on a tap.
//!
//! Definitions used here:
//! - **tap** — finger touched the panel and lifted without travelling
//!   more than `TAP_SLOP_PX` from its landing point.
//! - **drag** — finger travelled further than that before lifting.
//!
//! Draw strategy: **delta updates** against a tracked per-buffer
//! cross position. Each frame writes one row + one column (the new
//! cross) plus, if the cross was in a different place on this buffer
//! last time, the old row + old column erased back to BG. That's at
//! most 4 lines per frame (≈ 4 KB of pixel writes), vs a full
//! 460 KB fill — about a 100× reduction in PSRAM bandwidth per frame.
//!
//! Build & flash:
//!
//! ```sh
//! cd crates/ws-esp32-s3-touch-lcd-2-8c
//! cargo run --release --example hello_touch
//! ```
//!
//! Requires `espflash` and the Espressif Rust toolchain (`espup install`).

#![no_std]
#![no_main]

extern crate alloc;
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_2_8c as bsp;

esp_bootloader_esp_idf::esp_app_desc!();

/// RGB565 colours.
const BG: u16 = 0x0000; // black
const CROSS: u16 = 0xFFFF; // white
const FLASH: u16 = 0xFFFF; // white

/// Chebyshev distance below which a press+release counts as a tap.
const TAP_SLOP_PX: u32 = 15;
/// Flash duration on a recognised tap. 60 ms is short enough to feel
/// snappy but covers 2–3 full panel refresh cycles at ~43 Hz so the
/// flash is clearly perceived.
const FLASH_MS: u64 = 60;
/// Touch-poll interval. 20 ms = 50 Hz, well above the GT911's ~100 Hz
/// internal scan rate but slow enough that draw work has headroom.
const POLL_MS: u64 = 20;

struct Press {
    start: (u16, u16),
    last: (u16, u16),
    /// Chebyshev distance from `start` reached at any point in the gesture.
    max_dist: u32,
}

#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_psram(esp_hal::psram::PsramConfig {
            size: esp_hal::psram::PsramSize::Size(8 * 1024 * 1024),
            ..Default::default()
        }),
    );

    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    esp_println::logger::init_logger_from_env();

    let mut board = bsp::init(bsp::take_resources!(peripherals))
        .expect("board init");

    log::info!("board up — touch the panel");

    let mut press: Option<Press> = None;
    // Per-buffer state: what cross (if any) is currently drawn in each
    // buffer. Both start clean (init() zeros both buffers). `back_is_a`
    // mirrors the BSP's internal flag — we toggle it on each flip.
    let mut a_cross: Option<(u16, u16)> = None;
    let mut b_cross: Option<(u16, u16)> = None;
    let mut back_is_a = true;

    loop {
        let target: Option<(u16, u16)> = match board.touch.poll().await {
            Ok(Some(p)) => {
                let xy = (p.x, p.y);
                press = Some(match press.take() {
                    None => Press { start: xy, last: xy, max_dist: 0 },
                    Some(prev) => Press {
                        start: prev.start,
                        last: xy,
                        max_dist: prev.max_dist.max(chebyshev(prev.start, xy)),
                    },
                });
                Some(xy)
            }
            Ok(None) => {
                if let Some(p) = press.take() {
                    if p.max_dist <= TAP_SLOP_PX {
                        log::info!("tap @ ({}, {})", p.last.0, p.last.1);
                        flash(
                            &board.framebuffer,
                            FLASH,
                            &mut a_cross,
                            &mut b_cross,
                            &mut back_is_a,
                        )
                        .await;
                    } else {
                        log::info!(
                            "drag {:?} → {:?} ({} px)",
                            p.start, p.last, p.max_dist
                        );
                    }
                }
                None
            }
            Err(bsp::gt911::Error::NotReady) => {
                // No new data — keep showing whatever's on the panel.
                embassy_time::Timer::after(embassy_time::Duration::from_millis(POLL_MS)).await;
                continue;
            }
            Err(e) => {
                log::warn!("gt911: {:?}", e);
                embassy_time::Timer::after(embassy_time::Duration::from_millis(POLL_MS)).await;
                continue;
            }
        };

        // What's drawn in the buffer we're about to write into?
        let back_cross = if back_is_a { a_cross } else { b_cross };

        if back_cross != target {
            // Delta: erase old cross (if any), draw new cross (if any).
            if let Some((x, y)) = back_cross {
                draw_cross(&board.framebuffer, x, y, BG);
            }
            if let Some((x, y)) = target {
                draw_cross(&board.framebuffer, x, y, CROSS);
            }
            board.framebuffer.flip();

            // Track what's in this buffer; toggle which one is back.
            if back_is_a {
                a_cross = target;
            } else {
                b_cross = target;
            }
            back_is_a = !back_is_a;
        }

        embassy_time::Timer::after(embassy_time::Duration::from_millis(POLL_MS)).await;
    }
}

/// Draw or erase a full-screen crosshair at `(x, y)` with the given
/// colour. With `BG` this acts as an erase.
fn draw_cross(fb: &bsp::Framebuffer, x: u16, y: u16, colour: u16) {
    let x = (x as usize).min(bsp::WIDTH - 1);
    let y = (y as usize).min(bsp::HEIGHT - 1);
    fb.draw_row_solid(y, 0..bsp::WIDTH, colour);
    fb.draw_column(x, 0..bsp::HEIGHT, colour);
}

/// Briefly flash the panel to `colour`, then settle on clean BG.
///
/// Three fills + three flips total:
/// 1. Front transitions to `colour`.
/// 2. Hold for [`FLASH_MS`] — only the front needs to be `colour`, no
///    flips during the hold so the other buffer can stay stale.
/// 3. Front transitions to BG.
/// 4. New back (still holds the flash colour) is filled with BG so
///    subsequent delta updates have a clean starting state on either
///    buffer.
///
/// Costs ~3 × 8 ms (memset-fast fills) + `FLASH_MS` = ~85 ms wall.
/// The earlier four-fill version was ~140 ms.
async fn flash(
    fb: &bsp::Framebuffer,
    colour: u16,
    a_cross: &mut Option<(u16, u16)>,
    b_cross: &mut Option<(u16, u16)>,
    back_is_a: &mut bool,
) {
    fb.fill(colour);
    fb.flip();
    embassy_time::Timer::after(embassy_time::Duration::from_millis(FLASH_MS)).await;
    fb.fill(BG);
    fb.flip();
    fb.fill(BG);
    fb.flip();
    // Three flips toggle the back/front assignment exactly once. Both
    // buffers are now BG, so wipe the tracked cross positions.
    *a_cross = None;
    *b_cross = None;
    *back_is_a = !*back_is_a;
}

/// Chebyshev (L∞) distance — `max(|dx|, |dy|)`. Cheaper than Euclidean,
/// good enough for "did the finger stay within a square of side N".
fn chebyshev(a: (u16, u16), b: (u16, u16)) -> u32 {
    let dx = (a.0 as i32 - b.0 as i32).unsigned_abs();
    let dy = (a.1 as i32 - b.1 as i32).unsigned_abs();
    dx.max(dy)
}
