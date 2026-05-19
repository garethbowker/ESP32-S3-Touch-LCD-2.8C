//! Integration demo: every BSP peripheral running concurrently on
//! the shared async I²C bus, with on-screen readouts.
//!
//! Three spawned tasks sharing one `embassy_sync::Mutex<I2c>`:
//!
//! - **touch_task** — 50 Hz GT911 poll, beeps on every tap.
//! - **sensor_task** — 10 Hz IMU + 1 Hz RTC poll, hands readings
//!   to the renderer.
//! - **render_task** — repaints at ~10 Hz, but only the pixels
//!   that actually changed. Per-buffer state tracking eliminates
//!   the full-screen-fill flicker.
//!
//! Visuals fit the 480 px *inscribed circle* of the round panel —
//! all text and indicators stay inside the visible disc.
//!
//! Build & flash:
//!
//! ```sh
//! cd crates/ws-esp32-s3-touch-lcd-2-8c
//! cargo run --release --example hello_all --features rtc,imu
//! ```

#![no_std]
#![no_main]

extern crate alloc;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::Text,
};
use esp_backtrace as _;
use esp_hal::efuse::{Efuse, WAFER_VERSION_MAJOR, WAFER_VERSION_MINOR_HI, WAFER_VERSION_MINOR_LO};
use ws_esp32_s3_touch_lcd_2_8c::{
    self as bsp,
    rtc::{Date, Month, PrimitiveDateTime, Time},
    Buzzer, Framebuffer, Imu, Rtc, TouchPoller, HEIGHT, WIDTH,
};

esp_bootloader_esp_idf::esp_app_desc!();

// ---------------------------------------------------------------------------
// Visual constants
// ---------------------------------------------------------------------------

const BG:        Rgb565 = Rgb565::BLACK;
const TEXT:      Rgb565 = Rgb565::WHITE;
const BUBBLE:    Rgb565 = Rgb565::CYAN;
const REFERENCE: Rgb565 = Rgb565::new(8, 16, 8); // dim grey-green centre marker
const CROSS:     Rgb565 = Rgb565::YELLOW;
const FLASH:     Rgb565 = Rgb565::WHITE;

const FONT_W: i32 = 10;
const FONT_H: i32 = 20;

/// Inscribed-circle bounds for the 480×480 round panel — all
/// on-screen elements stay inside this disc.
const CENTRE_X: i32 = WIDTH as i32 / 2;
const CENTRE_Y: i32 = HEIGHT as i32 / 2;

const BUBBLE_R: i32 = 30;
const CROSS_HALF: i32 = 14;

const TOUCH_POLL_MS: u64 = 20;
const IMU_POLL_MS: u64 = 100;
const RTC_POLL_MS: u64 = 1_000;
const RENDER_TICK_MS: u64 = 100;
const FLASH_MS: u64 = 80;

/// EMA factor for IMU smoothing. New = old + (raw − old) / SMOOTH.
/// 4 = ~0.4 s time constant at the 100 ms sample rate — fast enough
/// to track real motion, slow enough that the per-sample noise is
/// well below the display's least-significant digit.
const SMOOTH: i32 = 4;

/// Sensor scale tables. The BSP's `Imu::new` configures the chip for
/// `AccelRange::G8` (4096 LSB/g, the driver default) and
/// `GyroRange::Dps64` (512 LSB/dps, overriding the driver default of
/// Dps512 so handheld-scale rotations have useful resolution).
const ACCEL_LSB_PER_G: i32 = 4_096;
const GYRO_LSB_PER_DPS: i32 = 512;

/// Hysteresis bands on the *displayed* value. The shown digits
/// only update when the smoothed reading moves further from the
/// current display value than this. 20 mg ≈ ±1° tilt detection;
/// 500 mdps ≈ slow rotation.
const ACCEL_HYST_MG:  i32 = 20;
/// Hysteresis around the currently-displayed gyro value. Wider than
/// the chip's per-LSB resolution so the displayed digits don't
/// flicker on noise, but narrow enough that real rotation registers
/// immediately.
const GYRO_HYST_MDPS: i32 = 500;
/// **Snap-to-zero** band. Smoothed gyro readings inside ±this are
/// shown as exactly 0 °/s. Without this, when a rotation stops the
/// hysteresis just leaves the display at whatever small value the
/// motion decayed to (e.g. +0.2 °/s). Picked wider than the chip's
/// post-warm-up zero-rate drift so a still board reads zero
/// regardless of which way it was just moving.
const GYRO_ZERO_BAND_MDPS: i32 = 500;

/// Quantisation step applied when the hysteresis band is crossed.
const ACCEL_QUANT_MG:  i32 = 10;
const GYRO_QUANT_MDPS: i32 = 100;

// ---------------------------------------------------------------------------
// Layout (x,y) — picked to fit the inscribed circle
// ---------------------------------------------------------------------------

const TIME_POS:  Point = Point::new(195, 80);
const ACCEL_POS: [Point; 3] = [
    Point::new(60, 110),
    Point::new(60, 135),
    Point::new(60, 160),
];
const GYRO_POS: [Point; 3] = [
    Point::new(295, 110),
    Point::new(295, 135),
    Point::new(295, 160),
];
const TAPS_POS:  Point = Point::new(195, 360);
const MAC_POS:   Point = Point::new(125, 390);
const CHIP_POS:  Point = Point::new(155, 420);

/// Region covered by a `n`-character FONT_10X20 string anchored at
/// `anchor` (which `embedded_graphics::Text::new` interprets as the
/// *baseline*-left corner). Used for erase-old-then-draw-new
/// delta updates.
const fn text_box(anchor: Point, n: i32) -> (Point, Size) {
    // FONT_10X20: baseline sits ~16 px below the top of the glyph.
    let top_y = anchor.y - 16;
    (Point::new(anchor.x, top_y), Size::new((n * FONT_W) as u32, FONT_H as u32))
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

static ACCEL_SIG: Signal<CriticalSectionRawMutex, [i16; 3]> = Signal::new();
static GYRO_SIG:  Signal<CriticalSectionRawMutex, [i16; 3]> = Signal::new();
static CLOCK_SIG: Signal<CriticalSectionRawMutex, PrimitiveDateTime> = Signal::new();
static TOUCH_SIG: Signal<CriticalSectionRawMutex, Option<(u16, u16)>> = Signal::new();
static FLASH_REQ: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static TAP_COUNT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
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

    let mut board = bsp::init(bsp::take_resources!(peripherals)).expect("board init");

    // ---- Device info ----
    let mac = Efuse::read_base_mac_address();
    let wafer_major: u8 = Efuse::read_field_le(WAFER_VERSION_MAJOR);
    let wafer_minor_lo: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR_LO);
    let wafer_minor_hi: u8 = Efuse::read_field_le(WAFER_VERSION_MINOR_HI);
    let wafer_minor = (wafer_minor_hi << 3) | (wafer_minor_lo & 0x7);
    let (block_major, block_minor) = Efuse::block_version();
    let mut mac_str = heapless::String::<24>::new();
    let _ = core::fmt::write(
        &mut mac_str,
        format_args!(
            "MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        ),
    );
    let mut chip_str = heapless::String::<24>::new();
    let _ = core::fmt::write(
        &mut chip_str,
        format_args!("ESP32-S3 r{}.{}", wafer_major, wafer_minor),
    );
    log::info!(
        "device: {} | chip ESP32-S3 r{}.{} | efuse block r{}.{} | flash-enc {}",
        mac_str.as_str(),
        wafer_major, wafer_minor,
        block_major, block_minor,
        Efuse::flash_encryption(),
    );

    let seed = PrimitiveDateTime::new(
        Date::from_calendar_date(2026, Month::May, 19).unwrap(),
        Time::from_hms(15, 0, 0).unwrap(),
    );
    if let Err(e) = board.rtc.set_datetime(&seed).await {
        log::warn!("rtc seed failed: {:?}", e);
    } else {
        log::info!("rtc seeded to {seed}");
    }

    let mut delay = Delay;
    board.imu.init(&mut delay).await.expect("imu init");
    log::info!("hello_all up — tilt to move bubble, tap to beep + flash");

    let touch = board.touch;
    let buzzer = board.buzzer;
    let imu = board.imu;
    let rtc = board.rtc;
    let framebuffer = board.framebuffer;
    core::mem::forget(board.backlight);

    spawner.spawn(touch_task(touch, buzzer)).expect("spawn touch");
    spawner.spawn(sensor_task(imu, rtc)).expect("spawn sensor");
    spawner.spawn(render_task(framebuffer, mac_str, chip_str)).expect("spawn render");

    loop {
        Timer::after(Duration::from_secs(10)).await;
        log::info!("hello_all alive (taps={})", TAP_COUNT.load(Ordering::Relaxed));
    }
}

// ---------------------------------------------------------------------------
// Touch + buzzer task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn touch_task(mut touch: TouchPoller, buzzer: Buzzer) {
    let mut pressed = false;
    loop {
        match touch.poll().await {
            Ok(Some(p)) => {
                TOUCH_SIG.signal(Some((p.x, p.y)));
                if !pressed {
                    pressed = true;
                    TAP_COUNT.fetch_add(1, Ordering::Relaxed);
                    log::info!("tap @ ({}, {})", p.x, p.y);
                    buzzer.beep_for(Duration::from_millis(60)).await;
                    FLASH_REQ.signal(());
                }
            }
            Ok(None) => {
                if pressed {
                    pressed = false;
                    TOUCH_SIG.signal(None);
                }
            }
            Err(bsp::gt911::Error::NotReady) => {}
            Err(e) => log::warn!("gt911: {:?}", e),
        }
        Timer::after(Duration::from_millis(TOUCH_POLL_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// IMU + RTC task — applies EMA smoothing so display values stop
// jittering on a still board
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn sensor_task(mut imu: Imu, mut rtc: Rtc) {
    // ---- Gyro zero-rate calibration ----
    //
    // Every QMI8658 has a non-zero gyro bias at rest. Cold-from-power-on,
    // the bias also drifts as the silicon thermally stabilises — sampling
    // immediately after init catches a "cold" value that doesn't reflect
    // steady-state. 1 s warm-up then 2 s of averaging gives a much more
    // representative estimate, at the cost of the user having to hold
    // the board still for ~3 s after flash.
    log::info!("gyro: warming up...");
    for _ in 0..20 {
        let _ = imu.driver_mut().read_gyro_raw().await;
        Timer::after(Duration::from_millis(50)).await;
    }
    log::info!("gyro: calibrating (hold board still for ~2 s)...");
    let mut bias_acc = [0i32; 3];
    let n_samples = 40i32;
    for _ in 0..n_samples {
        if let Ok(s) = imu.driver_mut().read_gyro_raw().await {
            bias_acc[0] += s.data.x as i32;
            bias_acc[1] += s.data.y as i32;
            bias_acc[2] += s.data.z as i32;
        }
        Timer::after(Duration::from_millis(50)).await;
    }
    let gyro_bias = [
        bias_acc[0] / n_samples,
        bias_acc[1] / n_samples,
        bias_acc[2] / n_samples,
    ];
    log::info!(
        "gyro zero-rate bias (raw LSB): x={} y={} z={}",
        gyro_bias[0], gyro_bias[1], gyro_bias[2],
    );

    let mut ema_a = [0i32; 3];
    let mut ema_g = [0i32; 3];
    let mut last_rtc = embassy_time::Instant::now();

    loop {
        if let Ok(s) = imu.driver_mut().read_accel_raw().await {
            let raw = [s.data.x as i32, s.data.y as i32, s.data.z as i32];
            for i in 0..3 {
                ema_a[i] += (raw[i] - ema_a[i]) / SMOOTH;
            }
            ACCEL_SIG.signal([ema_a[0] as i16, ema_a[1] as i16, ema_a[2] as i16]);
        }
        if let Ok(s) = imu.driver_mut().read_gyro_raw().await {
            // Bias-corrected raw counts.
            let raw = [
                (s.data.x as i32) - gyro_bias[0],
                (s.data.y as i32) - gyro_bias[1],
                (s.data.z as i32) - gyro_bias[2],
            ];
            for i in 0..3 {
                ema_g[i] += (raw[i] - ema_g[i]) / SMOOTH;
            }
            GYRO_SIG.signal([ema_g[0] as i16, ema_g[1] as i16, ema_g[2] as i16]);
        }

        let now = embassy_time::Instant::now();
        if (now - last_rtc).as_millis() >= RTC_POLL_MS {
            if let Ok(dt) = rtc.get_datetime().await {
                CLOCK_SIG.signal(dt);
            }
            last_rtc = now;
        }

        Timer::after(Duration::from_millis(IMU_POLL_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// Per-buffer rendered state. The render loop keeps one of these for
// each of the framebuffer's two physical buffers so it can erase old
// pixels and draw new ones without a full-screen fill.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct Snapshot {
    /// `false` means "this buffer hasn't been initialised yet" — the
    /// renderer paints everything from scratch on the next pass.
    initialised: bool,
    time:  heapless::String<10>,
    accel: [heapless::String<12>; 3],
    gyro:  [heapless::String<12>; 3],
    taps:  u32,
    bubble: (i32, i32),
    touch: Option<(i32, i32)>,
}

impl Snapshot {
    fn empty() -> Self {
        Self {
            initialised: false,
            time: heapless::String::new(),
            accel: [heapless::String::new(), heapless::String::new(), heapless::String::new()],
            gyro:  [heapless::String::new(), heapless::String::new(), heapless::String::new()],
            taps: 0,
            bubble: (CENTRE_X, CENTRE_Y),
            touch: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Render task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn render_task(
    framebuffer: Framebuffer,
    mac_str: heapless::String<24>,
    chip_str: heapless::String<24>,
) {
    let mut a_drawn = Snapshot::empty();
    let mut b_drawn = Snapshot::empty();
    let mut back_is_a = true;

    // Persistent hysteresis state — *displayed* values in milli-units.
    // Updated only when the smoothed reading moves further than the
    // hysteresis band, then quantised to a clean step. Without this
    // the last digit jitters constantly on a still board.
    let mut shown_accel_mg:  [i32; 3] = [0; 3];
    let mut shown_gyro_mdps: [i32; 3] = [0; 3];

    let mut latest_accel = [0i16; 3];
    let mut latest_gyro  = [0i16; 3];
    let mut latest_clock: Option<PrimitiveDateTime> = None;
    let mut latest_touch: Option<(u16, u16)> = None;

    loop {
        if let Some(a) = ACCEL_SIG.try_take() { latest_accel = a; }
        if let Some(g) = GYRO_SIG.try_take() { latest_gyro = g; }
        if let Some(c) = CLOCK_SIG.try_take() { latest_clock = Some(c); }
        if let Some(t) = TOUCH_SIG.try_take() { latest_touch = t; }

        // Apply hysteresis on the *displayed* value, in physical units.
        // Gyro: when the smoothed reading falls inside the snap-to-zero
        // band, force the display to exactly 0 — otherwise the
        // hysteresis just lingers at whatever tiny value a stopped
        // rotation decayed to.
        for i in 0..3 {
            let mg = raw_accel_to_mg(latest_accel[i]);
            if (mg - shown_accel_mg[i]).abs() >= ACCEL_HYST_MG {
                shown_accel_mg[i] = quantize(mg, ACCEL_QUANT_MG);
            }
            let mdps = raw_gyro_to_mdps(latest_gyro[i]);
            if mdps.abs() < GYRO_ZERO_BAND_MDPS {
                shown_gyro_mdps[i] = 0;
            } else if (mdps - shown_gyro_mdps[i]).abs() >= GYRO_HYST_MDPS {
                shown_gyro_mdps[i] = quantize(mdps, GYRO_QUANT_MDPS);
            }
        }

        // Tap flash: paint the back buffer white and flip. The flip
        // toggles BSP-internal back/front, so mirror that here or the
        // diff-based partial redraws after this point will be working
        // against the wrong baseline.
        //
        // We also flip *back* afterwards using a second white-fill so
        // both physical buffers are clean white before the heavy
        // post-flash redraw begins. Without the second flip the
        // following render iteration has to do a full-screen BG fill
        // (460 KB PSRAM write) immediately after returning from
        // sleep, and that has been observed to momentarily starve the
        // bounce-buffer EOF ISR — producing a single-frame ~1/3
        // vertical shift as the DMA loses alignment for one frame.
        // Two clean buffers + two short fills give the panel pipeline
        // a quiet 80 ms of identical content to resync against.
        if FLASH_REQ.try_take().is_some() {
            framebuffer.fill(FLASH.into_storage());
            framebuffer.flip();
            framebuffer.fill(FLASH.into_storage());
            framebuffer.flip();
            // Two flips net zero — `back_is_a` mirror stays where it
            // was. Both buffers now hold WHITE.
            Timer::after(Duration::from_millis(FLASH_MS)).await;
            a_drawn = Snapshot::empty();
            b_drawn = Snapshot::empty();
        }

        // Build this frame's intended state.
        let mut new = Snapshot::empty();
        new.initialised = true;
        new.taps = TAP_COUNT.load(Ordering::Relaxed);
        new.bubble = accel_to_screen(latest_accel[0], latest_accel[1]);
        new.touch = latest_touch.map(|(x, y)| (x as i32, y as i32));

        match latest_clock {
            Some(c) => {
                let _ = core::fmt::write(
                    &mut new.time,
                    format_args!("{:02}:{:02}:{:02}", c.hour(), c.minute(), c.second()),
                );
            }
            None => {
                let _ = new.time.push_str("--:--:--");
            }
        }
        for i in 0..3 {
            format_mg(&mut new.accel[i], ["AX", "AY", "AZ"][i], shown_accel_mg[i]);
            format_mdps(&mut new.gyro[i], ["GX", "GY", "GZ"][i], shown_gyro_mdps[i]);
        }

        // Diff against the buffer we're about to write to.
        let back = if back_is_a { &mut a_drawn } else { &mut b_drawn };
        let fresh = !back.initialised;

        if fresh {
            // Row-at-a-time fill instead of `framebuffer.fill()`. The
            // BSP's bulk `fill` is one giant `Cache_WriteBack_Addr`
            // (~14 ms with interrupts effectively disabled), which is
            // long enough to coalesce ~20 EOF firings of the
            // bounce-buffer ISR. Coalesced EOFs lose alignment and
            // produce a one-frame vertical shift after a tap flash.
            // Per-row writebacks are ~10 µs each and the ISR fires
            // happily between them.
            for y in 0..HEIGHT {
                framebuffer.draw_row_solid(y, 0..WIDTH, BG.into_storage());
            }
        }

        let mut target = FbTarget(&framebuffer);
        let style = MonoTextStyle::new(&FONT_10X20, TEXT);

        if fresh || back.time != new.time {
            redraw_text(&framebuffer, &mut target, style, TIME_POS, &new.time, 8);
        }
        for i in 0..3 {
            if fresh || back.accel[i] != new.accel[i] {
                redraw_text(&framebuffer, &mut target, style, ACCEL_POS[i], &new.accel[i], 10);
            }
            if fresh || back.gyro[i] != new.gyro[i] {
                redraw_text(&framebuffer, &mut target, style, GYRO_POS[i], &new.gyro[i], 10);
            }
        }
        if fresh || back.taps != new.taps {
            let mut s = heapless::String::<12>::new();
            let _ = core::fmt::write(&mut s, format_args!("TAPS {}", new.taps));
            redraw_text(&framebuffer, &mut target, style, TAPS_POS, &s, 10);
        }
        // Device info — static, only painted on the fresh-frame path.
        if fresh {
            redraw_text(&framebuffer, &mut target, style, MAC_POS, &mac_str, 23);
            redraw_text(&framebuffer, &mut target, style, CHIP_POS, &chip_str, 18);
        }

        if fresh || back.bubble != new.bubble {
            if !fresh {
                draw_disc(&framebuffer, back.bubble.0, back.bubble.1, BUBBLE_R, BG.into_storage());
                // Repaint the static centre reference where the old
                // bubble erased it.
                draw_centre_reference(&framebuffer, back.bubble.0, back.bubble.1);
            }
            draw_disc(&framebuffer, new.bubble.0, new.bubble.1, BUBBLE_R, BUBBLE.into_storage());
        }
        if fresh {
            // Static centre crosshair behind the bubble — gives the
            // eye an anchor so the bubble's offset reads as "tilt".
            draw_centre_reference(&framebuffer, CENTRE_X + 999, CENTRE_Y + 999);
        }

        if fresh || back.touch != new.touch {
            if let (false, Some((x, y))) = (fresh, back.touch) {
                draw_cross(&framebuffer, x, y, CROSS_HALF, BG.into_storage());
            }
            if let Some((x, y)) = new.touch {
                draw_cross(&framebuffer, x, y, CROSS_HALF, CROSS.into_storage());
            }
        }

        framebuffer.flip();

        *back = new;
        back_is_a = !back_is_a;

        Timer::after(Duration::from_millis(RENDER_TICK_MS)).await;
    }
}

/// Erase a rectangular text region (in BG) then draw the new string.
fn redraw_text(
    fb: &Framebuffer,
    target: &mut FbTarget<'_>,
    style: MonoTextStyle<'_, Rgb565>,
    anchor: Point,
    text: &str,
    max_chars: i32,
) {
    let (origin, size) = text_box(anchor, max_chars);
    fill_rect(fb, origin, size, BG.into_storage());
    let _ = Text::new(text, anchor, style).draw(target);
}

/// Convert raw accel sample (i16) to mg using the BSP/driver
/// default of `AccelRange::G8` → 4096 LSB/g.
fn raw_accel_to_mg(raw: i16) -> i32 {
    (raw as i32) * 1000 / ACCEL_LSB_PER_G
}

/// Convert raw gyro sample (i16) to mdps using the BSP/driver
/// default of `GyroRange::Dps512` → 64 LSB/dps.
fn raw_gyro_to_mdps(raw: i16) -> i32 {
    (raw as i32) * 1000 / GYRO_LSB_PER_DPS
}

/// Round-to-nearest-multiple-of-`step` for signed values.
fn quantize(value: i32, step: i32) -> i32 {
    let half = step / 2;
    if value >= 0 {
        (value + half) / step * step
    } else {
        (value - half) / step * step
    }
}

/// "LB +0.98" — sign + integer + 2 fractional digits.
fn format_mg(out: &mut heapless::String<12>, label: &str, mg: i32) {
    out.clear();
    let sign = if mg < 0 { '-' } else { '+' };
    let abs = mg.unsigned_abs();
    let whole = abs / 1000;
    let frac = (abs % 1000) / 10;
    let _ = core::fmt::write(out, format_args!("{} {}{}.{:02}", label, sign, whole, frac));
}

/// "LB +12.3" — sign + integer + 1 fractional digit.
fn format_mdps(out: &mut heapless::String<12>, label: &str, mdps: i32) {
    out.clear();
    let sign = if mdps < 0 { '-' } else { '+' };
    let abs = mdps.unsigned_abs();
    let whole = abs / 1000;
    let frac = (abs % 1000) / 100;
    let _ = core::fmt::write(out, format_args!("{} {}{}.{}", label, sign, whole, frac));
}

/// Map raw accel X/Y → screen pixels for a bubble-level display.
///
/// The QMI8658 on this carrier is mounted with the chip's axes
/// rotated relative to the screen: chip +X points along the
/// screen's *down* axis (so standing the board on its bottom edge
/// reads AX = +1 g), chip +Y points along the screen's *left*
/// axis (left side of screen down → AY = +1 g). To get a
/// bubble-level metaphor — bubble moves to the *high* side,
/// opposite gravity — we map:
///
/// - bubble screen-X = CENTRE_X + ay/SCALE   (chip +y → screen left → bubble right)
/// - bubble screen-Y = CENTRE_Y − ax/SCALE   (chip +x → screen down → bubble up)
///
/// SCALE chosen so a ~15° tilt (≈ 0.26 g = 1064 raw at G8) puts
/// the bubble about a third of the way to the edge.
fn accel_to_screen(ax: i16, ay: i16) -> (i32, i32) {
    const SCALE: i32 = 30;
    const Y_SWING: i32 = 40;
    const X_SWING: i32 = 60;
    let dx = (ay as i32) / SCALE;
    let dy = (ax as i32) / SCALE;
    let x = (CENTRE_X + dx).clamp(CENTRE_X - X_SWING, CENTRE_X + X_SWING);
    let y = (CENTRE_Y - dy).clamp(CENTRE_Y - Y_SWING, CENTRE_Y + Y_SWING);
    (x, y)
}

/// Draw the static centre crosshair — a small `+` at (CENTRE_X,
/// CENTRE_Y) that gives the bubble something to be "offset from".
/// `skip_x` / `skip_y` are the bubble's current position; if the
/// crosshair would land inside the bubble, suppress it so we don't
/// flicker the cyan disc with grey pixels.
fn draw_centre_reference(fb: &Framebuffer, skip_x: i32, skip_y: i32) {
    const ARM: i32 = 8;
    let dx = (CENTRE_X - skip_x).abs();
    let dy = (CENTRE_Y - skip_y).abs();
    if dx < BUBBLE_R + ARM && dy < BUBBLE_R + ARM {
        // bubble is covering or near the centre — don't draw the
        // reference, the bubble itself is the anchor.
        return;
    }
    let c = REFERENCE.into_storage();
    fb.draw_row_solid(
        CENTRE_Y as usize,
        (CENTRE_X - ARM) as usize..(CENTRE_X + ARM + 1) as usize,
        c,
    );
    fb.draw_column(
        CENTRE_X as usize,
        (CENTRE_Y - ARM) as usize..(CENTRE_Y + ARM + 1) as usize,
        c,
    );
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------

fn fill_rect(fb: &Framebuffer, origin: Point, size: Size, colour: u16) {
    let x0 = origin.x.max(0) as usize;
    let x1 = ((origin.x + size.width as i32).min(WIDTH as i32)) as usize;
    let y0 = origin.y.max(0) as usize;
    let y1 = ((origin.y + size.height as i32).min(HEIGHT as i32)) as usize;
    for y in y0..y1 {
        fb.draw_row_solid(y, x0..x1, colour);
    }
}

fn draw_disc(fb: &Framebuffer, cx: i32, cy: i32, r: i32, colour: u16) {
    let r2 = r * r;
    for dy in -r..=r {
        let y = cy + dy;
        if y < 0 || y >= HEIGHT as i32 {
            continue;
        }
        let dy2 = dy * dy;
        let mut dx_max = 0i32;
        while (dx_max + 1) * (dx_max + 1) + dy2 <= r2 {
            dx_max += 1;
        }
        let x0 = (cx - dx_max).max(0) as usize;
        let x1 = ((cx + dx_max + 1).min(WIDTH as i32)) as usize;
        if x0 < x1 {
            fb.draw_row_solid(y as usize, x0..x1, colour);
        }
    }
}

fn draw_cross(fb: &Framebuffer, x: i32, y: i32, half: i32, colour: u16) {
    let xc = x.clamp(0, WIDTH as i32 - 1) as usize;
    let yc = y.clamp(0, HEIGHT as i32 - 1) as usize;
    let xa = (x - half).max(0) as usize;
    let xb = ((x + half + 1).min(WIDTH as i32)) as usize;
    let ya = (y - half).max(0) as usize;
    let yb = ((y + half + 1).min(HEIGHT as i32)) as usize;
    fb.draw_row_solid(yc, xa..xb, colour);
    fb.draw_column(xc, ya..yb, colour);
}

// ---------------------------------------------------------------------------
// embedded-graphics adapter — per-pixel write to the BSP framebuffer
// ---------------------------------------------------------------------------

struct FbTarget<'a>(&'a Framebuffer);

impl OriginDimensions for FbTarget<'_> {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for FbTarget<'_> {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, colour) in pixels {
            if point.x < 0 || point.y < 0 {
                continue;
            }
            let x = point.x as usize;
            let y = point.y as usize;
            if x >= WIDTH || y >= HEIGHT {
                continue;
            }
            self.0.draw_row_solid(y, x..x + 1, colour.into_storage());
        }
        Ok(())
    }
}
