//! Integration demo: every BSP peripheral running concurrently on
//! the shared async I²C bus, with on-screen readouts.
//!
//! Three spawned tasks, all hitting the shared bus from independent
//! `I2cDevice` clones — the test of whether the shared-bus
//! serialisation does its job:
//!
//! - **touch_task** — polls the GT911 at 50 Hz, beeps the piezo on
//!   each tap (via the PCA9554), reports tap position + counter to
//!   the render task.
//! - **sensor_task** — polls the QMI8658 IMU at 10 Hz and the
//!   PCF85063A RTC at 1 Hz, hands the latest readings to the
//!   render task.
//! - **render_task** — repaints the panel at 10 Hz: time, accel,
//!   gyro, tap counter, a bubble-level dot following accel X/Y,
//!   and (while held) a touch crosshair.
//!
//! Tap the screen → beep + flash. Tilt the board → bubble drifts.
//! Watch the time tick. Move briskly → gyro values spike. If any
//! peripheral freezes or the screen tears, the shared-bus
//! serialisation is broken.
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
const CROSS:     Rgb565 = Rgb565::YELLOW;
const FLASH:     Rgb565 = Rgb565::WHITE;

const BUBBLE_HALF: i32 = 18;
const CROSS_HALF:  i32 = 14;
const FLASH_MS:    u64 = 80;
const TOUCH_POLL_MS: u64 = 20;
const IMU_POLL_MS:   u64 = 100;
const RTC_POLL_MS:   u64 = 1_000;
const RENDER_TICK_MS: u64 = 100;

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

static ACCEL: Signal<CriticalSectionRawMutex, [i16; 3]> = Signal::new();
static GYRO:  Signal<CriticalSectionRawMutex, [i16; 3]> = Signal::new();
static CLOCK: Signal<CriticalSectionRawMutex, PrimitiveDateTime> = Signal::new();
static TOUCH_POS: Signal<CriticalSectionRawMutex, Option<(u16, u16)>> = Signal::new();
static FLASH_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();
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
    spawner.spawn(render_task(framebuffer)).expect("spawn render");

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
                TOUCH_POS.signal(Some((p.x, p.y)));
                if !pressed {
                    pressed = true;
                    TAP_COUNT.fetch_add(1, Ordering::Relaxed);
                    log::info!("tap @ ({}, {})", p.x, p.y);
                    buzzer.beep_for(Duration::from_millis(60)).await;
                    FLASH_REQUEST.signal(());
                }
            }
            Ok(None) => {
                if pressed {
                    pressed = false;
                    TOUCH_POS.signal(None);
                }
            }
            Err(bsp::gt911::Error::NotReady) => {}
            Err(e) => log::warn!("gt911: {:?}", e),
        }
        Timer::after(Duration::from_millis(TOUCH_POLL_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// IMU + RTC task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn sensor_task(mut imu: Imu, mut rtc: Rtc) {
    let mut last_rtc = embassy_time::Instant::now();
    loop {
        match imu.driver_mut().read_accel_raw().await {
            Ok(s) => ACCEL.signal([s.data.x, s.data.y, s.data.z]),
            Err(e) => log::warn!("imu accel: {:?}", e),
        }
        match imu.driver_mut().read_gyro_raw().await {
            Ok(s) => GYRO.signal([s.data.x, s.data.y, s.data.z]),
            Err(e) => log::warn!("imu gyro: {:?}", e),
        }
        // RTC at its own slower cadence — no need to read it every
        // IMU tick. The Signal carries the latest snapshot.
        let now = embassy_time::Instant::now();
        if (now - last_rtc).as_millis() >= RTC_POLL_MS {
            match rtc.get_datetime().await {
                Ok(dt) => CLOCK.signal(dt),
                Err(e) => log::warn!("rtc: {:?}", e),
            }
            last_rtc = now;
        }
        Timer::after(Duration::from_millis(IMU_POLL_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// Render task
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn render_task(framebuffer: Framebuffer) {
    let mut accel = [0i16; 3];
    let mut gyro = [0i16; 3];
    let mut clock: Option<PrimitiveDateTime> = None;
    let mut touch_pos: Option<(u16, u16)> = None;

    loop {
        // Pick up the latest values that have arrived since last tick.
        if let Some(a) = ACCEL.try_take() { accel = a; }
        if let Some(g) = GYRO.try_take()  { gyro = g; }
        if let Some(c) = CLOCK.try_take() { clock = Some(c); }
        if let Some(t) = TOUCH_POS.try_take() { touch_pos = t; }

        // Tap flash: fill both buffers white briefly, then carry on.
        if FLASH_REQUEST.try_take().is_some() {
            framebuffer.fill(FLASH.into_storage());
            framebuffer.flip();
            framebuffer.fill(FLASH.into_storage());
            framebuffer.flip();
            Timer::after(Duration::from_millis(FLASH_MS)).await;
        }

        // Full-screen redraw each tick. With double-buffered PSRAM at
        // octal speeds this fills in ~ms, comfortably under the 100 ms
        // render tick.
        framebuffer.fill(BG.into_storage());

        let mut target = FbTarget(&framebuffer);
        let style = MonoTextStyle::new(&FONT_10X20, TEXT);

        // Time at the top centre.
        let mut buf = heapless::String::<24>::new();
        if let Some(c) = clock {
            let _ = core::fmt::write(
                &mut buf,
                format_args!(
                    "{:02}:{:02}:{:02}",
                    c.hour(), c.minute(), c.second()
                ),
            );
        } else {
            let _ = buf.push_str("--:--:--");
        }
        let _ = Text::new(&buf, Point::new(180, 60), style).draw(&mut target);

        // Accel on the left.
        for (i, v) in accel.iter().enumerate() {
            let mut s = heapless::String::<16>::new();
            let _ = core::fmt::write(&mut s, format_args!("{}{:>+6}", ["AX","AY","AZ"][i], v));
            let _ = Text::new(&s, Point::new(20, 110 + i as i32 * 28), style).draw(&mut target);
        }
        // Gyro on the right.
        for (i, v) in gyro.iter().enumerate() {
            let mut s = heapless::String::<16>::new();
            let _ = core::fmt::write(&mut s, format_args!("{}{:>+6}", ["GX","GY","GZ"][i], v));
            let _ = Text::new(&s, Point::new(310, 110 + i as i32 * 28), style).draw(&mut target);
        }

        // Tap counter near the bottom.
        let taps = TAP_COUNT.load(Ordering::Relaxed);
        let mut s = heapless::String::<16>::new();
        let _ = core::fmt::write(&mut s, format_args!("TAPS {}", taps));
        let _ = Text::new(&s, Point::new(190, 430), style).draw(&mut target);

        // Bubble-level dot — accel X/Y → screen X/Y.
        let (bx, by) = accel_to_screen(accel[0], accel[1]);
        draw_disc(&framebuffer, bx, by, BUBBLE_HALF, BUBBLE.into_storage());

        // Touch crosshair while finger is down.
        if let Some((tx, ty)) = touch_pos {
            draw_cross(&framebuffer, tx as i32, ty as i32, CROSS_HALF, CROSS.into_storage());
        }

        framebuffer.flip();
        Timer::after(Duration::from_millis(RENDER_TICK_MS)).await;
    }
}

// ---------------------------------------------------------------------------
// Bubble + crosshair drawing (direct framebuffer calls, faster than
// going through embedded-graphics for the big solid shapes)
// ---------------------------------------------------------------------------

/// Map raw accel X/Y to centre-of-bubble pixel coords. ±8 g full
/// scale → values up to ~4096 at 90° tilt; scale chosen so 90°
/// puts the bubble against the edge.
fn accel_to_screen(ax: i16, ay: i16) -> (i32, i32) {
    const SCALE: i32 = 28;
    let cx = WIDTH as i32 / 2;
    let cy = HEIGHT as i32 / 2;
    let x = cx + (ax as i32) / SCALE;
    let y = cy - (ay as i32) / SCALE; // invert: tilt-forward → bubble up
    (
        x.clamp(BUBBLE_HALF, WIDTH as i32 - BUBBLE_HALF),
        y.clamp(BUBBLE_HALF + 220, HEIGHT as i32 - BUBBLE_HALF - 60),
    )
}

fn draw_disc(fb: &Framebuffer, cx: i32, cy: i32, r: i32, colour: u16) {
    let r2 = r * r;
    for dy in -r..=r {
        let y = cy + dy;
        if y < 0 || y >= HEIGHT as i32 {
            continue;
        }
        let dy2 = dy * dy;
        // Integer ⌊√(r² − dy²)⌋ — avoids pulling in libm. r ≤ ~30
        // means this never iterates more than a few dozen times.
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
// embedded-graphics adapter for the BSP's Framebuffer
// ---------------------------------------------------------------------------

/// A thin DrawTarget that pokes individual pixels into the BSP's
/// back buffer via `draw_row_solid(y, x..x+1, …)`. Not super fast
/// — each pixel triggers one PSRAM write + one cache flush — but
/// for the modest amount of text in this demo it's well under the
/// 100 ms render budget.
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
