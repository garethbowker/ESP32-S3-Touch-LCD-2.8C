//! RTC demo: seeds the PCF85063A to a known datetime on boot, then
//! reads the running clock once a second and logs it.
//!
//! Watch the serial output — the seconds should tick. If you have
//! a coin-cell on the VBAT pad, power-cycling the board and
//! commenting out the `set_datetime` call should show the clock
//! continuing from where it left off (PCF85063A runs from VBAT
//! when Vdd drops).
//!
//! Build & flash:
//!
//! ```sh
//! cd crates/ws-esp32-s3-touch-lcd-2-8c
//! cargo run --release --example hello_rtc --features rtc
//! ```

#![no_std]
#![no_main]

extern crate alloc;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_2_8c::{
    self as bsp,
    rtc::{Date, Month, PrimitiveDateTime, Time},
};

esp_bootloader_esp_idf::esp_app_desc!();

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

    // Seed the RTC with a known datetime so we can see the seconds
    // tick. In production a real consumer would only do this from a
    // calibration source (GPS time, NTP, user input) — the
    // PCF85063A holds it across resets via VBAT.
    let seed = PrimitiveDateTime::new(
        Date::from_calendar_date(2026, Month::May, 19).unwrap(),
        Time::from_hms(14, 30, 0).unwrap(),
    );
    board.rtc.set_datetime(&seed).await.expect("rtc set");
    log::info!("rtc seeded to {seed}");

    loop {
        match board.rtc.get_datetime().await {
            Ok(dt) => log::info!("rtc: {dt}"),
            Err(e) => log::warn!("rtc read failed: {:?}", e),
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}
