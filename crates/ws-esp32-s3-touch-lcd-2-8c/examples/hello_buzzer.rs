//! Buzzer demo: short chirp on boot, then a chirp on every touch.
//!
//! Exercises the [`bsp::Buzzer`] first-class API end-to-end:
//! - PCA9554 controller drives io7 through the shared async I²C bus
//! - Buzzer + TouchPoller run concurrently on the same bus without
//!   contention
//!
//! Tap the panel — you should hear a 60 ms beep per tap. Holding the
//! finger down should *not* beep continuously (the example debounces
//! by waiting for the lift).
//!
//! Build & flash:
//!
//! ```sh
//! cd crates/ws-esp32-s3-touch-lcd-2-8c
//! cargo run --release --example hello_buzzer
//! ```

#![no_std]
#![no_main]

extern crate alloc;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_2_8c as bsp;

esp_bootloader_esp_idf::esp_app_desc!();

/// Poll the touch controller this often.
const POLL_MS: u64 = 20;
/// How long each touch-triggered beep lasts.
const TAP_BEEP_MS: u64 = 60;
/// Boot-time chirp duration — long enough to be clearly audible
/// over the panel-init click.
const BOOT_BEEP_MS: u64 = 200;

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

    log::info!("buzzer demo — boot beep, then beep on every touch");
    board.buzzer.beep_for(Duration::from_millis(BOOT_BEEP_MS)).await;

    let mut pressed = false;
    loop {
        match board.touch.poll().await {
            Ok(Some(p)) => {
                if !pressed {
                    pressed = true;
                    log::info!("tap @ ({}, {})", p.x, p.y);
                    board.buzzer.beep_for(Duration::from_millis(TAP_BEEP_MS)).await;
                }
            }
            Ok(None) => {
                if pressed {
                    pressed = false;
                }
            }
            // NotReady is the common case between scan cycles; anything
            // else is a bus error worth logging.
            Err(bsp::gt911::Error::NotReady) => {}
            Err(e) => log::warn!("gt911: {:?}", e),
        }
        Timer::after(Duration::from_millis(POLL_MS)).await;
    }
}
