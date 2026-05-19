//! IMU demo: initialises the QMI8658, then prints raw accel + gyro
//! counts ~10× per second.
//!
//! Pick the board up and rotate it — gyro values should pulse on the
//! axis you turn. Tilt it — accel readings should settle on the
//! gravity vector for the new orientation. Watch the serial output.
//!
//! Build & flash:
//!
//! ```sh
//! cd crates/ws-esp32-s3-touch-lcd-2-8c
//! cargo run --release --example hello_imu --features imu
//! ```

#![no_std]
#![no_main]

extern crate alloc;
use embassy_time::{Delay, Duration, Timer};
use esp_backtrace as _;
use ws_esp32_s3_touch_lcd_2_8c as bsp;

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

    // Run the async init handshake. embassy-time's `Delay` impls
    // `embedded_hal_async::delay::DelayNs` which is what the driver
    // wants. We use it for power-up timing and for re-reading the
    // device after CTRL9 handshakes.
    let mut delay = Delay;
    board.imu.init(&mut delay).await.expect("imu init");
    log::info!("imu initialised — pick up the board and rotate it");

    loop {
        let accel = board
            .imu
            .driver_mut()
            .read_accel_raw()
            .await
            .expect("accel read");
        let gyro = board
            .imu
            .driver_mut()
            .read_gyro_raw()
            .await
            .expect("gyro read");
        log::info!(
            "accel x={:>6} y={:>6} z={:>6}  |  gyro x={:>6} y={:>6} z={:>6}",
            accel.data.x, accel.data.y, accel.data.z,
            gyro.data.x, gyro.data.y, gyro.data.z,
        );
        Timer::after(Duration::from_millis(100)).await;
    }
}
