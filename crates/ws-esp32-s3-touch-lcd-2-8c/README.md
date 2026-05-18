# ws-esp32-s3-touch-lcd-2-8c

Board-support package for the [**Waveshare ESP32-S3-Touch-LCD-2.8C**][board]
— a 2.8" round 480×480 IPS panel (ST7701S) with a Goodix GT911
capacitive touch overlay, mounted on an ESP32-S3 carrier with 8 MB
octal PSRAM.

[board]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C

One call brings the board fully alive: panel init, PCA9554 wiring,
GT911 reset dance, DPI peripheral, PSRAM framebuffer, and the
DRAM bounce-buffer ring that works around the PSRAM→DPI drift
artefacts on this SoC.

## Quick start

```toml
[dependencies]
ws-esp32-s3-touch-lcd-2-8c = "1.0"
esp-hal           = { version = "=1.0.0-rc.0", features = ["esp32s3", "unstable", "psram"] }
esp-hal-embassy   = { version = "=0.9.0",      features = ["esp32s3"] }
esp-backtrace     = { version = "=0.17.0",     features = ["esp32s3", "panic-handler", "println"] }
esp-println       = { version = "=0.15.0",     features = ["esp32s3", "log-04"] }
esp-bootloader-esp-idf = { version = "=0.2.0", features = ["esp32s3"] }
esp-alloc         = "=0.8.0"
```

```rust,no_run
#![no_std]
#![no_main]

extern crate alloc;
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

    let mut board = bsp::init(bsp::take_resources!(peripherals))
        .expect("board init");

    // Solid red, then poll touch forever.
    board.framebuffer.fill(0xF800);

    loop {
        match board.touch.get_touch(&mut board.i2c) {
            Ok(Some(point)) => log::info!("touch @ ({}, {})", point.x, point.y),
            Ok(None) | Err(gt911::Error::NotReady) => {}
            Err(e) => log::warn!("gt911: {:?}", e),
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
    }
}
```

You'll also need a `.cargo/config.toml` with `target = "xtensa-esp32s3-none-elf"`,
`build-std`, and `ESP_HAL_CONFIG_PSRAM_MODE = "octal"` — see the
[`hello_touch`](examples/hello_touch.rs) example for a complete project layout.

## What `init` does

1. Brings up I²C0 at 400 kHz on the board's SDA/SCL.
2. Configures the PCA9554 I/O expander (`0x20`): bit 0 = LCD reset,
   bit 1 = touch reset, bit 2 = ST7701 chip-select.
3. Drives the ST7701 reset pulse and clocks out the panel init
   sequence over bit-banged 9-bit "3-wire" SPI.
4. Latches the GT911's I²C address to `0x5D` via the canonical
   INT-low / RST-pulse reset dance.
5. Allocates the 480×480 RGB565 framebuffer in PSRAM, configures the
   LCD_CAM peripheral's DPI block at 12 MHz PCLK / 480×480 active.
6. Sets up a DRAM bounce-buffer descriptor ring, starts a continuous
   DMA transfer feeding the panel, and installs the EOF interrupt
   handler that refills each half from PSRAM.
7. Drives the backlight GPIO high.

After return the panel is alive and black. CPU writes to
`board.framebuffer` are visible within one frame (~23 ms at 12 MHz).

## What you get back

```rust,ignore
pub struct Board<'d> {
    pub framebuffer: Framebuffer,
    pub i2c: esp_hal::i2c::master::I2c<'d, esp_hal::Blocking>,
    pub touch: gt911::Gt911Blocking<esp_hal::i2c::master::I2c<'d, esp_hal::Blocking>>,
    pub backlight: esp_hal::gpio::Output<'d>,
}
```

- `framebuffer` — write RGB565 pixels here, they appear on the panel.
- `i2c` — owned bus; pass `&mut board.i2c` to the GT911 driver per
  poll. The bus is yours after init; reuse it for other I²C devices
  if you like.
- `touch` — stateless GT911 driver. Methods all take `&mut I2C`.
- `backlight` — drive low to blank, or reconfigure as an LEDC channel
  for PWM dimming.

## Caveats

- **Call once per boot.** The bounce-buffer descriptor ring and EOF
  ISR live in module-level statics.
- The DPI DMA transfer is leaked intentionally — dropping it would
  stop the panel.
- Requires `esp_hal::init` to be called with PSRAM sized to at least
  ~450 KB (`8 * 1024 * 1024` covers it comfortably).
- Pinned to `esp-hal = =1.0.0-rc.0`. See the workspace
  [Cargo.toml](Cargo.toml) for why.

## Re-exports

```rust,ignore
pub use gt911;
pub use port_expander;
pub use st7701;
```

So you can do `bsp::gt911::Point` and `bsp::st7701::sequences` without
adding those crates to your own `[dependencies]`.

## License

MIT — see the workspace [LICENSE-MIT](../../LICENSE-MIT).
