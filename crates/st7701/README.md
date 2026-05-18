# st7701

`no_std` Rust driver for the Sitronix **ST7701** / ST7701S RGB-DPI panel
controller.

The ST7701 is the controller in a popular family of small round IPS
panels (notably the Waveshare 2.1" and 2.8" 480×480 modules). Pixel
data goes over a parallel RGB bus driven by whatever DPI / LCD
peripheral your SoC has — this crate covers only the one-shot **init
link**: a 9-bit-per-word "3-wire" SPI transfer that hardware SPI
peripherals can't natively frame.

## Scope

- Bit-banged 9-bit init transfer over [`embedded-hal`][eh] 1.0
  [`OutputPin`][outputpin] + [`DelayNs`][delayns].
- A `Step` table format for init sequences, matching the macros used
  in the C/C++ reference drivers (Espressif's `ESP32_Display_Panel`,
  LVGL, etc.).
- Canonical sequences as on-by-default cargo features
  ([`sequences::WAVESHARE_2_8C`] today; others welcome via PR).
- HAL-agnostic — works on esp-hal, embassy-stm32, rp-hal, anywhere
  `embedded-hal 1.0` does.

Not covered: pixel pushing. Once init returns, the chip is in
RGB-stream mode and your platform's DPI/RGB peripheral does the heavy
lifting.

[eh]: https://crates.io/crates/embedded-hal
[outputpin]: https://docs.rs/embedded-hal/latest/embedded_hal/digital/trait.OutputPin.html
[delayns]: https://docs.rs/embedded-hal/latest/embedded_hal/delay/trait.DelayNs.html

## Wiring

Four GPIOs plus a delay source. All push-pull outputs; no pull-ups.

| Pin | Direction | Notes                                    |
|-----|-----------|------------------------------------------|
| SCK | MCU → LCD | Idles low; data sampled on rising edge.  |
| SDA | MCU → LCD | MSB-first within the 9-bit word.         |
| CS  | MCU → LCD | Active-low; toggled per command.         |
| RST | MCU → LCD | Active-low hardware reset.               |

On boards where CS or RST hide behind an I²C I/O expander (the
Waveshare 2.8C runs both off a PCA9554), use the
[`port-expander`](https://crates.io/crates/port-expander) crate to
obtain pins that implement `OutputPin`.

## Pin error types

All four pins must share the same `OutputPin::Error` type. If you're
mixing pin backends with different error types, wrap the odd one out
in a small newtype that maps its error.

## Example

```rust,ignore
use st7701::{St7701, sequences};

let mut display = St7701::new(sck, sda, cs, rst, delay);
display.init(sequences::WAVESHARE_2_8C)?;
// Panel is now in RGB-stream mode; configure your DPI peripheral.

// Optional — reclaim the GPIOs:
let (sck, sda, cs, rst, delay) = display.release();
```

For an end-to-end ESP32-S3 + Waveshare-2.8C example see the
[`ws-esp32-s3-touch-lcd-2-8c`][bsp] BSP crate in the same
workspace.

[bsp]: ../ws-esp32-s3-touch-lcd-2-8c

## Features

| Feature        | Default | What it does                                  |
|----------------|:-------:|-----------------------------------------------|
| `waveshare-2-8c` | yes  | Bundles the Waveshare 2.8C init sequence.     |

## License

MIT — see the workspace [`LICENSE-MIT`](../../LICENSE-MIT).
