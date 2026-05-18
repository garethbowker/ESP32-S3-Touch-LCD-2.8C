# ESP32-S3-Touch-LCD-2.8C

Rust drivers for the [Waveshare ESP32-S3-Touch-LCD-2.8C][board] — a 2.8"
round 480×480 IPS panel with a Goodix GT911 capacitive touch overlay,
mounted on an ESP32-S3 carrier board.

[board]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C

## Crates

This workspace publishes two crates. Pick the one(s) you need:

| Crate | What it gives you | Depends on |
|---|---|---|
| [`st7701`](crates/st7701) | Generic Sitronix ST7701 init driver — bit-banged 9-bit 3-wire SPI over `embedded-hal` `OutputPin` + `DelayNs`. Includes the Waveshare-2.8C init sequence as a preset. Works on **any HAL**. | `embedded-hal 1.0` |
| [`ws-esp32-s3-touch-lcd-2-8c`](crates/ws-esp32-s3-touch-lcd-2-8c) | Board-support package for this exact board. One call wires up: ST7701 init, PCA9554 I/O expander, GT911 touch reset, DPI peripheral, PSRAM framebuffer, and the DRAM bounce-buffer ring needed to avoid the PSRAM→DPI drift on this SoC. | `esp-hal 1.0.0-rc.0`, `st7701`, [`port-expander`](https://crates.io/crates/port-expander), [`gt911`](https://crates.io/crates/gt911) |

Existing community crates cover the other two chips on the board:

- [`port-expander`](https://crates.io/crates/port-expander) — PCA9554
  I²C I/O expander (drives the LCD reset, ST7701 CS, and touch reset).
- [`gt911`](https://crates.io/crates/gt911) — Goodix GT911 capacitive
  touch controller.

The BSP crate re-exports those, so a downstream user only needs to
`cargo add ws-esp32-s3-touch-lcd-2-8c`.

## Picking a crate

- **Just want the LCD on a different HAL** (rp2040, stm32, …):
  `cargo add st7701` and supply your own pin types.
- **Touch only** (any board): `cargo add gt911`.
- **PCA9554 only** (any board): `cargo add port-expander`.
- **The full Waveshare 2.8C, ESP32-S3 target**:
  `cargo add ws-esp32-s3-touch-lcd-2-8c`.

## License

MIT — see [LICENSE-MIT](LICENSE-MIT).
