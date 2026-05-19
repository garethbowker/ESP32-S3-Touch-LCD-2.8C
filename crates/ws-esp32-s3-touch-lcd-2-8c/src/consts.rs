//! Hardware constants for the Waveshare ESP32-S3-Touch-LCD-2.8C carrier.
//!
//! Names match the silkscreen labels and the schematic on the
//! [Waveshare wiki][wiki]. Everything here is `pub const` — included
//! so consumers don't need to re-read the schematic to wire up the
//! peripherals the BSP doesn't claim (RTC, IMU, SD card, battery ADC,
//! USB).
//!
//! [wiki]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C

/// I²C bus 0. Used by the BSP for the PCA9554 expander, the GT911 touch
/// controller, and the off-board PCF85063 RTC and QMI8658 IMU.
pub mod i2c {
    /// SDA pin on the ESP32-S3.
    pub const SDA_GPIO: u8 = 15;
    /// SCL pin on the ESP32-S3.
    pub const SCL_GPIO: u8 = 7;
    /// Default bus frequency. The BSP brings up the bus at this rate
    /// — all four on-board peripherals tolerate 400 kHz cleanly.
    pub const FREQUENCY_HZ: u32 = 400_000;

    /// PCA9554 8-bit I/O expander.
    pub const PCA9554_ADDR: u8 = 0x20;
    /// GT911 capacitive touch controller (after the RST-with-INT-low
    /// reset latches the address — see the GT911 init dance in
    /// [`crate::init`]).
    pub const GT911_ADDR: u8 = 0x5D;
    /// PCF85063 RTC with battery backup.
    ///
    /// **Not claimed by [`crate::init`].** Bring your own driver
    /// (e.g. `pcf85063a` from crates.io) sharing this bus.
    pub const PCF85063_ADDR: u8 = 0x51;
    /// QMI8658C 6-axis IMU (accelerometer + gyroscope), L-variant —
    /// matches the address strap on this board.
    ///
    /// **Not claimed by [`crate::init`].**
    pub const QMI8658_ADDR: u8 = 0x6B;
}

/// PCA9554 expander pin assignments. Bit numbers match the chip
/// register layout (io0 = bit 0, io7 = bit 7), one less than the
/// Waveshare schematic's EXIO1..EXIO8 labelling.
pub mod pca9554 {
    /// LCD reset. Claimed by [`crate::init`] as output-high.
    pub const LCD_RST_BIT: u8 = 0;
    /// Touch-controller reset. Claimed by [`crate::init`] as
    /// output-high.
    pub const TOUCH_RST_BIT: u8 = 1;
    /// ST7701 chip-select for the bit-banged 9-bit init SPI. Claimed
    /// by [`crate::init`] as output-high.
    pub const ST7701_CS_BIT: u8 = 2;
    /// SD card D3 (enable). Drive HIGH to enable the SD card data
    /// path; the BSP leaves this pin alone, so consumers using SD
    /// must drive it themselves.
    pub const SD_D3_EN_BIT: u8 = 3;
    /// On-board piezo buzzer. Claimed by [`crate::init`] as
    /// output-low for silence.
    pub const BUZZER_BIT: u8 = 7;
    // Bits 4, 5, 6 are unused on this carrier — Waveshare's own demo
    // doesn't touch them. Available for consumer use via a fresh
    // `port_expander::dev::pca9554::Pca9554` instance on the BSP's
    // I²C bus.
}

/// SD/MMC pin layout. The board wires the SD card in 1-bit SDMMC mode
/// — only D0 is connected as a data line, with D3 routed via the
/// PCA9554 expander (see [`pca9554::SD_D3_EN_BIT`]).
///
/// **GPIO1 and GPIO2 are shared with the ST7701 init bit-bang SPI**.
/// [`crate::init`] uses them during panel bring-up and returns them
/// for SD use via [`crate::Board::sd_pins`].
///
/// **`esp-hal` 1.0.0-rc.0 does not yet expose an SDHOST/SDMMC driver
/// for the S3**, so no SD card driver can be wired up from pure Rust
/// today. The pin layout is documented here for when that support
/// arrives.
pub mod sd {
    /// SD CLK — also the ST7701 init SCK during panel bring-up.
    pub const CLK_GPIO: u8 = 2;
    /// SD CMD — also the ST7701 init MOSI during panel bring-up.
    pub const CMD_GPIO: u8 = 1;
    /// SD D0.
    pub const D0_GPIO: u8 = 42;
    /// SD D3 (enable) routes through the PCA9554 expander.
    pub const D3_PCA9554_BIT: u8 = super::pca9554::SD_D3_EN_BIT;
}

/// Battery voltage sense.
///
/// The carrier divides Vbat (Li-Po, 3.0–4.2 V nominal) onto GPIO4
/// with a roughly 3:1 resistor divider. Configure ADC1 channel 3
/// with 12-dB attenuation to read it.
///
/// **Not claimed by [`crate::init`].** [`crate::Board::battery_adc`]
/// hands the raw GPIO4 peripheral through so consumers can build
/// their own ADC oneshot.
pub mod battery {
    /// GPIO carrying the divider output.
    pub const ADC_GPIO: u8 = 4;
    /// ADC1 channel — pass to esp-hal's ADC API.
    pub const ADC1_CHANNEL: u8 = 3;
    /// Attenuation in dB. 12 dB gives a full-scale input of ~2.5 V
    /// at the pin, comfortably above the divider's ~1.4 V at 4.2 V
    /// Vbat.
    pub const ADC_ATTENUATION_DB: u8 = 12;
    /// Nominal divider ratio: Vbat / V_pin = 3.0. Multiply the
    /// ADC-read voltage by this to get Vbat. Real per-board values
    /// drift a few percent — Waveshare's own demo trims by an
    /// empirical 0.980952 multiplier on top.
    pub const DIVIDER_RATIO: f32 = 3.0;
}

/// USB-Serial-JTAG / USB-OTG D+ / D- pins.
///
/// These are hardware-fixed on the ESP32-S3 — the USB peripheral
/// bypasses the GPIO matrix — so the BSP doesn't need to claim them.
/// Documented so consumers don't accidentally configure them as
/// plain GPIOs.
pub mod usb {
    /// D-
    pub const DM_GPIO: u8 = 19;
    /// D+
    pub const DP_GPIO: u8 = 20;
}
