//! Sitronix **ST7701** / ST7701S init driver.
//!
//! The ST7701 is the controller in a popular family of small RGB-DPI
//! IPS panels (notably the round 2.1" and 2.8" Waveshare modules).
//! Pixel data goes over a parallel RGB bus driven by whatever DPI / LCD
//! peripheral your SoC has; configuration only happens once at boot
//! over a 9-bit-per-word "3-wire" SPI link.
//!
//! Hardware SPI peripherals can't natively frame 9-bit words, so this
//! driver bit-bangs the link over three [`OutputPin`]s. Initialisation
//! takes ~50 ms; after that the pins are unused and can be repurposed.
//!
//! ## What this crate is and isn't
//!
//! - **Is**: a HAL-agnostic init driver. It clocks out a sequence of
//!   `(command, [data...], delay_ms)` triples, where the sequence
//!   table is supplied per-panel.
//! - **Is**: a place to keep canonical init sequences for known panels
//!   (see [`sequences`]).
//! - **Isn't**: a pixel/framebuffer interface. Once init returns, the
//!   chip is in RGB-stream mode — pixel data is your platform's DPI
//!   peripheral's problem.
//!
//! ## Wiring
//!
//! Four GPIOs plus a delay source. All three logic pins are *push-pull*
//! outputs — no open-drain, no pull-ups required.
//!
//! | Pin | Direction | Notes                                          |
//! |-----|-----------|------------------------------------------------|
//! | SCK | MCU → LCD | Idles low; data sampled on rising edge.        |
//! | SDA | MCU → LCD | MSB-first within the 9-bit word.               |
//! | CS  | MCU → LCD | Active-low; toggled per command.               |
//! | RST | MCU → LCD | Active-low hardware reset before init.         |
//!
//! On boards where one or more of those pins live behind an I/O
//! expander (the Waveshare 2.8C runs CS *and* RST off a PCA9554 over
//! I²C), use [`port-expander`] or similar to obtain expander pins that
//! implement [`OutputPin`].
//!
//! [`port-expander`]: https://crates.io/crates/port-expander
//!
//! ## Pin error types
//!
//! All four pins must share the same [`OutputPin::Error`] type, since
//! the driver folds any pin failure into [`Error::Pin`]. If you're
//! mixing pin backends with different error types (e.g. local SoC GPIOs
//! for SCK/SDA, an expander pin for CS), wrap the odd one out in a
//! small newtype that maps its error into a common type.
//!
//! ## Example
//!
//! ```ignore
//! use st7701::{St7701, sequences};
//!
//! let mut display = St7701::new(sck, sda, cs, rst, delay);
//! display.init(sequences::WAVESHARE_2_8C)?;
//! // After init returns, the panel is in RGB-stream mode.
//! // Hand SCK / SDA / CS / RST back if you need them for something else:
//! let (sck, sda, cs, rst, delay) = display.release();
//! ```

#![cfg_attr(not(test), no_std)]
#![deny(missing_docs)]

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

#[cfg(feature = "waveshare-2-8c")]
pub mod sequences;

/// One entry in an ST7701 init sequence: a controller command byte,
/// zero or more parameter bytes, and an optional post-step settling
/// delay in milliseconds (0 = no delay).
///
/// The format mirrors the macro-driven init tables in the C/C++
/// reference drivers (Espressif's `ESP32_Display_Panel`, LVGL examples,
/// and the ST7701 datasheet appendix), which makes it straightforward
/// to translate sequences for other panels.
#[derive(Debug, Clone, Copy)]
pub struct Step {
    /// Command byte (the first 9-bit word, sent with the D/CX flag cleared).
    pub cmd: u8,
    /// Zero or more parameter bytes (each sent with D/CX set).
    pub data: &'static [u8],
    /// Settling delay after this step, in milliseconds. 0 = no delay.
    pub delay_ms: u32,
}

/// Driver errors.
///
/// The only failure mode during init is a GPIO toggle failing; on local
/// SoC pins that's typically [`core::convert::Infallible`], but on
/// expander pins driven over I²C any bus error from the expander
/// surfaces here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// A pin write returned an error. Contained value is the underlying
    /// `OutputPin::Error`.
    Pin(E),
}

impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::Pin(e)
    }
}

/// ST7701 init driver.
///
/// Type parameters are the four GPIOs (all [`OutputPin`] with a common
/// `Error` type) plus the delay source. See the crate-level docs for
/// the wiring and error model.
pub struct St7701<SCK, SDA, CS, RST, D> {
    sck: SCK,
    sda: SDA,
    cs: CS,
    rst: RST,
    delay: D,
}

impl<SCK, SDA, CS, RST, D, E> St7701<SCK, SDA, CS, RST, D>
where
    SCK: OutputPin<Error = E>,
    SDA: OutputPin<Error = E>,
    CS: OutputPin<Error = E>,
    RST: OutputPin<Error = E>,
    D: DelayNs,
{
    /// Construct the driver.
    ///
    /// Does not touch the pins; call [`Self::init`] (or the lower-level
    /// [`Self::reset`] / [`Self::run`] / [`Self::write_command`]
    /// primitives) to actually do anything.
    pub fn new(sck: SCK, sda: SDA, cs: CS, rst: RST, delay: D) -> Self {
        Self { sck, sda, cs, rst, delay }
    }

    /// Hardware-reset the controller, then clock out `sequence`.
    ///
    /// This is the one-shot entry point most users want: it does the
    /// 10 ms-low / 120 ms-high reset pulse from the datasheet, then
    /// runs the supplied step table.
    pub fn init(&mut self, sequence: &[Step]) -> Result<(), Error<E>> {
        self.reset()?;
        self.run(sequence)
    }

    /// Hardware reset, per the ST7701 datasheet: drive RST low for at
    /// least 10 µs, then high, then wait 120 ms before talking SPI.
    ///
    /// Idles SCK and SDA low as a side effect, so the line is in a
    /// known state when [`Self::run`] starts clocking bits.
    pub fn reset(&mut self) -> Result<(), Error<E>> {
        self.sck.set_low()?;
        self.sda.set_low()?;
        self.cs.set_high()?;
        self.rst.set_low()?;
        self.delay.delay_ms(10);
        self.rst.set_high()?;
        self.delay.delay_ms(120);
        Ok(())
    }

    /// Clock out `sequence` without doing a hardware reset first.
    ///
    /// Useful if you've already reset the chip yourself (e.g. via a
    /// board-level expander pin) and just want to send commands.
    pub fn run(&mut self, sequence: &[Step]) -> Result<(), Error<E>> {
        for step in sequence {
            self.write_command(step.cmd, step.data)?;
            if step.delay_ms > 0 {
                self.delay.delay_ms(step.delay_ms);
            }
        }
        Ok(())
    }

    /// Send a single command with its parameter bytes. CS is asserted
    /// low for the duration of the transaction and released high
    /// afterwards.
    pub fn write_command(&mut self, cmd: u8, data: &[u8]) -> Result<(), Error<E>> {
        self.cs.set_low()?;
        self.write_9bit(false, cmd)?;
        for &byte in data {
            self.write_9bit(true, byte)?;
        }
        self.cs.set_high()?;
        Ok(())
    }

    /// Hand the pins and delay back to the caller. Useful for
    /// repurposing the GPIOs after init — the ST7701 never needs the
    /// SPI link again once it's streaming RGB.
    pub fn release(self) -> (SCK, SDA, CS, RST, D) {
        (self.sck, self.sda, self.cs, self.rst, self.delay)
    }

    /// Clock out one 9-bit word, MSB first. Bit order on the wire is
    /// D/CX, then bits 7..=0 of `byte`. D/CX = 0 for commands, 1 for
    /// data; the controller samples SDA on the rising edge of SCK.
    ///
    /// The 1 µs per-step delay gives ~333 kHz SCK — well inside the
    /// chip's 10 MHz spec and well above its setup/hold minima, while
    /// being slow enough that the loop body's GPIO toggle overhead
    /// doesn't matter even on a fast MCU.
    fn write_9bit(&mut self, is_data: bool, byte: u8) -> Result<(), Error<E>> {
        self.write_bit(is_data)?;
        for i in (0..8).rev() {
            self.write_bit((byte >> i) & 1 != 0)?;
        }
        Ok(())
    }

    fn write_bit(&mut self, bit: bool) -> Result<(), Error<E>> {
        if bit {
            self.sda.set_high()?;
        } else {
            self.sda.set_low()?;
        }
        self.delay.delay_us(1);
        self.sck.set_high()?;
        self.delay.delay_us(1);
        self.sck.set_low()?;
        self.delay.delay_us(1);
        Ok(())
    }
}
