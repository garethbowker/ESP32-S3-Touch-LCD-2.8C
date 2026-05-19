//! Atomic read-modify-write controller for the PCA9554 I/O expander.
//!
//! The `port_expander` crate's PCA9554 driver caches the OutputPort
//! register and writes the whole byte on each `set()` — fine for a
//! single owner, but with multiple owners (buzzer + future SD-D3
//! enable + expansion pins) the caches can clobber each other. This
//! controller does a true read-modify-write of the chip register on
//! every operation, then wraps itself in an `embassy_sync::Mutex`
//! ([`SharedPcaController`]) so the RMW gap is closed across tasks.
//!
//! All the BSP's PCA9554-mediated peripherals — [`crate::Buzzer`],
//! and (when SD support lands) the SD card enable — share one
//! `SharedPcaController` instance.
//!
//! Used only internally by the BSP. Consumers wanting to drive
//! PCA9554 pins themselves (io4/5/6 are unused on this carrier)
//! can build their own `port_expander::Pca9554` on top of
//! [`crate::Board::i2c`] using the constants in
//! [`crate::consts::pca9554`].

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal_async::i2c::I2c;

use crate::shared_bus::BoardI2cDevice;

/// PCA9554 register addresses.
const REG_OUTPUT_PORT:   u8 = 0x01;
const REG_CONFIGURATION: u8 = 0x03;

/// Stateless PCA9554 controller — every operation reads the chip
/// register, modifies one bit, writes it back. Holds an
/// [`BoardI2cDevice`] for bus access.
pub(crate) struct Pca9554Controller {
    i2c: BoardI2cDevice,
    addr: u8,
}

impl Pca9554Controller {
    pub(crate) fn new(i2c: BoardI2cDevice, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Set or clear bit `bit` (0..=7) of the OutputPort register.
    /// Does not touch the Configuration register — caller is
    /// responsible for having configured the pin as an output
    /// (via [`Self::set_direction`]) before driving it.
    pub(crate) async fn set_output(&mut self, bit: u8, high: bool) -> Result<(), Error> {
        debug_assert!(bit < 8);
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[REG_OUTPUT_PORT], &mut buf)
            .await
            .map_err(|_| Error::Bus)?;
        let new = if high {
            buf[0] | (1 << bit)
        } else {
            buf[0] & !(1 << bit)
        };
        self.i2c
            .write(self.addr, &[REG_OUTPUT_PORT, new])
            .await
            .map_err(|_| Error::Bus)?;
        Ok(())
    }

    /// Configure bit `bit` as an output (`output = true`) or input
    /// (`output = false`). Output pins have bit cleared in the
    /// Configuration register; inputs have it set.
    #[allow(dead_code)] // exposed for completeness; nothing in the BSP toggles
                        // a pin's direction post-init today.
    pub(crate) async fn set_direction(&mut self, bit: u8, output: bool) -> Result<(), Error> {
        debug_assert!(bit < 8);
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[REG_CONFIGURATION], &mut buf)
            .await
            .map_err(|_| Error::Bus)?;
        let new = if output {
            buf[0] & !(1 << bit)
        } else {
            buf[0] | (1 << bit)
        };
        self.i2c
            .write(self.addr, &[REG_CONFIGURATION, new])
            .await
            .map_err(|_| Error::Bus)?;
        Ok(())
    }
}

/// PCA9554 controller errors. Boiled down — the I²C bus error type
/// is per-bus and not worth surfacing through the BSP's peripheral
/// API.
#[derive(Debug)]
pub enum Error {
    /// The bus transaction failed.
    Bus,
}

/// Mutex-wrapped controller for cross-task sharing.
pub(crate) type SharedPcaController = Mutex<CriticalSectionRawMutex, Pca9554Controller>;
