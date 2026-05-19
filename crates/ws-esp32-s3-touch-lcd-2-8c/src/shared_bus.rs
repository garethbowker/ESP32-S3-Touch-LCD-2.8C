//! Shared async I²C bus types.
//!
//! v2 of the BSP serialises access to the I²C bus through an
//! [`embassy_sync::mutex::Mutex`] so multiple drivers — touch, buzzer,
//! RTC, IMU, and anything the consumer brings — can use the bus from
//! independent tasks without stepping on each other.
//!
//! [`Board::i2c`](crate::Board::i2c) hands back a `'static` reference
//! to the wrapped bus; build an [`I2cDevice`] per consumer:
//!
//! ```ignore
//! use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
//! let mut my_device_bus = I2cDevice::new(board.i2c);
//! my_driver.read(&mut my_device_bus).await?;
//! ```
//!
//! Each [`I2cDevice`] is cheap (just a `&'static Mutex`) and acquires
//! the mutex per transaction. Idle waits are async — the executor
//! yields, no critical-section blocking.

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::{i2c::master::I2c, Async};

/// The wrapped, shared I²C bus. Stored in a `StaticCell` by
/// [`crate::init`]; consumers access it via
/// [`Board::i2c`](crate::Board::i2c).
pub type SharedI2c = Mutex<CriticalSectionRawMutex, I2c<'static, Async>>;

/// An `embedded-hal-async`-compatible handle to the shared bus.
/// Build one per consumer:
/// `let bus = I2cDevice::new(board.i2c);`
pub type BoardI2cDevice = I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>;
