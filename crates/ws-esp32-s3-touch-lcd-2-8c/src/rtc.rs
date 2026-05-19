//! PCF85063A real-time clock.
//!
//! Thin wrapper around the [`pcf85063a`] crate so the BSP can return
//! a ready-to-use [`Rtc`] handle from [`crate::init`]. The PCF85063A
//! doesn't need an async init dance — `new()` is purely structural
//! — so the consumer can call [`Rtc::get_datetime`] immediately
//! after the BSP returns.
//!
//! For richer functionality (alarms, periodic interrupts, software
//! reset, RAM-byte storage) call [`Rtc::driver_mut`] to get at the
//! underlying [`pcf85063a::PCF85063`].

pub use pcf85063a::{Error, PCF85063};
#[allow(unused_imports)] // re-exported for consumer convenience
pub use time::{Date, Month, PrimitiveDateTime, Time, Weekday};

use crate::shared_bus::BoardI2cDevice;

/// First-class handle to the on-board PCF85063A RTC.
pub struct Rtc {
    inner: PCF85063<BoardI2cDevice>,
}

impl Rtc {
    pub(crate) fn new(bus: BoardI2cDevice) -> Self {
        Self {
            inner: PCF85063::new(bus),
        }
    }

    /// Read the current date/time.
    pub async fn get_datetime(
        &mut self,
    ) -> Result<PrimitiveDateTime, Error<embassy_embedded_hal::shared_bus::I2cDeviceError<esp_hal::i2c::master::Error>>>
    {
        self.inner.get_datetime().await
    }

    /// Set the date/time. The PCF85063A holds it across resets so
    /// long as VBAT is connected (the carrier provides a coin-cell
    /// backup socket).
    pub async fn set_datetime(
        &mut self,
        datetime: &PrimitiveDateTime,
    ) -> Result<(), Error<embassy_embedded_hal::shared_bus::I2cDeviceError<esp_hal::i2c::master::Error>>>
    {
        self.inner.set_datetime(datetime).await
    }

    /// Borrow the underlying driver mutably. Use for alarms,
    /// software reset, RAM byte, periodic interrupts — anything
    /// not on the fast path of get/set datetime.
    pub fn driver_mut(&mut self) -> &mut PCF85063<BoardI2cDevice> {
        &mut self.inner
    }
}
