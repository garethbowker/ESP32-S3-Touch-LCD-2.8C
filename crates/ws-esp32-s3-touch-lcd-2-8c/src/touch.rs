//! GT911 capacitive touch poller.
//!
//! [`TouchPoller`] owns its own [`BoardI2cDevice`] clone plus a
//! scratch buffer sized for `gt911::get_touch`. Move the whole thing
//! into a task and call [`TouchPoller::poll`] in a loop — no need
//! to plumb the I²C bus through the task's arguments.

use crate::shared_bus::BoardI2cDevice;

/// What a single GT911 read returned.
pub type Point = gt911::Point;
/// GT911 driver errors. Re-exported so consumers don't have to
/// match on a deeply-nested generic error type.
pub type Error = gt911::Error<embassy_embedded_hal::shared_bus::I2cDeviceError<esp_hal::i2c::master::Error>>;

/// Touch-controller poller. Self-contained — owns its bus handle
/// and scratch buffer.
pub struct TouchPoller {
    driver: gt911::Gt911<BoardI2cDevice>,
    bus: BoardI2cDevice,
    buf: [u8; gt911::GET_TOUCH_BUF_SIZE],
}

impl TouchPoller {
    pub(crate) fn new(driver: gt911::Gt911<BoardI2cDevice>, bus: BoardI2cDevice) -> Self {
        Self {
            driver,
            bus,
            buf: [0u8; gt911::GET_TOUCH_BUF_SIZE],
        }
    }

    /// Poll the GT911 for a single touch report.
    ///
    /// Returns:
    /// - `Ok(Some(point))` — finger down at `point`.
    /// - `Ok(None)` — finger lifted since last poll.
    /// - `Err(Error::NotReady)` — polled faster than the chip's
    ///   internal scan rate; not an error, just wait and retry.
    /// - `Err(other)` — I²C or protocol failure.
    pub async fn poll(&mut self) -> Result<Option<Point>, Error> {
        self.driver.get_touch(&mut self.bus, &mut self.buf).await
    }
}
