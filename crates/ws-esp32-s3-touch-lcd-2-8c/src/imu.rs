//! QMI8658C 6-axis IMU.
//!
//! Wrapper around the [`ph_qmi8658`] crate. The Waveshare carrier
//! doesn't break out the IMU's INT1/INT2 lines to a GPIO, so the
//! BSP constructs the driver with both pins set to `None`.
//!
//! The IMU isn't initialised by [`crate::init`] — `Qmi8658::init`
//! is async and needs a `DelayNs` instance, neither of which is
//! easy to thread through a sync BSP init. The consumer calls
//! [`Imu::init`] once after [`crate::init`] returns, then uses the
//! driver normally.

pub use ph_qmi8658::{Config, Error, Qmi8658I2c};
use embedded_hal_async::delay::DelayNs;

use crate::consts;
use crate::shared_bus::BoardI2cDevice;

/// First-class handle to the on-board QMI8658C IMU.
///
/// Call [`Self::init`] once before the first read; the driver does
/// the device handshake and applies the embedded [`Config`].
pub struct Imu {
    inner: Qmi8658I2c<BoardI2cDevice>,
}

impl Imu {
    pub(crate) fn new(bus: BoardI2cDevice) -> Self {
        let config = Config::new();
        let i2c_config = ph_qmi8658::I2cConfig::new(consts::i2c::QMI8658_ADDR);
        let inner = Qmi8658I2c::with_i2c_config(bus, None, None, config, i2c_config);
        Self { inner }
    }

    /// Probe the chip, apply the default config (accel + gyro at
    /// the crate's defaults), and bring the device out of standby.
    /// Pass any [`DelayNs`] implementation — embassy-time's `Delay`
    /// is the obvious choice on this platform.
    pub async fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error> {
        self.inner.init(delay).await
    }

    /// Borrow the underlying driver. Use for everything beyond
    /// init: reading samples, applying custom configs, FIFO, WOM,
    /// self-test.
    pub fn driver_mut(&mut self) -> &mut Qmi8658I2c<BoardI2cDevice> {
        &mut self.inner
    }
}
