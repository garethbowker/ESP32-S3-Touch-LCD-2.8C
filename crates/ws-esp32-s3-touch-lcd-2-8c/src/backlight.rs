//! Backlight handle.
//!
//! Thin wrapper around the GPIO6 [`Output`]. Provides
//! ergonomic on/off and reflects the boot state set by
//! [`crate::init`] (driven high — full brightness).
//!
//! Wraps rather than passes through the raw `Output` so future
//! firmware versions can swap to LEDC-based PWM dimming without
//! changing the consumer-side API.

use esp_hal::gpio::{Level, Output};

/// LCD backlight handle.
pub struct Backlight {
    pin: Output<'static>,
}

impl Backlight {
    pub(crate) fn new(pin: Output<'static>) -> Self {
        Self { pin }
    }

    /// Full brightness (drive GPIO6 high). The backlight is in this
    /// state on return from [`crate::init`].
    pub fn on(&mut self) {
        self.pin.set_high();
    }

    /// Blank (drive GPIO6 low). Useful for power-saving when the UI
    /// is idle.
    pub fn off(&mut self) {
        self.pin.set_low();
    }

    /// Set the backlight on (`true`) or off (`false`).
    pub fn set(&mut self, on: bool) {
        self.pin.set_level(if on { Level::High } else { Level::Low });
    }

    /// Consume the wrapper and return the underlying `Output` for
    /// reconfiguration as an LEDC channel (PWM dimming).
    pub fn into_inner(self) -> Output<'static> {
        self.pin
    }
}
