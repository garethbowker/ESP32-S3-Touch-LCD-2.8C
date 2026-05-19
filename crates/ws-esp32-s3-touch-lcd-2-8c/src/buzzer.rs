//! On-board piezo buzzer, driven through the PCA9554 expander.
//!
//! [`Buzzer`] holds a `'static` reference to the BSP's shared
//! [`SharedPcaController`] so it can be moved into a task and
//! interleaved freely with any other PCA9554-mediated peripheral
//! (today: only itself; tomorrow: SD-D3 enable).
//!
//! The buzzer is left silent by [`crate::init`] (output mode, level
//! low). Drive it via [`Buzzer::on`] / [`Buzzer::off`] or pulse it
//! with [`Buzzer::beep_for`].

use embassy_time::{Duration, Timer};

use crate::consts;
use crate::pca9554::SharedPcaController;

/// Handle to the on-board piezo buzzer.
pub struct Buzzer {
    pca: &'static SharedPcaController,
}

impl Buzzer {
    pub(crate) fn new(pca: &'static SharedPcaController) -> Self {
        Self { pca }
    }

    /// Turn the buzzer on. Errors are swallowed — a flaky I²C bus
    /// shouldn't take down the rest of the firmware over a piezo.
    pub async fn on(&self) {
        let mut pca = self.pca.lock().await;
        let _ = pca.set_output(consts::pca9554::BUZZER_BIT, true).await;
    }

    /// Turn the buzzer off. Errors are swallowed.
    pub async fn off(&self) {
        let mut pca = self.pca.lock().await;
        let _ = pca.set_output(consts::pca9554::BUZZER_BIT, false).await;
    }

    /// Beep for `duration`, then go silent. Holds the PCA9554 mutex
    /// only across the two register writes — the in-between
    /// `Timer::after(...)` releases it, so other PCA9554 consumers
    /// aren't blocked for the beep duration.
    pub async fn beep_for(&self, duration: Duration) {
        self.on().await;
        Timer::after(duration).await;
        self.off().await;
    }
}
