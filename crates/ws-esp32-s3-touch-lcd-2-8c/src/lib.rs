//! Board-support package for the **Waveshare ESP32-S3-Touch-LCD-2.8C** —
//! a 2.8" round 480×480 IPS panel with a Goodix GT911 capacitive touch
//! overlay, mounted on an ESP32-S3 carrier with 8 MB octal PSRAM.
//!
//! See the [board's wiki][board] for the hardware.
//!
//! [board]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C
//!
//! ## One-call setup
//!
//! ```ignore
//! use esp_hal::{psram::PsramConfig, psram::PsramSize};
//! use ws_esp32_s3_touch_lcd_2_8c as bsp;
//!
//! let peripherals = esp_hal::init(
//!     esp_hal::Config::default().with_psram(PsramConfig {
//!         size: PsramSize::Size(8 * 1024 * 1024),
//!         ..Default::default()
//!     }),
//! );
//!
//! let board = bsp::init(bsp::take_resources!(peripherals))
//!     .expect("board init");
//!
//! board.framebuffer.fill(0x07E0); // green
//! // ...your render loop here...
//!
//! // Poll the touch controller:
//! let mut i2c = board.i2c;
//! match board.touch.get_touch(&mut i2c) {
//!     Ok(Some(point)) => log::info!("touch @ ({}, {})", point.x, point.y),
//!     Ok(None) => {} // finger lifted, no current touch
//!     Err(gt911::Error::NotReady) => {} // poll faster than the chip — ignore
//!     Err(e) => log::warn!("gt911: {:?}", e),
//! }
//! ```
//!
//! ## What [`init`] does
//!
//! 1. Brings up the I²C0 bus at 400 kHz on the board's SDA/SCL pins.
//! 2. Configures the PCA9554 I/O expander (address `0x20`): bit 0 →
//!    LCD reset (high), bit 1 → touch reset (high), bit 2 → ST7701
//!    chip-select (high), bit 7 → on-board piezo (low, i.e. silent).
//!    The PCA9554 powers up with every pin in input mode, so the
//!    direction register *must* be written — not just the output
//!    latch — for any of these signals to actually reach their pins.
//! 3. Drives the ST7701 reset pulse and clocks out the panel's init
//!    sequence over bit-banged 9-bit "3-wire" SPI.
//! 4. Latches the GT911's I²C address to `0x5D` via the canonical
//!    INT-low / RST-pulse reset dance.
//! 5. Allocates the 480×480 RGB565 framebuffer in PSRAM and configures
//!    the LCD_CAM peripheral's DPI block with the board's documented
//!    timings.
//! 6. Sets up a DRAM bounce-buffer descriptor ring and starts a
//!    continuous DMA transfer feeding pixels to the panel. Installs
//!    the EOF interrupt handler that refills each bounce half from
//!    PSRAM (works around the PSRAM→DPI drift artefacts on this SoC).
//! 7. Drives the backlight GPIO high.
//!
//! On return, the panel is alive and the framebuffer is black. CPU
//! writes via [`Framebuffer::write_row`] / [`Framebuffer::fill`] become
//! visible within one frame (~23 ms at 12 MHz PCLK).
//!
//! The returned [`Board`] also exposes pins for the off-board
//! peripherals the BSP doesn't drive itself: the SD card slot
//! ([`Board::sd_pins`]) and the battery-voltage divider
//! ([`Board::battery_adc`]). The PCF85063 RTC and QMI8658 IMU sit on
//! the I²C bus already (see [`consts::i2c`]); bring your own driver.
//!
//! ## Caveats
//!
//! - **Must be called at most once per boot.** The bounce-buffer
//!   descriptor ring and EOF ISR use module-level statics; a second
//!   call would corrupt them.
//! - The DPI DMA transfer is leaked intentionally (`mem::forget`) —
//!   dropping it would stop the panel.
//! - Requires `esp_hal::init` to have been called with PSRAM sized to
//!   at least the framebuffer (~450 KB).

#![no_std]
#![deny(missing_docs)]

mod bounce_buffer;
mod framebuffer;
mod pca9554;
mod shared_bus;

mod backlight;
mod buzzer;
mod touch;
#[cfg(feature = "rtc")]
pub mod rtc;
#[cfg(feature = "imu")]
pub mod imu;

pub mod consts;

pub use backlight::Backlight;
pub use buzzer::Buzzer;
pub use shared_bus::{BoardI2cDevice, SharedI2c};
pub use touch::{Point, TouchPoller};
#[cfg(feature = "rtc")]
pub use rtc::Rtc;
#[cfg(feature = "imu")]
pub use imu::Imu;

use core::cell::RefCell;
use core::convert::Infallible;

use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::i2c::RefCellDevice;

use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    lcd_cam::{
        lcd::{
            dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
            ClockMode, Phase as SpiPhase, Polarity,
        },
        LcdCam,
    },
    peripherals,
    time::Rate,
};

// Re-export the underlying driver crates so consumers don't need to
// pull them in separately (or worry about version skew).
pub use gt911;
pub use port_expander;
pub use st7701;

pub use bounce_buffer::BounceRing;
pub use framebuffer::{
    Framebuffer, BYTES as FRAMEBUFFER_BYTES, HEIGHT, PSRAM_BYTES_REQUIRED, WIDTH,
};

// ---------------------------------------------------------------------------
// Public board peripherals/resources
// ---------------------------------------------------------------------------

/// All board peripherals and pins consumed by [`init`].
///
/// Populated either by hand from an `esp_hal::peripherals::Peripherals`,
/// or — preferred — via the [`take_resources!`] macro which
/// destructures the standard pinout for you. Future BSP releases
/// may add fields here; the macro is owned by the BSP and updated
/// in lockstep, so consumers using it are insulated.
#[allow(missing_docs)]
pub struct Resources<'d> {
    pub i2c0:        peripherals::I2C0<'d>,
    pub lcd_cam:     peripherals::LCD_CAM<'d>,
    pub dma_ch0:     peripherals::DMA_CH0<'d>,
    pub psram:       peripherals::PSRAM<'d>,

    pub sda:         peripherals::GPIO15<'d>,
    pub scl:         peripherals::GPIO7<'d>,
    // GPIO2 (`st7701_sck`) and GPIO1 (`st7701_mosi`) double as SD CLK
    // and SD CMD — see [`consts::sd`]. `init` only uses them as
    // outputs during the ST7701 bit-bang and returns the originals
    // via [`Board::sd_pins`].
    pub st7701_sck:  peripherals::GPIO2<'d>,
    pub st7701_mosi: peripherals::GPIO1<'d>,
    pub touch_int:   peripherals::GPIO16<'d>,
    pub backlight:   peripherals::GPIO6<'d>,

    pub vsync:       peripherals::GPIO39<'d>,
    pub hsync:       peripherals::GPIO38<'d>,
    pub de:          peripherals::GPIO40<'d>,
    pub pclk:        peripherals::GPIO41<'d>,

    pub data: DpiDataPins<'d>,

    /// Battery-divider sense (ADC1 channel 3). Passed through to
    /// [`Board::battery_adc`]; the BSP itself doesn't touch the ADC.
    pub battery_adc: peripherals::GPIO4<'d>,
    /// SD card data line 0. Passed through to [`Board::sd_pins`].
    pub sd_d0:       peripherals::GPIO42<'d>,
}

/// The 16 DPI data pins.
///
/// The board wires the 16-bit RGB565 bus as B0..B4 / G0..G5 / R0..R4
/// (5 + 6 + 5 = 16 lines). [`Resources::take`] populates these
/// automatically; only construct one by hand if you've rewired the
/// board.
#[allow(missing_docs)]
pub struct DpiDataPins<'d> {
    pub b0: peripherals::GPIO5<'d>,
    pub b1: peripherals::GPIO45<'d>,
    pub b2: peripherals::GPIO48<'d>,
    pub b3: peripherals::GPIO47<'d>,
    pub b4: peripherals::GPIO21<'d>,
    pub g0: peripherals::GPIO14<'d>,
    pub g1: peripherals::GPIO13<'d>,
    pub g2: peripherals::GPIO12<'d>,
    pub g3: peripherals::GPIO11<'d>,
    pub g4: peripherals::GPIO10<'d>,
    pub g5: peripherals::GPIO9<'d>,
    pub r0: peripherals::GPIO46<'d>,
    pub r1: peripherals::GPIO3<'d>,
    pub r2: peripherals::GPIO8<'d>,
    pub r3: peripherals::GPIO18<'d>,
    pub r4: peripherals::GPIO17<'d>,
}

/// Pluck the board's standard pins and peripherals out of an
/// `esp_hal::peripherals::Peripherals`, returning a [`Resources`]
/// struct ready to pass to [`init`].
///
/// Implemented as a macro rather than a function so other peripheral
/// fields (UART, USB, the unused GPIOs, the timer group you want for
/// embassy, …) remain accessible afterwards. A function would have to
/// take `Peripherals` by value, consuming the whole struct.
///
/// ```ignore
/// let peripherals = esp_hal::init(/* ... */);
///
/// // Pluck timers and anything else you need *first* —
/// let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
/// esp_hal_embassy::init(timg0.timer0);
///
/// // — then hand the rest to the BSP.
/// let board = bsp::init(bsp::take_resources!(peripherals))?;
/// ```
#[macro_export]
macro_rules! take_resources {
    ($p:expr) => {
        $crate::Resources {
            i2c0:        $p.I2C0,
            lcd_cam:     $p.LCD_CAM,
            dma_ch0:     $p.DMA_CH0,
            psram:       $p.PSRAM,
            sda:         $p.GPIO15,
            scl:         $p.GPIO7,
            st7701_sck:  $p.GPIO2,
            st7701_mosi: $p.GPIO1,
            touch_int:   $p.GPIO16,
            backlight:   $p.GPIO6,
            vsync:       $p.GPIO39,
            hsync:       $p.GPIO38,
            de:          $p.GPIO40,
            pclk:        $p.GPIO41,
            data: $crate::DpiDataPins {
                b0: $p.GPIO5,  b1: $p.GPIO45, b2: $p.GPIO48, b3: $p.GPIO47, b4: $p.GPIO21,
                g0: $p.GPIO14, g1: $p.GPIO13, g2: $p.GPIO12, g3: $p.GPIO11, g4: $p.GPIO10, g5: $p.GPIO9,
                r0: $p.GPIO46, r1: $p.GPIO3,  r2: $p.GPIO8,  r3: $p.GPIO18, r4: $p.GPIO17,
            },
            battery_adc: $p.GPIO4,
            sd_d0:       $p.GPIO42,
        }
    };
}

// ---------------------------------------------------------------------------
// Public board handle
// ---------------------------------------------------------------------------

/// What [`init`] hands back. Every on-board peripheral the BSP can
/// drive itself is exposed as a first-class handle with its own
/// methods — no need to thread I²C buses or scratch buffers
/// through your tasks. Pass-through fields ([`Self::sd_pins`],
/// [`Self::battery_adc`]) cover the peripherals that don't have an
/// async-Rust driver yet.
///
/// `Board` has no lifetime parameter — everything inside is
/// `'static`. The shared I²C bus lives in a `StaticCell` owned by
/// the BSP, and `esp_hal::init` returns peripherals with `'static`
/// lifetime already.
///
/// Marked `#[non_exhaustive]` so future additions won't break
/// pattern-matching consumers.
#[non_exhaustive]
pub struct Board {
    /// CPU-side handle to the PSRAM framebuffer. Write pixels here;
    /// they appear on the panel within one frame.
    pub framebuffer: Framebuffer,

    /// The shared I²C bus. Build an
    /// [`I2cDevice`](embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice)
    /// per consumer to use it from independent tasks. The BSP's
    /// own peripherals already hold their own clones; this handle
    /// is for anything *you* want to put on the bus.
    pub i2c: &'static SharedI2c,

    /// GT911 capacitive touch poller. Self-contained: owns its
    /// `I2cDevice` clone and scratch buffer. Move into a task and
    /// call [`TouchPoller::poll`] in a loop.
    pub touch: TouchPoller,

    /// On-board piezo buzzer. Left silent by [`init`]; pulse with
    /// [`Buzzer::beep_for`] or drive directly with
    /// [`Buzzer::on`] / [`Buzzer::off`].
    pub buzzer: Buzzer,

    /// LCD backlight, currently full-on. Drive low for blank, or
    /// consume into a raw `Output` via [`Backlight::into_inner`]
    /// to reconfigure as an LEDC PWM channel.
    pub backlight: Backlight,

    /// PCF85063A real-time clock. Reads/writes datetime via I²C —
    /// already on the shared bus.
    #[cfg(feature = "rtc")]
    pub rtc: Rtc,

    /// QMI8658C 6-axis IMU. Call [`Imu::init`] once before reading
    /// samples — the driver's init handshake is async and isn't
    /// run by [`init`].
    #[cfg(feature = "imu")]
    pub imu: Imu,

    /// Raw GPIO4 for the battery-divider sense. The BSP doesn't
    /// configure the ADC — bring up `Adc<ADC1>` yourself and pass
    /// this pin. See [`consts::battery`] for the channel/attenuation
    /// to use and the divider ratio.
    pub battery_adc: peripherals::GPIO4<'static>,

    /// SD card pins, released after [`init`] used GPIO1/2 for the
    /// ST7701 init bit-bang. See [`SdPins`] for caveats.
    pub sd_pins: SdPins,
}

/// SD/MMC pins exposed by the BSP for consumer-managed SD card
/// support.
///
/// **`esp-hal` 1.0.0-rc.0 doesn't yet expose an SDHOST driver for
/// the S3**, so these pins can't actually be wired up from pure
/// Rust today — but the BSP's pin-ownership handoff (ST7701 init
/// → SD use) is already done, so a future driver only needs the
/// protocol code, not the BSP plumbing.
///
/// The SD card's D3 line runs through the PCA9554 expander — see
/// [`consts::pca9554::SD_D3_EN_BIT`]. When a future SD driver
/// lands the BSP will expose a first-class `SdCard` handle that
/// drives this bit itself; for now consumers needing it can build
/// a `port_expander::Pca9554` on top of [`Board::i2c`].
#[non_exhaustive]
pub struct SdPins {
    /// SD CLK (GPIO2).
    pub clk: peripherals::GPIO2<'static>,
    /// SD CMD (GPIO1).
    pub cmd: peripherals::GPIO1<'static>,
    /// SD D0 (GPIO42).
    pub d0:  peripherals::GPIO42<'static>,
}

/// What can go wrong during [`init`].
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// `esp_hal::i2c::master::I2c::new` rejected its config — wrong
    /// frequency, wrong peripheral state, etc.
    I2cConfig,
    /// PCA9554 didn't ack a write. Either it's not on the bus
    /// (wiring), or the bus pull-ups are missing/wrong, or the address
    /// strap pins were latched differently than expected.
    Pca9554,
    /// ST7701 init bit-bang failed — only possible if the CS pin (a
    /// PCA9554 line) failed mid-sequence, i.e. I²C bus error.
    St7701,
    /// `esp_hal::lcd_cam::lcd::dpi::Dpi::new` rejected its config.
    Dpi,
    /// PSRAM mapped region is smaller than [`PSRAM_BYTES_REQUIRED`]
    /// (two framebuffers, ~920 KB). Make sure `esp_hal::init` was
    /// called with the right `PsramConfig`.
    PsramTooSmall,
    /// The DPI DMA transfer failed to start.
    DpiStart,
    /// GT911 didn't ack, or returned an unexpected product ID. Usually
    /// means the reset sequence didn't latch the address correctly.
    Gt911,
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Bring the board up.
///
/// See the module docs for what's configured and the call-once caveat.
pub fn init(r: Resources<'static>) -> Result<Board, Error> {
    let delay = Delay::new();

    // ----- I²C bus + PCA9554 setup ------------------------------------------
    let i2c = I2c::new(
        r.i2c0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| Error::I2cConfig)?
    .with_sda(r.sda)
    .with_scl(r.scl);

    // ST7701 init pins (bit-banged 3-wire SPI).
    //
    // GPIO2 / GPIO1 are *shared* with the SD card's CLK/CMD lines on
    // this carrier. We need them as `Output` for the init bit-bang
    // and then have to hand them back to the consumer in
    // `Board::sd_pins`. `Peripheral::reborrow` creates a
    // shorter-lifetime peripheral handle that the `Output` consumes;
    // when the `Output` is dropped at the end of the init scope, the
    // reborrow goes with it and the originals (`sd_clk`, `sd_cmd`)
    // are usable again.
    let mut sd_clk = r.st7701_sck;
    let mut sd_cmd = r.st7701_mosi;
    let sck  = Output::new(sd_clk.reborrow(), Level::Low, OutputConfig::default());
    let mosi = Output::new(sd_cmd.reborrow(), Level::Low, OutputConfig::default());

    // The PCA9554, the ST7701 init, and the GT911 reset dance all
    // share the I²C bus, and need to finish before the GT911 driver
    // can take it over. Scope it so the RefCell exclusivity boundary
    // is well-defined.
    let i2c_cell = RefCell::new(i2c);
    {
        let dev = RefCellDevice::new(&i2c_cell);
        let mut pca = port_expander::dev::pca9554::Pca9554::new(dev, false, false, false);
        let pins = pca.split();

        // Drive the three control lines high *and* flip them to
        // output mode. `into_output_high` writes the output latch
        // before the direction register, so the pin transitions
        // input → output already at HIGH and the panel/touch never
        // see a glitch low.
        //
        // Using `set_high` alone (which `Pin<QuasiBidirectional, _>`
        // permits) only writes the OutputPort latch and leaves the
        // Configuration register at its 0xFF power-on default — all
        // pins stay electrically high-Z. That works *after* a
        // software reset (the PCA9554 keeps its registers), but on a
        // cold boot the LCD reset pulse never reaches the ST7701 and
        // the panel stays in whatever state it was last left in
        // (usually black). v1.0.0 had that bug.
        let lcd_rst       = pins.io0.into_output_high().map_err(|_| Error::Pca9554)?;
        let mut touch_rst = pins.io1.into_output_high().map_err(|_| Error::Pca9554)?;
        let st7701_cs     = pins.io2.into_output_high().map_err(|_| Error::Pca9554)?;

        // Silence the on-board piezo. The factory firmware leaves
        // PCA9554 io7 high (= constant tone); without claiming the
        // pin here, the BSP would inherit that state across resets.
        // `into_output` clears the output latch before flipping the
        // direction, so the buzzer never beeps on boot.
        let _buzzer = pins.io7.into_output().map_err(|_| Error::Pca9554)?;

        // ST7701 init.
        //
        // The crate is HAL-agnostic but requires all four GPIOs to
        // share an OutputPin::Error type. SCK/MOSI are esp-hal Output
        // (Error = Infallible) while CS/RST are port-expander pins
        // (Error = something I²C-flavoured). Wrap CS and the
        // (separate) RST pin to make all four Infallible — an I²C
        // failure mid-init is unrecoverable anyway, so panicking is
        // the only sensible thing to do.
        let mut st = st7701::St7701::new(
            sck,
            mosi,
            UnwrapPin::new(st7701_cs, "ST7701 CS (PCA9554 io2)"),
            UnwrapPin::new(lcd_rst, "ST7701 RST (PCA9554 io0)"),
            delay,
        );
        st.init(st7701::sequences::WAVESHARE_2_8C).map_err(|_| Error::St7701)?;
        // Drop the driver — pins are released; we no longer touch SCK
        // or MOSI. (RST stays high inside the UnwrapPin until the
        // PCA9554 itself goes out of scope.)
        let _ = st.release();

        // GT911 reset sequence. The chip latches its 7-bit I²C address
        // on the rising edge of RST based on the INT level: INT low →
        // 0x5D (we want), INT high → 0x14.
        let mut touch_int = Output::new(r.touch_int, Level::Low, OutputConfig::default());
        delay.delay_millis(10);
        touch_rst.set_low().map_err(|_| Error::Pca9554)?;
        delay.delay_millis(10);
        touch_rst.set_high().map_err(|_| Error::Pca9554)?;
        // Chip needs ~200 ms post-release before it answers I²C.
        delay.delay_millis(200);
        touch_int.set_high();
        // Drop the Output to release INT to high-Z; from now on the
        // GT911 owns the line (and drives it low to flag new data).
        drop(touch_int);
    }
    // PCA9554, RefCellDevice, and the borrow of `i2c_cell` are all
    // dropped here. The cell is exclusively ours again.
    let mut i2c = i2c_cell.into_inner();

    // ----- GT911 init (still blocking — one-shot at boot) ------------------
    let touch_init = gt911::Gt911Blocking::default();
    touch_init.init(&mut i2c).map_err(|_| Error::Gt911)?;

    // ----- DPI peripheral ---------------------------------------------------
    // Timings from `esp-arduino-libs/ESP32_Display_Panel`'s
    // BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_2_8_C.h. 12 MHz PCLK gives
    // ~43 Hz refresh — within the panel's spec and comfortable for
    // the bounce-buffer ISR's refill workload.
    let lcd_cam = LcdCam::new(r.lcd_cam);
    let dpi_config = DpiConfig::default()
        .with_frequency(Rate::from_mhz(12))
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: SpiPhase::ShiftLow,
        })
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: 480,
            horizontal_total_width: 548,
            horizontal_blank_front_porch: 50,
            vertical_active_height: 480,
            vertical_total_height: 508,
            vertical_blank_front_porch: 8,
            hsync_width: 8,
            vsync_width: 2,
            hsync_position: 0,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low);

    let dpi = Dpi::new(lcd_cam.lcd, r.dma_ch0, dpi_config)
        .map_err(|_| Error::Dpi)?
        .with_vsync(r.vsync)
        .with_hsync(r.hsync)
        .with_de(r.de)
        .with_pclk(r.pclk)
        .with_data0(r.data.b0)
        .with_data1(r.data.b1)
        .with_data2(r.data.b2)
        .with_data3(r.data.b3)
        .with_data4(r.data.b4)
        .with_data5(r.data.g0)
        .with_data6(r.data.g1)
        .with_data7(r.data.g2)
        .with_data8(r.data.g3)
        .with_data9(r.data.g4)
        .with_data10(r.data.g5)
        .with_data11(r.data.r0)
        .with_data12(r.data.r1)
        .with_data13(r.data.r2)
        .with_data14(r.data.r3)
        .with_data15(r.data.r4);

    // ----- PSRAM framebuffer + bounce-buffer ring ---------------------------
    let (psram_ptr, psram_size) = esp_hal::psram::psram_raw_parts(&r.psram);
    // SAFETY: `psram_raw_parts` returns the mapped, DMA-capable region.
    // We treat the first 2× FRAMEBUFFER_BYTES of it as ours (front +
    // back); the rest is the user's to do what they like with.
    let (initial_front_ptr, framebuffer) =
        unsafe { framebuffer::Framebuffer::split_from_psram(psram_ptr, psram_size) }
            .ok_or(Error::PsramTooSmall)?;

    // SAFETY: Module-level statics; must be called at most once per
    // boot. Documented in lib.rs. `initial_front_ptr` points to the
    // buffer the ISR will refill from until the user calls
    // `Framebuffer::flip()` for the first time.
    let bounce = unsafe { bounce_buffer::BounceRing::new(initial_front_ptr as *const u8) };

    // ----- Start the continuous DPI transfer + EOF ISR ----------------------
    let transfer = dpi.send(true, bounce).map_err(|_| Error::DpiStart)?;
    // Never tear down — the panel needs continuous refresh.
    core::mem::forget(transfer);
    // SAFETY: precondition is "called outside any interrupt context
    // and after the DPI transfer is running"; both true here.
    unsafe { bounce_buffer::enable_eof_interrupt() };

    // ----- Backlight on -----------------------------------------------------
    let backlight_pin = Output::new(r.backlight, Level::High, OutputConfig::default());

    // ----- Switch I²C to async + share via mutex ---------------------------
    //
    // Up to this point the I²C bus was blocking, which is exactly what
    // we want for the synchronous init dance (no executor running). For
    // runtime we want async + shareable across tasks (touch poll,
    // buzzer, RTC, IMU, whatever the consumer adds). esp-hal supports
    // the conversion via `I2c::into_async()`, after which we wrap in an
    // embassy `Mutex` and stash that in a `StaticCell` so the resulting
    // `&'static SharedI2c` can be cloned cheaply into each consumer's
    // `I2cDevice`.
    static I2C_BUS: StaticCell<SharedI2c> = StaticCell::new();
    let bus: &'static SharedI2c = I2C_BUS.init(Mutex::new(i2c.into_async()));

    // ----- PCA9554 runtime controller --------------------------------------
    //
    // Buzzer (and SD-D3 enable, once that lands) need to manipulate
    // PCA9554 register bits without clobbering each other's cached
    // state. The controller does true read-modify-write of the chip
    // registers, and the surrounding `Mutex` closes the RMW window
    // across tasks. One instance for the whole BSP.
    static PCA_CTRL: StaticCell<pca9554::SharedPcaController> = StaticCell::new();
    let pca = PCA_CTRL.init(Mutex::new(pca9554::Pca9554Controller::new(
        embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(bus),
        consts::i2c::PCA9554_ADDR,
    )));

    // ----- Build the first-class peripheral handles ------------------------
    let touch_driver = gt911::Gt911::default();
    let touch_bus = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(bus);
    let touch = TouchPoller::new(touch_driver, touch_bus);

    let buzzer = Buzzer::new(pca);
    let backlight = Backlight::new(backlight_pin);

    #[cfg(feature = "rtc")]
    let rtc = Rtc::new(embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(bus));

    #[cfg(feature = "imu")]
    let imu = Imu::new(embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(bus));

    Ok(Board {
        framebuffer,
        i2c: bus,
        touch,
        buzzer,
        backlight,
        #[cfg(feature = "rtc")]
        rtc,
        #[cfg(feature = "imu")]
        imu,
        battery_adc: r.battery_adc,
        sd_pins: SdPins {
            clk: sd_clk,
            cmd: sd_cmd,
            d0:  r.sd_d0,
        },
    })
}

// ---------------------------------------------------------------------------
// Pin error unwrap adapter
// ---------------------------------------------------------------------------

/// Wrap an [`OutputPin`] whose `Error` is fallible into one whose
/// `Error` is [`Infallible`], panicking with `ctx` if the underlying
/// pin ever errors. Used during one-shot setup paths where any pin
/// failure is unrecoverable anyway.
struct UnwrapPin<P> {
    pin: P,
    ctx: &'static str,
}

impl<P> UnwrapPin<P> {
    fn new(pin: P, ctx: &'static str) -> Self {
        Self { pin, ctx }
    }
}

impl<P: OutputPin> ErrorType for UnwrapPin<P>
where
    P::Error: core::fmt::Debug,
{
    type Error = Infallible;
}

impl<P: OutputPin> OutputPin for UnwrapPin<P>
where
    P::Error: core::fmt::Debug,
{
    fn set_low(&mut self) -> Result<(), Infallible> {
        self.pin
            .set_low()
            .unwrap_or_else(|e| panic!("{}: set_low failed: {e:?}", self.ctx));
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Infallible> {
        self.pin
            .set_high()
            .unwrap_or_else(|e| panic!("{}: set_high failed: {e:?}", self.ctx));
        Ok(())
    }
}
