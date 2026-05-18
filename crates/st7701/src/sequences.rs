//! Canonical init sequences for known ST7701-based panels.
//!
//! Each sequence is a `&[Step]` table that can be passed straight to
//! [`crate::St7701::init`] / [`crate::St7701::run`]. The byte values
//! come from each panel's reference driver — *don't* mix and match
//! between sequences: the gamma table, MADCTL byte, and pixel-format
//! command are all subtly panel-specific.

use crate::Step;

/// Init sequence for the [Waveshare ESP32-S3-Touch-LCD-2.8C][board] —
/// a 480×480 round IPS panel with an ST7701S controller.
///
/// Values are taken verbatim from
/// `esp-arduino-libs/ESP32_Display_Panel`'s
/// `BOARD_WAVESHARE_ESP32_S3_TOUCH_LCD_2_8_C.h`, which is the reference
/// any C++/ESP-IDF user of this panel ends up using.
///
/// [board]: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.8C
pub const WAVESHARE_2_8C: &[Step] = &[
    // Command2 BK3 select
    Step { cmd: 0xFF, data: &[0x77, 0x01, 0x00, 0x00, 0x13], delay_ms: 0 },
    Step { cmd: 0xEF, data: &[0x08], delay_ms: 0 },
    // Command2 BK0 select
    Step { cmd: 0xFF, data: &[0x77, 0x01, 0x00, 0x00, 0x10], delay_ms: 0 },
    Step { cmd: 0xC0, data: &[0x3B, 0x00], delay_ms: 0 },
    Step { cmd: 0xC1, data: &[0x10, 0x0C], delay_ms: 0 },
    Step { cmd: 0xC2, data: &[0x07, 0x0A], delay_ms: 0 },
    Step { cmd: 0xC7, data: &[0x00], delay_ms: 0 },
    Step { cmd: 0xCC, data: &[0x10], delay_ms: 0 },
    Step { cmd: 0xCD, data: &[0x08], delay_ms: 0 },
    Step {
        cmd: 0xB0,
        data: &[
            0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09, 0x09, 0x23, 0x05, 0x52, 0x0F, 0x67,
            0x2C, 0x11,
        ],
        delay_ms: 0,
    },
    Step {
        cmd: 0xB1,
        data: &[
            0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08, 0x08, 0x22, 0x03, 0x51, 0x11, 0x66,
            0x2B, 0x0F,
        ],
        delay_ms: 0,
    },
    // Command2 BK1 select
    Step { cmd: 0xFF, data: &[0x77, 0x01, 0x00, 0x00, 0x11], delay_ms: 0 },
    Step { cmd: 0xB0, data: &[0x5D], delay_ms: 0 },
    Step { cmd: 0xB1, data: &[0x3E], delay_ms: 0 },
    Step { cmd: 0xB2, data: &[0x81], delay_ms: 0 },
    Step { cmd: 0xB3, data: &[0x80], delay_ms: 0 },
    Step { cmd: 0xB5, data: &[0x4E], delay_ms: 0 },
    Step { cmd: 0xB7, data: &[0x85], delay_ms: 0 },
    Step { cmd: 0xB8, data: &[0x20], delay_ms: 0 },
    Step { cmd: 0xC1, data: &[0x78], delay_ms: 0 },
    Step { cmd: 0xC2, data: &[0x78], delay_ms: 0 },
    Step { cmd: 0xD0, data: &[0x88], delay_ms: 0 },
    Step { cmd: 0xE0, data: &[0x00, 0x00, 0x02], delay_ms: 0 },
    Step {
        cmd: 0xE1,
        data: &[0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33],
        delay_ms: 0,
    },
    Step {
        cmd: 0xE2,
        data: &[0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00],
        delay_ms: 0,
    },
    Step { cmd: 0xE3, data: &[0x00, 0x00, 0x11, 0x11], delay_ms: 0 },
    Step { cmd: 0xE4, data: &[0x44, 0x44], delay_ms: 0 },
    Step {
        cmd: 0xE5,
        data: &[
            0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 0x30, 0xF0, 0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3,
            0x30, 0xF0,
        ],
        delay_ms: 0,
    },
    Step { cmd: 0xE6, data: &[0x00, 0x00, 0x11, 0x11], delay_ms: 0 },
    Step { cmd: 0xE7, data: &[0x44, 0x44], delay_ms: 0 },
    Step {
        cmd: 0xE8,
        data: &[
            0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 0x30, 0xF0, 0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2,
            0x30, 0xF0,
        ],
        delay_ms: 0,
    },
    Step { cmd: 0xE9, data: &[0x36, 0x01], delay_ms: 0 },
    Step { cmd: 0xEB, data: &[0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40], delay_ms: 0 },
    Step {
        cmd: 0xED,
        data: &[
            0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF, 0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA,
            0x01, 0xFF,
        ],
        delay_ms: 0,
    },
    Step { cmd: 0xEF, data: &[0x08, 0x08, 0x08, 0x45, 0x3F, 0x54], delay_ms: 0 },
    // Back to BK0
    Step { cmd: 0xFF, data: &[0x77, 0x01, 0x00, 0x00, 0x00], delay_ms: 0 },
    // Sleep Out — 120 ms wait per the ST7701 datasheet
    Step { cmd: 0x11, data: &[], delay_ms: 120 },
    // Interface Pixel Format: 0x66 = RGB666 on the controller side; the
    // ST7701 maps from a 16-bit RGB565 bus internally.
    Step { cmd: 0x3A, data: &[0x66], delay_ms: 0 },
    // Memory Access Control: 0x00 = no rotation, no BGR (straight RGB).
    Step { cmd: 0x36, data: &[0x00], delay_ms: 0 },
    // Tearing Effect Line On (V-blank only).
    Step { cmd: 0x35, data: &[0x00], delay_ms: 0 },
    // Display On.
    Step { cmd: 0x29, data: &[], delay_ms: 0 },
];
