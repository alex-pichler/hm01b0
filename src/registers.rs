//! HM01B0 register addresses and the power-on register table.
//!
//! The first block below is every register the power-on configuration in
//! [`DEFAULT_REGISTERS`] touches or that this driver writes at run time. The
//! second block holds registers that exist in the datasheet but are outside
//! that configuration; they are marked individually.

// ---------------------------------------------------------------------------
// Registers in the power-on configuration
// ---------------------------------------------------------------------------

/// Model ID, high byte. Reads `0x01` on an HM01B0.
pub const MODEL_ID_H: u16 = 0x0000;
/// Model ID, low byte. Reads `0xB0` on an HM01B0.
pub const MODEL_ID_L: u16 = 0x0001;
/// Mode select. See [`Mode`](crate::Mode) for the values.
pub const MODE_SELECT: u16 = 0x0100;
/// Software reset. Written with `0x00`.
pub const SW_RESET: u16 = 0x0103;
/// Analog gain.
pub const ANALOG_GAIN: u16 = 0x0205;
/// Digital gain, high byte (integer part).
pub const DIGITAL_GAIN_H: u16 = 0x020E;
/// Digital gain, low byte (fractional part).
pub const DIGITAL_GAIN_L: u16 = 0x020F;
/// Digital gain control.
pub const DGAIN_CONTROL: u16 = 0x0350;
/// Test pattern mode. See [`TestPattern`](crate::TestPattern).
pub const TEST_PATTERN_MODE: u16 = 0x0601;
/// Black level calibration configuration.
pub const BLC_CFG: u16 = 0x1000;
/// Black level calibration dither.
pub const BLC_DITHER: u16 = 0x1001;
/// Black level calibration dark pixel target.
pub const BLC_DARKPIXEL: u16 = 0x1002;
/// Black level calibration target.
pub const BLC_TGT: u16 = 0x1003;
/// Black level interpolation enable.
pub const BLI_EN: u16 = 0x1006;
/// Second black level calibration target.
pub const BLC2_TGT: u16 = 0x1007;
/// Defect pixel correction control.
pub const DPC_CTRL: u16 = 0x1008;
/// Defect pixel cluster hot threshold.
pub const CLUSTER_THR_HOT: u16 = 0x1009;
/// Defect pixel cluster cold threshold.
pub const CLUSTER_THR_COLD: u16 = 0x100A;
/// Single defect pixel hot threshold.
pub const SINGLE_THR_HOT: u16 = 0x100B;
/// Single defect pixel cold threshold.
pub const SINGLE_THR_COLD: u16 = 0x100C;
/// VSYNC/HSYNC/pixel shift enable. Cleared by
/// [`Hm01b0::configure`](crate::Hm01b0::configure) before streaming.
pub const VSYNC_HSYNC_PIXEL_SHIFT_EN: u16 = 0x1012;
/// Statistics engine control.
pub const STATISTIC_CTRL: u16 = 0x2000;
/// Motion-detection ROI, X start, high byte.
pub const MD_LROI_X_START_H: u16 = 0x2011;
/// Motion-detection ROI, X start, low byte.
pub const MD_LROI_X_START_L: u16 = 0x2012;
/// Motion-detection ROI, Y start, high byte.
pub const MD_LROI_Y_START_H: u16 = 0x2013;
/// Motion-detection ROI, Y start, low byte.
pub const MD_LROI_Y_START_L: u16 = 0x2014;
/// Motion-detection ROI, X end, high byte.
pub const MD_LROI_X_END_H: u16 = 0x2015;
/// Motion-detection ROI, X end, low byte.
pub const MD_LROI_X_END_L: u16 = 0x2016;
/// Motion-detection ROI, Y end, high byte.
pub const MD_LROI_Y_END_H: u16 = 0x2017;
/// Motion-detection ROI, Y end, low byte.
pub const MD_LROI_Y_END_L: u16 = 0x2018;
/// Auto-exposure control. Bit 0 enables the AE loop.
pub const AE_CTRL: u16 = 0x2100;
/// Auto-exposure target mean brightness.
pub const AE_TARGET_MEAN: u16 = 0x2101;
/// Auto-exposure minimum mean brightness.
pub const AE_MIN_MEAN: u16 = 0x2102;
/// Auto-exposure convergence-in threshold.
pub const CONVERGE_IN_TH: u16 = 0x2103;
/// Auto-exposure convergence-out threshold.
pub const CONVERGE_OUT_TH: u16 = 0x2104;
/// Auto-exposure maximum integration time, high byte (in lines).
///
/// [`DEFAULT_REGISTERS`] writes `0x0214` (532 lines) across this and [`MAX_INTG_L`],
/// which is [`VENDOR_MAX_INTEGRATION_LINES`](crate::VENDOR_MAX_INTEGRATION_LINES)
/// and sits deliberately below the frame-rate knee. This is the most
/// consequential default in the table; see
/// [`Hm01b0::set_max_integration_lines`](crate::Hm01b0::set_max_integration_lines).
pub const MAX_INTG_H: u16 = 0x2105;
/// Auto-exposure maximum integration time, low byte (in lines). See
/// [`MAX_INTG_H`].
pub const MAX_INTG_L: u16 = 0x2106;
/// Auto-exposure minimum integration time (in lines). [`DEFAULT_REGISTERS`]
/// writes [`VENDOR_MIN_INTEGRATION_LINES`](crate::VENDOR_MIN_INTEGRATION_LINES).
pub const MIN_INTG: u16 = 0x2107;
/// Auto-exposure maximum analog gain, full resolution.
pub const MAX_AGAIN_FULL: u16 = 0x2108;
/// Auto-exposure maximum analog gain, 2x2 binned.
pub const MAX_AGAIN_BIN2: u16 = 0x2109;
/// Auto-exposure minimum analog gain.
pub const MIN_AGAIN: u16 = 0x210A;
/// Auto-exposure maximum digital gain.
pub const MAX_DGAIN: u16 = 0x210B;
/// Auto-exposure minimum digital gain.
pub const MIN_DGAIN: u16 = 0x210C;
/// Auto-exposure damping factor.
pub const DAMPING_FACTOR: u16 = 0x210D;
/// Flicker-suppression control.
pub const FS_CTRL: u16 = 0x210E;
/// Flicker suppression, 60 Hz, high byte.
pub const FS_60HZ_H: u16 = 0x210F;
/// Flicker suppression, 60 Hz, low byte.
pub const FS_60HZ_L: u16 = 0x2110;
/// Flicker suppression, 50 Hz, high byte.
pub const FS_50HZ_H: u16 = 0x2111;
/// Flicker suppression, 50 Hz, low byte.
pub const FS_50HZ_L: u16 = 0x2112;
/// Motion-detection control.
pub const MD_CTRL: u16 = 0x2150;
/// Clears the motion-detection interrupt when written with `1`.
pub const I2C_CLEAR: u16 = 0x2153;
/// Motion-detection threshold (low).
pub const MD_THL: u16 = 0x215B;
/// Output bit-depth / bus control.
pub const BIT_CONTROL: u16 = 0x3059;
/// Oscillator clock divider. Bit 5 selects gated-clock output mode.
pub const OSC_CLK_DIV: u16 = 0x3060;

// ---------------------------------------------------------------------------
// Registers outside the power-on configuration.
//
// These are what make manual exposure control possible; see the crate-level
// docs. Nothing in the known-good configuration writes them, so treat them as
// datasheet-derived rather than field-proven.
// ---------------------------------------------------------------------------

/// Group parameter hold.
///
/// Write `1` before changing exposure/gain and `0` afterwards so the sensor
/// latches the whole group on the same frame boundary instead of applying the
/// bytes piecemeal.
pub const GRP_PARAM_HOLD: u16 = 0x0104;
/// Coarse integration time, high byte, in lines.
pub const INTEGRATION_H: u16 = 0x0202;
/// Coarse integration time, low byte, in lines.
pub const INTEGRATION_L: u16 = 0x0203;

/// The power-on register table, in the exact order coralmicro writes it.
///
/// Transcribed from `CameraTask::SetDefaultRegisters()` in
/// `libs/camera/camera.cc`, whose own comment reads *"Taken from Tensorflow's
/// configuration in the person detection sample"*. Register values are not
/// invented here and must not be "cleaned up". See in particular the
/// `0x3044` to `0x3058` block, which coralmicro flags as RESERVED in the
/// datasheet but necessary: *"These registers are RESERVED in the datasheet,
/// but without them the picture is bad."*
///
/// Note that entry `AE_CTRL = 0x01` leaves auto-exposure enabled; see
/// [`Hm01b0::set_auto_exposure`](crate::Hm01b0::set_auto_exposure). Note also
/// that `MAX_INTG = 0x0214` caps that loop below the frame-rate knee, which
/// makes it underexpose rather than slow down; see
/// [`Hm01b0::set_max_integration_lines`](crate::Hm01b0::set_max_integration_lines).
pub const DEFAULT_REGISTERS: [(u16, u8); 45] = [
    // Analog settings.
    (BLC_TGT, 0x08),
    (BLC2_TGT, 0x08),
    // RESERVED in the datasheet, but the image is bad without them.
    // (coralmicro camera.cc, verbatim comment)
    (0x3044, 0x0A),
    (0x3045, 0x00),
    (0x3047, 0x0A),
    (0x3050, 0xC0),
    (0x3051, 0x42),
    (0x3052, 0x50),
    (0x3053, 0x00),
    (0x3054, 0x03),
    (0x3055, 0xF7),
    (0x3056, 0xF8),
    (0x3057, 0x29),
    (0x3058, 0x1F),
    (BIT_CONTROL, 0x1E),
    // Digital settings.
    (BLC_CFG, 0x43),
    (BLC_DITHER, 0x40),
    (BLC_DARKPIXEL, 0x32),
    (DGAIN_CONTROL, 0x7F),
    (BLI_EN, 0x01),
    (DPC_CTRL, 0x00),
    (CLUSTER_THR_HOT, 0xA0),
    (CLUSTER_THR_COLD, 0x60),
    (SINGLE_THR_HOT, 0x90),
    (SINGLE_THR_COLD, 0x40),
    // Auto-exposure settings.
    (STATISTIC_CTRL, 0x07),
    (AE_CTRL, 0x01),
    (AE_TARGET_MEAN, 0x5F),
    (AE_MIN_MEAN, 0x0A),
    (CONVERGE_IN_TH, 0x03),
    (CONVERGE_OUT_TH, 0x05),
    (MAX_INTG_H, 0x02),
    (MAX_INTG_L, 0x14),
    (MIN_INTG, 0x02),
    (MAX_AGAIN_FULL, 0x03),
    (MAX_AGAIN_BIN2, 0x03),
    (MIN_AGAIN, 0x00),
    (MAX_DGAIN, 0x80),
    (MIN_DGAIN, 0x40),
    (DAMPING_FACTOR, 0x20),
    // 60 Hz flicker suppression.
    (FS_CTRL, 0x03),
    (FS_60HZ_H, 0x00),
    (FS_60HZ_L, 0x85),
    (FS_50HZ_H, 0x00),
    (FS_50HZ_L, 0xA0),
];
