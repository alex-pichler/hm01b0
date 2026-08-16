#![no_std]
#![deny(missing_docs)]
//! A platform-agnostic driver for the Himax HM01B0 image sensor.
//!
//! The HM01B0 is a 324×324 CMOS sensor with a Bayer colour filter array. It is
//! controlled over I2C (7-bit address `0x24`, 16-bit register addresses, 8-bit
//! register data) and delivers pixels over a separate 8-bit parallel bus, one
//! sample per pixel, which some host peripheral has to receive. Each sample is
//! one colour channel (see [`CFA_PATTERN`]), so a raw frame is *mosaiced*, not
//! greyscale, and a consumer that wants colour has to demosaic it.
//!
//! This crate implements the control side only, on nothing but
//! [`embedded_hal`] 1.0 traits, so it works anywhere and is testable on the
//! host.
//!
//! # Where the register values come from
//!
//! The addresses are the datasheet's. The power-on values are the
//! configuration Google's coralmicro SDK uses on the Coral Dev Board Micro,
//! which is the one known to produce a good image on this part; nothing here
//! is invented. See [`registers`] for the table and its provenance.
//!
//! Three registers sit outside that configuration:
//! [`registers::GRP_PARAM_HOLD`], [`registers::INTEGRATION_H`] and
//! [`registers::INTEGRATION_L`], which come from the datasheet and are what
//! [`Hm01b0::set_exposure`] needs. They are flagged as such.
//!
//! # Colour
//!
//! The raw frame is a Bayer mosaic in BGGR order: `(even, even)` is blue,
//! `(odd, odd)` is red and the two mixed positions are green, in the raw
//! frame's own coordinates with no rotation applied. That is
//! [`CFA_PATTERN`] / [`cfa_color_at`], which carries the evidence for the
//! phase.
//!
//! It is real colour, not a nominal filter over a monochrome part. Averaging
//! the four CFA phases over frames captured on this hardware under warm indoor
//! light gives R = 121.8, G = 113.6, B = 77.4, a spread far too wide to be
//! noise.
//!
//! Demosaicing, white balance, rescaling and rotation are deliberately out of
//! scope; they are host-side pixel processing, not sensor control.
//!
//! # Exposure, frame period and the vendor ceiling
//!
//! The frame period is flat at [`MIN_FRAME_PERIOD_US`] (17.503 ms, 57.1 Hz)
//! until integration time passes the readout time, and grows linearly with
//! integration after that, at [`LINE_PERIOD_NS`] (31.07 µs) per integration
//! line. The crossover is [`FRAME_PERIOD_KNEE_LINES`], ~564 lines, and
//! [`frame_period_us`] is that whole relationship as a function.
//!
//! The power-on table sets the auto-exposure ceiling
//! ([`registers::MAX_INTG_H`] / `_L`) to [`VENDOR_MAX_INTEGRATION_LINES`],
//! 532 lines. 532 is not arbitrary: it sits just below the knee, so the
//! ceiling is placed such that auto-exposure can never cost a frame. That is a
//! defensible choice but an invisible one, because in dim light AE runs into
//! the ceiling and underexposes instead of slowing down, and nothing reports
//! that it happened. On this hardware raising the ceiling to 1800 lines
//! (~55.9 ms, 17.9 Hz worst case, and free in bright light because AE then
//! sits well below the knee) took the loop from railed to converged. See
//! [`Hm01b0::set_max_integration_lines`].
//!
//! Long integration is also motion blur, which is what makes the trade real on
//! anything that moves. Two operating points measured on this hardware:
//!
//! | integration | analogue gain | frame rate | temporal noise |
//! |---|---|---|---|
//! | 1200 lines (~37.3 ms) | 8× | 26.7 Hz | 9.55 LSB |
//! | 3000 lines (~93.2 ms) | 4× | 10.7 Hz | 5.83 LSB |
//!
//! Longer integration buys signal-to-noise; gain does not, it only rescales
//! what the photodiode already collected. Spending frame rate is how you get a
//! quieter image, and [`frame_period_us`] says what it costs.
//!
//! # Example
//!
//! ```no_run
//! # fn example<I, D>(i2c: I, mut delay: D) -> Result<(), hm01b0::Error<I::Error>>
//! # where I: embedded_hal::i2c::I2c, D: embedded_hal::delay::DelayNs {
//! use hm01b0::{Exposure, Hm01b0, Mode};
//!
//! let mut cam = Hm01b0::new(i2c);
//! cam.init(&mut delay)?; // verifies the model ID, then loads the defaults
//!
//! // Optional: pin the exposure instead of leaving auto-exposure on.
//! cam.set_exposure(&Exposure {
//!     integration_lines: 200,
//!     analog_gain: 0x00,
//!     digital_gain: 0x0100,
//! })?;
//!
//! cam.set_mode(Mode::Streaming)?;
//! // ... CSI peripheral now receives 324x324 8-bit Bayer samples ...
//! cam.stop()?;
//! # Ok(())
//! # }
//! ```
//!
//! # Cargo features
//!
//! * `defmt`: derives [`defmt::Format`](https://docs.rs/defmt) on the public
//!   enums, structs and [`Error`]. Off by default; the crate has no default
//!   features at all.
//!
//! # Not implemented
//!
//! Windowing and subsampling (QVGA/QQVGA), `embedded-hal-async`, and pixel
//! clock control. All three are real gaps, and all three are additive.

use embedded_hal::digital::OutputPin;

mod driver;
pub mod registers;

pub use driver::*;

#[cfg(test)]
mod tests;

// ---------------------------------------------------------------------------
// Geometry and identity
// ---------------------------------------------------------------------------

/// 7-bit I2C address of the HM01B0 as wired on the Coral Dev Board Micro.
pub const I2C_ADDRESS: u8 = 0x24;

/// Expected value of the 16-bit model ID (`MODEL_ID_H:MODEL_ID_L`).
pub const MODEL_ID: u16 = 0x01B0;

/// Native image width in pixels.
pub const WIDTH: u16 = 324;

/// Native image height in pixels.
pub const HEIGHT: u16 = 324;

/// Bytes per pixel on the parallel output bus.
///
/// One 8-bit sample per photosite, and each photosite sees one colour of
/// [`CFA_PATTERN`]. A frame is a Bayer mosaic, not three channels and not
/// greyscale.
pub const BYTES_PER_PIXEL: usize = 1;

/// Size of one native frame in bytes (`WIDTH * HEIGHT * BYTES_PER_PIXEL`).
pub const FRAME_SIZE: usize = WIDTH as usize * HEIGHT as usize * BYTES_PER_PIXEL;

// ---------------------------------------------------------------------------
// Colour filter array
// ---------------------------------------------------------------------------

/// The colour of one photosite in the sensor's colour filter array.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CfaColor {
    /// Red-filtered photosite.
    Red,
    /// Green-filtered photosite. Half the array is green.
    Green,
    /// Blue-filtered photosite.
    Blue,
}

/// The sensor's Bayer colour filter array, as the 2×2 tile it repeats.
///
/// Index it `CFA_PATTERN[(y % 2) as usize][(x % 2) as usize]`, or use
/// [`cfa_color_at`]. The layout is BGGR in the raw frame's own
/// coordinates, origin top-left, no rotation:
///
/// ```text
///      x=0  x=1
/// y=0   B    G
/// y=1   G    R
/// ```
///
/// A raw frame is therefore a mosaic and must be demosaiced to get colour (or,
/// if you only want luminance, averaged). The raw bytes on their own are not
/// luminance: the blue phase is roughly 0.6× the red phase under warm indoor
/// light.
///
/// # Provenance
///
/// Confirmed against coralmicro's `BayerInternal()` in `libs/camera/camera.cc`:
/// its nearest-neighbour walk starts at `y = 2, x = 2` with `blue = true`, and
/// there takes the blue sample from `(x, y)`, the red sample from
/// `(x + 1, y + 1)` and the green samples from `(x + 1, y)` and `(x + 2,
/// y + 1)`. `blue` flips every row, which is the same statement as "the phase
/// is `(x % 2, y % 2)`".
pub const CFA_PATTERN: [[CfaColor; 2]; 2] = [
    [CfaColor::Blue, CfaColor::Green],
    [CfaColor::Green, CfaColor::Red],
];

/// The colour of the photosite at `(x, y)` in a raw frame.
///
/// See [`CFA_PATTERN`]. Coordinates are in the raw frame, origin top-left; if
/// your capture path rotates or crops the image, apply this before that.
///
/// ```
/// use hm01b0::{cfa_color_at, CfaColor};
///
/// assert_eq!(cfa_color_at(0, 0), CfaColor::Blue);
/// assert_eq!(cfa_color_at(1, 1), CfaColor::Red);
/// assert_eq!(cfa_color_at(1, 0), CfaColor::Green);
/// assert_eq!(cfa_color_at(0, 1), CfaColor::Green);
/// ```
pub const fn cfa_color_at(x: u16, y: u16) -> CfaColor {
    CFA_PATTERN[(y & 1) as usize][(x & 1) as usize]
}

// ---------------------------------------------------------------------------
// Frame timing
// ---------------------------------------------------------------------------

/// Time added to the frame period by one line of integration, in nanoseconds
/// (31.07 µs).
///
/// Measured on the Coral Dev Board Micro at native resolution with the
/// register table in [`registers::DEFAULT_REGISTERS`], by sweeping
/// [`Hm01b0::set_integration_lines`] and timing frame arrivals: above
/// [`FRAME_PERIOD_KNEE_LINES`] the frame period is linear in the integration
/// count with this slope. It is the sensor's line period, so it is a function
/// of the pixel clock and the line length rather than of the board, but the
/// pixel clock is not configurable through this driver, so treat it as fixed.
///
/// Expressed in nanoseconds only so it stays an exact integer;
/// [`frame_period_us`] does the arithmetic for you.
pub const LINE_PERIOD_NS: u32 = 31_070;

/// The shortest frame period the sensor will produce, in microseconds
/// (17.503 ms, 57.1 Hz).
///
/// Frame readout, not exposure: below [`FRAME_PERIOD_KNEE_LINES`] the frame
/// period does not depend on integration time at all, so shortening the
/// exposure past that point buys nothing in frame rate. Measured at the DMA
/// level on the Coral Dev Board Micro.
///
/// This is the rate the sensor emits frames at, which is not necessarily the
/// rate an application sees them: host-side buffering and copying add to it.
/// That part belongs to a board support crate, not here.
pub const MIN_FRAME_PERIOD_US: u32 = 17_503;

/// The integration time, in lines, at which the frame period starts to grow
/// (~564 lines, ~17.5 ms).
///
/// Below this the sensor is readout-limited and integration is free; above it
/// every extra line costs [`LINE_PERIOD_NS`] of frame period. Derived from
/// [`MIN_FRAME_PERIOD_US`] and [`LINE_PERIOD_NS`], and consistent with the
/// knee observed on hardware at ~560 lines.
pub const FRAME_PERIOD_KNEE_LINES: u16 =
    ((MIN_FRAME_PERIOD_US as u64 * 1_000).div_ceil(LINE_PERIOD_NS as u64)) as u16;

/// The auto-exposure integration ceiling in [`registers::DEFAULT_REGISTERS`]
/// (`MAX_INTG` = `0x0214` = 532 lines).
///
/// This sits just below [`FRAME_PERIOD_KNEE_LINES`], so with the stock
/// defaults auto-exposure can never lengthen the frame period, and therefore
/// gives up on exposure rather than on frame rate in dim light. See
/// [`Hm01b0::set_max_integration_lines`].
pub const VENDOR_MAX_INTEGRATION_LINES: u16 = 0x0214;

/// The auto-exposure integration floor in [`registers::DEFAULT_REGISTERS`]
/// (`MIN_INTG` = 2 lines).
pub const VENDOR_MIN_INTEGRATION_LINES: u16 = 2;

// The vendor ceiling sitting below the knee is the finding, not a coincidence.
// If either measured constant is ever revised this stops compiling, rather than
// quietly invalidating everything written about it above.
const _: () = assert!(VENDOR_MAX_INTEGRATION_LINES < FRAME_PERIOD_KNEE_LINES);
const _: () = assert!(VENDOR_MIN_INTEGRATION_LINES < VENDOR_MAX_INTEGRATION_LINES);

/// The frame period, in microseconds, for a given integration time in lines.
///
/// `max(MIN_FRAME_PERIOD_US, integration_lines * LINE_PERIOD_NS)`: flat while
/// the sensor is readout-limited, linear once integration dominates. This is
/// the whole exposure/frame-rate trade, and it is why
/// [`VENDOR_MAX_INTEGRATION_LINES`] is where it is.
///
/// ```
/// use hm01b0::{frame_period_us, MIN_FRAME_PERIOD_US, VENDOR_MAX_INTEGRATION_LINES};
///
/// // The vendor ceiling costs exactly nothing in frame rate...
/// assert_eq!(frame_period_us(VENDOR_MAX_INTEGRATION_LINES), MIN_FRAME_PERIOD_US);
/// // ...and 1800 lines costs ~56 ms, i.e. ~17.9 Hz worst case.
/// assert_eq!(frame_period_us(1800), 55_926);
/// ```
pub const fn frame_period_us(integration_lines: u16) -> u32 {
    let integration_us = (integration_lines as u32 * LINE_PERIOD_NS + 500) / 1_000;
    if integration_us > MIN_FRAME_PERIOD_US {
        integration_us
    } else {
        MIN_FRAME_PERIOD_US
    }
}

/// The longest integration time, in lines, that still fits in `period_us`.
///
/// The inverse of [`frame_period_us`]: answer to "I need at least N Hz, what
/// may I let auto-exposure spend?". Saturates at `u16::MAX`; for a period at
/// or below [`MIN_FRAME_PERIOD_US`] the answer is the largest integration that
/// is still free, `FRAME_PERIOD_KNEE_LINES - 1`.
///
/// ```
/// use hm01b0::max_integration_lines_for_period_us;
///
/// // "I want to stay above 20 Hz" -> 50 ms -> ~1609 lines.
/// assert_eq!(max_integration_lines_for_period_us(50_000), 1609);
/// ```
pub const fn max_integration_lines_for_period_us(period_us: u32) -> u16 {
    if period_us <= MIN_FRAME_PERIOD_US {
        return FRAME_PERIOD_KNEE_LINES - 1;
    }
    // `frame_period_us` rounds to the nearest microsecond, so the largest
    // integration that fits a budget of `p` is the largest `l` satisfying
    // `(l * LINE_PERIOD_NS + 500) / 1000 <= p`, which rearranges to
    // `l <= (p * 1000 + 499) / LINE_PERIOD_NS`. Dividing `p * 1000` alone
    // ignores that rounding and gives an answer one line short of the maximum
    // for about 1.5% of budgets.
    let lines = (period_us as u64 * 1_000 + 499) / LINE_PERIOD_NS as u64;
    if lines > u16::MAX as u64 {
        u16::MAX
    } else {
        lines as u16
    }
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

/// Number of model-ID read attempts made by [`Hm01b0::reset`].
///
/// The sensor may not answer immediately after its rails come up, so the read
/// is retried. This bound is what keeps the call from hanging.
pub const ID_ATTEMPTS: u8 = 10;

/// Settle time applied after each software reset, in milliseconds.
pub const RESET_SETTLE_MS: u32 = 10;

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

/// Errors returned by this driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error<E> {
    /// The underlying I2C bus returned an error.
    I2c(E),
    /// The part did not identify itself as an HM01B0.
    ///
    /// The sensor answered on the bus but reported a model ID other than
    /// [`MODEL_ID`]. A value of `0xFFFF` usually means the camera rails are
    /// off rather than that the wrong part is fitted.
    ModelId {
        /// The 16-bit model ID actually read back.
        found: u16,
    },
    /// A motion-detection region of interest fell outside the sensor array or
    /// had its start beyond its end.
    InvalidRoi,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Self {
        Error::I2c(err)
    }
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::I2c(e) => write!(f, "I2C error: {e:?}"),
            Error::ModelId { found } => write!(
                f,
                "not an HM01B0: model ID is {found:#06x}, expected {MODEL_ID:#06x}"
            ),
            Error::InvalidRoi => write!(f, "motion-detection ROI out of range"),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

// ---------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------

/// Sensor operating mode. The discriminants are the values written to
/// [`registers::MODE_SELECT`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Mode {
    /// Standby. The sensor stops driving the parallel bus and drops to a
    /// low-power state (~200 µW against ~4 mW streaming) while keeping its
    /// register configuration, so it can restart without re-running
    /// [`Hm01b0::configure`].
    Standby = 0,
    /// Free-running capture: the sensor emits frames continuously, one every
    /// [`frame_period_us`] of the current integration time.
    Streaming = 1,
    /// Hardware-triggered single-frame capture.
    ///
    /// One frame is emitted per assertion of the sensor's external trigger
    /// pin. That pin is a board GPIO, not an I2C register, so the driver
    /// cannot pulse it. See [`TriggerLine`].
    Trigger = 5,
}

/// Built-in test patterns. The discriminants are the values written to
/// [`registers::TEST_PATTERN_MODE`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TestPattern {
    /// No test pattern; the sensor outputs real pixel data.
    None = 0x00,
    /// Colour-bar pattern.
    ColorBar = 0x01,
    /// Walking-ones pattern, useful for checking the parallel data bus wiring.
    WalkingOnes = 0x11,
}

/// A rectangular region of interest for in-sensor motion detection.
///
/// Coordinates are inclusive pixel indices in the native
/// [`WIDTH`]×[`HEIGHT`] array.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionRoi {
    /// Left-most pixel of the detection zone.
    pub x0: u16,
    /// Top-most pixel of the detection zone.
    pub y0: u16,
    /// Right-most pixel of the detection zone.
    pub x1: u16,
    /// Bottom-most pixel of the detection zone.
    pub y1: u16,
}

impl MotionRoi {
    /// The whole sensor array.
    pub const fn full() -> Self {
        Self {
            x0: 0,
            y0: 0,
            x1: WIDTH - 1,
            y1: HEIGHT - 1,
        }
    }

    /// Whether the region lies inside the array and is not inverted.
    pub(crate) fn validate(&self) -> bool {
        self.x0 <= self.x1 && self.y0 <= self.y1 && self.x1 < WIDTH && self.y1 < HEIGHT
    }
}

impl Default for MotionRoi {
    fn default() -> Self {
        Self::full()
    }
}

/// A fixed exposure operating point, applied by [`Hm01b0::set_exposure`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Exposure {
    /// Coarse integration time in line periods.
    ///
    /// One line is [`LINE_PERIOD_NS`] (31.07 µs), and
    /// [`frame_period_us`] turns a value here into the frame period it
    /// implies: free up to [`FRAME_PERIOD_KNEE_LINES`], linear above it.
    /// [`VENDOR_MIN_INTEGRATION_LINES`] and [`VENDOR_MAX_INTEGRATION_LINES`]
    /// bound the *AE loop*, not this register: with AE off the sensor will
    /// honour much longer integrations, at the cost of frame rate and motion
    /// blur.
    pub integration_lines: u16,
    /// Raw value for [`registers::ANALOG_GAIN`].
    ///
    /// Per the datasheet this is a power-of-two gain selector in bits 6:4
    /// (`n << 4` gives 2ⁿ×, i.e. `0x00` = 1×, `0x10` = 2×, `0x30` = 8×). Only
    /// `0x00` appears anywhere in the known-good configuration, so that
    /// encoding is datasheet-derived rather than field-proven, hence a raw
    /// byte rather than an enum.
    ///
    /// Gain amplifies signal and noise alike. Prefer spending integration
    /// time, and reach for gain only when the frame period this implies is
    /// already as long as the application can tolerate.
    pub analog_gain: u8,
    /// Raw 16-bit value for [`registers::DIGITAL_GAIN_H`] /
    /// [`registers::DIGITAL_GAIN_L`].
    ///
    /// `0x0100` is unity gain. This is a multiply after the ADC, so it
    /// rescales what was already captured without collecting any extra
    /// photons.
    pub digital_gain: u16,
}

impl Default for Exposure {
    /// The vendor auto-exposure ceiling at unity gain. This is the brightest
    /// the stock configuration can get, and the point the AE loop rails at in
    /// dim light. See [`VENDOR_MAX_INTEGRATION_LINES`].
    fn default() -> Self {
        Self {
            integration_lines: VENDOR_MAX_INTEGRATION_LINES,
            analog_gain: 0x00,
            digital_gain: 0x0100,
        }
    }
}

// ---------------------------------------------------------------------------
// Trigger line
// ---------------------------------------------------------------------------

/// The sensor's external trigger input, used with [`Mode::Trigger`].
///
/// The HM01B0 has no I2C "capture now" command: in trigger mode it captures
/// while its trigger pin is asserted. That pin is a board GPIO, `GPIO8_IO27`
/// on the Coral Dev Board Micro, so this wrapper keeps the lifecycle explicit
/// without dragging a platform HAL into the driver.
///
/// The line is a *level*, not a pulse: assert it to request a frame and
/// deassert it only once that frame has been read out of the receiving
/// peripheral. That is [`TriggerLine::trigger`] and [`TriggerLine::release`].
///
/// ```no_run
/// # fn example<P: embedded_hal::digital::OutputPin>(pin: P) -> Result<(), P::Error> {
/// let mut trigger = hm01b0::TriggerLine::new(pin);
/// trigger.trigger()?;              // ask for one frame
/// // ... wait for the frame to arrive, then ...
/// trigger.release()?;              // re-arm for the next request
/// # Ok(())
/// # }
/// ```
#[derive(Debug)]
pub struct TriggerLine<P> {
    pin: P,
    armed: bool,
}

impl<P: OutputPin> TriggerLine<P> {
    /// Wraps an output pin connected to the sensor's trigger input.
    ///
    /// The pin is left as it is; call [`TriggerLine::release`] to be sure it
    /// starts deasserted.
    pub fn new(pin: P) -> Self {
        Self { pin, armed: false }
    }

    /// Asserts the trigger line, requesting one frame.
    pub fn trigger(&mut self) -> Result<(), P::Error> {
        self.pin.set_high()?;
        self.armed = true;
        Ok(())
    }

    /// Deasserts the trigger line.
    ///
    /// Call this once the requested frame has been consumed; the next
    /// [`TriggerLine::trigger`] then requests a fresh one.
    pub fn release(&mut self) -> Result<(), P::Error> {
        self.pin.set_low()?;
        self.armed = false;
        Ok(())
    }

    /// Whether a frame has been requested and not yet released.
    ///
    /// Check this before waiting on a frame, so that asking for an image that
    /// was never triggered fails instead of blocking forever.
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Returns the wrapped pin.
    pub fn free(self) -> P {
        self.pin
    }
}
