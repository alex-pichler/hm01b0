//! The [`Hm01b0`] driver struct and its register-level operations.
//!
//! Everything here needs an I2C bus. The standalone types and constants it
//! works with ([`Mode`], [`Exposure`], [`MotionRoi`], [`Error`], the timing
//! and geometry constants) live in the crate root and are usable without it.
//!
//! This module is re-exported from the crate root; there is no `hm01b0::driver`
//! path in the public API.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::{
    registers, Error, Exposure, Mode, MotionRoi, TestPattern, I2C_ADDRESS, ID_ATTEMPTS, MODEL_ID,
    RESET_SETTLE_MS,
};

/// Driver for the Himax HM01B0 image sensor.
///
/// The driver owns only the I2C control channel. Pixel data leaves the sensor
/// on a separate 8-bit parallel bus that some host peripheral (a CSI/DCMI
/// controller, a PIO block, ...) has to receive; that is deliberately out of
/// scope so this crate stays platform-independent. The samples arriving there
/// are a Bayer mosaic; see [`CFA_PATTERN`](crate::CFA_PATTERN).
#[derive(Debug)]
pub struct Hm01b0<I2C> {
    i2c: I2C,
    address: u8,
    mode: Mode,
    motion: Option<MotionRoi>,
}

impl<I2C, E> Hm01b0<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a driver on the default address, [`I2C_ADDRESS`].
    ///
    /// No bus traffic happens here; call [`Hm01b0::init`] next.
    pub fn new(i2c: I2C) -> Self {
        Self::with_address(i2c, I2C_ADDRESS)
    }

    /// Creates a driver on a non-default 7-bit I2C address.
    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self {
            i2c,
            address,
            mode: Mode::Standby,
            motion: None,
        }
    }

    /// Returns the wrapped I2C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Borrows the wrapped I2C bus, for sharing it with other devices.
    pub fn bus(&mut self) -> &mut I2C {
        &mut self.i2c
    }

    /// The last mode written with [`Hm01b0::set_mode`].
    ///
    /// This is driver state, not a register read; the sensor is assumed to be
    /// in [`Mode::Standby`] before [`Hm01b0::set_mode`] is first called, which
    /// is where a software reset leaves it.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    // -- raw register access -------------------------------------------------

    /// Writes one 8-bit register.
    ///
    /// The wire format is a single write of three bytes: the 16-bit register
    /// address big-endian, then the data byte.
    pub fn write_register(&mut self, reg: u16, value: u8) -> Result<(), Error<E>> {
        let bytes = [(reg >> 8) as u8, reg as u8, value];
        self.i2c.write(self.address, &bytes).map_err(Error::I2c)
    }

    /// Reads one 8-bit register.
    ///
    /// The wire format is a write of the 16-bit big-endian register address,
    /// a repeated start, then a one-byte read.
    pub fn read_register(&mut self, reg: u16) -> Result<u8, Error<E>> {
        let addr = [(reg >> 8) as u8, reg as u8];
        let mut buf = [0u8; 1];
        self.i2c
            .write_read(self.address, &addr, &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    /// Reads a register, applies `f` to it and writes the result back.
    pub fn modify_register<F>(&mut self, reg: u16, f: F) -> Result<(), Error<E>>
    where
        F: FnOnce(u8) -> u8,
    {
        let value = self.read_register(reg)?;
        self.write_register(reg, f(value))
    }

    /// Writes a 16-bit value to an adjacent high/low register pair.
    fn write_pair(&mut self, high: u16, low: u16, value: u16) -> Result<(), Error<E>> {
        self.write_register(high, (value >> 8) as u8)?;
        self.write_register(low, value as u8)
    }

    // -- identity ------------------------------------------------------------

    /// Reads the 16-bit model ID (`MODEL_ID_H` << 8 | `MODEL_ID_L`).
    ///
    /// Two single-byte reads, rather than relying on address auto-increment.
    pub fn model_id(&mut self) -> Result<u16, Error<E>> {
        let high = self.read_register(registers::MODEL_ID_H)?;
        let low = self.read_register(registers::MODEL_ID_L)?;
        Ok(u16::from(high) << 8 | u16::from(low))
    }

    /// Checks that the part identifies as an HM01B0.
    ///
    /// Returns [`Error::ModelId`] with the value actually read on mismatch.
    pub fn verify_identity(&mut self) -> Result<(), Error<E>> {
        let found = self.model_id()?;
        if found == MODEL_ID {
            Ok(())
        } else {
            Err(Error::ModelId { found })
        }
    }

    // -- bring-up ------------------------------------------------------------

    /// Verifies the identity and software-resets the sensor.
    ///
    /// Reads the model ID, writes [`registers::SW_RESET`], compares, and
    /// retries up to [`ID_ATTEMPTS`] times, because the sensor may not answer
    /// while its rails are still coming up.
    /// I2C errors are treated as retryable and only surface if the last
    /// attempt also fails, so a busy-bus NACK during power-up does not abort
    /// bring-up.
    ///
    /// The loop is bounded; this call cannot hang. Worst case it costs
    /// `ID_ATTEMPTS * RESET_SETTLE_MS` = 100 ms.
    ///
    /// Leaves the sensor in [`Mode::Standby`] with default register contents.
    pub fn reset<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        let mut last = Error::ModelId { found: 0xFFFF };
        for _ in 0..ID_ATTEMPTS {
            match self.model_id() {
                Ok(found) => {
                    // The reset is issued on every attempt, including the
                    // successful one, so the defaults are always written to a
                    // freshly reset sensor.
                    let reset = self.write_register(registers::SW_RESET, 0x00);
                    delay.delay_ms(RESET_SETTLE_MS);
                    if let Err(err) = reset {
                        last = err;
                        continue;
                    }
                    if found == MODEL_ID {
                        self.mode = Mode::Standby;
                        self.motion = None;
                        return Ok(());
                    }
                    last = Error::ModelId { found };
                }
                Err(err) => {
                    last = err;
                    delay.delay_ms(RESET_SETTLE_MS);
                }
            }
        }
        Err(last)
    }

    /// Loads the power-on register configuration.
    ///
    /// In order: select gated-clock output on [`registers::OSC_CLK_DIV`]
    /// (bit 5), write [`registers::DEFAULT_REGISTERS`], write the
    /// motion-detection registers for the current configuration, then clear
    /// [`registers::VSYNC_HSYNC_PIXEL_SHIFT_EN`]. That is 1 read and 48
    /// writes.
    ///
    /// Does not start capture; call [`Hm01b0::set_mode`] afterwards.
    pub fn configure(&mut self) -> Result<(), Error<E>> {
        // Gated clock mode: the sensor only clocks the parallel bus while
        // pixel data is valid, so the receiving peripheral does not have to
        // gate on the sync signals itself.
        self.modify_register(registers::OSC_CLK_DIV, |v| v | (1 << 5))?;
        self.apply_default_registers()?;
        // Shifting.
        self.write_register(registers::VSYNC_HSYNC_PIXEL_SHIFT_EN, 0x00)
    }

    /// [`Hm01b0::reset`] followed by [`Hm01b0::configure`].
    ///
    /// After this the sensor is configured and idle in [`Mode::Standby`], with
    /// auto-exposure enabled and capped at
    /// [`VENDOR_MAX_INTEGRATION_LINES`](crate::VENDOR_MAX_INTEGRATION_LINES)
    /// (that is what the vendor register table says). If the scene is not
    /// bright, raise that cap with [`Hm01b0::set_max_integration_lines`], or
    /// take exposure over entirely with [`Hm01b0::set_exposure`].
    pub fn init<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.reset(delay)?;
        self.configure()
    }

    /// Writes [`registers::DEFAULT_REGISTERS`] followed by the
    /// motion-detection registers.
    ///
    /// [`Hm01b0::set_test_pattern`] re-runs this when a pattern is turned off,
    /// which is why it is public. Note that it also restores the stock
    /// auto-exposure ceiling, so any [`Hm01b0::set_max_integration_lines`]
    /// call has to be repeated afterwards.
    pub fn apply_default_registers(&mut self) -> Result<(), Error<E>> {
        for (reg, value) in registers::DEFAULT_REGISTERS {
            self.write_register(reg, value)?;
        }
        self.apply_motion_detection_registers()
    }

    // -- mode ----------------------------------------------------------------

    /// Writes [`registers::MODE_SELECT`].
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.write_register(registers::MODE_SELECT, mode as u8)?;
        self.mode = mode;
        Ok(())
    }

    /// Starts free-running capture. Equivalent to `set_mode(Mode::Streaming)`.
    ///
    /// The first frames after this are not usable: auto-exposure needs on the
    /// order of a hundred frames to settle, and the pipeline is several frames
    /// deep on top of that. With a fixed exposure set through
    /// [`Hm01b0::set_exposure`] there is nothing to converge and only the
    /// pipeline depth remains.
    pub fn start_streaming(&mut self) -> Result<(), Error<E>> {
        self.set_mode(Mode::Streaming)
    }

    /// Arms hardware-triggered single-frame capture. Equivalent to
    /// `set_mode(Mode::Trigger)`.
    ///
    /// Frames are produced by the external trigger pin, not over I2C; see
    /// [`TriggerLine`](crate::TriggerLine).
    pub fn start_triggered(&mut self) -> Result<(), Error<E>> {
        self.set_mode(Mode::Trigger)
    }

    /// Stops capture and puts the sensor in [`Mode::Standby`].
    ///
    /// The register configuration survives, so capture can be restarted with
    /// [`Hm01b0::start_streaming`] without re-running [`Hm01b0::configure`].
    pub fn stop(&mut self) -> Result<(), Error<E>> {
        self.set_mode(Mode::Standby)
    }

    // -- exposure ------------------------------------------------------------

    /// Enables or disables the on-sensor auto-exposure loop
    /// ([`registers::AE_CTRL`] bit 0).
    ///
    /// [`registers::DEFAULT_REGISTERS`] leaves this enabled. Disabling it is
    /// the prerequisite for a repeatable frame period, because AE varies the
    /// integration time and therefore the frame timing.
    pub fn set_auto_exposure(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.write_register(registers::AE_CTRL, u8::from(enable))
    }

    /// Disables auto-exposure and pins integration time and gains.
    ///
    /// The whole group is written between [`registers::GRP_PARAM_HOLD`] `1`
    /// and `0` so the sensor latches it on one frame boundary rather than
    /// applying the bytes piecemeal. The latch happens at a frame boundary and
    /// the frame already in flight was exposed under the old setting, so
    /// discard a few frames before measuring the result.
    ///
    /// [`frame_period_us`](crate::frame_period_us) gives the frame period the
    /// `integration_lines` value implies.
    ///
    /// The integration-time registers written here are datasheet-derived: the
    /// known-good configuration sets the AE loop's *bounds*
    /// (`MIN_INTG`/`MAX_INTG`/`MIN_AGAIN`/…) but never the operating point, so
    /// it does not corroborate them.
    pub fn set_exposure(&mut self, exposure: &Exposure) -> Result<(), Error<E>> {
        self.write_register(registers::GRP_PARAM_HOLD, 0x01)?;
        self.write_register(registers::AE_CTRL, 0x00)?;
        self.write_pair(
            registers::INTEGRATION_H,
            registers::INTEGRATION_L,
            exposure.integration_lines,
        )?;
        self.write_register(registers::ANALOG_GAIN, exposure.analog_gain)?;
        self.write_pair(
            registers::DIGITAL_GAIN_H,
            registers::DIGITAL_GAIN_L,
            exposure.digital_gain,
        )?;
        self.write_register(registers::GRP_PARAM_HOLD, 0x00)
    }

    /// Sets the coarse integration time in line periods, leaving gains and the
    /// auto-exposure enable alone.
    ///
    /// Group-held like [`Hm01b0::set_exposure`]. Has no lasting effect while
    /// auto-exposure is enabled, since the AE loop will overwrite it.
    pub fn set_integration_lines(&mut self, lines: u16) -> Result<(), Error<E>> {
        self.write_register(registers::GRP_PARAM_HOLD, 0x01)?;
        self.write_pair(registers::INTEGRATION_H, registers::INTEGRATION_L, lines)?;
        self.write_register(registers::GRP_PARAM_HOLD, 0x00)
    }

    /// Sets the auto-exposure loop's maximum integration time in lines
    /// ([`registers::MAX_INTG_H`] / [`registers::MAX_INTG_L`]).
    ///
    /// This is the single most consequential knob on the sensor, and the
    /// vendor default is a trap.
    ///
    /// The frame period is flat at
    /// [`MIN_FRAME_PERIOD_US`](crate::MIN_FRAME_PERIOD_US) (17.503 ms) until
    /// integration reaches
    /// [`FRAME_PERIOD_KNEE_LINES`](crate::FRAME_PERIOD_KNEE_LINES) (~564
    /// lines), and grows at [`LINE_PERIOD_NS`](crate::LINE_PERIOD_NS)
    /// (31.07 µs) per line after that. See
    /// [`frame_period_us`](crate::frame_period_us). The power-on table writes
    /// [`VENDOR_MAX_INTEGRATION_LINES`](crate::VENDOR_MAX_INTEGRATION_LINES)
    /// (532) here, which sits just under the knee. The vendor ceiling is
    /// therefore placed so that auto-exposure can never cost a frame, so in
    /// dim light AE rails against the ceiling and underexposes rather than
    /// slowing down, and it does so without reporting anything.
    ///
    /// Raising the ceiling is two register writes and it is the smallest
    /// honest fix for an underexposed scene, because nothing is wrong with the
    /// AE loop: it was simply forbidden from spending. On this hardware 1800
    /// lines took the loop from railed to converged; that is a worst case of
    /// `frame_period_us(1800)` = 55.9 ms (17.9 Hz) on a dark scene and costs
    /// nothing at all on a bright one, where AE settles well below the knee.
    ///
    /// The trade to weigh, both points measured on this hardware:
    ///
    /// | integration | analogue gain | frame rate | temporal noise |
    /// |---|---|---|---|
    /// | 1200 lines (~37.3 ms) | 8× | 26.7 Hz | 9.55 LSB |
    /// | 3000 lines (~93.2 ms) | 4× | 10.7 Hz | 5.83 LSB |
    ///
    /// Integration time is also motion blur, so on a moving platform the
    /// ceiling is a shutter-speed limit as much as a frame-rate one.
    /// [`max_integration_lines_for_period_us`](crate::max_integration_lines_for_period_us)
    /// converts a frame-rate budget into a value for this call.
    ///
    /// Note that [`Hm01b0::apply_default_registers`] (and therefore
    /// [`Hm01b0::init`], and `set_test_pattern(TestPattern::None)`) puts the
    /// vendor ceiling back.
    pub fn set_max_integration_lines(&mut self, lines: u16) -> Result<(), Error<E>> {
        self.write_pair(registers::MAX_INTG_H, registers::MAX_INTG_L, lines)
    }

    /// Sets the auto-exposure target mean brightness
    /// ([`registers::AE_TARGET_MEAN`]). The vendor default is `0x5F` (95).
    ///
    /// Raising the target only helps if the loop has room to reach it; if AE
    /// is already railed against
    /// [`Hm01b0::set_max_integration_lines`], raise that first.
    pub fn set_ae_target_mean(&mut self, target: u8) -> Result<(), Error<E>> {
        self.write_register(registers::AE_TARGET_MEAN, target)
    }

    // -- test patterns -------------------------------------------------------

    /// Selects a built-in test pattern, or returns to real pixel data.
    ///
    /// Enabling a pattern first disables auto-exposure, black-level
    /// calibration and defect-pixel correction and forces unity gain, because
    /// otherwise those blocks act on the synthetic image. Disabling it
    /// restores the whole default register table.
    pub fn set_test_pattern(&mut self, pattern: TestPattern) -> Result<(), Error<E>> {
        if pattern == TestPattern::None {
            self.apply_default_registers()?;
        } else {
            self.write_register(registers::AE_CTRL, 0x00)?;
            self.write_register(registers::BLC_CFG, 0x00)?;
            self.write_register(registers::DPC_CTRL, 0x00)?;
            self.write_register(registers::ANALOG_GAIN, 0x00)?;
            self.write_register(registers::DIGITAL_GAIN_H, 0x01)?;
            self.write_register(registers::DIGITAL_GAIN_L, 0x00)?;
        }
        self.write_register(registers::TEST_PATTERN_MODE, pattern as u8)
    }

    // -- motion detection ----------------------------------------------------

    /// Configures in-sensor motion detection, or disables it with `None`.
    ///
    /// When enabled the sensor raises its interrupt pin on motion inside the
    /// region; that pin is a board GPIO handled outside this driver, and the
    /// interrupt must be acknowledged with
    /// [`Hm01b0::clear_motion_interrupt`].
    ///
    /// Requires [`Mode::Streaming`] to be useful. The setting is remembered
    /// and re-applied by [`Hm01b0::apply_default_registers`].
    pub fn set_motion_detection(&mut self, roi: Option<MotionRoi>) -> Result<(), Error<E>> {
        if let Some(roi) = roi {
            if !roi.validate() {
                return Err(Error::InvalidRoi);
            }
        }
        // `apply_motion_detection_registers` reads `self.motion`, so the new
        // value has to be in place before the writes. If a write fails the old
        // value goes back: a cached setting the sensor never received would be
        // worse than the failure itself, and `set_mode` above is careful the
        // same way.
        let previous = self.motion;
        self.motion = roi;
        match self.apply_motion_detection_registers() {
            Ok(()) => Ok(()),
            Err(error) => {
                self.motion = previous;
                Err(error)
            }
        }
    }

    fn apply_motion_detection_registers(&mut self) -> Result<(), Error<E>> {
        match self.motion {
            Some(roi) => {
                self.write_register(registers::MD_CTRL, 3)?;
                self.write_register(registers::MD_THL, 1)?;
                self.write_pair(
                    registers::MD_LROI_X_START_H,
                    registers::MD_LROI_X_START_L,
                    roi.x0,
                )?;
                self.write_pair(
                    registers::MD_LROI_Y_START_H,
                    registers::MD_LROI_Y_START_L,
                    roi.y0,
                )?;
                self.write_pair(
                    registers::MD_LROI_X_END_H,
                    registers::MD_LROI_X_END_L,
                    roi.x1,
                )?;
                self.write_pair(
                    registers::MD_LROI_Y_END_H,
                    registers::MD_LROI_Y_END_L,
                    roi.y1,
                )?;
                self.write_register(registers::I2C_CLEAR, 1)
            }
            None => self.write_register(registers::MD_CTRL, 0),
        }
    }

    /// Acknowledges a motion-detection interrupt
    /// ([`registers::I2C_CLEAR`] = 1).
    ///
    /// Call this from the handler for the sensor's interrupt line, otherwise
    /// no further motion interrupts are raised.
    pub fn clear_motion_interrupt(&mut self) -> Result<(), Error<E>> {
        self.write_register(registers::I2C_CLEAR, 1)
    }
}
