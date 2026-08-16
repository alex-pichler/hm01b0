//! Host-side unit tests.
//!
//! The file is in two halves, and the boundary is marked with a banner:
//!
//! * everything above it exercises the standalone types and constants in
//!   `lib.rs` and needs no I2C bus at all;
//! * everything below it exercises [`Hm01b0`] against [`MockI2c`], a fake bus
//!   that records every transaction and answers reads from a canned register
//!   map, so the whole driver is testable without hardware.
//!
//! ```text
//! cargo test
//! ```

extern crate std;

use std::collections::BTreeMap;
use std::vec::Vec;

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{self, ErrorType, I2c, Operation};

use crate::registers as reg;
use crate::{
    Error, Exposure, Hm01b0, Mode, MotionRoi, TestPattern, ID_ATTEMPTS, MODEL_ID,
    VENDOR_MAX_INTEGRATION_LINES,
};

// ---------------------------------------------------------------------------
// Types and constants: no driver, no bus.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

#[test]
fn the_native_geometry_is_the_datasheets() {
    assert_eq!((crate::WIDTH, crate::HEIGHT), (324, 324));
    assert_eq!(crate::BYTES_PER_PIXEL, 1);
    assert_eq!(crate::FRAME_SIZE, 324 * 324);
    assert_eq!(crate::I2C_ADDRESS, 0x24);
    assert_eq!(MODEL_ID, 0x01B0);
}

// ---------------------------------------------------------------------------
// Colour filter array
// ---------------------------------------------------------------------------

#[test]
fn the_cfa_is_bggr_in_raw_frame_coordinates() {
    use crate::{cfa_color_at, CfaColor::*};
    // The 2x2 tile. See `CFA_PATTERN` for where the phase comes from.
    assert_eq!(cfa_color_at(0, 0), Blue);
    assert_eq!(cfa_color_at(1, 0), Green);
    assert_eq!(cfa_color_at(0, 1), Green);
    assert_eq!(cfa_color_at(1, 1), Red);
}

#[test]
fn the_cfa_repeats_with_period_two_over_the_whole_array() {
    use crate::{cfa_color_at, CFA_PATTERN, HEIGHT, WIDTH};
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            assert_eq!(
                cfa_color_at(x, y),
                CFA_PATTERN[(y % 2) as usize][(x % 2) as usize]
            );
        }
    }
    // The (2, 2) start point the phase was confirmed at, spelled out.
    assert_eq!(cfa_color_at(2, 2), crate::CfaColor::Blue);
    assert_eq!(cfa_color_at(3, 3), crate::CfaColor::Red);
}

#[test]
fn exactly_half_the_photosites_are_green() {
    use crate::{cfa_color_at, CfaColor, HEIGHT, WIDTH};
    let mut green = 0usize;
    let mut red = 0usize;
    let mut blue = 0usize;
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            match cfa_color_at(x, y) {
                CfaColor::Green => green += 1,
                CfaColor::Red => red += 1,
                CfaColor::Blue => blue += 1,
            }
        }
    }
    assert_eq!(green, crate::FRAME_SIZE / 2);
    assert_eq!(red, crate::FRAME_SIZE / 4);
    assert_eq!(blue, crate::FRAME_SIZE / 4);
}

// ---------------------------------------------------------------------------
// Frame timing
// ---------------------------------------------------------------------------

#[test]
fn the_frame_period_is_flat_below_the_knee_and_linear_above_it() {
    use crate::{frame_period_us, FRAME_PERIOD_KNEE_LINES, LINE_PERIOD_NS, MIN_FRAME_PERIOD_US};

    // Readout-limited: integration is free.
    assert_eq!(frame_period_us(0), MIN_FRAME_PERIOD_US);
    assert_eq!(frame_period_us(2), MIN_FRAME_PERIOD_US);
    assert_eq!(
        frame_period_us(FRAME_PERIOD_KNEE_LINES - 1),
        MIN_FRAME_PERIOD_US
    );
    // First line that costs something.
    assert!(frame_period_us(FRAME_PERIOD_KNEE_LINES) > MIN_FRAME_PERIOD_US);
    // Integration-limited: one more line costs one line period.
    let a = frame_period_us(2_000);
    let b = frame_period_us(2_100);
    assert_eq!(b - a, 100 * LINE_PERIOD_NS / 1_000);
    // The top of the register range does not overflow.
    assert_eq!(frame_period_us(u16::MAX), 2_036_172);
}

#[test]
fn the_vendor_ae_ceiling_sits_just_below_the_knee() {
    use crate::{frame_period_us, FRAME_PERIOD_KNEE_LINES, MIN_FRAME_PERIOD_US};
    // This is the whole finding: 532 is not arbitrary, it is a hair under the
    // knee, so the vendor AE loop is structurally unable to trade frame rate
    // for exposure and can only underexpose instead.
    // (`VENDOR_MAX_INTEGRATION_LINES < FRAME_PERIOD_KNEE_LINES` is asserted at
    // compile time in lib.rs.)
    assert_eq!(
        frame_period_us(VENDOR_MAX_INTEGRATION_LINES),
        MIN_FRAME_PERIOD_US
    );
    assert_eq!(VENDOR_MAX_INTEGRATION_LINES, 532);
    assert_eq!(crate::VENDOR_MIN_INTEGRATION_LINES, 2);
    // ...and the knee derived from the two measured constants lands on the
    // ~560 lines observed on hardware.
    assert_eq!(FRAME_PERIOD_KNEE_LINES, 564);
}

#[test]
fn the_measured_operating_points_reproduce() {
    use crate::frame_period_us;
    // 1200 lines measured 26.7 Hz, 3000 lines measured 10.7 Hz, and the
    // raised AE ceiling of 1800 lines is a 17.9 Hz worst case.
    for (lines, hz) in [(1_200u16, 26.7f64), (3_000, 10.7), (1_800, 17.9)] {
        let measured = 1e6 / f64::from(frame_period_us(lines));
        assert!(
            (measured - hz).abs() < 0.15,
            "{lines} lines: model says {measured:.2} Hz, hardware said {hz} Hz"
        );
    }
}

#[test]
fn the_frame_rate_budget_helper_inverts_the_frame_period() {
    use crate::{
        frame_period_us, max_integration_lines_for_period_us, FRAME_PERIOD_KNEE_LINES,
        MIN_FRAME_PERIOD_US,
    };
    // 17_523 is the first budget where the rounding in `frame_period_us`
    // matters: 564 lines land on exactly 17_523 us, so an inverse that ignores
    // the rounding answers 563 and is no longer the maximum it claims to be.
    for budget in [17_523u32, 20_000, 37_300, 55_926, 93_210, 200_000] {
        let lines = max_integration_lines_for_period_us(budget);
        assert!(frame_period_us(lines) <= budget, "{budget} µs -> {lines}");
        assert!(
            frame_period_us(lines + 1) > budget,
            "{budget} µs -> {lines}"
        );
    }

    // Both halves of the property, over every budget rather than a handful:
    // never over the budget, and never one line short of the maximum.
    for budget in (MIN_FRAME_PERIOD_US + 1)..200_000 {
        let lines = max_integration_lines_for_period_us(budget);
        assert!(
            frame_period_us(lines) <= budget,
            "over budget: {budget} µs -> {lines} lines = {} µs",
            frame_period_us(lines)
        );
        assert!(
            frame_period_us(lines + 1) > budget,
            "not maximal: {budget} µs -> {lines} lines, but {} also fits",
            lines + 1
        );
    }
    // Asking for a period the sensor cannot beat still returns free lines.
    assert_eq!(
        max_integration_lines_for_period_us(1_000),
        FRAME_PERIOD_KNEE_LINES - 1
    );
    // And the register width saturates rather than wrapping.
    assert_eq!(max_integration_lines_for_period_us(u32::MAX), u16::MAX);
}

// ---------------------------------------------------------------------------
// Standalone types
// ---------------------------------------------------------------------------

#[test]
fn the_default_exposure_matches_the_vendor_ae_ceiling() {
    let e = Exposure::default();
    assert_eq!(e.integration_lines, VENDOR_MAX_INTEGRATION_LINES);
    assert_eq!(e.integration_lines, 0x0214);
    assert_eq!(e.digital_gain, 0x0100); // unity
}

#[test]
fn the_default_roi_is_the_whole_array() {
    let roi = MotionRoi::full();
    assert_eq!(roi, MotionRoi::default());
    assert_eq!((roi.x1, roi.y1), (crate::WIDTH - 1, crate::HEIGHT - 1));
    assert!(roi.validate());
}

#[test]
fn an_inverted_or_oversized_roi_does_not_validate() {
    for bad in [
        MotionRoi {
            x0: 0,
            y0: 0,
            x1: crate::WIDTH,
            y1: 10,
        },
        MotionRoi {
            x0: 0,
            y0: 0,
            x1: 10,
            y1: crate::HEIGHT,
        },
        MotionRoi {
            x0: 20,
            y0: 0,
            x1: 10,
            y1: 10,
        },
    ] {
        assert!(!bad.validate());
    }
}

#[test]
fn errors_report_what_they_saw() {
    use std::string::ToString;
    let bad: Error<core::convert::Infallible> = Error::ModelId { found: 0xFFFF };
    assert_eq!(
        bad.to_string(),
        "not an HM01B0: model ID is 0xffff, expected 0x01b0"
    );
    let roi: Error<core::convert::Infallible> = Error::InvalidRoi;
    assert_eq!(roi.to_string(), "motion-detection ROI out of range");
}

// ---------------------------------------------------------------------------
// Trigger line
// ---------------------------------------------------------------------------

#[derive(Default)]
struct MockPin {
    levels: Vec<bool>,
}

impl embedded_hal::digital::ErrorType for MockPin {
    type Error = core::convert::Infallible;
}

impl embedded_hal::digital::OutputPin for MockPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.levels.push(false);
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.levels.push(true);
        Ok(())
    }
}

#[test]
fn the_trigger_line_is_a_level_held_until_the_frame_is_taken() {
    let mut line = crate::TriggerLine::new(MockPin::default());
    assert!(!line.is_armed());
    line.trigger().unwrap();
    assert!(line.is_armed());
    line.release().unwrap();
    assert!(!line.is_armed());
    assert_eq!(line.free().levels, [true, false]);
}

// ---------------------------------------------------------------------------
// Driver: everything below here needs `Hm01b0` and a bus.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Fakes
// ---------------------------------------------------------------------------

/// One recorded bus transaction.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Txn {
    /// A plain write of `n` bytes.
    Write(Vec<u8>),
    /// A write of the register address followed by a repeated-start read.
    WriteRead(Vec<u8>, Vec<u8>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockError;

impl i2c::Error for MockError {
    fn kind(&self) -> i2c::ErrorKind {
        i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Address)
    }
}

struct MockI2c {
    /// Every transaction, in order.
    txns: Vec<Txn>,
    /// Canned register contents for reads; unlisted registers read `0xFF`.
    regs: BTreeMap<u16, u8>,
    /// Addresses the transactions were addressed to.
    addresses: Vec<u8>,
    /// Number of leading transactions to fail with `MockError`.
    fail_first: usize,
    /// Transactions attempted so far, including failed ones.
    attempts: usize,
}

impl MockI2c {
    fn new() -> Self {
        let mut regs = BTreeMap::new();
        regs.insert(reg::MODEL_ID_H, 0x01);
        regs.insert(reg::MODEL_ID_L, 0xB0);
        // Arbitrary non-zero power-on value, so the read-modify-write of the
        // gated-clock bit is visible in the recorded transactions.
        regs.insert(reg::OSC_CLK_DIV, 0x0A);
        Self {
            txns: Vec::new(),
            regs,
            addresses: Vec::new(),
            fail_first: 0,
            attempts: 0,
        }
    }

    fn with_model_id(mut self, id: u16) -> Self {
        self.regs.insert(reg::MODEL_ID_H, (id >> 8) as u8);
        self.regs.insert(reg::MODEL_ID_L, id as u8);
        self
    }

    fn failing_first(mut self, n: usize) -> Self {
        self.fail_first = n;
        self
    }

    /// The recorded writes as `(register, value)` pairs, reads dropped.
    fn writes(&self) -> Vec<(u16, u8)> {
        self.txns
            .iter()
            .filter_map(|t| match t {
                Txn::Write(bytes) => {
                    assert_eq!(bytes.len(), 3, "register writes are 3 bytes");
                    Some(((u16::from(bytes[0]) << 8) | u16::from(bytes[1]), bytes[2]))
                }
                Txn::WriteRead(..) => None,
            })
            .collect()
    }

    fn reads(&self) -> Vec<u16> {
        self.txns
            .iter()
            .filter_map(|t| match t {
                Txn::WriteRead(addr, _) => Some((u16::from(addr[0]) << 8) | u16::from(addr[1])),
                Txn::Write(_) => None,
            })
            .collect()
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl I2c for MockI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.attempts += 1;
        if self.attempts <= self.fail_first {
            return Err(MockError);
        }
        self.addresses.push(address);
        match operations {
            [Operation::Write(bytes)] => {
                self.txns.push(Txn::Write(bytes.to_vec()));
            }
            [Operation::Write(addr), Operation::Read(buf)] => {
                assert_eq!(addr.len(), 2, "register addresses are 16-bit");
                let register = (u16::from(addr[0]) << 8) | u16::from(addr[1]);
                for (i, byte) in buf.iter_mut().enumerate() {
                    *byte = *self.regs.get(&(register + i as u16)).unwrap_or(&0xFF);
                }
                self.txns.push(Txn::WriteRead(addr.to_vec(), buf.to_vec()));
            }
            other => panic!("unexpected transaction shape: {} ops", other.len()),
        }
        Ok(())
    }
}

/// Records how long the driver asked to be delayed, so the bounded-wait
/// guarantee can be asserted.
#[derive(Default)]
struct MockDelay {
    total_ns: u64,
    calls: usize,
}

impl DelayNs for MockDelay {
    fn delay_ns(&mut self, ns: u32) {
        self.total_ns += u64::from(ns);
        self.calls += 1;
    }
}

impl MockDelay {
    fn total_ms(&self) -> u64 {
        self.total_ns / 1_000_000
    }
}

fn driver() -> Hm01b0<MockI2c> {
    Hm01b0::new(MockI2c::new())
}

// ---------------------------------------------------------------------------
// Wire format
// ---------------------------------------------------------------------------

#[test]
fn write_register_is_address_big_endian_then_data() {
    let mut cam = driver();
    cam.write_register(0x1234, 0x56).unwrap();
    assert_eq!(cam.bus().txns, [Txn::Write(std::vec![0x12, 0x34, 0x56])]);
    assert_eq!(cam.bus().addresses, [crate::I2C_ADDRESS]);
}

#[test]
fn read_register_is_write_address_then_repeated_start_read() {
    let mut cam = driver();
    let value = cam.read_register(reg::OSC_CLK_DIV).unwrap();
    assert_eq!(value, 0x0A);
    assert_eq!(
        cam.bus().txns,
        [Txn::WriteRead(std::vec![0x30, 0x60], std::vec![0x0A])]
    );
}

#[test]
fn a_custom_address_is_honoured() {
    let mut cam = Hm01b0::with_address(MockI2c::new(), 0x48);
    cam.write_register(0x0000, 0x00).unwrap();
    cam.read_register(0x0000).unwrap();
    assert_eq!(cam.bus().addresses, [0x48, 0x48]);
}

#[test]
fn modify_register_reads_then_writes_back() {
    let mut cam = driver();
    cam.modify_register(reg::OSC_CLK_DIV, |v| v | (1 << 5))
        .unwrap();
    assert_eq!(
        cam.bus().txns,
        [
            Txn::WriteRead(std::vec![0x30, 0x60], std::vec![0x0A]),
            Txn::Write(std::vec![0x30, 0x60, 0x2A]),
        ]
    );
}

#[test]
fn i2c_errors_are_wrapped() {
    let mut cam = Hm01b0::new(MockI2c::new().failing_first(1));
    assert_eq!(cam.write_register(0x0000, 0x00), Err(Error::I2c(MockError)));
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

#[test]
fn model_id_is_two_single_byte_reads_high_first() {
    let mut cam = driver();
    assert_eq!(cam.model_id().unwrap(), MODEL_ID);
    assert_eq!(cam.model_id().unwrap(), 0x01B0);
    assert_eq!(
        cam.bus().reads(),
        [
            reg::MODEL_ID_H,
            reg::MODEL_ID_L,
            reg::MODEL_ID_H,
            reg::MODEL_ID_L
        ]
    );
}

#[test]
fn verify_identity_accepts_an_hm01b0() {
    assert_eq!(driver().verify_identity(), Ok(()));
}

#[test]
fn verify_identity_reports_the_id_it_found() {
    let mut cam = Hm01b0::new(MockI2c::new().with_model_id(0x01B6));
    assert_eq!(cam.verify_identity(), Err(Error::ModelId { found: 0x01B6 }));
}

#[test]
fn an_absent_sensor_reads_as_all_ones() {
    // Camera rails off: the bus answers but every bit floats high.
    let mut cam = Hm01b0::new(MockI2c::new().with_model_id(0xFFFF));
    assert_eq!(cam.verify_identity(), Err(Error::ModelId { found: 0xFFFF }));
}

// ---------------------------------------------------------------------------
// Bring-up
// ---------------------------------------------------------------------------

/// The exact writes `init()` must produce, spelled out as literals rather than
/// referencing `registers::*`, so that an accidental edit to the register table
/// fails this test instead of silently changing what the sensor is programmed
/// with. The order is reset, gated clock, `DEFAULT_REGISTERS`, the
/// motion-detection registers, then the pixel-shift clear.
const EXPECTED_INIT_WRITES: [(u16, u8); 49] = [
    (0x0103, 0x00), // SW_RESET, from the identity/reset loop
    (0x3060, 0x2A), // OSC_CLK_DIV, read 0x0A with bit 5 set for gated clock
    // SetDefaultRegisters(): analog
    (0x1003, 0x08),
    (0x1007, 0x08),
    // reserved-but-required block
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
    (0x3059, 0x1E),
    // digital
    (0x1000, 0x43),
    (0x1001, 0x40),
    (0x1002, 0x32),
    (0x0350, 0x7F),
    (0x1006, 0x01),
    (0x1008, 0x00),
    (0x1009, 0xA0),
    (0x100A, 0x60),
    (0x100B, 0x90),
    (0x100C, 0x40),
    // auto-exposure
    (0x2000, 0x07),
    (0x2100, 0x01), // AE_CTRL: auto-exposure stays ON
    (0x2101, 0x5F),
    (0x2102, 0x0A),
    (0x2103, 0x03),
    (0x2104, 0x05),
    (0x2105, 0x02), // MAX_INTG_H \_ 532 lines: the ceiling on the frame-rate
    (0x2106, 0x14), // MAX_INTG_L /  knee, see set_max_integration_lines()
    (0x2107, 0x02),
    (0x2108, 0x03),
    (0x2109, 0x03),
    (0x210A, 0x00),
    (0x210B, 0x80),
    (0x210C, 0x40),
    (0x210D, 0x20),
    // 60 Hz flicker
    (0x210E, 0x03),
    (0x210F, 0x00),
    (0x2110, 0x85),
    (0x2111, 0x00),
    (0x2112, 0xA0),
    // SetMotionDetectionRegisters(), disabled
    (0x2150, 0x00),
    // shifting
    (0x1012, 0x00),
];

#[test]
fn init_writes_the_power_on_sequence_in_order() {
    let mut cam = driver();
    let mut delay = MockDelay::default();
    cam.init(&mut delay).unwrap();

    assert_eq!(cam.bus().writes(), EXPECTED_INIT_WRITES);
    // Two identity reads plus the OSC_CLK_DIV read-modify-write.
    assert_eq!(
        cam.bus().reads(),
        [reg::MODEL_ID_H, reg::MODEL_ID_L, reg::OSC_CLK_DIV]
    );
    // 48 of the 49 writes belong to configure(); the reset write does not.
    assert_eq!(EXPECTED_INIT_WRITES.len() - 1, 48);
}

#[test]
fn the_vendor_table_programs_the_documented_ae_ceiling() {
    // Ties the constant to the bytes actually put on the wire.
    let mut cam = driver();
    cam.init(&mut MockDelay::default()).unwrap();
    let writes = cam.bus().writes();
    let byte = |r| writes.iter().find(|(reg, _)| *reg == r).unwrap().1;
    let ceiling = u16::from(byte(reg::MAX_INTG_H)) << 8 | u16::from(byte(reg::MAX_INTG_L));
    assert_eq!(ceiling, VENDOR_MAX_INTEGRATION_LINES);
    assert_eq!(
        u16::from(byte(reg::MIN_INTG)),
        crate::VENDOR_MIN_INTEGRATION_LINES
    );
}

#[test]
fn init_leaves_the_sensor_in_standby() {
    let mut cam = driver();
    cam.init(&mut MockDelay::default()).unwrap();
    assert_eq!(cam.mode(), Mode::Standby);
    // Nothing was written to MODE_SELECT.
    assert!(!cam
        .bus()
        .writes()
        .iter()
        .any(|(r, _)| *r == reg::MODE_SELECT));
}

#[test]
fn init_settles_once_after_the_reset_on_the_happy_path() {
    let mut cam = driver();
    let mut delay = MockDelay::default();
    cam.init(&mut delay).unwrap();
    assert_eq!(delay.calls, 1);
    assert_eq!(delay.total_ms(), u64::from(crate::RESET_SETTLE_MS));
}

#[test]
fn init_gives_up_after_a_bounded_number_of_attempts() {
    let mut cam = Hm01b0::new(MockI2c::new().with_model_id(0x00FF));
    let mut delay = MockDelay::default();

    assert_eq!(cam.init(&mut delay), Err(Error::ModelId { found: 0x00FF }));
    // Exactly ID_ATTEMPTS rounds of (2 reads + 1 reset write), and no more:
    // the retry loop is bounded, a wrong part cannot hang bring-up.
    assert_eq!(cam.bus().reads().len(), 2 * usize::from(ID_ATTEMPTS));
    assert_eq!(cam.bus().writes().len(), usize::from(ID_ATTEMPTS));
    assert!(cam
        .bus()
        .writes()
        .iter()
        .all(|(r, v)| *r == reg::SW_RESET && *v == 0x00));
    assert_eq!(delay.calls, usize::from(ID_ATTEMPTS));
    assert_eq!(
        delay.total_ms(),
        u64::from(ID_ATTEMPTS) * u64::from(crate::RESET_SETTLE_MS)
    );
}

#[test]
fn init_rides_out_a_bus_error_while_the_rails_come_up() {
    // The first four transactions NACK, as they do when the sensor is still
    // powering up; the driver must retry rather than give up.
    let mut cam = Hm01b0::new(MockI2c::new().failing_first(4));
    let mut delay = MockDelay::default();
    assert_eq!(cam.init(&mut delay), Ok(()));
    assert_eq!(cam.bus().writes(), EXPECTED_INIT_WRITES);
}

#[test]
fn a_permanently_dead_bus_surfaces_the_i2c_error() {
    let mut cam = Hm01b0::new(MockI2c::new().failing_first(usize::MAX));
    assert_eq!(
        cam.init(&mut MockDelay::default()),
        Err(Error::I2c(MockError))
    );
}

#[test]
fn configure_alone_skips_the_identity_check() {
    let mut cam = driver();
    cam.configure().unwrap();
    assert_eq!(cam.bus().writes().len(), 48);
    assert_eq!(cam.bus().reads(), [reg::OSC_CLK_DIV]);
}

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

#[test]
fn mode_transitions_write_mode_select_with_the_documented_values() {
    let mut cam = driver();
    cam.start_streaming().unwrap();
    assert_eq!(cam.mode(), Mode::Streaming);
    cam.start_triggered().unwrap();
    assert_eq!(cam.mode(), Mode::Trigger);
    cam.stop().unwrap();
    assert_eq!(cam.mode(), Mode::Standby);
    cam.set_mode(Mode::Streaming).unwrap();
    assert_eq!(cam.mode(), Mode::Streaming);

    assert_eq!(
        cam.bus().writes(),
        [
            (0x0100, 1), // kStreaming
            (0x0100, 5), // kTrigger
            (0x0100, 0), // HandleDisableRequest()
            (0x0100, 1),
        ]
    );
}

#[test]
fn a_failed_mode_write_does_not_update_the_cached_mode() {
    let mut cam = Hm01b0::new(MockI2c::new().failing_first(1));
    assert!(cam.set_mode(Mode::Streaming).is_err());
    assert_eq!(cam.mode(), Mode::Standby);
}

// ---------------------------------------------------------------------------
// Exposure
// ---------------------------------------------------------------------------

#[test]
fn set_auto_exposure_toggles_bit_zero_of_ae_ctrl() {
    let mut cam = driver();
    cam.set_auto_exposure(false).unwrap();
    cam.set_auto_exposure(true).unwrap();
    assert_eq!(cam.bus().writes(), [(0x2100, 0x00), (0x2100, 0x01)]);
}

#[test]
fn set_exposure_disables_ae_inside_a_group_hold() {
    let mut cam = driver();
    cam.set_exposure(&Exposure {
        integration_lines: 0x0123,
        analog_gain: 0x30,
        digital_gain: 0x0180,
    })
    .unwrap();

    assert_eq!(
        cam.bus().writes(),
        [
            (0x0104, 0x01), // GRP_PARAM_HOLD on
            (0x2100, 0x00), // AE off
            (0x0202, 0x01), // INTEGRATION_H
            (0x0203, 0x23), // INTEGRATION_L
            (0x0205, 0x30), // ANALOG_GAIN
            (0x020E, 0x01), // DIGITAL_GAIN_H
            (0x020F, 0x80), // DIGITAL_GAIN_L
            (0x0104, 0x00), // GRP_PARAM_HOLD off
        ]
    );
}

#[test]
fn integration_and_ae_bounds_split_into_high_low_pairs() {
    let mut cam = driver();
    cam.set_integration_lines(0x02AB).unwrap();
    cam.set_max_integration_lines(0x0214).unwrap();
    cam.set_ae_target_mean(0x40).unwrap();
    assert_eq!(
        cam.bus().writes(),
        [
            (0x0104, 0x01),
            (0x0202, 0x02),
            (0x0203, 0xAB),
            (0x0104, 0x00),
            (0x2105, 0x02),
            (0x2106, 0x14),
            (0x2101, 0x40),
        ]
    );
}

#[test]
fn raising_the_ae_ceiling_past_the_knee_is_two_writes() {
    // The fix this crate exists to make possible: 1800 lines instead of 532.
    let mut cam = driver();
    cam.set_max_integration_lines(1_800).unwrap();
    assert_eq!(cam.bus().writes(), [(0x2105, 0x07), (0x2106, 0x08)]);
    assert_eq!(0x0708, 1_800);
}

// ---------------------------------------------------------------------------
// Test patterns
// ---------------------------------------------------------------------------

#[test]
fn enabling_a_test_pattern_disables_the_blocks_that_would_corrupt_it() {
    let mut cam = driver();
    cam.set_test_pattern(TestPattern::WalkingOnes).unwrap();
    assert_eq!(
        cam.bus().writes(),
        [
            (0x2100, 0x00), // AE_CTRL
            (0x1000, 0x00), // BLC_CFG
            (0x1008, 0x00), // DPC_CTRL
            (0x0205, 0x00), // ANALOG_GAIN
            (0x020E, 0x01), // DIGITAL_GAIN_H
            (0x020F, 0x00), // DIGITAL_GAIN_L
            (0x0601, 0x11), // TEST_PATTERN_MODE = kWalkingOnes
        ]
    );
}

#[test]
fn the_colour_bar_pattern_uses_the_documented_value() {
    let mut cam = driver();
    cam.set_test_pattern(TestPattern::ColorBar).unwrap();
    assert_eq!(cam.bus().writes().last(), Some(&(0x0601, 0x01)));
}

#[test]
fn disabling_the_test_pattern_restores_the_whole_default_table() {
    let mut cam = driver();
    cam.set_test_pattern(TestPattern::None).unwrap();
    let writes = cam.bus().writes();
    // 45 defaults + MD_CTRL + TEST_PATTERN_MODE.
    assert_eq!(writes.len(), reg::DEFAULT_REGISTERS.len() + 2);
    assert_eq!(writes[0], (0x1003, 0x08));
    assert_eq!(writes.last(), Some(&(0x0601, 0x00)));
}

// ---------------------------------------------------------------------------
// Motion detection
// ---------------------------------------------------------------------------

#[test]
fn motion_detection_splits_the_roi_across_high_low_registers() {
    let mut cam = driver();
    cam.set_motion_detection(Some(MotionRoi {
        x0: 0x0004,
        y0: 0x0102,
        x1: 0x0141,
        y1: 0x0143,
    }))
    .unwrap();
    assert_eq!(
        cam.bus().writes(),
        [
            (0x2150, 0x03), // MD_CTRL
            (0x215B, 0x01), // MD_THL
            (0x2011, 0x00), // X start high
            (0x2012, 0x04), // X start low
            (0x2013, 0x01), // Y start high
            (0x2014, 0x02), // Y start low
            (0x2015, 0x01), // X end high
            (0x2016, 0x41), // X end low
            (0x2017, 0x01), // Y end high
            (0x2018, 0x43), // Y end low
            (0x2153, 0x01), // I2C_CLEAR
        ]
    );
}

#[test]
fn an_out_of_range_roi_is_rejected_without_touching_the_bus() {
    let mut cam = driver();
    for bad in [
        MotionRoi {
            x0: 0,
            y0: 0,
            x1: crate::WIDTH,
            y1: 10,
        },
        MotionRoi {
            x0: 0,
            y0: 0,
            x1: 10,
            y1: crate::HEIGHT,
        },
        MotionRoi {
            x0: 20,
            y0: 0,
            x1: 10,
            y1: 10,
        },
    ] {
        assert_eq!(cam.set_motion_detection(Some(bad)), Err(Error::InvalidRoi));
    }
    assert!(cam.bus().txns.is_empty());
}

#[test]
fn a_configured_roi_is_re_applied_by_the_default_table() {
    let mut cam = driver();
    cam.set_motion_detection(Some(MotionRoi::full())).unwrap();
    cam.bus().txns.clear();

    cam.apply_default_registers().unwrap();
    let writes = cam.bus().writes();
    assert_eq!(writes.len(), reg::DEFAULT_REGISTERS.len() + 11);
    assert_eq!(writes[reg::DEFAULT_REGISTERS.len()], (0x2150, 0x03));
    assert_eq!(writes.last(), Some(&(0x2153, 0x01)));
}

#[test]
fn a_failed_motion_write_leaves_the_cached_roi_alone() {
    // The driver remembers the region so `apply_default_registers` can put it
    // back. If a write fails part way through, the remembered value has to stay
    // what the sensor actually has, or the cache and the hardware disagree with
    // nothing to say so.
    let mut cam = Hm01b0::new(MockI2c::new().failing_first(1));
    assert!(cam
        .set_motion_detection(Some(MotionRoi {
            x0: 0x0004,
            y0: 0x0102,
            x1: 0x0141,
            y1: 0x0143,
        }))
        .is_err());

    // Motion was never configured, so re-applying must write the "off" value
    // and no region at all. A cached region here would mean the driver
    // believes in a configuration the sensor rejected.
    cam.apply_default_registers().unwrap();
    let region: Vec<(u16, u8)> = cam
        .bus()
        .writes()
        .into_iter()
        .filter(|(reg, _)| (0x2011..=0x2018).contains(reg))
        .collect();
    assert!(
        region.is_empty(),
        "failed set_motion_detection cached a region anyway: {region:?}"
    );
    assert_eq!(
        cam.bus().writes().last(),
        Some(&(0x2150, 0x00)),
        "motion should still be off"
    );
}

#[test]
fn motion_detection_can_be_switched_back_off() {
    let mut cam = driver();
    cam.set_motion_detection(None).unwrap();
    assert_eq!(cam.bus().writes(), [(0x2150, 0x00)]);
}

#[test]
fn clearing_the_interrupt_writes_i2c_clear() {
    let mut cam = driver();
    cam.clear_motion_interrupt().unwrap();
    assert_eq!(cam.bus().writes(), [(0x2153, 0x01)]);
}
