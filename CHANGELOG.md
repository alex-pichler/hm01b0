# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-08-16

Initial release: a platform-agnostic `no_std` `embedded-hal` 1.0 driver for the
Himax HM01B0, covering sensor control over I2C. Its power-on configuration is
the one coralmicro uses on the Coral Dev Board Micro.

### Added

- `Hm01b0<I2C>`: bring-up, streaming and hardware-triggered modes, exposure,
  test patterns and motion detection.
- Manual exposure control, and the `AE_CTRL` enable it needs.
- The frame-period model measured on hardware, and the finding that the stock
  auto-exposure ceiling sits just below the frame-rate knee, so auto-exposure
  underexposes in dim light rather than slowing down.
- The Bayer colour filter array as public API: `CFA_PATTERN` and
  `cfa_color_at`, BGGR in raw-frame coordinates.
- `TriggerLine<P>` for the sensor's external trigger, which is a board GPIO
  rather than a register.
- The `registers` module: every address the power-on configuration uses, with
  provenance, plus the three datasheet-only registers manual exposure needs.
- `defmt`: derive `defmt::Format` on the public types. Off by default.
- Host tests against a mock I2C bus that records transactions.

### Notes

- MSRV is 1.81, set by `core::error::Error`.
- Not (yet) implemented: windowing and subsampling (QVGA, QQVGA), pixel-clock
  configuration and `embedded-hal-async`.

[Unreleased]: https://github.com/alex-pichler/hm01b0/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/alex-pichler/hm01b0/releases/tag/v0.1.0
