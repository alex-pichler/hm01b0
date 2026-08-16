# hm01b0

Platform-agnostic `no_std` [`embedded-hal`] 1.0 driver for the Himax HM01B0, a 324x324
Bayer-CFA image sensor. The driver handles sensor control over I2C: bring-up,
streaming and hardware-triggered modes, exposure, test patterns and motion
detection. The sensor writes pixel data to a parallel bus that this crate does
not touch, so you capture the frames yourself.

[`embedded-hal`]: https://crates.io/crates/embedded-hal

## Usage

```rust
use hm01b0::{Hm01b0, Mode};

let mut cam = Hm01b0::new(i2c);
cam.init(&mut delay)?;           // verify the model ID, reset, load defaults
cam.set_mode(Mode::Streaming)?;  // pixel data is now written to the parallel bus
```

Auto-exposure is enabled by default after `init`. To manually control exposure:

```rust
use hm01b0::Exposure;

cam.set_auto_exposure(false)?;
cam.set_exposure(&Exposure {
    integration_lines: 1200,    // also affects frame period, 37ms here
    analog_gain: 0x30,
    digital_gain: 0x0100,
})?;
```

## Output format

The sensor has a Bayer colour filter array in a BGGR pattern, so each byte is
one colour sample and reconstructing colour needs a demosaic. `CFA_PATTERN` and
`cfa_color_at` give the pattern.

## Features

- `defmt`: derive `defmt::Format` on the public types.

## Not (yet) implemented

Windowing and subsampling (QVGA, QQVGA), pixel-clock configuration and
`embedded-hal-async`.

