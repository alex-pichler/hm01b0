# Contributing

## Layout

```
src/lib.rs         crate documentation, constants, CFA, error and value types
src/driver.rs      the Hm01b0 type and its methods
src/registers.rs   register addresses and the vendor bring-up table
src/tests.rs       host tests
```

## Tests

Everything runs on the host against a mock I2C bus that records transactions,
so no hardware is needed:

```
cargo test                    # default features
cargo test --all-features     # adds defmt
```

