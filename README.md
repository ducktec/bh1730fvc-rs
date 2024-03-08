# BH1730FVC Rust Driver

A Rust no-std driver for the BH1730FVC ambient light sensor.

[![Crates.io](https://img.shields.io/crates/v/bh1730fvc-rs.svg)](https://crates.io/crates/bh1730fvc-rs)
[![Docs.rs](https://docs.rs/bh1730fvc-rs/badge.svg)](https://docs.rs/bh1730fvc-rs)

## Description

This library provides a Rust interface to the BH1730FVC ambient light sensor. It is designed to be used in no-std environments, making it suitable for use in embedded systems.

## Supported features

* Single-shot and continuous measurement mode
* Configurable integration time and gain
* Reading the part number and revision id of the sensor
* Converting the read raw values into the ambient light intensity in lux

## Unsupported features

* Interrupt functionality
* Configuring the thresholds
* Async support

## Dependencies

This library depends on the `embedded-hal` crate for hardware abstraction. It uses the `log` crate for logging.

For development, it uses the `embedded-hal-mock` crate for mocking the hardware abstraction layer. For the example, it uses `linux-embedded-hal`, to be tentatively run on an RPI (not tested yet!).

## Usage

To use this library, add the following to your `Cargo.toml` file:

```toml
[dependencies]
bh1730fvc = "0.1.0"
```

Then, you can use it in your Rust code like this:

```rust
// Create the necessary hardware objects
let mut i2c = hal::I2cdev::new("/dev/i2c-1").unwrap();
let mut delay = hal::Delay;

// Create a new BH1730FVC instance
let mut bh1730fvc = BH1730FVC::new(&mut delay, &mut i2c).unwrap();

// Read the device ID
let device_id = bh1730fvc.read_id(&mut i2c).unwrap();
log::info!(
    "Device ID: (Part Number: 0x{:02X}, Revision ID: 0x{:02X}",
    device_id.0,
    device_id.1,
);

// Do single shot measurement once
match bh1730fvc.get_ambient_light_intensity_single_shot(&mut delay, &mut i2c) {
    Ok(reading) => log::info!("Single shot measurement - lux value: {}", reading),
    Err(e) => log::error!("Error reading sensor: {:?}", e),
}
```

## Examples
There is an example available in the examples directory. To build it (cross-compiling for the RPI), run the following command:

```bash
cargo build --example rpi_demo --target=arm-unknown-linux-gnueabihf
```

## License
This library is licensed under either of
* MIT license (LICENSE-MIT or http://opensource.org/licenses/MIT)
* Apache License, Version 2.0 (LICENSE-APACHE or http://www.apache.org/licenses/LICENSE-2.0)
at your option.

## Contribution
Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.

