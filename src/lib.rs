//! This crate provides a platform agnostic no_std driver for the BH1730FVC ambient light sensor.
//! The driver is compatible with the [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.
//!
//! The datasheet of the sensor can be found [here](https://fscdn.rohm.com/en/products/databook/datasheet/ic/sensor/light/bh1730fvc-e.pdf).
//!
//! ## Supported features
//! * Single-shot and continuous measurement mode
//! * Configurable integration time and gain
//! * Reading the part number and revision id of the sensor
//! * Converting the read raw values into the ambient light intensity in lux
//!
//! ## Unsupported features
//! * Interrupt functionality
//! * Threshold functionality
//!
//! ## Usage
//!
//! ### Creating a driver instance (blocking mode)
//!
//! ```rust
//! use bh1730fvc::{BH1730FVC, Gain, Mode};
//! use embedded_hal::blocking::i2c::{Write, WriteRead};
//! use embedded_hal::blocking::delay::DelayMs;
//!
//!
//! fn main() {
//!     let mut delay = MockNoop::new();
//!     let mut i2c = MockI2c::new();
//!     let mut sensor = BH1730FVC::new(&mut delay, &mut i2c).unwrap();
//! }
//! ```
//!
//! ### Reading the ambient light intensity (blocking mode)
//!
//! ```rust
//! use bh1730fvc::{BH1730FVC, Gain, Mode};
//! use embedded_hal::blocking::i2c::{Write, WriteRead};
//! use embedded_hal::blocking::delay::DelayMs;
//!
//! fn main() {
//!     let mut delay = MockNoop::new();
//!     let mut i2c = MockI2c::new();
//!     let mut sensor = BH1730FVC::new(&mut delay, &mut i2c).unwrap();
//!
//!     // Read sensor value once, sensor goes back to sleep after this
//!     // (This blocks the thread for the duration of the measurement)
//!     let lux = sensor.get_ambient_light_intensity_single_shot(&mut delay, &mut i2c).unwrap();
//!
//!     println!("Ambient light intensity: {} lux", lux);
//! }
//! ```
//!
//! ### Continuous measurement (blocking mode)
//!
//! ```rust
//! use bh1730fvc::{BH1730FVC, Gain, Mode};
//! use embedded_hal::blocking::i2c::{Write, WriteRead};
//! use embedded_hal::blocking::delay::DelayMs;
//!
//! fn main() {
//!     let mut delay = MockNoop::new();
//!     let mut i2c = MockI2c::new();
//!     let mut sensor = BH1730FVC::new(&mut delay, &mut i2c).unwrap();
//!
//!     // Start continuous measurement mode, sensor will keep measuring and overwriting the
//!     // last measured value until stopped
//!     sensor.start_continuous_measurement(&mut i2c).unwrap();
//!
//!     // Read the last measured value (this does not block, but will return error
//!     // BH1730FVCError::NoDataAvailable if no valid data is available (yet))
//!     let lux = sensor.get_last_ambient_light_intensity(&mut i2c).unwrap();
//!
//!     // Stop continuous measurement mode
//!     sensor.stop_continuous_measurement(&mut i2c).unwrap();
//!
//!     // Print the last measured value
//!     println!("Ambient light intensity: {} lux", lux);
//! }
//! ```

#![cfg_attr(not(test), no_std)]

#[cfg(feature = "async")]
mod r#async;

#[cfg(not(feature = "async"))]
mod blocking;

/// I2C address for the BH1730FVC sensor.
pub const BH1730FVC_ADDR: u8 = 0x29;

/// BH1730FVC register address for reading the ID information.
const BH1730FVC_RESET_CMD: u8 = 0xE4;

/// Internal clock interval
const TINT_US: f32 = 2.8; // 2.8 us (typical value)

/// Represents an I2C-connected BH1730FVC sensor.
#[derive(Copy, Clone, Debug)]
pub struct BH1730FVC<I2C, D> {
    /// Marker to satisfy the compiler.
    _delay: core::marker::PhantomData<D>,

    /// I2C Interface for communcating with the sensor.
    _i2c: core::marker::PhantomData<I2C>,

    /// The gain of the sensor.
    gain: Gain,

    /// The integration time of the sensor.
    integration_time_ms: f32,
}

/// Calculates the light intensity in lux from the raw sensor data
fn calculate_lux(data0: u16, data1: u16, gain: f32, integration_time_ms: f32) -> f32 {
    let data0 = data0 as f32;
    let data1 = data1 as f32;
    let ratio = data1 / data0;

    let lux = if ratio < 0.26 {
        (1.290 * data0 - 2.733 * data1) / (gain * 102.6 / integration_time_ms)
    } else if ratio < 0.55 {
        (0.795 * data0 - 0.859 * data1) / (gain * 102.6 / integration_time_ms)
    } else if ratio < 1.09 {
        (0.510 * data0 - 0.345 * data1) / (gain * 102.6 / integration_time_ms)
    } else if ratio < 2.13 {
        (0.276 * data0 - 0.130 * data1) / (gain * 102.6 / integration_time_ms)
    } else {
        0.0
    };

    lux
}

/// Converts the integration time in milliseconds to the value to be written to the timing register
fn itime_ms_to_itime(itime_ms: f32) -> u8 {
    let itime = 256.0 - (itime_ms * 1000.0 / (TINT_US * 964.0));
    itime as u8
}

/// Converts the value in the timing register to the integration time in milliseconds
fn itime_to_itime_ms(itime: u8) -> f32 {
    let itime_ms = (TINT_US * 964.0 * (256.0 - itime as f32)) / 1000.0;
    itime_ms
}

/// Shorthand for all functions returning an error in this module.
type Result<T> = core::result::Result<T, BH1730FVCError>;

/// Represents any error that may happen during communication.
#[derive(Copy, Clone, Debug, Ord, PartialOrd, Eq, PartialEq)]
pub enum BH1730FVCError {
    /// An error occurred while reading from the sensor.
    ReadI2CError,
    /// An error occurred while writing to the sensor.
    WriteI2CError,
    /// No valid data is available for the raw light intensity values based on the last measurement.
    NoDataAvailable,
}

// All data registers of the BH1730FVC sensor.
pub enum DataRegister {
    /// Mode control register
    Control = 0x00,

    /// Timing control register
    Timing = 0x01,

    /// Interrupt control register
    Interrupt = 0x02,

    /// Interrupt threshold register (Low byte)
    InterruptLowThresholdLow = 0x03,

    /// Interrupt threshold register (High byte)
    InterruptLowThresholdHigh = 0x04,

    /// Interrupt threshold register (Low byte)
    InterruptHighThresholdLow = 0x05,

    /// Interrupt threshold register (High byte)
    InterruptHighThresholdHigh = 0x06,

    /// Gain control register
    Gain = 0x07,

    /// ID register
    ID = 0x12,

    /// Data0 value (Low byte)
    Data0Low = 0x14,

    /// Data0 value (High byte)
    Data0High = 0x15,

    /// Data1 value (Low byte)
    Data1Low = 0x16,

    /// Data1 value (High byte)
    Data1High = 0x17,
}

/// The gain of the BH1730FVC sensor.
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u8)]
pub enum Gain {
    X1 = 0x0,
    X2 = 0x1,
    X64 = 0x2,
    X128 = 0x3,
}

impl From<Gain> for f32 {
    fn from(gain: Gain) -> Self {
        match gain {
            Gain::X1 => 1.0,
            Gain::X2 => 2.0,
            Gain::X64 => 64.0,
            Gain::X128 => 128.0,
        }
    }
}

impl Gain {
    /// Converts Gain value into the corresponding register value
    pub fn into_reg_value(self) -> u8 {
        self as u8
    }
}

/// The mode of the BH1730FVC sensor.
#[derive(Copy, Clone, Debug)]
pub enum Mode {
    PowerDown = 0x00,
    SingleShot = 0x01,
    Continuous = 0x02,
}
