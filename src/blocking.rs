//! # Blocking API
//!
//! This module contains the blocking API for the BH1730FVC sensor.
//! It is based on the `embedded-hal` traits and is intended to be used
//! with synchronous blocking code.
//!
//! The methods return `Result` with the error type being `BH1730FVCError`.
//!
//! ## Examples
//! ### Creating a driver instance
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
//! ### Reading the ambient light intensity
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
//! ### Continuous measurement
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

use crate::{
    calculate_lux, itime_ms_to_itime, itime_to_itime_ms, BH1730FVCError, DataRegister, Gain, Mode,
    Result, BH1730FVC_ADDR, BH1730FVC_RESET_CMD,
};

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

impl<I2C, D> BH1730FVC<I2C, D>
where
    D: embedded_hal::blocking::delay::DelayMs<u32>,
    I2C: embedded_hal::blocking::i2c::Read
        + embedded_hal::blocking::i2c::Write
        + embedded_hal::blocking::i2c::WriteRead,
{
    /// Creates a connection with an BH1730FVC sensor via I2C.
    ///
    /// This method will reset the sensor and wait for it to start up.
    /// It will also initialize the sensor with the default configuration values
    /// for gain and integration time.
    pub fn new(delay: &mut D, i2c: &mut I2C) -> Result<Self> {
        let result = i2c.write(BH1730FVC_ADDR, &[BH1730FVC_RESET_CMD]);
        if result.is_err() {
            return Err(BH1730FVCError::WriteI2CError);
        }

        // The sensor per datasheet needs more than 2 ms to startup, so just wait a bit longer
        delay.delay_ms(10);

        Ok(Self {
            _delay: core::marker::PhantomData::default(),
            _i2c: core::marker::PhantomData::default(),
            gain: Gain::X1,
            integration_time_ms: 102.6,
        })
    }

    /// Perform a single-shot measurement of the ambient light intensity in lux.
    ///
    /// This method will set the sensor to single-shot mode, wait for the
    /// measurement to complete, and then read the result.
    pub fn get_ambient_light_intensity_single_shot(
        &mut self,
        delay: &mut D,
        i2c: &mut I2C,
    ) -> Result<f32> {
        self.set_mode(Mode::SingleShot, i2c)?;
        delay.delay_ms(self.integration_time_ms as u32);

        self.read_ambient_light_intensity(i2c)
    }

    /// Start continuous measurement mode.
    pub fn start_continuous_measurement(&mut self, i2c: &mut I2C) -> Result<()> {
        self.set_mode(Mode::Continuous, i2c)
    }

    /// Stop continuous measurement mode.
    pub fn stop_continuous_measurement(&mut self, i2c: &mut I2C) -> Result<()> {
        self.set_mode(Mode::PowerDown, i2c)
    }

    /// Get the last measured ambient light intensity in lux.
    pub fn get_last_ambient_light_intensity(&mut self, i2c: &mut I2C) -> Result<f32> {
        self.read_ambient_light_intensity(i2c)
    }

    /// Write the integration time to the sensor.
    pub fn set_integration_time(&mut self, time_ms: f32, i2c: &mut I2C) -> Result<()> {
        self.integration_time_ms = time_ms;
        self.write_register(DataRegister::Timing, itime_ms_to_itime(time_ms), i2c)
    }

    /// Read the integration time of the sensor.
    pub fn read_integration_time(&mut self, i2c: &mut I2C) -> Result<f32> {
        let itime = self.read_register(DataRegister::Timing, i2c)?;
        Ok(itime_to_itime_ms(itime))
    }

    /// Set the gain of the sensor.
    pub fn set_gain(&mut self, gain: Gain, i2c: &mut I2C) -> Result<()> {
        self.gain = gain;
        self.write_register(DataRegister::Gain, gain.into_reg_value(), i2c)
    }

    /// Read the gain of the sensor.
    pub fn read_gain(&mut self, i2c: &mut I2C) -> Result<Gain> {
        let gain = self.read_register(DataRegister::Gain, i2c)?;
        // Mask the GAIN bits and match the value to the Gain enum
        Ok(match gain & 0x07 {
            0x0 => Gain::X1,
            0x1 => Gain::X2,
            0x2 => Gain::X64,
            0x3 => Gain::X128,
            _ => panic!("Invalid gain value"),
        })
    }

    /// Set the mode of the sensor.
    pub fn set_mode(&mut self, mode: Mode, i2c: &mut I2C) -> Result<()> {
        // Set the right values for the ONE_TIME, ADC_EN and POWER fields in the register based on mode
        let control_register_value = match mode {
            Mode::PowerDown => 0x00,
            Mode::SingleShot => 0x0b,
            Mode::Continuous => 0x03,
        };

        self.write_register(DataRegister::Control, control_register_value, i2c)
    }

    /// Reads the ambient light intensity in lux in single-shot mode using the read_light_intensity function
    pub fn read_ambient_light_intensity(&mut self, i2c: &mut I2C) -> Result<f32> {
        // Check if the sensor has valid data available (ADC_VALID bit in the Control register is set to 1)
        if (self.read_register(DataRegister::Control, i2c)? & 0x10) == 0 {
            return Err(BH1730FVCError::NoDataAvailable);
        }

        let (data0, data1) = self.read_light_raw_values(i2c)?;

        let lux = calculate_lux(data0, data1, f32::from(self.gain), self.integration_time_ms);

        Ok(lux)
    }

    /// Writes a new value to a specific register
    pub fn write_register(
        &mut self,
        register: DataRegister,
        data: u8,
        i2c: &mut I2C,
    ) -> Result<()> {
        let register_address = register as u8;
        let command_reg_value = register_address | 0x80; // Set MSB to 1 to indicate address mode
        let write_data = [command_reg_value, data];

        i2c.write(BH1730FVC_ADDR, &write_data)
            .map_err(|_| BH1730FVCError::WriteI2CError)?;

        Ok(())
    }

    /// Reads the value of a specific register
    pub fn read_register(&mut self, register: DataRegister, i2c: &mut I2C) -> Result<u8> {
        let register_address = register as u8;
        let command_reg_value = register_address | 0x80; // Set MSB to 1 to indicate address mode
        let mut read_data = [0; 1];

        i2c.write_read(BH1730FVC_ADDR, &[command_reg_value], &mut read_data)
            .map_err(|_| BH1730FVCError::ReadI2CError)?;

        Ok(read_data[0])
    }

    /// Read all 4 data registers with the light intensity values in one i2c write-then-read operation
    pub fn read_light_raw_values(&mut self, i2c: &mut I2C) -> Result<(u16, u16)> {
        let mut read_data = [0; 4];

        i2c.write_read(
            BH1730FVC_ADDR,
            &[DataRegister::Data0Low as u8 | 0x80],
            &mut read_data,
        )
        .map_err(|_| BH1730FVCError::ReadI2CError)?;

        log::info!("Read raw values: {:?}", read_data);

        let data0 = ((read_data[1] as u16) << 8) | read_data[0] as u16;
        let data1 = ((read_data[3] as u16) << 8) | read_data[2] as u16;

        Ok((data0, data1))
    }

    /// Reads the part number and the revision id of the sensor.
    pub fn read_id(&mut self, i2c: &mut I2C) -> Result<(u8, u8)> {
        // Reads temperature
        let id_raw = self.read_register(DataRegister::ID, i2c)?;

        Ok((id_raw >> 4, id_raw & 0x0F))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::i2c::Mock as I2cMock;
    use embedded_hal_mock::MockError;
    use embedded_hal_mock::{delay::MockNoop as DelayMock, i2c::Transaction as I2cTransaction};

    #[test]
    fn test_get_ambient_light_intensity_single_shot() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xe4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x80, 0x0b].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x80].to_vec(), [0x10].to_vec()),
            I2cTransaction::write_read(
                BH1730FVC_ADDR,
                [(0x14 | 0x80)].to_vec(),
                [0xA4, 0x0, 0x0, 0x1].to_vec(),
            ),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut bh1730fvc = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let lux_val = bh1730fvc
            .get_ambient_light_intensity_single_shot(&mut delay_mock, &mut i2c_mock)
            .unwrap();
        assert_eq!(lux_val, 11.984001);

        i2c_mock.done();
    }

    #[test]
    fn test_continous_measurement() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xe4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x80, 0x03].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x80].to_vec(), [0x10].to_vec()),
            I2cTransaction::write_read(
                BH1730FVC_ADDR,
                [(0x14 | 0x80)].to_vec(),
                [0xA4, 0x0, 0x0, 0x1].to_vec(),
            ),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x80].to_vec(), [0x0f].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x80, 0x00].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut bh1730fvc = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        bh1730fvc
            .start_continuous_measurement(&mut i2c_mock)
            .unwrap();

        // happy path, we assume valid data is available
        let lux_val = bh1730fvc
            .get_last_ambient_light_intensity(&mut i2c_mock)
            .unwrap();
        assert_eq!(lux_val, 11.984001);

        let result = bh1730fvc.get_last_ambient_light_intensity(&mut i2c_mock);
        assert!(result.is_err());

        bh1730fvc
            .stop_continuous_measurement(&mut i2c_mock)
            .unwrap();

        i2c_mock.done();
    }

    #[test]
    fn test_calculate_lux() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x07 | 0x80, 0x01].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x01 | 0x80, 0xB5].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations); // No I2C transactions are expected
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let integration_time_ms = 200.0;
        sensor.set_gain(Gain::X2, &mut i2c_mock).unwrap();
        sensor
            .set_integration_time(integration_time_ms, &mut i2c_mock)
            .unwrap();

        // Test case 1: ratio < 0.26
        let data0 = 100;
        let data1 = 20;
        let expected_lux = (1.290 * data0 as f32 - 2.733 * data1 as f32)
            / (f32::from(sensor.gain) * 102.6 / sensor.integration_time_ms);
        assert_eq!(
            calculate_lux(data0, data1, f32::from(Gain::X2), integration_time_ms),
            expected_lux
        );

        // Test case 2: 0.26 <= ratio < 0.55
        let data0 = 100;
        let data1 = 40;
        let expected_lux = (0.795 * data0 as f32 - 0.859 * data1 as f32)
            / (f32::from(sensor.gain) * 102.6 / sensor.integration_time_ms);
        assert_eq!(
            calculate_lux(data0, data1, f32::from(Gain::X2), integration_time_ms),
            expected_lux
        );

        // Test case 3: 0.55 <= ratio < 1.09
        let data0 = 100;
        let data1 = 70;
        let expected_lux = (0.510 * data0 as f32 - 0.345 * data1 as f32)
            / (f32::from(sensor.gain) * 102.6 / sensor.integration_time_ms);
        assert_eq!(
            calculate_lux(data0, data1, f32::from(Gain::X2), integration_time_ms),
            expected_lux
        );

        // Test case 4: 1.09 <= ratio < 2.13
        let data0 = 100;
        let data1 = 120;
        let expected_lux = (0.276 * data0 as f32 - 0.130 * data1 as f32)
            / (f32::from(sensor.gain) * 102.6 / sensor.integration_time_ms);
        assert_eq!(
            calculate_lux(data0, data1, f32::from(Gain::X2), integration_time_ms),
            expected_lux
        );

        // Test case 5: ratio >= 2.13
        let data0 = 100;
        let data1 = 250;
        let expected_lux = 0.0;
        assert_eq!(
            calculate_lux(data0, data1, f32::from(Gain::X2), integration_time_ms),
            expected_lux
        );
    }

    #[test]
    fn test_write_error() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x80, 0x00].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x80, 0x00].to_vec())
                .with_error(MockError::Io(std::io::ErrorKind::Other)),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.write_register(DataRegister::Control, 0x00, &mut i2c_mock);
        assert_eq!(result, Ok(()));

        let result = sensor.write_register(DataRegister::Control, 0x00, &mut i2c_mock);
        assert_eq!(result, Err(BH1730FVCError::WriteI2CError));

        i2c_mock.done();
    }

    #[test]
    fn test_read_error() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x80].to_vec(), [0x00].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x80].to_vec(), [0x00].to_vec())
                .with_error(MockError::Io(std::io::ErrorKind::Other)),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.read_register(DataRegister::Control, &mut i2c_mock);
        assert_eq!(result, Ok(0x00));

        let result = sensor.read_register(DataRegister::Control, &mut i2c_mock);
        assert_eq!(result, Err(BH1730FVCError::ReadI2CError));

        i2c_mock.done();
    }

    #[test]
    fn test_read_id() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x92].to_vec(), [0x12].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.read_id(&mut i2c_mock);
        assert_eq!(result, Ok((0x1, 0x2)));

        i2c_mock.done();
    }

    #[test]
    fn test_set_integration_time() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x81, 0xC8].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.set_integration_time(150.0, &mut i2c_mock);
        assert_eq!(result, Ok(()));

        i2c_mock.done();
    }

    #[test]
    fn test_read_integration_time() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x81].to_vec(), [0x2a].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.read_integration_time(&mut i2c_mock);
        assert_eq!(result, Ok(577.6288));

        i2c_mock.done();
    }

    #[test]
    fn test_set_gain() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write(BH1730FVC_ADDR, [0x87, 0x02].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.set_gain(Gain::X64, &mut i2c_mock);
        assert_eq!(result, Ok(()));

        i2c_mock.done();
    }

    #[test]
    fn test_read_gain() {
        let expectations = [
            I2cTransaction::write(BH1730FVC_ADDR, [0xE4].to_vec()),
            I2cTransaction::write_read(BH1730FVC_ADDR, [0x87].to_vec(), [0x03].to_vec()),
        ];

        let mut i2c_mock = I2cMock::new(&expectations);
        let mut delay_mock = DelayMock::new();

        let mut sensor = BH1730FVC::new(&mut delay_mock, &mut i2c_mock).unwrap();
        let result = sensor.read_gain(&mut i2c_mock).unwrap();
        assert_eq!(result, Gain::X128);

        i2c_mock.done();
    }

    #[test]
    fn test_itime() {
        let itime_ms = 250.0;

        let itime: u8 = itime_ms_to_itime(itime_ms);
        assert_eq!(itime, 0xA3);

        let itime_ms = itime_to_itime_ms(itime);

        // check that itime_ms is within 1% of the original value
        assert!((itime_ms - 250.0).abs() < 2.5);
    }
}
