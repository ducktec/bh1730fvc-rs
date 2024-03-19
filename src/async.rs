//! Blocking API
//!
//! This module contains the blocking API for the BH1730FVC sensor.

use crate::{
    calculate_lux, itime_ms_to_itime, itime_to_itime_ms, BH1730FVCError, DataRegister, Gain, Mode,
    Result, BH1730FVC, BH1730FVC_ADDR, BH1730FVC_RESET_CMD,
};

impl<I2C, D> BH1730FVC<I2C, D>
where
    D: embedded_hal_async::delay::DelayNs,
    I2C: embedded_hal_async::i2c::I2c<embedded_hal_async::i2c::SevenBitAddress>,
{
    /// Creates a connection with an BH1730FVC sensor via I2C.
    ///
    /// This method will reset the sensor and wait for it to start up.
    /// It will also initialize the sensor with the default configuration values
    /// for gain and integration time.
    pub async fn new(delay: &mut D, i2c: &mut I2C) -> Result<Self> {
        let result = i2c.write(BH1730FVC_ADDR, &[BH1730FVC_RESET_CMD]).await;
        if result.is_err() {
            return Err(BH1730FVCError::WriteI2CError);
        }

        // The sensor per datasheet needs more than 2 ms to startup, so just wait a bit longer
        delay.delay_ms(10).await;

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
    pub async fn get_ambient_light_intensity_single_shot(
        &mut self,
        delay: &mut D,
        i2c: &mut I2C,
    ) -> Result<f32> {
        self.set_mode(Mode::SingleShot, i2c).await?;
        delay.delay_ms(self.integration_time_ms as u32).await;

        self.read_ambient_light_intensity(i2c).await
    }

    /// Start continuous measurement mode.
    pub async fn start_continuous_measurement(&mut self, i2c: &mut I2C) -> Result<()> {
        self.set_mode(Mode::Continuous, i2c).await
    }

    /// Stop continuous measurement mode.
    pub async fn stop_continuous_measurement(&mut self, i2c: &mut I2C) -> Result<()> {
        self.set_mode(Mode::PowerDown, i2c).await
    }

    /// Get the last measured ambient light intensity in lux.
    pub async fn get_last_ambient_light_intensity(&mut self, i2c: &mut I2C) -> Result<f32> {
        self.read_ambient_light_intensity(i2c).await
    }

    /// Write the integration time to the sensor.
    pub async fn set_integration_time(&mut self, time_ms: f32, i2c: &mut I2C) -> Result<()> {
        self.integration_time_ms = time_ms;
        self.write_register(DataRegister::Timing, itime_ms_to_itime(time_ms), i2c)
            .await
    }

    /// Read the integration time of the sensor.
    pub async fn read_integration_time(&mut self, i2c: &mut I2C) -> Result<f32> {
        let itime = self.read_register(DataRegister::Timing, i2c).await?;
        Ok(itime_to_itime_ms(itime))
    }

    /// Set the gain of the sensor.
    pub async fn set_gain(&mut self, gain: Gain, i2c: &mut I2C) -> Result<()> {
        self.gain = gain;
        self.write_register(DataRegister::Gain, gain.into_reg_value(), i2c)
            .await
    }

    /// Read the gain of the sensor.
    pub async fn read_gain(&mut self, i2c: &mut I2C) -> Result<Gain> {
        let gain = self.read_register(DataRegister::Gain, i2c).await?;
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
    pub async fn set_mode(&mut self, mode: Mode, i2c: &mut I2C) -> Result<()> {
        // Set the right values for the ONE_TIME, ADC_EN and POWER fields in the register based on mode
        let control_register_value = match mode {
            Mode::PowerDown => 0x00,
            Mode::SingleShot => 0x0b,
            Mode::Continuous => 0x03,
        };

        self.write_register(DataRegister::Control, control_register_value, i2c)
            .await
    }

    /// Reads the ambient light intensity in lux in single-shot mode using the read_light_intensity function
    pub async fn read_ambient_light_intensity(&mut self, i2c: &mut I2C) -> Result<f32> {
        // Check if the sensor has valid data available (ADC_VALID bit in the Control register is set to 1)
        if (self.read_register(DataRegister::Control, i2c).await? & 0x10) == 0 {
            return Err(BH1730FVCError::NoDataAvailable);
        }

        let (data0, data1) = self.read_light_raw_values(i2c).await?;

        let lux = calculate_lux(data0, data1, f32::from(self.gain), self.integration_time_ms);

        Ok(lux)
    }

    /// Writes a new value to a specific register
    pub async fn write_register(
        &mut self,
        register: DataRegister,
        data: u8,
        i2c: &mut I2C,
    ) -> Result<()> {
        let register_address = register as u8;
        let command_reg_value = register_address | 0x80; // Set MSB to 1 to indicate address mode
        let write_data = [command_reg_value, data];

        i2c.write(BH1730FVC_ADDR, &write_data)
            .await
            .map_err(|_| BH1730FVCError::WriteI2CError)?;

        Ok(())
    }

    /// Reads the value of a specific register
    pub async fn read_register(&mut self, register: DataRegister, i2c: &mut I2C) -> Result<u8> {
        let register_address = register as u8;
        let command_reg_value = register_address | 0x80; // Set MSB to 1 to indicate address mode
        let mut read_data = [0; 1];

        i2c.write_read(BH1730FVC_ADDR, &[command_reg_value], &mut read_data)
            .await
            .map_err(|_| BH1730FVCError::ReadI2CError)?;

        Ok(read_data[0])
    }

    /// Read all 4 data registers with the light intensity values in one i2c write-then-read operation
    pub async fn read_light_raw_values(&mut self, i2c: &mut I2C) -> Result<(u16, u16)> {
        let mut read_data = [0; 4];

        i2c.write_read(
            BH1730FVC_ADDR,
            &[DataRegister::Data0Low as u8 | 0x80],
            &mut read_data,
        )
        .await
        .map_err(|_| BH1730FVCError::ReadI2CError)?;

        log::info!("Read raw values: {:?}", read_data);

        let data0 = ((read_data[1] as u16) << 8) | read_data[0] as u16;
        let data1 = ((read_data[3] as u16) << 8) | read_data[2] as u16;

        Ok((data0, data1))
    }

    /// Reads the part number and the revision id of the sensor.
    pub async fn read_id(&mut self, i2c: &mut I2C) -> Result<(u8, u8)> {
        // Reads temperature
        let id_raw = self.read_register(DataRegister::ID, i2c).await?;

        Ok((id_raw >> 4, id_raw & 0x0F))
    }
}

// async mocking of I2C currently not supported by embedded-hal-mock
// so no UT right now
