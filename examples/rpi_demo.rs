// This example demonstrates how to use the BH1730FVC sensor with a Raspberry Pi.
// It is so far untested, but should be a good reference for any kind of embedded system.

use bh1730fvc::BH1730FVC;
use embedded_hal::blocking::delay::DelayMs;
use linux_embedded_hal as hal;

fn main() {
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

    // Do continuous measurement for 5 minutes
    match bh1730fvc.start_continuous_measurement(&mut i2c) {
        Ok(_) => log::info!("Continuous measurement started"),
        Err(e) => log::error!("Error starting continuous measurement: {:?}", e),
    }

    for _ in 0..300 {
        // Here we always get the latest reading from the sensor
        // Given the delay, we'll skip some readings
        match bh1730fvc.get_last_ambient_light_intensity(&mut i2c) {
            Ok(reading) => log::info!("Lux Value: {}", reading),
            Err(e) => log::error!("Error reading sensor: {:?}", e),
        }
        delay.delay_ms(1000u32);
    }

    // Stop continuous measurement (sensor will go to idle mode)
    match bh1730fvc.stop_continuous_measurement(&mut i2c) {
        Ok(_) => log::info!("Continuous measurement stopped"),
        Err(e) => log::error!("Error stopping continuous measurement: {:?}", e),
    }

    // Just some loop so we never return
    loop {
        delay.delay_ms(1000u32);
    }
}
