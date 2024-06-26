#![cfg_attr(not(test), no_std)]
extern crate alloc;
use alloc::boxed::Box;

pub const EEPROM_DATALOGGER_SETTINGS_SIZE: usize = 64;
pub const EEPROM_SENSOR_SETTINGS_SIZE: usize = 64;
pub const EEPROM_TOTAL_SENSOR_SLOTS: usize = 12;

enum AdcSelect {
    Internal,
    External
}

pub trait RXProcessor: Send + Sync {
    fn process_character(&'static self, character: u8);
}

// Board Services Used by Control Logic and Drivers
macro_rules! control_services {
    () => {
        fn serial_send(&self, string: &str);    
        fn delay_ms(&mut self, ms: u16);
        fn timestamp(&mut self) -> u32;
    };
}

pub trait RRIVBoard: Send {
    
    // Core Services
    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>);
    fn critical_section<T, F>(&self, f: F) -> T where F: Fn() -> T;

    // Storage Services
    fn store_datalogger_settings(&mut self, bytes: &[u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn retrieve_datalogger_settings(&mut self, buffer: &mut [u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn store_sensor_settings(&mut self, slot: u8, bytes: &[u8; EEPROM_SENSOR_SETTINGS_SIZE] );
    fn retrieve_sensor_settings(&mut self, buffer: &mut [u8; EEPROM_SENSOR_SETTINGS_SIZE * EEPROM_TOTAL_SENSOR_SLOTS]);

    // Board Services Used by Control Logic and Drivers
    control_services!();

    fn get_sensor_driver_services(&mut self) -> &mut dyn SensorDriverServices;
    fn get_actuator_driver_services(&mut self) -> &mut dyn ActuatorDriverServices;
    fn get_telemetry_driver_services(&mut self) -> &mut dyn TelemetryDriverServices;


}


// pub trait SensorMeasurementInterfaces {
//     adc,
//     i2c,
//     modbus,
// }

// pub trait ActuatorInterfaces {
//     i2c,
//     modbug
// }

// pub trait TelemeterInterfaces {
//     spi
// }

pub trait SensorDriverServices {
    // future functions for ADC interface
    // fn get_adc_capabilities(&mut self); // minimum functionality return of adcs
    // fn get power on status of each adc
    // fn change power on status of each adc
    // fn query adc by index
    
    fn query_internal_adc(&mut self, port: u8) -> u16;
    fn query_external_adc(&mut self, port: u8) -> u32;

    control_services!();

}

pub trait ActuatorDriverServices {
    control_services!();
}

pub trait TelemetryDriverServices {
    control_services!();
}


pub trait RRIVBoardBuilder {
    fn setup(&mut self);
    // fn build(self) -> dyn RRIVBoard;
}