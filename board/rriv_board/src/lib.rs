#![cfg_attr(not(test), no_std)]
extern crate alloc;
use alloc::boxed::Box;

pub const EEPROM_DATALOGGER_SETTINGS_SIZE: usize = 64;
pub const EEPROM_SENSOR_SETTINGS_SIZE: usize = 64;
pub const EEPROM_TOTAL_SENSOR_SLOTS: usize = 2;

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
        fn serial_debug(&self, string: &str);    
        fn delay_ms(&mut self, ms: u16);
        fn timestamp(&mut self) -> i64;
    };
}

pub trait RRIVBoard: Send {

    // Run loop services
    fn run_loop_iteration(&mut self);

    // Core Services
    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>);
    fn critical_section<T, F>(&self, f: F) -> T where F: Fn() -> T;

    // Storage Services
    fn store_datalogger_settings(&mut self, bytes: &[u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn retrieve_datalogger_settings(&mut self, buffer: &mut [u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn store_sensor_settings(&mut self, slot: u8, bytes: &[u8; EEPROM_SENSOR_SETTINGS_SIZE] );
    fn retrieve_sensor_settings(&mut self, buffer: &mut [u8; EEPROM_SENSOR_SETTINGS_SIZE * EEPROM_TOTAL_SENSOR_SLOTS]);

    // Modes
    fn set_debug(&mut self, debug: bool);

    // Data Logging
    fn write_log_file(&mut self, data: &str);
    fn flush_log_file(&mut self);


    // Time
    fn set_epoch(&mut self, epoch: i64);
    fn epoch_timestamp(&mut self) -> i64;

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

pub trait OneWireBusInterface {
    // fn send_command(
    //     &mut self,
    //     command: u8,
    //     address: Option<&Address>,
    //     delay: &mut impl DelayUs<u16>,
    // ) -> OneWireResult<(), E>
}


pub trait SensorDriverServices {
    // future functions for ADC interface
    // fn get_adc_capabilities(&mut self); // minimum functionality return of adcs
    // fn get power on status of each adc
    // fn change power on status of each adc
    // fn query adc by index
    
    fn query_internal_adc(&mut self, port: u8) -> u16;
    fn query_external_adc(&mut self, port: u8) -> u32;
    fn ic2_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), ()>;
    fn ic2_write(&mut self, addr: u8, message: &[u8]) -> Result<(), ()>;

    fn write_gpio_pin(&mut self, pin: u8, value: bool);
    // fn read_gpio_pin(&mut self, pin: u8) -> bool;

    // fn borrow_one_wire_bus(&mut self) -> &mut dyn OneWireBusInterface;

    fn one_wire_send_command(&mut self, command: u8, address: u64);
    fn one_wire_reset(&mut self);
    fn one_wire_skip_address(&mut self);
    fn one_wire_write_byte(&mut self, byte: u8);
    fn one_wire_match_address(&mut self, address: u64);
    fn one_wire_read_bytes(&mut self, output: &mut [u8] );
    fn one_wire_bus_start_search(&mut self);
    fn one_wire_bus_search(&mut self) -> Option<u64>;

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