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

pub trait RRIVBoard: Send {
    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>);
    fn serial_send(&self, string: &str);
    fn critical_section<T, F>(&self, f: F) -> T where F: Fn() -> T;
    fn store_datalogger_settings(&mut self, bytes: &[u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn retrieve_datalogger_settings(&mut self, buffer: &mut [u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn store_sensor_settings(&mut self, slot: u8, bytes: &[u8; EEPROM_SENSOR_SETTINGS_SIZE] );
    fn retrieve_sensor_settings(&mut self, buffer: &mut [u8; EEPROM_SENSOR_SETTINGS_SIZE * EEPROM_TOTAL_SENSOR_SLOTS]);
    fn delay_ms(&mut self, ms: u16);
    fn timestamp(&mut self) -> u32;
    fn get_adc_interface(&mut self) -> &mut dyn ADCInterface;
}

pub trait ADCInterface {
    // future functions for ADC interface
    // fn get_adc_capabilities(&mut self); // minimum functionality return of adcs
    // fn get power on status of each adc
    // fn change power on status of each adc
    // fn query adc by index
    
    fn query_internal_adc(&mut self, port: u8) -> u16;
    fn query_external_adc(&mut self, port: u8) -> u32;
}


pub trait RRIVBoardBuilder {
    fn setup(&mut self);
    // fn build(self) -> dyn RRIVBoard;
}