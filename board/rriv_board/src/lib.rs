#![cfg_attr(not(test), no_std)]
extern crate alloc;
use alloc::boxed::Box;

pub const EEPROM_DATALOGGER_SETTINGS_SIZE: usize = 64;
pub const EEPROM_SENSOR_SETTINGS_SIZE: usize = 64;
pub const EEPROM_TOTAL_SENSOR_SLOTS: usize = 12;

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
}


pub trait RRIVBoardBuilder {
    fn setup(&mut self);
    // fn build(self) -> dyn RRIVBoard;
}