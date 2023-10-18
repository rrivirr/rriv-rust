#![cfg_attr(not(test), no_std)]
extern crate alloc;
use alloc::boxed::Box;

pub const EEPROM_DATALOGGER_SETTINGS_SIZE: usize = 64;
pub const EEPROM_SENSOR_SETTINGS_SIZE: usize = 64;

pub trait RXProcessor: Send + Sync {
    fn process_character(&'static self, character: u8);
}

pub trait RRIVBoard: Send {
    fn setup(&mut self);
    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>);
    fn serial_send(&self, string: &str);
    fn critical_section<T, F>(&self, f: F) -> T where F: Fn() -> T;
    fn store_datalogger_settings(&mut self, bytes: &[u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
    fn retrieve_datalogger_settings(&mut self, buffer: &mut [u8;EEPROM_DATALOGGER_SETTINGS_SIZE]);
}
