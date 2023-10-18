use embedded_hal::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_i2c_Read};
use rriv_board::RRIVBoard;

// implementation specific consts
const EEPROM_I2C_ADDRESS: u8 = 0x50;

const EEPROM_RESET_VALUE: u16 = 255; // max value of a byte

const EEPROM_UUID_ADDRESS_START: u8 = 0;
const EEPROM_UUID_ADDRESS_END: u8 = 15;
const UUID_LENGTH: u8 = 12; // STM32 has a 12 byte UUID, leave extra space for the future 16

const EEPROM_DATALOGGER_SETTINGS_START: u8 = 16;
const EEPROM_DATALOGGER_SENSOR_SETTINGS_START: u8 = 80;
const EEPROM_TOTAL_SENSOR_SLOTS: u8 = 12;

pub fn write_bytes_to_eeprom(board: &mut crate::Board, start_address: u8, bytes: &[u8]) {

  if let Some(i2c) = & mut board.i2c1 { 
    i2c.write(start_address, bytes); // does this work, or do we need to send each byte separately
  }
}

pub fn read_bytes_from_eeprom(board: &mut crate::Board, start_address: u8, buffer: &mut [u8]) {
  if let Some(i2c) = &mut board.i2c1 { 
    match i2c.read(start_address, buffer) {
        Ok(_) => return,
        Err(_) => {
          board.serial_send("EEPROM read failure, restarting");
          // board.restart();
        }, // what do we do if we fail?  panic?  retry?  restart the board?
    }
  }
}


pub fn write_datalogger_settings_to_eeprom(board: &mut crate::Board, bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE]) {
  write_bytes_to_eeprom(board, EEPROM_DATALOGGER_SENSOR_SETTINGS_START, bytes);
}

pub fn read_datalogger_settings_from_eeprom(board: &mut crate::Board, buffer: &mut [u8]) {
  read_bytes_from_eeprom(board, EEPROM_DATALOGGER_SENSOR_SETTINGS_START, buffer);
}