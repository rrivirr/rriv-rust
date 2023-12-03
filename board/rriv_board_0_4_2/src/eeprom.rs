use core::hash::BuildHasher;

use embedded_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};
use nb::block;
use rriv_board::RRIVBoard;
use rtt_target::rprint;

// implementation specific consts
const EEPROM_I2C_ADDRESS: u8 = 0x50;

const EEPROM_RESET_VALUE: u16 = 255; // max value of a byte

const EEPROM_UUID_ADDRESS_START: u8 = 0;
const EEPROM_UUID_ADDRESS_END: u8 = 15;
const UUID_LENGTH: u8 = 12; // STM32 has a 12 byte UUID, leave extra space for the future 16

const EEPROM_DATALOGGER_SETTINGS_START: u8 = 16;
const EEPROM_SENSOR_SETTINGS_START: u8 = 80;
const EEPROM_TOTAL_SENSOR_SLOTS: u8 = 12;

pub fn write_bytes_to_eeprom(board: &mut crate::Board, start_address: u8, bytes: &[u8]) {
    let mut address = start_address;
    for byte in bytes {
        let bytes_to_send = [address, *byte];
        match board
            .i2c1
            .write(EEPROM_I2C_ADDRESS, &bytes_to_send)
        {
            Ok(_) => {
                // rprint!("wrote {} address {}\n", byte, address);
                board.delay_ms(5_u16);
            }
            Err(error) => {
                rprint!("error: {:?}", error);
            }
        }
        address = address + 1;
    }
}

pub fn read_bytes_from_eeprom(board: &mut crate::Board, start_address: u8, buffer: &mut [u8]) {
        let mut i: usize = 0;

        while i < buffer.len() {
            let mut b: [u8; 1] = [254];
            let message = [start_address + i as u8];
            match board.i2c1.write_read(EEPROM_I2C_ADDRESS, &message, &mut b) {
                Ok(_) => {
                    // rprint!("read {} address {}\n", b[0], message[0]);
                    buffer[i] = b[0];
                }
                Err(_) => {
                    board.serial_send("EEPROM read failure, restarting");
                    // board.restart();
                    return;
                } // what do we do if we fail?  panic?  retry?  restart the board?
            }
            i = i + 1;
        }
    
}

pub fn write_datalogger_settings_to_eeprom(
    board: &mut crate::Board,
    bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
) {
    write_bytes_to_eeprom(board, EEPROM_DATALOGGER_SETTINGS_START, bytes);
}

pub fn read_datalogger_settings_from_eeprom(board: &mut crate::Board, buffer: &mut [u8]) {
    read_bytes_from_eeprom(board, EEPROM_DATALOGGER_SETTINGS_START, buffer);
}
