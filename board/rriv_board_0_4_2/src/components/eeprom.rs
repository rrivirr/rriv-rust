
use embedded_hal::prelude::{
    _embedded_hal_blocking_i2c_Read, _embedded_hal_blocking_i2c_Write,
    _embedded_hal_blocking_i2c_WriteRead,
};
use crate::Board;
use rriv_board::RRIVBoard;
use rtt_target::{rprint, rprintln};

// implementation specific consts
const EEPROM_I2C_ADDRESS: u8 = 0x50;

const _EEPROM_RESET_VALUE: u16 = 255; // max value of a byte

const _EEPROM_UUID_ADDRESS_START: u8 = 0;
const _EEPROM_UUID_ADDRESS_END: u8 = 15;
const _UUID_LENGTH: u8 = 12; // STM32 has a 12 byte UUID, leave extra space for the future 16

const EEPROM_DATALOGGER_SETTINGS_START: u8 = 16;
const _EEPROM_SENSOR_SETTINGS_START: u8 = 80;


pub fn write_bytes_to_eeprom(board: &mut crate::Board, block: u8, start_address: u8, bytes: &[u8]) {
    let device_address = EEPROM_I2C_ADDRESS + block;
    let mut address = start_address;
    for byte in bytes {
        let bytes_to_send = [address, *byte];
        match board
            .i2c1
            .write(device_address, &bytes_to_send)
        {
            Ok(_) => {
                // rprint!("wrote {} address {}\n", byte, address);
                board.delay_ms(5_u16);
            }
            Err(error) => {
                rprintln!("write error: {:?}", error);
            }
        }
        address = address + 1;
    }
    rprintln!("done with EEPROM writes");
}

pub fn read_bytes_from_eeprom(board: &mut crate::Board, block: u8, start_address: u8, buffer: &mut [u8]) {
        let mut i: usize = 0;
        let device_address = EEPROM_I2C_ADDRESS + block;


        while i < buffer.len() {
            let mut b: [u8; 1] = [254];
            let message = [start_address + i as u8];
            match board.i2c1.write_read(device_address, &message, &mut b) {
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
    board: &mut Board,
    bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
) {
    write_bytes_to_eeprom(board, 0, EEPROM_DATALOGGER_SETTINGS_START, bytes);
}

pub fn read_datalogger_settings_from_eeprom(board: &mut Board, buffer: &mut [u8]) {
    read_bytes_from_eeprom(board, 0, EEPROM_DATALOGGER_SETTINGS_START, buffer);
}

struct MemoryPosition {
    pub block: u8,
    pub offset: u8,
    pub address: u8
}

fn calculate_memory_position(slot: u8) -> MemoryPosition {

    let block = slot / 4 + 1;
    let offset = slot % 4;
    let address = offset as u8 * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE as u8;

    MemoryPosition { 
        block, 
        offset,
        address
    }
}

pub fn write_sensor_configuration_to_eeprom(board: &mut Board, slot: u8, bytes: &[u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {
   
    let memory_position = calculate_memory_position(slot);
    write_bytes_to_eeprom(board, memory_position.block, memory_position.address,bytes);
    
}

pub fn read_sensor_configuration_from_eeprom(board: &mut Board, slot: u8, buffer: &mut [u8]) {

    let memory_position = calculate_memory_position(slot);
    read_bytes_from_eeprom(board, memory_position.block, memory_position.address, buffer)

}

