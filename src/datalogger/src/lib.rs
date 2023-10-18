#![cfg_attr(not(test), no_std)]

mod command_service;
mod datalogger_commands;
use datalogger_commands::*;
use rriv_board::{RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE};
use bitflags::bitflags;


#[derive(Copy,Clone)]
struct DataloggerSettings {
    deployment_identifier: [u8;16],
    logger_name: [u8;8],
    site_name: [u8;8],
    deployment_timestamp: u64,
    interval: u16,
    burst_repetitions: u16,
    start_up_delay: u16,
    delay_between_bursts: u16,
    mode: char,
    // external_adc_enabled: u8:1, // how we we handle bitfields?
    // debug_includes_values: u8:1,
    // withold_incomplete_readings: u8:1,
    // low_raw_data: u8:1,
    // reserved: u8:4
}

impl DataloggerSettings {
    pub fn new() -> Self {
        let mut settings = DataloggerSettings { 
            deployment_identifier: [b'\0'; 16], 
            logger_name: [b'\0'; 8], 
            site_name: [b'\0'; 8], 
            deployment_timestamp: 0, 
            interval: 60, 
            burst_repetitions: 1, 
            start_up_delay: 0, 
            delay_between_bursts: 0, 
            mode: 'i' 
        };
        let logger_name_types = "MyLogger".as_bytes();
        settings.logger_name.copy_from_slice(logger_name_types);
        settings
    }

    pub fn new_from_bytes(bytes: &[u8;EEPROM_DATALOGGER_SETTINGS_SIZE] ) -> DataloggerSettings {
        let settings = bytes.as_ptr().cast::<DataloggerSettings>();
        unsafe {
            *settings
        }
    }
}

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts(
        (p as *const T) as *const u8,
        ::core::mem::size_of::<T>(),
    )
}


pub struct DataLogger {
    settings: DataloggerSettings
}

impl DataLogger {
    pub fn new() -> Self {
        DataLogger {
            settings: DataloggerSettings::new()
        }
    }

    extern "C" fn test_exec(buf: *const i8) {
        let cmd_str = unsafe { from_c_str(buf) };
        rtt_target::rprintln!("command executed! {}", cmd_str);
    }

    extern "C" fn unknown_command(buf: *const i8) {
        let cmd_str = unsafe { from_c_str(buf) };
        rtt_target::rprintln!("unknown command or invalid json. {}", cmd_str);
    }

    fn retrieve_settings(&self, board: &mut impl RRIVBoard) -> DataloggerSettings {
        let mut bytes: [u8;EEPROM_DATALOGGER_SETTINGS_SIZE] = [b'\0'; EEPROM_DATALOGGER_SETTINGS_SIZE];
        board.retrieve_datalogger_settings(&mut bytes);
        return DataloggerSettings::new_from_bytes(& bytes);
        // convert the bytes pack into a DataloggerSettings
    }

    fn store_settings(&self, board: &mut impl RRIVBoard){
        let bytes: &[u8] = unsafe { any_as_u8_slice(&self.settings) };
        let mut bytes_sized: [u8;EEPROM_DATALOGGER_SETTINGS_SIZE] = [0; EEPROM_DATALOGGER_SETTINGS_SIZE];
        bytes_sized.copy_from_slice(&bytes[0..EEPROM_DATALOGGER_SETTINGS_SIZE]);
        board.store_datalogger_settings(&bytes_sized);
    }

    pub fn setup(&mut self, board: &mut impl RRIVBoard) {

        self.settings = self.retrieve_settings(board);
        // setup each service
        command_service::setup(board);
        // self.command_service
        //     .register_command("datalogger", "set", Self::test_exec);
        // self.command_service
        //     .register_command("unknown", "unknown", Self::unknown_command);
    }

    pub fn run_loop_iteration(&mut self, board: &mut impl RRIVBoard) {
        // todo: refactor to use Result<T,E>
        let get_command_result = command_service::get_pending_command(board);
        if let Some(get_command_result) = get_command_result {
            match get_command_result {
                Ok(command_payload) => {
                    self.executeCommand(board, command_payload);
                },
                Err(error) => {
                    board.serial_send("Error processing command");
                    // board.serial_send(error); // TODO how to get the string from the error

                    // CommandPayload::InvalidPayload() => {
                    //     board.serial_send("invalid payload\n");
                    // }
                    // CommandPayload::UnrecognizedCommand() => {
                    //     board.serial_send("unrecognized command\n");
                    // }
                }
            }
        }
    }
    
    
    pub fn executeCommand(&mut self, board: &mut impl RRIVBoard, command_payload: CommandPayload) {
        match command_payload {
            CommandPayload::SetCommandPayload(payload) => {
                self.update_datalogger_settings(board, payload);
                board.serial_send("updated datalogger settings\n");
            },
            CommandPayload::GetCommandPayload(_) => todo!(),
           
        }
    }

    pub fn update_datalogger_settings(&mut self, board: &mut impl RRIVBoard, set_command_payload: DataloggerSetCommandPayload) {
        if let Some(interval) = set_command_payload.interval {
            self.settings.interval = interval;
        }
        self.store_settings(board)
    }

}

use core::{str, future::pending};

unsafe fn from_c_str<'a>(ptr: *const i8) -> &'a str {
    let mut len = 0;
    while *ptr.offset(len) != 0 {
        len += 1;
    }
    let slice = core::slice::from_raw_parts(ptr as *const u8, len as usize);
    str::from_utf8_unchecked(slice)
}
