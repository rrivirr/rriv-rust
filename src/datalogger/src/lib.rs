#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]


mod command_service;
mod datalogger_commands;
use datalogger_commands::*;
use rriv_board::{RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE};
use bitflags::bitflags;
extern crate alloc;
use alloc::format;
use rtt_target::rprintln;


#[derive(Copy,Clone,Debug)]
struct DataloggerSettings {
    deployment_identifier: [u8;16],
    logger_name: [u8;8],
    site_name: [u8;8],
    deployment_timestamp: u64,
    interval: u16,
    start_up_delay: u16,
    delay_between_bursts: u16,
    burst_repetitions: u8,
    mode: u8,
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
            mode: b'i' 
        };
        let deployment_identifier_default = "bcdefghijklmnopq".as_bytes();
        settings.deployment_identifier.copy_from_slice(deployment_identifier_default);

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

    pub fn configure_defaults(&mut self) {
        
        if self.burst_repetitions == 0 || self.burst_repetitions > 20 {
            self.burst_repetitions = 1;
        }

        if self.delay_between_bursts > 300{
            self.delay_between_bursts = 0;
        }

        if self.interval > 60*24 {
            self.interval = 15;
        }

        if self.start_up_delay > 60 {
            self.start_up_delay = 0;
        }

        if !check_alphanumeric(&self.logger_name) {
            self.logger_name.clone_from_slice("logger".as_bytes());
        }

        if !check_alphanumeric(&self.site_name) {
            self.site_name.clone_from_slice("site".as_bytes());
        }

        if !check_alphanumeric(&self.deployment_identifier) {
            self.deployment_identifier.clone_from_slice("site".as_bytes());
        }
   

    }
}

pub fn check_alphanumeric( array: &[u8]) -> bool {
    let checks = array.iter();
    let checks = checks.map(|x| (*x as char).is_alphanumeric());
    checks.fold(true,
            |acc, check|
             if !check || !acc {false} else {true}
            )
}


unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts(
        (p as *const T) as *const u8,
        ::core::mem::size_of::<T>(),
    )
}


pub struct DataLogger {
    settings: DataloggerSettings,

    debug_values: bool,  // serial out of values as they are read
    log_raw_data: bool,  // both raw and summary data writting to storage
}

impl DataLogger {
    pub fn new() -> Self {
        DataLogger {
            settings: DataloggerSettings::new(),
            debug_values: true,
            log_raw_data: true,
        }
    }

    fn retrieve_settings(&self, board: &mut impl RRIVBoard) -> DataloggerSettings {
        let mut bytes: [u8;EEPROM_DATALOGGER_SETTINGS_SIZE] = [b'\0'; EEPROM_DATALOGGER_SETTINGS_SIZE];
        board.retrieve_datalogger_settings(&mut bytes);
        rprintln!("retrieved {:?}", bytes);
        let mut settings = DataloggerSettings::new_from_bytes(& bytes);         // convert the bytes pack into a DataloggerSettings

        settings.configure_defaults();

        settings
    }

    fn store_settings(&self, board: &mut impl RRIVBoard){
        let bytes: &[u8] = unsafe { any_as_u8_slice(&self.settings) };
        let mut bytes_sized: [u8;EEPROM_DATALOGGER_SETTINGS_SIZE] = [0; EEPROM_DATALOGGER_SETTINGS_SIZE];
        let copy_size = if bytes.len() >= EEPROM_DATALOGGER_SETTINGS_SIZE { EEPROM_DATALOGGER_SETTINGS_SIZE } else { bytes.len() };
        bytes_sized[..bytes.len()].copy_from_slice(&bytes[0..copy_size]);
        board.store_datalogger_settings(&bytes_sized);
        rprintln!("stored {:?}", bytes_sized);
    }

    pub fn setup(&mut self, board: &mut impl RRIVBoard) {

        // enable power to the eeprom and bring i2c online
    
        rprintln!("retrieving settings");
        self.settings = self.retrieve_settings(board);
        rprintln!("retrieved settings {:?}", self.settings);

        // setup each service
        command_service::setup(board);


        // self.settings = DataloggerSettings::new();
        // self.settings.deployment_identifier = [b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e',b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e'];
        // self.settings.logger_name = [b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e'];
        // rprintln!("attempting to store settings for test purposes");
        // self.store_settings(board);



  
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
                    board.serial_send(format!("{:?}", error).as_str()); // TODO how to get the string from the error

                    // CommandPayload::InvalidPayload() => {
                    //     board.serial_send("invalid payload\n");
                    // }
                    // CommandPayload::UnrecognizedCommand() => {
                    //     board.serial_send("unrecognized command\n");
                    // }
                }
            }
        }

        // do the measurement cycle stuff
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
        
        if let Some(logger_name) = set_command_payload.logger_name {
            match serde_json::from_value(logger_name) {
                Ok(logger_name) => self.settings.logger_name = logger_name,
                Err(_) => todo!(),
            }
        }

        if let Some(site_name) = set_command_payload.site_name {
            match serde_json::from_value(site_name) {
                Ok(site_name) => self.settings.site_name = site_name,
                Err(_) => todo!(),
            }
        }

        if let Some(deployment_identifier) = set_command_payload.deployment_identifier {
            match serde_json::from_value(deployment_identifier) {
                Ok(deployment_identifier) => self.settings.deployment_identifier = deployment_identifier,
                Err(_) => todo!(),
            }
        }

        if let Some(interval) = set_command_payload.interval {
            self.settings.interval = interval;
        }

        if let Some(burst_repetitions) = set_command_payload.burst_repetitions {
            self.settings.burst_repetitions = burst_repetitions;
        }

        if let Some(start_up_delay) = set_command_payload.start_up_delay {
            self.settings.start_up_delay = start_up_delay;
        }

        self.store_settings(board)
    }

}



