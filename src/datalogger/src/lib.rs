#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]


mod command_service;
mod datalogger_commands;
use core::panic;

use datalogger_commands::*;
use rriv_board::{RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE};
use bitflags::bitflags;
extern crate alloc;
use alloc::format;
use rtt_target::rprintln;
use alloc::boxed::Box;

mod drivers;
use drivers::*;

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

/* start registry WIP */

const SENSOR_NAMES: [&str; 4] = [
    "no_match",
    "generic_analog",
    "atlas_ec",
    "aht22"
];

fn sensor_type_id_from_name( name: &str) -> usize {
    for i in 0..SENSOR_NAMES.len() {
        if name == SENSOR_NAMES[i] {
            return i
        }
    }
    return 0;
}


type SpecialSettingsSlice = [u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE];

#[macro_export]
macro_rules!  driver_create_function {
    ($driver:ty) => {
        |settings: SensorDriverGeneralConfiguration, special_settings_slice: &SpecialSettingsSlice | -> Box<dyn SensorDriver> { 
            let driver = <$driver>::new(settings, special_settings_slice);
            Box::new(driver)
        }
    };
}

fn get_registry() -> [ Option<fn(SensorDriverGeneralConfiguration,  &SpecialSettingsSlice) -> Box<dyn SensorDriver>>; 256 ] {

    const ARRAY_INIT_VALUE: Option<fn(SensorDriverGeneralConfiguration,  &SpecialSettingsSlice) -> Box<dyn SensorDriver>  > = None;
    let mut driver_create_functions: [ Option<fn(SensorDriverGeneralConfiguration,  &SpecialSettingsSlice) -> Box<dyn SensorDriver> >; 256] = [ARRAY_INIT_VALUE; 256];
    driver_create_functions[0] = Some( driver_create_function!(GenericAnalog) );
    driver_create_functions[1] = None;
    driver_create_functions[2] = Some( driver_create_function!(AHT22) );

    driver_create_functions
}


/* end registry WIP */


pub struct DataLogger {
    settings: DataloggerSettings,
    driver_array: [Option<Box<dyn SensorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],



    debug_values: bool,  // serial out of values as they are read
    log_raw_data: bool,  // both raw and summary data writting to storage
}

impl DataLogger {
    pub fn new() -> Self {
        const ARRAY_INIT_VALUE: core::option::Option<Box<dyn drivers::SensorDriver>> = None;
        DataLogger {
            settings: DataloggerSettings::new(),
            driver_array: [ARRAY_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            debug_values: true,
            log_raw_data: true,
        }
    }
        
    fn retrieve_settings(&self, board: &mut impl RRIVBoard) -> DataloggerSettings {
        let mut bytes: [u8;EEPROM_DATALOGGER_SETTINGS_SIZE] = [b'\0'; EEPROM_DATALOGGER_SETTINGS_SIZE];
        board.retrieve_datalogger_settings(&mut bytes);
        rprintln!("retrieved {:?}", bytes);
        let mut settings: DataloggerSettings = DataloggerSettings::new_from_bytes(& bytes);         // convert the bytes pack into a DataloggerSettings

        settings.configure_defaults();
        let mut settings: DataloggerSettings = DataloggerSettings::new_from_bytes(& bytes);         // convert the bytes pack into a DataloggerSettings

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

        // enable power to the eeprom and bring i2bufferc online
    
        rprintln!("retrieving settings");
        self.settings = self.retrieve_settings(board);
        rprintln!("retrieved settings {:?}", self.settings);

        // setup each service
        command_service::setup(board);


        self.settings = DataloggerSettings::new();
        self.settings.deployment_identifier = [b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e',b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e'];
        self.settings.logger_name = [b'n',b'a',b'm',b'e',b'e',b'e',b'e',b'e'];
        rprintln!("attempting to store settings for test purposes");
        self.store_settings(board);


        // read all the sensors from EEPROMSELF
        let registry = get_registry();
        let mut sensor_config_bytes: [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS] = [0; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS];
        board.retrieve_sensor_settings(&mut sensor_config_bytes);
        for i in 0..rriv_board::EEPROM_TOTAL_SENSOR_SLOTS {
            let base = i*rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;
            let general_settings_slice = &sensor_config_bytes[base..(base + rriv_board::EEPROM_SENSOR_SETTINGS_SIZE/2)];
            let special_settings_slice: &[u8] = &sensor_config_bytes[(base + rriv_board::EEPROM_SENSOR_SETTINGS_SIZE/2)..(i+1)*rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];

            let settings: SensorDriverGeneralConfiguration = SensorDriverGeneralConfiguration::new_from_bytes(general_settings_slice);         // convert the bytes pack into a DataloggerSettings
            
            let create_function = registry[usize::from(settings.sensor_type_id)];
            if let Some(function) = create_function {
                let mut s:[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE] = [0; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE];
                s.clone_from_slice(special_settings_slice);
                let driver = function(settings, &s);
                self.driver_array[i] = Some(driver);
            }
    
        }
       
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
            CommandPayload::DataloggerSetCommandPayload(payload) => {
                self.update_datalogger_settings(board, payload);
                board.serial_send("updated datalogger settings\n");
            },
            CommandPayload::DataloggerGetCommandPayload(_) => todo!(),
            CommandPayload::SensorSetCommandPayload(_) => {
                
                // look for id in sensor list
                // if id is not present, insert
                // if present, update

            },
            CommandPayload::SensorGetCommandPayload(_) => todo!(),
            CommandPayload::SensorRemoveCommandPayload(_) => todo!(),
            CommandPayload::SensorListCommandPayload(_) => todo!(),
           
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



