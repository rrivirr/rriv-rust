#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]

mod command_service;
mod datalogger_commands;

use bitflags::bitflags;
use datalogger_commands::*;
use rriv_board::{RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE, EEPROM_SENSOR_SETTINGS_SIZE};
extern crate alloc;
use alloc::boxed::Box;
use alloc::format;
use rtt_target::rprintln;

mod drivers;
use drivers::*;
use serde::de::value;
use serde_json::json;
use crate::alloc::string::ToString;
use alloc::string::String;


use core::str::from_utf8;

const DATALOGGER_SETTINGS_UNUSED_BYTES: usize = 16;

#[derive(Debug, Clone, Copy)]
struct DataloggerSettings {
    deployment_identifier: [u8; 16],
    logger_name: [u8; 8],
    site_name: [u8; 8],
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
    reserved: [u8; DATALOGGER_SETTINGS_UNUSED_BYTES],
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
            mode: b'i',
            reserved: [b'\0'; DATALOGGER_SETTINGS_UNUSED_BYTES],
        };
        let deployment_identifier_default = "bcdefghijklmnopq".as_bytes();
        settings
            .deployment_identifier
            .copy_from_slice(deployment_identifier_default);

        let logger_name_types = "MyLogger".as_bytes();
        settings.logger_name.copy_from_slice(logger_name_types);
        settings
    }

    pub fn new_from_bytes(bytes: [u8; EEPROM_DATALOGGER_SETTINGS_SIZE]) -> DataloggerSettings {
        let settings = bytes.as_ptr().cast::<DataloggerSettings>();
        unsafe { *settings }
    }

    pub fn configure_defaults(&mut self) {
        if self.burst_repetitions == 0 || self.burst_repetitions > 20 {
            self.burst_repetitions = 1;
        }

        if self.delay_between_bursts > 300_u16 {
            self.delay_between_bursts = 0_u16
        }

        if self.interval > 60_u16 * 24_u16 {
            self.interval = 15_u16;
        }

        if self.start_up_delay > 60_u16 {
            self.start_up_delay = 0;
        }

        if !check_alphanumeric(&self.logger_name) {
            let default = "logger";
            self.logger_name[0..default.len()].clone_from_slice(default.as_bytes());
        }

        if !check_alphanumeric(&self.site_name) {
            let default = "site";
            self.site_name[0..default.len()].clone_from_slice(default.as_bytes());
        }

        if !check_alphanumeric(&self.deployment_identifier) {
            let default = "site";
            self.deployment_identifier[0..default.len()].clone_from_slice(default.as_bytes());
        }
    }
}

pub fn check_alphanumeric(array: &[u8]) -> bool {
    let checks = array.iter();
    let checks = checks.map(|x| (*x as char).is_alphanumeric());
    checks.fold(true, |acc, check| if !check || !acc { false } else { true })
}

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts((p as *const T) as *const u8, ::core::mem::size_of::<T>())
}

/* start registry WIP */

const SENSOR_NAMES: [&str; 4] = ["no_match", "generic_analog", "atlas_ec", "aht22"];

fn sensor_type_id_from_name(name: &str) -> u16 {
    for i in 0..SENSOR_NAMES.len() {
        if name == SENSOR_NAMES[i] {
            return u16::try_from(i).ok().unwrap();
        }
    }
    return 0;
}

fn sensor_name_from_type_id(id: usize) -> [u8; 16] {
    let mut rval = [b'\0'; 16];
    let name = SENSOR_NAMES[id].clone();
    rval[..name.len()].copy_from_slice(name.as_bytes());
    rval
}

#[macro_export]
macro_rules! driver_create_functions {
    ($driver:ty, $special_settings_type:ty) => {
        (
            |general_settings: SensorDriverGeneralConfiguration,
             special_settings_values: serde_json::Value|
             -> (Box<dyn SensorDriver>, [u8; SENSOR_SETTINGS_PARTITION_SIZE]) {
                let special_settings =
                    <$special_settings_type>::new_from_values(special_settings_values); // ok
                let driver = <$driver>::new(general_settings, special_settings); // seems ok
                
                let bytes: &[u8] = unsafe { any_as_u8_slice(&special_settings) }; // must be this one, maybe size comes back wrong
                if bytes.len() != SENSOR_SETTINGS_PARTITION_SIZE {
                    // special_settings_type does not confrm to expected size.  this is a development fault
                    rprintln!("{} is wrong size", "<$special_settings_type>");
                    // causes crash later.  other way to gracefully handle?
                }
                let mut bytes_sized: [u8; SENSOR_SETTINGS_PARTITION_SIZE] =
                    [0; SENSOR_SETTINGS_PARTITION_SIZE];
                let copy_size = if bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE { // this was supposed to make it safe...
                    SENSOR_SETTINGS_PARTITION_SIZE
                } else {
                    bytes.len()
                };
                bytes_sized[..SENSOR_SETTINGS_PARTITION_SIZE].copy_from_slice(&bytes[0..copy_size]);

                (Box::new(driver), bytes_sized)
            },
            |general_settings: SensorDriverGeneralConfiguration,
             special_settings_slice: &[u8]|
             -> Box<dyn SensorDriver> {
                let mut bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE] =
                    [0; SENSOR_SETTINGS_PARTITION_SIZE];
                bytes.clone_from_slice(special_settings_slice);
                let special_settings = <$special_settings_type>::new_from_bytes(bytes);
                let driver = <$driver>::new(general_settings, special_settings);
                Box::new(driver)
            },
        )
    };
}

type DriverCreateFunctions = Option<(
    fn(SensorDriverGeneralConfiguration, serde_json::Value) -> ( Box<dyn SensorDriver>, [u8; SENSOR_SETTINGS_PARTITION_SIZE] ),
    fn(SensorDriverGeneralConfiguration, &[u8]) -> Box<dyn SensorDriver>,
)>;
fn get_registry() -> [DriverCreateFunctions; 256] {
    const ARRAY_INIT_VALUE: DriverCreateFunctions = None;
    let mut driver_create_functions: [DriverCreateFunctions; 256] = [ARRAY_INIT_VALUE; 256];
    driver_create_functions[0] = None;
    driver_create_functions[1] = Some(driver_create_functions!(
        GenericAnalog,
        GenericAnalogSpecialConfiguration
    )); //distinct settings? characteristic settings?
    driver_create_functions[2] = None;
    // driver_create_functions[2] = Some(driver_create_function!(AHT22));

    driver_create_functions
}

/* end registry WIP */

pub struct DataLogger {
    settings: DataloggerSettings,
    sensor_drivers: [Option<Box<dyn SensorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    actuator_drivers: [Option<Box<dyn ActuatorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    telemeter_drivers: [Option<Box<dyn TelemeterDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],

    debug_values: bool, // serial out of values as they are read
    log_raw_data: bool, // both raw and summary data writting to storage
}
const SENSOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::SensorDriver>> = None;
const ACTUATOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::ActuatorDriver>> = None;
const TELEMETER_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::TelemeterDriver>> = None;

impl DataLogger {
    pub fn new() -> Self {
        DataLogger {
            settings: DataloggerSettings::new(),
            sensor_drivers: [SENSOR_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            actuator_drivers: [ACTUATOR_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            telemeter_drivers: [TELEMETER_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            debug_values: true,
            log_raw_data: true,
        }
    }

    fn retrieve_settings(&self, board: &mut impl RRIVBoard) -> DataloggerSettings {
        let mut bytes: [u8; EEPROM_DATALOGGER_SETTINGS_SIZE] =
            [b'\0'; EEPROM_DATALOGGER_SETTINGS_SIZE];
        board.retrieve_datalogger_settings(&mut bytes);
        rprintln!("retrieved {:?}", bytes);
        let mut settings: DataloggerSettings = DataloggerSettings::new_from_bytes(bytes); // convert the bytes pack into a DataloggerSettings

        settings.configure_defaults();

        settings
    }

    fn store_settings(&self, board: &mut impl RRIVBoard) {
        let bytes: &[u8] = unsafe { any_as_u8_slice(&self.settings) };
        let mut bytes_sized: [u8; EEPROM_DATALOGGER_SETTINGS_SIZE] =
            [0; EEPROM_DATALOGGER_SETTINGS_SIZE];
        let copy_size = if bytes.len() >= EEPROM_DATALOGGER_SETTINGS_SIZE {
            EEPROM_DATALOGGER_SETTINGS_SIZE
        } else {
            bytes.len()
        };
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
        self.settings.deployment_identifier = [
            b'n', b'a', b'm', b'e', b'e', b'e', b'e', b'e', b'n', b'a', b'm', b'e', b'e', b'e',
            b'e', b'e',
        ];
        self.settings.logger_name = [b'n', b'a', b'm', b'e', b'e', b'e', b'e', b'e'];
        rprintln!("attempting to store settings for test purposes");
        self.store_settings(board);

        // read all the sensors from EEPROM
        let registry = get_registry();
        let mut sensor_config_bytes: [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE
            * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS] =
            [0; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS];
        board.retrieve_sensor_settings(&mut sensor_config_bytes);
        for i in 0..rriv_board::EEPROM_TOTAL_SENSOR_SLOTS {
            let base = i * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;
            let general_settings_slice =
                &sensor_config_bytes[base..(base + SENSOR_SETTINGS_PARTITION_SIZE)];
            let special_settings_slice: &[u8] = &sensor_config_bytes[(base
                + SENSOR_SETTINGS_PARTITION_SIZE)
                ..(i + 1) * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];

            let mut s: [u8; SENSOR_SETTINGS_PARTITION_SIZE] = [0; SENSOR_SETTINGS_PARTITION_SIZE];
            s.clone_from_slice(general_settings_slice); // explicitly define the size of the array
            let settings: SensorDriverGeneralConfiguration =
                SensorDriverGeneralConfiguration::new_from_bytes(&s);

            let sensor_type_id = usize::from(settings.sensor_type_id);
            if sensor_type_id > registry.len() {
                continue;
            }
            let create_function = registry[usize::from(settings.sensor_type_id)];
            if let Some(functions) = create_function {
                let mut s: [u8; SENSOR_SETTINGS_PARTITION_SIZE] =
                    [0; SENSOR_SETTINGS_PARTITION_SIZE];
                s.clone_from_slice(special_settings_slice); // explicitly define the size of the arra
                let driver = functions.1(settings, &s);
                self.sensor_drivers[i] = Some(driver);
            }
        }
        rprintln!("done loading sensors");
    }

    pub fn run_loop_iteration(&mut self, board: &mut impl RRIVBoard) {
        // todo: refactor to use Result<T,E>
        let get_command_result = command_service::get_pending_command(board);
        if let Some(get_command_result) = get_command_result {
            match get_command_result {
                Ok(command_payload) => {
                    self.execute_command(board, command_payload);
                }
                Err(error) => {
                    board.serial_send("Error processing command");
                    board.serial_send(format!("{:?}", error).as_str()); // TODO how to get the string from the error

                    // CommandPayload::InvalidPayload() => {
                    //     board.serial_send("invalid payload\n");sensor_type_id
                    // }
                    // CommandPayload::UnrecognizedCommand() => {
                    //     board.serial_send("unrecognized command\n");
                    // }
                }
            }
        }

        // do the measurement cycle stuff
    }

    pub fn execute_command(&mut self, board: &mut impl RRIVBoard, command_payload: CommandPayload) {
        match command_payload {
            CommandPayload::DataloggerSetCommandPayload(payload) => {
                self.update_datalogger_settings(board, payload);
                board.serial_send("updated datalogger settings\n");
            }
            CommandPayload::DataloggerGetCommandPayload(_) => {
                board.serial_send("get datalogger settings not implemented\n");
            }
            CommandPayload::SensorSetCommandPayload(payload, values) => {
                let registry = get_registry();
                // let sensor_type: Result<&str, core::str::Utf8Error> = core::str::from_utf8(&payload.r#type);
                let sensor_type_id = match payload.r#type {
                    serde_json::Value::String(sensor_type) => {
                        sensor_type_id_from_name(&sensor_type)
                    }
                    _ => {
                        board.serial_send("{\"error\": \"sensor type not specified\"}");
                        return;
                    }
                    // Ok(sensor_type) => sensor_type_id_from_name(&sensor_type),
                    // Err(_) => 0,
                };

                if sensor_type_id == 0 {
                    board.serial_send("{\"error\": \"sensor type not found\"}");
                    return;
                }

                let mut sensor_id: [u8; 6] = [b'0'; 6]; // base default value
                let mut id_exists = false;

                if let Some(payload_id) = payload.id {
                    sensor_id = match payload_id {
                        serde_json::Value::String(id) => {
                            let mut prepared_id: [u8; 6] = [0; 6];
                            prepared_id.copy_from_slice(id.as_bytes());
                            id_exists = true;
                            prepared_id
                        }
                        _ => {
                            // default to unique id
                            sensor_id
                        }
                    };
                }

                // make sure it is unique
                if !id_exists {
                    for i in 0..self.sensor_drivers.len() {
                        if let Some(driver) = &mut self.sensor_drivers[i] {
                            let existing_id = driver.get_id();
                            for i in 0..existing_id.len() {
                                while existing_id[i] == sensor_id[i] {
                                    sensor_id[i] = sensor_id[i] + 1;
                                }
                            }
                        }
                    }
                }

                // find the slot
                let mut slot = usize::MAX;
                let mut empty_slot = usize::MAX;
                for i in 0..self.sensor_drivers.len() {
                    if let Some(driver) = &mut self.sensor_drivers[i] {
                        if sensor_id == driver.get_id() {
                            slot = i;
                        }
                    } else {
                        if empty_slot == usize::MAX {
                            empty_slot = i;
                        }
                    }
                }

                if slot == usize::MAX {
                    slot = empty_slot
                };

                rprintln!("looking up funcs");
                let create_function = registry[usize::from(sensor_type_id)];

                if let Some(functions) = create_function {
        
                    let general_settings =
                        SensorDriverGeneralConfiguration::new(sensor_id, sensor_type_id);
                    rprintln!("calling func 0"); // TODO: crashed here
                    let (driver, special_settings_bytes) = functions.0(general_settings, values); // could just convert values to special settings bytes directly, store, then load
                    self.sensor_drivers[slot] = Some(driver);

                    // get the generic settings as bytes
                    let generic_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&general_settings) };
                    let mut bytes_sized: [u8; EEPROM_SENSOR_SETTINGS_SIZE] =
                        [0; EEPROM_SENSOR_SETTINGS_SIZE];
                    let copy_size = if generic_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
                        SENSOR_SETTINGS_PARTITION_SIZE
                    } else {
                        generic_settings_bytes.len()
                    };
                    bytes_sized[..copy_size].copy_from_slice(&generic_settings_bytes[0..copy_size]);
                    

                    // get the special settings as bytes
                    let copy_size = if special_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
                        SENSOR_SETTINGS_PARTITION_SIZE
                    } else {
                        special_settings_bytes.len()
                    };
                    bytes_sized[SENSOR_SETTINGS_PARTITION_SIZE..(SENSOR_SETTINGS_PARTITION_SIZE+copy_size)].copy_from_slice(&special_settings_bytes[0..copy_size]);


                    board.store_sensor_settings(slot.try_into().unwrap(), &bytes_sized);
                    board.serial_send("updated sensor configuration\n");
                }
            }
            CommandPayload::SensorGetCommandPayload(_) => {
                board.serial_send("get sensor settings not implemented\n");
            }
            CommandPayload::SensorRemoveCommandPayload(_) => todo!(),
            CommandPayload::SensorListCommandPayload(_) => {

                board.serial_send("{");
                for i in 0..self.sensor_drivers.len() {
                    // create json and output it
                    let test = & self.sensor_drivers[i];
                    if let Some(driver) = &mut self.sensor_drivers[i] {

                        let id = driver.get_id();
                        let type_id = driver.get_type_id();
                        let json = json!({
                            "id": id,
                            "type": sensor_name_from_type_id(type_id.into())
                        });
                        let string = json.to_string();
                        let str = string.as_str();
                        board.serial_send(str);
                        board.serial_send("\n");
                    }
                }
                board.serial_send("}");
            },
        }
    }

    pub fn update_datalogger_settings(
        &mut self,
        board: &mut impl RRIVBoard,
        set_command_payload: DataloggerSetCommandPayload,
    ) {
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
                Ok(deployment_identifier) => {
                    self.settings.deployment_identifier = deployment_identifier
                }
                Err(_) => todo!()
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
