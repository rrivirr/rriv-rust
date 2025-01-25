#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]

mod command_service;
mod datalogger_commands;

use bitflags::bitflags;
use datalogger_commands::*;
use ds18b20::{Ds18b20, Ds18b20SpecialConfiguration};
use ring_temperature::{RingTemperatureDriver, RingTemperatureDriverSpecialConfiguration};
use rriv_board::{
    RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE, EEPROM_SENSOR_SETTINGS_SIZE,
    EEPROM_TOTAL_SENSOR_SLOTS,
};
extern crate alloc;
use alloc::boxed::Box;
use alloc::format;
use rtt_target::rprintln;

mod drivers;
use crate::alloc::string::ToString;
use drivers::{*,types::*};
use mcp9808::*;
use generic_analog::*;

use serde::de::value;
use serde_json::{json, Value};

use core::{ffi::CStr, str::from_utf8};

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

const SENSOR_NAMES: [&str; 7] = [
    "no_match",
    "generic_analog",
    "atlas_ec",
    "aht22",
    "mcp_9808",
    "ring_temperature",
    "ds18b20"
];

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
    fn(
        SensorDriverGeneralConfiguration,
        serde_json::Value,
    ) -> (Box<dyn SensorDriver>, [u8; SENSOR_SETTINGS_PARTITION_SIZE]),
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
    driver_create_functions[3] = None;
    driver_create_functions[4] = Some(driver_create_functions!(
        MCP9808TemperatureDriver,
        MCP9808TemperatureDriverSpecialConfiguration
    )); //distinct settings? characteristic settings?
    driver_create_functions[5] = Some(driver_create_functions!(
        RingTemperatureDriver,
        RingTemperatureDriverSpecialConfiguration
    ));
    driver_create_functions[6] = Some(driver_create_functions!(
        Ds18b20,
        Ds18b20SpecialConfiguration
    ));
    // driver_create_functions[2] = Some(driver_create_function!(AHT22));

    driver_create_functions
}

/* end registry WIP */

pub enum DataLoggerMode {
    Interactive,
    Watch,
    Quiet
}


pub struct DataLogger {
    settings: DataloggerSettings,
    sensor_drivers: [Option<Box<dyn SensorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    actuator_drivers: [Option<Box<dyn ActuatorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    telemeter_drivers: [Option<Box<dyn TelemeterDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],

    last_interactive_log_time: i64,

    debug_values: bool, // serial out of values as they are read
    log_raw_data: bool, // both raw and summary data writting to storage

    mode: DataLoggerMode,
}
const SENSOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::SensorDriver>> = None;
const ACTUATOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::ActuatorDriver>> = None;
const TELEMETER_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::TelemeterDriver>> = None;

impl DataLogger {
    pub fn new() -> Self {
        DataLogger {
            settings: DataloggerSettings::new(),
            sensor_drivers: [SENSOR_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            actuator_drivers: [ACTUATOR_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            telemeter_drivers: [TELEMETER_DRIVER_INIT_VALUE; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
            last_interactive_log_time: 0,
            debug_values: true,
            log_raw_data: true,
            mode: DataLoggerMode::Interactive
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
        //todo: refactor to use Result<T,E>
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
        // implement interactive mode logging first

        let interactive_mode_logging = true;
        if interactive_mode_logging {
            if board.timestamp() > self.last_interactive_log_time + 1 {
                // notify(F("interactive log"));
                self.measure_sensor_values(board); // measureSensorValues(false);
                self.write_last_measurement_to_serial(board); //outputLastMeasurement();
                                                              // Serial2.print(F("CMD >> "));
                                                              // writeRawMeasurementToLogFile();
                                                              // fileSystemWriteCache->flushCache();
                self.last_interactive_log_time = board.timestamp();
            }
        }
    }

    fn measure_sensor_values(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {;
                driver.take_measurement(board.get_sensor_driver_services());
            }
        }
    }

    fn write_column_headers_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {

        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                let sensor_name = driver.get_id(); // always output the id for now, later add bit to control append prefix behavior, default to false
                                                   // let mut prefix: &str = "";
                                                   // if let Some(sensor_name) = sensor_name {
                let mut prefix = core::str::from_utf8(&sensor_name.clone())
                    .unwrap()
                    .to_string();
                prefix = (prefix + "_").clone();
                // }
                for i in 0..driver.get_measured_parameter_count() {
                    let identifier = driver.get_measured_parameter_identifier(i);
                    let identifier_str = core::str::from_utf8(&identifier).unwrap();
                    board.serial_send(&prefix);
                    let end = identifier.iter().position(|&x| x == b'\0').unwrap_or_else(|| 1);
                    let var = &identifier_str[ 0..end];
                    board.serial_send(var);
                    if i != driver.get_measured_parameter_count() - 1 {
                        board.serial_send(",");
                    }
                }
                board.serial_send("\n");
            }
        }

    }

    fn write_measured_parameters_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                for i in 0..driver.get_measured_parameter_count() {
                    let value = driver.get_measured_parameter_value(i);
                    let output = format!("{:.4}", value);
                    rprintln!("{}",value);
                    board.serial_send(&output);
                    if i != driver.get_measured_parameter_count() - 1 {
                        board.serial_send(",");
                    }
                }
                board.serial_send("\n");
            }
        }
    }


    fn write_last_measurement_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {

        // first output the column headers
        match self.mode {
            DataLoggerMode::Interactive => self.write_column_headers_to_serial(board),
            _ => {},
        }

        // then output the last measurement values
        match self.mode {
            DataLoggerMode::Interactive | DataLoggerMode::Watch => self.write_measured_parameters_to_serial(board),
            _ => {}
        }
        
    }

    pub fn execute_command(&mut self, board: &mut impl RRIVBoard, command_payload: CommandPayload) {
        match command_payload {
            CommandPayload::DataloggerSetCommandPayload(payload) => {
                self.update_datalogger_settings(board, payload);
                board.serial_send("updated datalogger settings\n");
            }
            CommandPayload::DataloggerGetCommandPayload(_) => {
                board.serial_send("get datalogger settings not implemented\n");
            },
            CommandPayload::DataloggerSetModeCommandPayload(payload) => {

                if let Some(mode) = payload.mode {
                    match mode {
                        Value::String(mode) => {
                            let mode = mode.as_str();
                            match mode {
                                "watch" => { 
                                    self.write_column_headers_to_serial(board);
                                    self.mode = DataLoggerMode::Watch
                                },
                                "quiet" => self.mode = DataLoggerMode::Quiet,
                                _ => self.mode = DataLoggerMode::Interactive
                            }
                        }
                        _ => {}
                    }
                }

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
                    } // Ok(sensor_type) => sensor_type_id_from_name(&sensor_type),
                      // Err(_) => 0,
                };

                if sensor_type_id == 0 {
                    board.serial_send("{\"error\": \"sensor type not found\"}");
                    return;
                }

                let mut sensor_id: [u8; 6] = [b'0'; 6]; // base default value
                let mut id_provided: bool = false;

                if let Some(payload_id) = payload.id {
                    sensor_id = match payload_id {
                        serde_json::Value::String(id) => {
                            let mut prepared_id: [u8; 6] = [0; 6];
                            prepared_id.copy_from_slice(id.as_bytes());
                            id_provided = true;
                            prepared_id
                        }
                        _ => {
                            // default to unique id
                            sensor_id
                        }
                    };
                }

                // create a new unique id
                if !id_provided {
                    // get all the current ids
                    let mut driver_ids: [[u8; 6]; EEPROM_TOTAL_SENSOR_SLOTS] =
                        [[0; 6]; EEPROM_TOTAL_SENSOR_SLOTS];
                    for i in 0..self.sensor_drivers.len() {
                        if let Some(driver) = &mut self.sensor_drivers[i] {
                            driver_ids[i] = driver.get_id();
                        }
                    }

                    let mut unique = false;
                    while unique == false {
                        let mut i = 0;
                        let mut changed = false;
                        let sensor_id_scan = sensor_id.clone();
                        while i < driver_ids.len() && changed == false {
                            let mut same = true;
                            for (j, (u1, u2)) in
                                driver_ids[i].iter().zip(sensor_id_scan.iter()).enumerate()
                            {
                                // if hey are equal..
                                if u1 != u2 {
                                    same = false;
                                    break;
                                }
                            }
                            if same {
                                // we need to make a change
                                sensor_id[sensor_id.len() - 1] = sensor_id[sensor_id.len() - 1] + 1;
                                changed = true;
                            }
                            i = i + 1;
                        }
                        unique = !changed;
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
                    let generic_settings_bytes: &[u8] =
                        unsafe { any_as_u8_slice(&general_settings) };
                    let mut bytes_sized: [u8; EEPROM_SENSOR_SETTINGS_SIZE] =
                        [0; EEPROM_SENSOR_SETTINGS_SIZE];
                    let copy_size =
                        if generic_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
                            SENSOR_SETTINGS_PARTITION_SIZE
                        } else {
                            generic_settings_bytes.len()
                        };
                    bytes_sized[..copy_size].copy_from_slice(&generic_settings_bytes[0..copy_size]);

                    // get the special settings as bytes
                    let copy_size =
                        if special_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
                            SENSOR_SETTINGS_PARTITION_SIZE
                        } else {
                            special_settings_bytes.len()
                        };
                    bytes_sized[SENSOR_SETTINGS_PARTITION_SIZE
                        ..(SENSOR_SETTINGS_PARTITION_SIZE + copy_size)]
                        .copy_from_slice(&special_settings_bytes[0..copy_size]);

                    board.store_sensor_settings(slot.try_into().unwrap(), &bytes_sized);
                    board.serial_send("updated sensor configuration\n");
                }
            }
            CommandPayload::SensorGetCommandPayload(payload) => {
                board.serial_send("get sensor settings not implemented\n");
            }
            CommandPayload::SensorRemoveCommandPayload(payload) => {
                let sensor_id = match payload.id {
                    serde_json::Value::String(id) => {
                        let mut prepared_id: [u8; 6] = [0; 6];
                        prepared_id.copy_from_slice(id.as_bytes());
                        prepared_id
                    }
                    _ => {
                        board.serial_send("Sensor not found");
                        return;
                    }
                };

                for i in 0..self.sensor_drivers.len() {
                    if let Some(driver) = &mut self.sensor_drivers[i] {
                        let mut found = i;
                        for (j, (u1, u2)) in
                            driver.get_id().iter().zip(sensor_id.iter()).enumerate()
                        {
                            if u1 != u2 {
                                found = 256; // 256 mneans not found
                                break;
                            }
                        }
                        if usize::from(found) < EEPROM_TOTAL_SENSOR_SLOTS {
                            // remove the sensor driver and write null to EEPROM
                            let bytes: [u8; EEPROM_SENSOR_SETTINGS_SIZE] =
                                [0; EEPROM_DATALOGGER_SETTINGS_SIZE];
                            if let Some(found_u8) = found.try_into().ok() {
                                board.store_sensor_settings(found_u8, &bytes);
                                self.sensor_drivers[found] = None;
                                board.serial_send("Sensor removed\n");
                                return;
                            }
                        }
                    }
                }
            }
            CommandPayload::SensorListCommandPayload(_) => {
                board.serial_send("{");
                for i in 0..self.sensor_drivers.len() {
                    // create json and output it
                    let test = &self.sensor_drivers[i];
                    if let Some(driver) = &mut self.sensor_drivers[i] {
                        let id_bytes = driver.get_id();
                        // let id = "id";
                        // let id = core::str::from_utf8_unchecked(&id_bytes);

                        // let id_cstr = CStr::from_bytes_with_nul(&id_bytes).unwrap_or_default();
                        // let id_str = id_cstr.to_str().unwrap_or_default();

                        let id_str = core::str::from_utf8(&id_bytes).unwrap_or_default();

                        let type_id = driver.get_type_id();
                        let json = json!({
                            "id": id_str,
                            "type": sensor_name_from_type_id(type_id.into())
                        });
                        let string = json.to_string();
                        let str = string.as_str();
                        board.serial_send(str);
                        board.serial_send("\n");
                    }
                }
                board.serial_send("}");
            }
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
