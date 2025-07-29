#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]

mod datalogger_commands;
mod services;

use datalogger_commands::*;
use ds18b20::{Ds18b20, Ds18b20SpecialConfiguration};
use heater::{Heater, HeaterSpecialConfiguration};
use ring_temperature::{RingTemperatureDriver, RingTemperatureDriverSpecialConfiguration};
use rriv_board::{
    RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE, EEPROM_SENSOR_SETTINGS_SIZE,
    EEPROM_TOTAL_SENSOR_SLOTS,
};
use util::{any_as_u8_slice, check_alphanumeric};
extern crate alloc;
use crate::{
    alloc::string::ToString, drivers::k30_co2::{K30CO2SpecialConfiguration, K30CO2}, protocol::responses, services::*, telemetry::telemeters::lorawan::RakWireless3172
};
use alloc::boxed::Box;
use alloc::format;
use rtt_target::rprintln;

mod drivers;
use drivers::{types::*, *};
use generic_analog::*;
use mcp9808::*;

mod protocol;
mod registry;
mod telemetry;
use registry::*;

use serde_json::{json, Value};

const DATALOGGER_SETTINGS_UNUSED_BYTES: usize = 16;

static mut EPOCH_TIMESTAMP: i64 = 0;

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
    // external_adc_enabled: u8:1, // how we we handle bitfields? -> bitfield_struct crate works well
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

/* start registry WIP */

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
                    rprintln!("{} is wrong size {}", "<$special_settings_type>", bytes.len());
                    // causes crash later.  other way to gracefully handle?
                    panic!();
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
    driver_create_functions[6] = Some(driver_create_functions!(Heater, HeaterSpecialConfiguration));
    driver_create_functions[7] = Some(driver_create_functions!(
        Ds18b20,
        Ds18b20SpecialConfiguration
    ));
     driver_create_functions[8] = Some(driver_create_functions!(
        K30CO2,
        K30CO2SpecialConfiguration
    ));
    // driver_create_functions[2] = Some(driver_create_function!(AHT22));

    driver_create_functions
}

/* end registry WIP */

pub enum DataLoggerMode {
    Interactive,
    Watch,
    Quiet,
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

    // naive calibration value book keeping
    // not memory efficient
    calibration_point_values: [Option<Box<[CalibrationPair]>>; EEPROM_TOTAL_SENSOR_SLOTS],

    telemeter: telemetry::telemeters::lorawan::RakWireless3172,
}

const SENSOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::SensorDriver>> = None;
const ACTUATOR_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::ActuatorDriver>> =
    None;
const TELEMETER_DRIVER_INIT_VALUE: core::option::Option<Box<dyn drivers::types::TelemeterDriver>> =
    None;
const CALIBRATION_REPEAT_VALUE: core::option::Option<Box<[types::CalibrationPair]>> = None;

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
            mode: DataLoggerMode::Interactive,
            calibration_point_values: [CALIBRATION_REPEAT_VALUE; EEPROM_TOTAL_SENSOR_SLOTS],
            telemeter: RakWireless3172::new(),
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

    fn get_driver_index_by_id(&self, id: &str) -> Option<usize> {
        let drivers = &self.sensor_drivers;
        for i in 0..self.sensor_drivers.len() {
            // create json and output it
            if let Some(driver) = &drivers[i] {
                let id_bytes = driver.get_id();
                let id_str = core::str::from_utf8(&id_bytes).unwrap_or_default();
                if id == id_str {
                    return Some(i);
                }
            }
        }
        return None;
    }

    pub fn setup(&mut self, board: &mut impl RRIVBoard) {
        // enable power to the eeprom and bring i2bufferc online

        // rprintln!("retrieving settings");
        self.settings = self.retrieve_settings(board);
        // rprintln!("retrieved settings {:?}", self.settings);

        // setup each service
        command_service::setup(board);
        usart_service::setup(board);

        // For smaller EERPOM, this would overwrite sensor drivers config
        // self.settings = DataloggerSettings::new();
        // self.settings.deployment_identifier = [
        //     b'n', b'a', b'm', b'e', b'e', b'e', b'e', b'e', b'n', b'a', b'm', b'e', b'e', b'e',
        //     b'e', b'e',
        // ];
        // self.settings.logger_name = [b'n', b'a', b'm', b'e', b'e', b'e', b'e', b'e'];
        // rprintln!("attempting to store settings for test purposes");
        // self.store_settings(board);

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
                let mut driver = functions.1(settings, &s);
                driver.setup();
                self.sensor_drivers[i] = Some(driver);
            }
        }
        rprintln!("done loading sensors");

        self.write_column_headers_to_storage(board);
        rprintln!("done with setup");
    }

    pub fn run_loop_iteration(&mut self, board: &mut impl RRIVBoard) {
        //
        // Process incoming commands
        //

        //todo: refactor to use Result<T,E>
        let get_command_result = command_service::get_pending_command(board);
        if let Some(get_command_result) = get_command_result {
            match get_command_result {
                Ok(command_payload) => {
                    self.execute_command(board, command_payload);
                }
                Err(error) => {
                    responses::send_command_response_error(
                        board,
                        "Error processing command",
                        format!("{:?}\n", error).as_str(),
                    );
                }
            }
        }

        //
        //  Process any telemetry setup or QOS
        //
        self.telemeter.run_loop_iteration(board);

        //
        // Do the measurement cycle
        //
        //
        // implement interactive mode logging first

        self.update_actuators(board);
        let interactive_mode_logging = true;
        if interactive_mode_logging {
            if board.timestamp() > self.last_interactive_log_time + 1 {
                // need to separate logic here.
                // notify(F("interactive log"));
                unsafe {
                    EPOCH_TIMESTAMP = board.epoch_timestamp();
                }
                self.measure_sensor_values(board); // measureSensorValues(false);
                self.write_last_measurement_to_serial(board); //outputLastMeasurement();
                                                              // Serial2.print(F("CMD >> "));
                                                              // writeRawMeasurementToLogFile();
                                                              // fileSystemWriteCache->flushCache();
                self.write_last_measurement_to_storage(board);

                self.process_telemetry(board); // telemeterize the measured values

                self.last_interactive_log_time = board.timestamp();
            }
        }
    }

    fn process_telemetry(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        self.telemeter.process_events(board);

        if !self.telemeter.ready_to_transmit(board) {
            return;
        }

        // TODO: this book-keeping to get the sensor values is not correct / robust / fully functional
        let mut values: [f32; 12] = [0_f32; 12];
        let mut bits: [u8; 12] = [0_u8; 12];
        let mut j = 0; // index of value into the values array
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                // TODO: iterate values
                match driver.get_measured_parameter_value(0) {
                    Ok(value) => {
                        values[j] = value as f32;
                        j = j + 1;
                    }
                    Err(_) => rprintln!("error reading value"),
                }

                // TODO: returning bits instead of full f64 is a way to use less space in the payload
                //  Bits is a number of bits that should be used when encoding.
                // match driver.get_measured_parameters_bits(0){
                //     Ok(bits) => bits[i] = bits,
                //     Err(_) => todo!(),
                // }
            }
        }

        let timestamp_hour_offset = 0; // TODO get the timestamp offset from the beginning of the utc hour
        let bits = [13_u8; 12];

        let payload = telemetry::codecs::naive_codec::encode(board.epoch_timestamp(), &values);

        // stateful deltas codec
        // let payload = telemetry::codecs::first_differences_codec::encode(timestamp_hour_offset, values, bits);let p

        self.telemeter.transmit(board, &payload);
    }

    fn measure_sensor_values(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                driver.take_measurement(board.get_sensor_driver_services());
            }
        }
    }

    fn update_actuators(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                driver.update_actuators(board.get_sensor_driver_services());
            }
        }
    }

    fn write_column_headers_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        board.usb_serial_send("timestamp,");

        let mut first = true;
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                if first {
                    first = false;
                } else {
                    board.usb_serial_send(",");
                }

                let sensor_name = driver.get_id(); // always output the id for now, later add bit to control append prefix behavior, default to false
                                                   // let mut prefix: &str = "";
                                                   // if let Some(sensor_name) = sensor_name {
                let mut prefix = core::str::from_utf8(&sensor_name.clone())
                    .unwrap()
                    .to_string();
                prefix = (prefix + "_").clone();
                // }
                for j in 0..driver.get_measured_parameter_count() {
                    let identifier = driver.get_measured_parameter_identifier(j);
                    let identifier_str = core::str::from_utf8(&identifier).unwrap();
                    board.usb_serial_send(&prefix);
                    let end = identifier
                        .iter()
                        .position(|&x| x == b'\0')
                        .unwrap_or_else(|| 1);
                    let var = &identifier_str[0..end];
                    board.usb_serial_send(var);
                    if j != driver.get_measured_parameter_count() - 1 {
                        board.usb_serial_send(",");
                    }
                }
            }
        }
        board.usb_serial_send("\n");
    }

    fn write_column_headers_to_storage(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        board.write_log_file("timestamp,");

        let mut first = true;
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                if first {
                    first = false;
                } else {
                    board.write_log_file(",");
                }

                let sensor_name = driver.get_id(); // always output the id for now, later add bit to control append prefix behavior, default to false
                                                   // let mut prefix: &str = "";
                                                   // if let Some(sensor_name) = sensor_name {
                let mut prefix = core::str::from_utf8(&sensor_name.clone())
                    .unwrap()
                    .to_string();
                prefix = (prefix + "_").clone();
                // }
                for j in 0..driver.get_measured_parameter_count() {
                    let identifier = driver.get_measured_parameter_identifier(j);
                    let identifier_str = core::str::from_utf8(&identifier).unwrap();
                    board.write_log_file(&prefix);
                    let end = identifier
                        .iter()
                        .position(|&x| x == b'\0')
                        .unwrap_or_else(|| 1);
                    let var = &identifier_str[0..end];
                    board.write_log_file(var);
                    if j != driver.get_measured_parameter_count() - 1 {
                        board.write_log_file(",");
                    }
                }
            }
        }
        board.write_log_file("\n");
    }

    fn write_measured_parameters_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        let epoch = board.epoch_timestamp();
        let output = format!("{},", epoch);
        board.usb_serial_send(&output);

        let mut first = true;
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                if first {
                    first = false;
                } else {
                    board.usb_serial_send(",");
                }

                for j in 0..driver.get_measured_parameter_count() {
                    match driver.get_measured_parameter_value(j) {
                        Ok(value) => {
                            let output = format!("{:.4}", value);
                            rprintln!("{}", value);
                            board.usb_serial_send(&output);
                        }
                        Err(_) => {
                            rprintln!("{}", "Error");
                            board.usb_serial_send("Error");
                        }
                    }

                    if j != driver.get_measured_parameter_count() - 1 {
                        board.usb_serial_send(",");
                    }
                }
            }
        }
        board.usb_serial_send("\n");
    }

    fn write_last_measurement_to_storage(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        let epoch = board.epoch_timestamp();
        let output = format!("{},", epoch);
        board.write_log_file(&output);

        let mut first = true;
        for i in 0..self.sensor_drivers.len() {
            if let Some(ref mut driver) = self.sensor_drivers[i] {
                if first {
                    first = false;
                } else {
                    board.write_log_file(",");
                }

                for j in 0..driver.get_measured_parameter_count() {
                    match driver.get_measured_parameter_value(j) {
                        Ok(value) => {
                            let output = format!("{:.4}", value);
                            rprintln!("{}", value);
                            board.write_log_file(&output);
                        }
                        Err(_) => {
                            rprintln!("{}", "Error");
                            board.write_log_file("Error");
                        }
                    }

                    if j != driver.get_measured_parameter_count() - 1 {
                        board.write_log_file(",");
                    }
                }
            }
        }
        board.write_log_file("\n");
    }

    fn write_last_measurement_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        // first output the column headers
        match self.mode {
            DataLoggerMode::Interactive => self.write_column_headers_to_serial(board),
            _ => {}
        }

        // then output the last measurement values
        match self.mode {
            DataLoggerMode::Interactive | DataLoggerMode::Watch => {
                self.write_measured_parameters_to_serial(board)
            }
            _ => {}
        }
    }

    pub fn execute_command(&mut self, board: &mut impl RRIVBoard, command_payload: CommandPayload) {
        // rprintln!("executing command {:?}", command_payload);
        match command_payload {
            CommandPayload::DataloggerSetCommandPayload(payload) => {
                        self.update_datalogger_settings(board, payload);
                        board.usb_serial_send("updated datalogger settings\n");
                    }
            CommandPayload::DataloggerGetCommandPayload(_) => {
                        board.usb_serial_send("get datalogger settings not implemented\n");
                    }
            CommandPayload::DataloggerSetModeCommandPayload(payload) => {
                        if let Some(mode) = payload.mode {
                            match mode {
                                Value::String(mode) => {
                                    let mode = mode.as_str();
                                    match mode {
                                        "watch" => {
                                            self.write_column_headers_to_serial(board);
                                            self.mode = DataLoggerMode::Watch;
                                            board.set_debug(false);
                                            self.telemeter.set_watch(true);
                                        }
                                        "watch-debug" => {
                                            self.write_column_headers_to_serial(board);
                                            self.mode = DataLoggerMode::Watch;
                                            board.set_debug(true);
                                            self.telemeter.set_watch(true);
                                        }
                                        "quiet" => {
                                            self.mode = DataLoggerMode::Quiet;
                                            board.set_debug(false);
                                            self.telemeter.set_watch(false);
                                        }
                                        _ => {
                                            self.mode = DataLoggerMode::Interactive;
                                            board.set_debug(true);
                                            self.telemeter.set_watch(false);
                                        }
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
                                registry::sensor_type_id_from_name(&sensor_type)
                            }
                            _ => {
                                board.usb_serial_send("{\"error\": \"sensor type not specified\"}");
                                return;
                            } // Ok(sensor_type) => sensor_type_id_from_name(&sensor_type),
                              // Err(_) => 0,
                        };

                        if sensor_type_id == 0 {
                            board.usb_serial_send("{\"error\": \"sensor type not found\"}");
                            return;
                        }

                        let mut sensor_id: [u8; 6] = [b'0'; 6]; // base default value
                        let mut id_provided: bool = false;

                        if let Some(payload_id) = payload.id {
                            sensor_id = match payload_id {
                                serde_json::Value::String(id) => {
                                    let mut prepared_id: [u8; 6] = [0; 6];
                                    prepared_id[0..id.len()].copy_from_slice(id.as_bytes());
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
                            let (mut driver, special_settings_bytes) =
                                functions.0(general_settings, values); // could just convert values to special settings bytes directly, store, then load
                            driver.setup();
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
                            board.usb_serial_send("updated sensor configuration\n");
                        }
                    }
            CommandPayload::SensorGetCommandPayload(payload) => {
                        for i in 0..self.sensor_drivers.len() {
                            // create json and output it
                            if let Some(driver) = &mut self.sensor_drivers[i] {
                                let id_bytes = driver.get_id();
                                let id_str = core::str::from_utf8(&id_bytes).unwrap_or_default();

                                match payload.id {
                                    serde_json::Value::String(ref payload_id) => {
                                        if payload_id == id_str {
                                            //echo the details and return

                                            let configuration_payload = driver.get_configuration_json();

                                            // TODO: get the other sensor values

                                            let string = configuration_payload.to_string();
                                            let str = string.as_str();
                                            board.usb_serial_send(str);
                                            board.usb_serial_send("\n");
                                            return;
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }

                        responses::send_command_response_message(board, "didn't find the sensor");
                    }
            CommandPayload::SensorRemoveCommandPayload(payload) => {
                        let sensor_id = match payload.id {
                            serde_json::Value::String(id) => {
                                let mut prepared_id: [u8; 6] = [0; 6];
                                prepared_id.copy_from_slice(id.as_bytes());
                                prepared_id
                            }
                            _ => {
                                responses::send_command_response_message(board, "Sensor not found");
                                return;
                            }
                        };

                        for i in 0..self.sensor_drivers.len() {
                            if let Some(driver) = &mut self.sensor_drivers[i] {
                                let mut found = i;

                                // bytewise comparison of sensor id to delete with sensor id of loaded sensor driver
                                for (j, (u1, u2)) in
                                    driver.get_id().iter().zip(sensor_id.iter()).enumerate()
                                {
                                    if u1 != u2 {
                                        found = 256; // 256 mneans not found
                                        break;
                                    }
                                }

                                // do the removal if we matched, and then return
                                if usize::from(found) < EEPROM_TOTAL_SENSOR_SLOTS {
                                    // remove the sensor driver and write null to EEPROM
                                    let bytes: [u8; EEPROM_SENSOR_SETTINGS_SIZE] =
                                        [0xFF; EEPROM_DATALOGGER_SETTINGS_SIZE];
                                    if let Some(found_u8) = found.try_into().ok() {
                                        board.store_sensor_settings(found_u8, &bytes);
                                        self.sensor_drivers[found] = None;
                                        responses::send_command_response_message(board, "sensor removed");
                                        return;
                                    }
                                }
                            }
                        }
                    }
            CommandPayload::SensorListCommandPayload(_) => {
                        board.usb_serial_send("[");
                        let mut first = true;
                        for i in 0..self.sensor_drivers.len() {
                            // create json and output it
                            if let Some(driver) = &mut self.sensor_drivers[i] {
                                if first {
                                    first = false
                                } else {
                                    board.usb_serial_send(",");
                                }

                                let mut id_bytes = driver.get_id();
                                let id_str = match util::str_from_utf8(&mut id_bytes){
                                    Ok(str) => str,
                                    Err(_) => "error",
                                };

                                let type_id = driver.get_type_id();
                                let mut sensor_name_bytes = sensor_name_from_type_id(type_id.into());
                                let sensor_name_str = match util::str_from_utf8(&mut sensor_name_bytes){
                                    Ok(str) => str,
                                    Err(_) => "error",
                                };

                                let json = json!({
                                    "id": id_str,
                                    "type": sensor_name_str
                                });
                                let string = json.to_string();
                                let str = string.as_str();
                                board.usb_serial_send(str);
                            }
                        }
                        board.usb_serial_send("]");
                        board.usb_serial_send("\n");
                    }
            CommandPayload::BoardRtcSetPayload(payload) => {
                        match payload.epoch {
                            serde_json::Value::Number(epoch) => {
                                let epoch = epoch.as_i64();
                                if let Some(epoch) = epoch {
                                    board.set_epoch(epoch);
                                    responses::send_command_response_message(board, "Epoch set");
                                } else {
                                    responses::send_command_response_message(board, "Bad epoch in command");
                                }
                            }
                            err => {
                                responses::send_command_response_error(
                                    board,
                                    "Bad epoch in command",
                                    format!("{:?}", err).as_str(),
                                );
                                return;
                            }
                        };
                    }
            CommandPayload::BoardGetPayload(payload) => {
                        protocol::commands::get_board(board, payload);
                    }
            CommandPayload::SensorCalibratePointPayload(payload) => {
                        // we want to do the book keeping here for point payloads
                        // i guess we use a box again
                        rprintln!("Sensor calibrate point payload");

                        let id = match payload.id {
                            serde_json::Value::String(ref payload_id) => payload_id,
                            _ => {
                                responses::send_command_response_message(
                                    board,
                                    "Invalid sensor id specified",
                                );
                                return;
                            }
                        };

                        let point = payload.point.as_f64();
                        let point = match point {
                            Some(point) => point,
                            None => {
                                responses::send_command_response_message(
                                    board,
                                    "Invalid calibration point specified",
                                );
                                return;
                            }
                        };

                        for i in 0..self.sensor_drivers.len() {
                            // create json and output it
                            if let Some(driver) = &mut self.sensor_drivers[i] {
                                let id_bytes = driver.get_id();
                                let id_str = core::str::from_utf8(&id_bytes).unwrap_or_default();

                                if id == id_str {
                                    // read sensor values
                                    driver.take_measurement(board.get_sensor_driver_services());

                                    let count = driver.get_measured_parameter_count() / 2; // TODO: get_measured_parameter_count, vs get_output_parameter_count
                                    let mut values = Box::new([0_f64; 10]); // TODO: max of 10, should we make this dynamic?
                                    for j in 0..count {
                                        rprintln!("{:?}", j);
                                        let value = match driver.get_measured_parameter_value(j * 2) {
                                            Ok(value) => value,
                                            Err(_) => {
                                                // board.usb_serial_send("missing parameter value\n");
                                                0_f64
                                            }
                                        };
                                        values[j] = value;
                                    }

                                    // store the point
                                    // TODO: we are only storing one point for now
                                    let calibration_pair = CalibrationPair {
                                        point: point,
                                        values: values,
                                    };
                                    if self.calibration_point_values[i].is_some() {
                                        let pairs: &Option<Box<[CalibrationPair]>> =
                                            &self.calibration_point_values[i];
                                        if let Some(pairs) = pairs {
                                            let arr = [calibration_pair, pairs[0].clone()];
                                            let arr_box = Box::new(arr);
                                            self.calibration_point_values[i] = Some(arr_box);
                                        }
                                    } else {
                                        let arr = [calibration_pair];
                                        let arr_box = Box::new(arr);
                                        self.calibration_point_values[i] = Some(arr_box);
                                    }
                                    responses::send_command_response_message(
                                        board,
                                        "Stored a single calibration point",
                                    );
                                }
                            } else {
                                responses::send_command_response_message(board, "Didn't find the sensor");
                                board.usb_serial_send("didn't find the sensor\n");
                            }
                        }
                    }
            CommandPayload::SensorCalibrateListPayload(payload) => {
                        board.usb_serial_send("[");

                        let payload_values = match payload.convert() {
                            Ok(payload_values) => payload_values,
                            Err(message) => {
                                responses::send_command_response_message(
                                    board,
                                    format!("{}", message).as_str(),
                                );
                                return;
                            }
                        };

                        // list the values
                        if let Some(index) = self.get_driver_index_by_id(payload_values.id) {
                            rprintln!("driver index{}", index);
                            let pairs: &Option<Box<[CalibrationPair]>> =
                                &self.calibration_point_values[index];
                            if let Some(pairs) = pairs {
                                for i in 0..pairs.len() {
                                    rprintln!("calib pair{:?}", i);
                                    let pair = &pairs[i];
                                    board.usb_serial_send(
                                        format!("{{'point': {}, 'values': [", pair.point).as_str(),
                                    );
                                    for i in 0..pair.values.len() {
                                        board.usb_serial_send(format!("{}", pair.values[i]).as_str());
                                        if i < pair.values.len() - 1 {
                                            board.usb_serial_send(",");
                                        }
                                    }

                                    board.usb_serial_send("] }}");
                                    if i < pairs.len() - 1 {
                                        board.usb_serial_send(",");
                                    }
                                }
                            }
                        }

                        board.usb_serial_send("]\n");
                    }
            CommandPayload::SensorCalibrateRemovePayload(payload) => {
                        responses::send_command_response_message(
                            board,
                            "This command is not implemented yet",
                        );
                        return;

                        // TO DO: implement removal by tag

                        let payload_values = match payload.convert() {
                            Ok(payload_values) => payload_values,
                            Err(message) => {
                                board.usb_serial_send(format!("{}", message).as_str());
                                return;
                            }
                        };

                        if let Some(index) = self.get_driver_index_by_id(payload_values.id) {
                            let pairs: &Option<Box<[CalibrationPair]>> =
                                &self.calibration_point_values[index];
                            if let Some(pairs) = pairs {
                                for i in 0..pairs.len() {
                                    let pair: CalibrationPair = pairs[i];
                                }
                            }
                        }
                    }
            CommandPayload::SensorCalibrateFitPayload(payload) => {
                        let payload_values = match payload.convert() {
                            Ok(payload_values) => payload_values,
                            Err(message) => {
                                responses::send_command_response_message(
                                    board,
                                    format!("{}", message).as_str(),
                                );
                                return;
                            }
                        };

                        if let Some(index) = self.get_driver_index_by_id(payload_values.id) {
                            if let Some(driver) = &mut self.sensor_drivers[index] {
                                let pairs: &Option<Box<[CalibrationPair]>> =
                                    &self.calibration_point_values[index];
                                // if let Some(pairs) = pairs {
                                //     for i in 0..pairs.len() {
                                //         let pair = &pairs[i];
                                //         rprintln!("calib pair{:?} {} {}", i, pair.point, pair.values[i]);
                                //     }
                                // }

                                if let Some(pairs) = pairs {
                                    for i in 0..pairs.len() {
                                        let pair = &pairs[i];
                                        rprintln!("calib pair{:?} {} {}", i, pair.point, pair.values[0]);
                                    }
                            
                                    driver.clear_calibration();
                                    match driver.fit(pairs) {
                                        Ok(_) => {
                                            let mut storage = [0u8; EEPROM_SENSOR_SETTINGS_SIZE];
                                            driver.get_configuration_bytes(&mut storage);

                                            board.store_sensor_settings(index as u8, &storage);
                                            responses::send_command_response_message(board, "Fit OK");
                                        }
                                        Err(_) => {
                                            responses::send_command_response_message(
                                                board,
                                                "something went wrong",
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
            CommandPayload::SensorCalibrateClearPayload(payload) => {
                        let payload_values = match payload.convert() {
                            Ok(payload_values) => payload_values,
                            Err(message) => {
                                board.usb_serial_send(format!("{}", message).as_str());
                                return;
                            }
                        };

                        if let Some(index) = self.get_driver_index_by_id(payload_values.id) {
                            if let Some(driver) = &mut self.sensor_drivers[index] {
                                driver.clear_calibration();
                                responses::send_command_response_message(board, "Cleared payload");
                            }
                        }
                    }
            CommandPayload::BoardSerialSendPayload(payload) => {
                        let payload_values = match payload.convert() {
                            Ok(payload_values) => payload_values,
                            Err(error) => {
                                responses::send_command_response_error(
                                    board,
                                    "Problem with message",
                                    error,
                                );
                                return;
                            }
                        };

                        let message = match core::str::from_utf8(
                            &payload_values.message[0..payload_values.message_len as usize],
                        ) {
                            Ok(message) => message,
                            Err(error) => {
                                responses::send_command_response_error(
                                    board,
                                    "Problem sending message",
                                    format!(" {} \n", error).as_str(),
                                );
                                return;
                            }
                        };

                        let prepared_message = format!("{}\r\n", message);
                        let prepared_message = prepared_message.as_str();
                        rprintln!("message {}", prepared_message);
                        board.usart_send(prepared_message);
                        // rprintln!("{}", "\r\n");
                        // board.usart_send("\r\n");
                        // rprintln!("just line feed");
                        // board.usart_send("\r");

                        board.delay_ms(500);

                        let response = match usart_service::take_command(board) {
                            Ok(message) => message,
                            Err(_) => {
                                rprintln!("no usart response");
                                responses::send_command_response_message(
                                    board,
                                    "No response received on serial",
                                );
                                return;
                            }
                        };

                        let length = response.len();
                        for b in &response[0..length] {
                            let c = *b as char;
                            rprintln!("{}", c);
                        }

                        match core::str::from_utf8(&response) {
                            Ok(response) => {
                                let end = response.find("\0");
                                let mut response = response;
                                match end {
                                    Some(end) => response = &response[0..end],
                                    None => {}
                                }
                                responses::send_command_response_message(board, response);
                            }
                            Err(error) => {
                                responses::send_command_response_error(
                                    board,
                                    "Problem receiving message",
                                    format!(" {} \n", error).as_str(),
                                );
                            }
                        };

                        return;
                    }
            CommandPayload::TelemeterGet => {
                match self.telemeter.get_identity(board) {
                    Ok(message) => {
                        board.usb_serial_send(format!("{}\n", message.as_str()).as_str());
                    },
                    Err(_) => {
                        responses::send_command_response_message(board, "Failed to get identifiers");
                    }
                }
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
