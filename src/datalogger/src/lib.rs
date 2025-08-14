#![cfg_attr(not(test), no_std)]
#![feature(array_methods)]

mod services;
mod datalogger;

use datalogger::settings::*;
use datalogger::commands::*;


use rriv_board::{
    RRIVBoard, EEPROM_DATALOGGER_SETTINGS_SIZE, EEPROM_SENSOR_SETTINGS_SIZE,
    EEPROM_TOTAL_SENSOR_SLOTS,
};
use util::{any_as_u8_slice};
extern crate alloc;
use crate::datalogger::modes::DataLoggerMode;
use crate::datalogger::modes::DataLoggerSerialTxMode;
use crate::{
    alloc::string::ToString, protocol::responses, services::*, telemetry::telemeters::lorawan::RakWireless3172
};
use alloc::boxed::Box;
use alloc::format;
use rtt_target::rprintln;

mod drivers;
use drivers::{types::*, *};

mod protocol;
mod registry;
mod telemetry;
use registry::*;

use serde_json::{json, Value};



pub struct DataLogger {
    settings: DataloggerSettings,
    sensor_drivers: [Option<Box<dyn SensorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    actuator_drivers: [Option<Box<dyn ActuatorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    telemeter_drivers: [Option<Box<dyn TelemeterDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],

    last_interactive_log_time: i64,

    debug_values: bool, // serial out of values as they are read
    log_raw_data: bool, // both raw and summary data writting to storage

    mode: DataLoggerMode,
    serial_tx_mode: DataLoggerSerialTxMode,

    // naive calibration value book keeping
    // not memory efficient
    calibration_point_values: [Option<Box<[CalibrationPair]>>; EEPROM_TOTAL_SENSOR_SLOTS],

    telemeter: telemetry::telemeters::lorawan::RakWireless3172,

    // measurement cycle
    completed_bursts: u8,
    readings_completed_in_current_burst: u8,

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
            serial_tx_mode: DataLoggerSerialTxMode::Normal,
            calibration_point_values: [CALIBRATION_REPEAT_VALUE; EEPROM_TOTAL_SENSOR_SLOTS],
            telemeter: RakWireless3172::new(),
            completed_bursts: 0,
            readings_completed_in_current_burst: 0,
        }
    }

    fn retrieve_settings(&self, board: &mut impl RRIVBoard) -> DataloggerSettings {
        let mut bytes: [u8; EEPROM_DATALOGGER_SETTINGS_SIZE] =
            [b'\0'; EEPROM_DATALOGGER_SETTINGS_SIZE];
        board.retrieve_datalogger_settings(&mut bytes);
        rprintln!("retrieved {:?}", bytes);
        let settings: DataloggerSettings = DataloggerSettings::new_from_bytes(bytes); // convert the bytes pack into a DataloggerSettings

        let settings = settings.configure_defaults();

        settings
    }

    fn store_settings(&mut self, board: &mut impl RRIVBoard) {
        let bytes = unsafe { self.settings.get_bytes() };
        board.store_datalogger_settings(&bytes);
        rprintln!("stored {:?}", bytes);
    }

    fn get_driver_index_by_id(&self, id: &str) -> Option<usize> {
        let drivers = &self.sensor_drivers;
        for i in 0..self.sensor_drivers.len() {
            // create json and output it
            if let Some(driver) = &drivers[i] {
                let mut id_bytes = driver.get_id();
                let id_str = util::str_from_utf8(&mut id_bytes).unwrap_or_default();
                if id == id_str {
                    return Some(i);
                }
            }
        }
        return None;
    }

    fn get_driver_index_by_id_value(&self, id: Value) -> Option<usize> {
        let id = match id {
            serde_json::Value::String(ref payload_id) => payload_id,
            _ => {
                // responses::send_command_response_message(
                //     board,
                //     "Invalid sensor id specified",
                // );
                return None;
            }
        };

        return self.get_driver_index_by_id(id);
    }

    pub fn setup(&mut self, board: &mut impl RRIVBoard) {
        // enable power to the eeprom and bring i2bufferc online

        // rprintln!("retrieving settings");
        self.settings = self.retrieve_settings(board);
        rprintln!("retrieved settings {:?}", self.settings);

        // setup each service
        command_service::setup(board);
        usart_service::setup(board);

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
                driver.setup(board.get_sensor_driver_services());
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

        match self.mode {
            DataLoggerMode::Interactive  => {
                
                // process CLI
                // process telemetry
                // process actuators
                
                if board.timestamp() >= self.last_interactive_log_time + self.settings.interactive_logging_interval as i64 {
                    
                    // process a single measurement
                    // is this called a 'single measurement cycle' ?
                  
                    self.measure_sensor_values(board); // measureSensorValues(false);
                    self.write_last_measurement_to_serial(board); //outputLastMeasurement();
                                                                  // Serial2.print(F("CMD >> "));
                                                                  // writeRawMeasurementToLogFile();
                                                                  // fileSystemWriteCache->flushCache();
                    self.write_raw_measurement_to_storage(board);

                    self.process_telemetry(board); // telemeterize the measured values

                    self.last_interactive_log_time = board.timestamp();
                }
            }
            DataLoggerMode::Field => {

                // run measurement cycle
                // maybe we have sleep_interval (minutes) and interactive_logged_interval (seconds)

                self.run_measurement_cycle(board);
                if self.measurement_cycle_completed() {
                //     // go to sleep until the next in interval (in minutes)
                    board.delay_ms(self.settings.interactive_logging_interval); // TODO: using interactive while developing.
                    
                //     // start the next measurement cycle
                    self.initialize_measurement_cycle();
                }
            },
            DataLoggerMode::HibernateUntil => { 
                // this block is processed when we start up, don't get an interactive mode interrupt, and are in HibernateUntil mode
                // or when we have just entered this mode
                // TODO: hibernate until the requested wake time
            }
            
        }
    }

    fn initialize_measurement_cycle(&mut self){
        self.completed_bursts = 0;
        self.readings_completed_in_current_burst = 0;
    }

    fn measurement_cycle_completed(&self) -> bool {
        self.completed_bursts >= self.settings.bursts_per_measurement_cycle
    }


    fn run_measurement_cycle(&mut self, board: &mut impl rriv_board::RRIVBoard) {

        // calculate the total number of readings per burst, max of readings requests on each sensor.
        // allow burst length to be set per sensors OR overwridden

        let readings_per_burst = 10; // TODO get it from settings (need to be added there)

        // get next raw reading
        self.measure_sensor_values(board);
        self.readings_completed_in_current_burst = self.readings_completed_in_current_burst + 1;

        // output raw data to serial?
        let output_raw_data_to_serial  = true;
        if output_raw_data_to_serial {
            self.write_last_measurement_to_serial(board)
        }

        // write raw data to storage
        self.write_raw_measurement_to_storage(board);

        // store values into the summarizer

        // check on progress
        if(self.readings_completed_in_current_burst >= readings_per_burst){
            self.completed_bursts = self.completed_bursts + 1;
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
        board.write_log_file("type,site,logger,deployment,deployed_at,uid,time.s,battery.V,");

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


    // TODO: this function and the next one can be DRY by passing a closure
    fn write_measured_parameters_to_serial(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        let epoch = board.epoch_timestamp();
        let millis = board.get_millis() % 1000;
        let output = format!("{}.{},", epoch, millis);
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

    fn write_raw_measurement_to_storage(&mut self, board: &mut impl rriv_board::RRIVBoard) {
        let epoch = board.epoch_timestamp();
        let millis = board.get_millis() % 1000;
        // "type,site,logger,deployment,deployed_at,uid,time.s,battery.V"
        
        // TODO: find a better way to print this uid, or generate and use a UUID that doesn't come from the MCU's uid
        let uid = board.get_uid();
        let output = format!("raw,{},{},{},-,{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?}{:X?},{}.{},{},",
            util::str_from_utf8(&mut self.settings.site_name).unwrap_or_default(),
            util::str_from_utf8(&mut self.settings.logger_name).unwrap_or_default(),
            util::str_from_utf8(&mut self.settings.deployment_identifier).unwrap_or_default(),
            uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],uid[6],uid[7],uid[8],uid[9],uid[10],uid[11],
            epoch, millis,
            board.get_battery_level()
        );
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
        match self.serial_tx_mode {
            DataLoggerSerialTxMode::Normal => self.write_column_headers_to_serial(board),
            _ => {}
        }

        // then output the last measurement values
        match self.serial_tx_mode {
            DataLoggerSerialTxMode::Watch => {
                self.write_measured_parameters_to_serial(board)
            }
            _ => {}
        }
    }

    pub fn set_mode(&mut self, board: &mut impl RRIVBoard, mode: Value) {
        match mode {
                        Value::String(mode) => {
                            let mode = mode.as_str();
                            // TODO: perhaps watch should be separate from mode, so you can watch any mode
                            // TODO: and rrivctl can still accept commands when in watch mode
                            match mode {
                                "watch" => {
                                    self.write_column_headers_to_serial(board);
                                    self.serial_tx_mode = DataLoggerSerialTxMode::Watch;
                                    board.set_debug(false);
                                    self.telemeter.set_watch(true);
                                }
                                "watch-debug" => {
                                    self.write_column_headers_to_serial(board);
                                    self.serial_tx_mode = DataLoggerSerialTxMode::Watch;
                                    board.set_debug(true);
                                    self.telemeter.set_watch(true);
                                }
                                "quiet" => {
                                    self.serial_tx_mode = DataLoggerSerialTxMode::Quiet;
                                    board.set_debug(false);
                                    self.telemeter.set_watch(false);
                                },
                                "field" => {
                                    self.mode = DataLoggerMode::Field;
                                },
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

    pub fn execute_command(&mut self, board: &mut impl RRIVBoard, command_payload: CommandPayload) {
        // rprintln!("executing command {:?}", command_payload);
        match command_payload {
            CommandPayload::DataloggerSetCommandPayload(payload) => {
                self.update_datalogger_settings(board, payload);
                responses::send_command_response_message(board, "updated datalogger settings");
            }
            CommandPayload::DataloggerGetCommandPayload(_) => {
                let values = json!({
                   "site_name": util::str_from_utf8(&mut self.settings.site_name).unwrap_or_default(),
                   "logger_name" : util::str_from_utf8(&mut self.settings.logger_name).unwrap_or_default(),
                   "deployment_identifier" : util::str_from_utf8(&mut self.settings.deployment_identifier).unwrap_or_default(),
                   "deployment_timestamp" : self.settings.deployment_timestamp,
                   "interactive_logging_interval" : self.settings.interactive_logging_interval,
                   "sleep_interval" : self.settings.sleep_interval,
                   "start_up_delay" : self.settings.start_up_delay,
                   "delay_between_bursts" : self.settings.delay_between_bursts,
                   "bursts_per_measurement_cycle" : self.settings.bursts_per_measurement_cycle,
                   "mode" : datalogger::modes::mode_text(&self.mode)
                });
                responses::send_command_response_message(board, values.to_string().as_str());
            }
            CommandPayload::DataloggerSetModeCommandPayload(payload) => {
                if let Some(mode) = payload.mode {
                    self.set_mode(board, mode);
                }
                
                responses::send_command_response_message(board, "mode set");

            }
            CommandPayload::SensorSetCommandPayload(payload, values) => {
                let registry = get_registry();
                // let sensor_type: Result<&str, core::str::Utf8Error> = core::str::from_utf8(&payload.r#type);
                let sensor_type_id = match payload.r#type {
                    serde_json::Value::String(sensor_type) => {
                        registry::sensor_type_id_from_name(&sensor_type)
                    }
                    _ => {
                        responses::send_command_response_message(board, "ensor type not specified");
                        return;
                    }
                };

                if sensor_type_id == 0 {
                    responses::send_command_response_message(board, "sensor type not found");
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
                    driver.setup(board.get_sensor_driver_services());
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
                    responses::send_command_response_message(board, "updated sensor configuration");
                }
            }
            CommandPayload::SensorGetCommandPayload(payload) => {
                if let Some(index) = self.get_driver_index_by_id_value(payload.id) {
                    if let Some(driver) = &mut self.sensor_drivers[index] {
                        let configuration_payload = driver.get_configuration_json();
                        let string = configuration_payload.to_string();
                        let str = string.as_str();
                        responses::send_command_response_message(board, str);
                        return;
                    }
                }

                responses::send_command_response_message(board, "didn't find the sensor");
            }
            CommandPayload::SensorRemoveCommandPayload(payload) => {
                let sensor_id = match payload.id {
                    serde_json::Value::String(id) => {
                        let mut prepared_id: [u8; 6] = [0; 6];
                        prepared_id[0..id.as_bytes().len()].copy_from_slice(id.as_bytes());
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
                        let id_str = match util::str_from_utf8(&mut id_bytes) {
                            Ok(str) => str,
                            Err(_) => "error",
                        };

                        let type_id = driver.get_type_id();
                        let mut sensor_name_bytes = sensor_name_from_type_id(type_id.into());
                        let sensor_name_str = match util::str_from_utf8(&mut sensor_name_bytes) {
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
                datalogger::commands::get_board(board, payload);
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

                if let Some(index) = self.get_driver_index_by_id(id) {
                    if let Some(driver) = &mut self.sensor_drivers[index] {
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
                        if self.calibration_point_values[index].is_some() {
                            let pairs: &Option<Box<[CalibrationPair]>> =
                                &self.calibration_point_values[index];
                            if let Some(pairs) = pairs {
                                let arr = [calibration_pair, pairs[0].clone()];
                                let arr_box = Box::new(arr);
                                self.calibration_point_values[index] = Some(arr_box);
                            }
                        } else {
                            let arr = [calibration_pair];
                            let arr_box = Box::new(arr);
                            self.calibration_point_values[index] = Some(arr_box);
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
            CommandPayload::SensorCalibrateListPayload(payload) => {
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

                    board.usb_serial_send("{ pairs: [");

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

                            board.usb_serial_send("] }");
                            if i < pairs.len() - 1 {
                                board.usb_serial_send(",");
                            }
                        }
                    }

                    board.usb_serial_send("]}");
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
            CommandPayload::TelemeterGet => match self.telemeter.get_identity(board) {
                Ok(message) => {
                    board.usb_serial_send(format!("{}\n", message.as_str()).as_str());
                }
                Err(_) => {
                    responses::send_command_response_message(board, "Failed to get identifiers");
                }
            },
        }
    }

    pub fn update_datalogger_settings(
        &mut self,
        board: &mut impl RRIVBoard,
        set_command_payload: DataloggerSetCommandPayload,
    ) {
        let mode = set_command_payload.mode.clone(); // TODO: clean this up
        let values = set_command_payload.values();
        self.settings = self.settings.with_values(values);
         if let Some(mode) = mode {
            self.set_mode(board, mode);
        }
        self.store_settings(board)
    }
}
