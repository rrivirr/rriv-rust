use super::types::*;

pub const EMPTY_SIZE: usize = 32;
pub const NUMBER_OF_MEASURED_PARAMETERS: usize = 2;

#[derive(Copy, Clone, Debug)]
pub struct Ds18b20SpecialConfiguration {
    // calibrate data?
    // power mode
    // resolution mode
    _empty: [u8; 32]
}

impl Ds18b20SpecialConfiguration {
    pub fn new_from_values (
        value: serde_json::Value,
    ) -> Ds18b20SpecialConfiguration {
        Self {_empty:[b'\0'; EMPTY_SIZE]}
    }

    pub fn new_from_bytes (
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE]
    ) -> Ds18b20SpecialConfiguration {
        Self {_empty: [b'\0'; EMPTY_SIZE]}
    }
}

pub struct Ds18b20 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: Ds18b20SpecialConfiguration,
    measured_parameter_values: [f64; NUMBER_OF_MEASURED_PARAMETERS]
}

impl Ds18b20 {
    pub fn new (
        general_config: SensorDriverGeneralConfiguration,
        special_config: Ds18b20SpecialConfiguration
    ) -> Ds18b20 {
        Ds18b20 {
            general_config,
            special_config,
            measured_parameter_values: [0.0; NUMBER_OF_MEASURED_PARAMETERS]
        }
    }
}

impl SensorDriver for Ds18b20 {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        return NUMBER_OF_MEASURED_PARAMETERS;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> f64 {
        self.measured_parameter_values[index]
        // if index < NUMBER_OF_MEASURED_PARAMETERS {
        //     return Ok(sel.measured_parameter_values[index]);
        // } else {
        //     return Err(());
        // }
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        let identifiers: [&str; NUMBER_OF_MEASURED_PARAMETERS] = ["T_raw","T_cal"];
        let mut buf: [u8; 16] = [0_u8; 16];

        let mut identifier: &str = "invalid";
        if index <= NUMBER_OF_MEASURED_PARAMETERS {
            identifier = identifiers[index];
        }
        for i in 0..identifier.len() {
            buf[i] = identifier.as_bytes()[i];
        }
        return buf;
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        todo!();
    }
}   