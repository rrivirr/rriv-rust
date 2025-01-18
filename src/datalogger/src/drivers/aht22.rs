use super::types::*;

const EMPTY_SIZE: usize = 32;

#[derive(Copy, Clone, Debug)]
pub struct Aht22SpecialConfiguration {
    // calibration data storage?
    // power mode (jake)
    // resolution mode (jake)
    _empty: [u8; EMPTY_SIZE]
}

impl Aht22SpecialConfiguration {
    pub fn new_from_values (
        value: serde_json::Value,
    ) -> Aht22SpecialConfiguration {
        Self {_empty:[b'\0'; EMPTY_SIZE]}
    }

    pub fn new_from_bytes (
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE]
    ) -> Aht22SpecialConfiguration {
        Self {_empty:[b'\0'; EMPTY_SIZE]}
    }
}

const NUMBER_OF_MEASURED_PARAMETERS: usize = 4;

pub struct Aht22 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: Aht22SpecialConfiguration,
    measured_parameter_values: [f64; NUMBER_OF_MEASURED_PARAMETERS]
}


impl Aht22 {
    pub fn new (
        general_config: SensorDriverGeneralConfiguration, 
        special_config: Aht22SpecialConfiguration
    ) -> Aht22 {
        return Aht22 {
            general_config,
            special_config,
            measured_parameter_values: [0.0; NUMBER_OF_MEASURED_PARAMETERS]
        };
    }
}

impl SensorDriver for Aht22 {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        return NUMBER_OF_MEASURED_PARAMETERS;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        if index < NUMBER_OF_MEASURED_PARAMETERS {
            return Ok(self.measured_parameter_values[index]);
        } else {
            return Err(());
        }
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        let identifiers = ["T_raw", "T_cal", "H_raw", "H_cal"];
        let mut buf = [0u8;16];
        let mut identifier = "invalid";
        if index <= NUMBER_OF_MEASURED_PARAMETERS {
            identifier = identifiers[index];
        }
        for i in 0..identifier.len() {
            buf[i] = identifier.as_bytes()[i];
        }
        return buf;
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        todo!("take mesaurement")
    }
}