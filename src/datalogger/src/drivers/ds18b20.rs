use super::types::*;

pub const EMPTY_SIZE: usize = 32;

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
    special_config: Ds18b20SpecialConfiguration
}

impl SensorDriver for Ds18b20 {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        todo!()
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> f64 {
        todo!()
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        todo!()
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        todo!()
    }
}