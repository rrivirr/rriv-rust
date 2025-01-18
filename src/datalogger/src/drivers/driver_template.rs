use super::types::*;

pub const EMPTY_SIZE: usize = 32;

#[derive(Copy, Clone, Debug)]
pub struct DRIVERTEMPLATESpecialConfiguration {
    _empty: [u8; 32],
}

// should this adopt a trait??
impl DRIVERTEMPLATESpecialConfiguration {
    pub fn new_from_values(
        value: serde_json::Value,
    ) -> RingTemperatureDriverSpecialConfiguration {
        Self {address_offset: 0, empty: [b'\0'; 31] } // Just using default address offset of 0 for now, need to optionally read from JSON
    }
    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> RingTemperatureDriverSpecialConfiguration {
        Self { address_offset: bytes[0], empty: [b'\0'; 31] }
    }
}

pub struct DRIVERTEMPLATE {
    general_config: SensorDriverGeneralConfiguration,
    special_config: DRIVERTEMPLATESpecialConfiguration,
}


impl SensorDriver for DRIVERTEMPLATE {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        todo!()
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        todo!()
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        todo!()
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        todo!()
    }
}