use rtt_target::rprint;
use serde_json::json;

use crate::sensor_name_from_type_id;

use super::types::*;

#[derive(Copy, Clone)]
pub struct AHT22SpecialConfiguration {
    wait_time: usize,
    empty: [u8; 28],
}

impl AHT22SpecialConfiguration {
    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> AHT22SpecialConfiguration {
        let settings = bytes.as_ptr().cast::<AHT22SpecialConfiguration>();
        unsafe { *settings }
    }
}

pub struct AHT22 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: AHT22SpecialConfiguration,
}

pub struct ConfigurationPayload {}

impl SensorDriver for AHT22 {

    fn get_configuration_json(&mut self) -> serde_json::Value  {

        let sensor_name_bytes = sensor_name_from_type_id(self.get_type_id().into());
        let sensor_name_str = core::str::from_utf8(&sensor_name_bytes).unwrap_or_default();

        json!({ 
            "id" : self.get_id(),
            "type" : sensor_name_str,
            "wait_time": self.special_config.wait_time
        })
    }

       

    fn setup(&mut self) {
        rprint!("not implemented");
    }

    getters!();

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        rprint!("not implemented");
    }

    fn get_measured_parameter_count(&mut self) -> usize {
        todo!()
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        todo!()
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        todo!()
    }

    fn fit(&mut self, pairs: &[CalibrationPair]) -> Result<(), ()>{
        let _ = pairs;
        todo!()
    }
    
    fn clear_calibration(&mut self) {
        rprint!("not implemented");
    }
    
    fn get_configuration_bytes(&self, storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {
        rprint!("not implemented");
    }
       
    fn update_actuators(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
    }
}

impl AHT22 {
    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        specific_config_bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> Self {
        let special_config = AHT22SpecialConfiguration::new_from_bytes(specific_config_bytes);
        AHT22 {
            general_config,
            special_config,
        }
    }
}
