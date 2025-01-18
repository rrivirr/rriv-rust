use super::types::*;

#[derive(Copy, Clone, Debug)]
pub struct AHT22SpecialConfiguration {
    wait_time: usize,
    empty: [u8; 28],
}

impl AHT22SpecialConfiguration {
    pub fn new_from_bytes(
        bytes: &[u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> AHT22SpecialConfiguration {
        let settings = bytes.as_ptr().cast::<AHT22SpecialConfiguration>();
        unsafe { *settings }
    }
}

pub struct AHT22 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: AHT22SpecialConfiguration,
}

impl SensorDriver for AHT22 {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        todo!()
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
        todo!()
    }
       
}

impl AHT22 {
    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        specific_config_bytes: &[u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> Self {
        let special_config = AHT22SpecialConfiguration::new_from_bytes(specific_config_bytes);
        AHT22 {
            general_config,
            special_config,
        }
    }
}
