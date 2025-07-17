use crate::sensor_name_from_type_id;

use super::types::*;
use bitfield_struct::bitfield;
use alloc::boxed::Box;
use serde_json::json;


pub struct GenericAnalog {
    general_config: SensorDriverGeneralConfiguration,
    special_config: GenericAnalogSpecialConfiguration,
    measured_parameter_values: [f64; 2],
}


impl SensorDriver for GenericAnalog {

    fn get_configuration_json(&mut self) -> serde_json::Value  {

        let sensor_name_bytes = sensor_name_from_type_id(self.get_type_id().into());
        let sensor_name_str = core::str::from_utf8(&sensor_name_bytes).unwrap_or_default();

        let adc_select = match self.special_config.settings.adc_select() {
            0 => "internal",
            1 => "external",
            _ => "invalid"
        };

        json!({ 
            "id" : self.get_id(),
            "type" : sensor_name_str,
            "m": self.special_config.m, 
            "b" : self.special_config.b,
            "sensor_port": self.special_config.sensor_port,
            "adc_select": adc_select
         })
    }

    fn setup(&mut self) {
        
    }

    getters!();

    // fn get_measured_parameter_values(&mut self) -> [f32; 2] {
    //     return self.measured_parameter_values.clone();
    // }

    // fn get_measured_parameter_identifiers(&mut self) -> [&str] {
    //     return ["raw"];
    // }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        let mut value = 0;
        match self.special_config.settings.adc_select() {
            0 => {
                value = board.query_internal_adc(self.special_config.sensor_port);
            }
            1 => todo!("exadc not impl"),
            2_usize.. => todo!("other adcs not implemented"),
        }
        self.measured_parameter_values[0] = value.into();
        self.measured_parameter_values[0] = value.into();

    }

    fn get_measured_parameter_count(&mut self) -> usize {
        return 2;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        return Ok(self.measured_parameter_values[index]);
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        return single_raw_or_cal_parameter_identifiers(index, None);
    }

    fn update_actuators(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        // not actuators
    }
    
    // fn take_measurement(&mut self, board: Box<&mut impl rriv_board::RRIVBoard>) {
    //     // implement exadc and intadc
    //     match self.special_config.settings.adc_select {
    //         0 => {
    //             board.query_internal_adc(self.special_config.sensor_port);
    //         },
    //         1 => todo!("exadc not implemented"),
    //     }
    // }

    fn fit(&mut self, pairs: &[CalibrationPair]) -> Result<(), ()>{
        let _ = pairs;
        todo!()
    }
    
    fn clear_calibration(&mut self) {
        todo!()
    }
    
    fn get_configuration_bytes(&self, storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {
        todo!()
    }
    
  
       
}

impl GenericAnalog {
    // pub fn new(general_config: SensorDriverGeneralConfiguration, special_config_bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE]) -> Self {

    //     let special_config = GenericAnalogSpecialConfiguration::new_from_bytes(special_config_bytes);
    //     GenericAnalog {
    //         general_config,
    //         special_config
    //     }
    // }

    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        special_config: GenericAnalogSpecialConfiguration,
    ) -> Self {
        GenericAnalog {
            general_config,
            special_config,
            measured_parameter_values: [0.0; 2],
        }
    }
}



#[bitfield(u8)]
struct GenericAnalogDriverBitfield {
    #[bits(2)]
    adc_select: usize,

    unused_1: bool,
    unused_2: bool,
    unused_3: bool,
    unused_4: bool,
    unused_5: bool,
    unused_6: bool,
}

#[derive(Copy, Clone, Debug)]
pub struct GenericAnalogSpecialConfiguration {
    m: f64,          //8
    b: f64,          // 8
    sensor_port: u8, // 1
    settings: GenericAnalogDriverBitfield,
    empty: [u8; 14], // 15
}

impl GenericAnalogSpecialConfiguration {
    pub fn new_from_values(value: serde_json::Value) -> GenericAnalogSpecialConfiguration {
        // should we return a Result object here? because we are parsing?  parse_from_values?
        let mut sensor_port: u8 = 0;
        match &value["sensor_port"] {
            serde_json::Value::Number(number) => {
                if let Some(number) = number.as_u64() {
                    let number: Result<u8, _> = number.try_into();
                    match number {
                        Ok(number) => {
                            sensor_port = number;
                        }
                        Err(_) => todo!("need to handle invalid number"),
                    }
                }
            }
            _ => {
                todo!("need to handle missing sensor port")
            }
        }

        let mut bits: u8 = 0;
        match &value["adc_select"] {
            serde_json::Value::String(string) => match string.as_str() {
                "internal" => {
                    bits = 0;
                }
                "external" => {
                    bits = 1;
                }
                _ => {
                    todo!("need to handle bad adc select string");
                }
            },
            _ => {
                todo!("need to handle missing adc selection")
            }
        }

        return Self {
            m: 0.0,
            b: 0.0,
            sensor_port: sensor_port,
            settings: GenericAnalogDriverBitfield::from_bits(bits),
            empty: [b'\0'; 14],
        };
    }

    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> GenericAnalogSpecialConfiguration {
        // panic if bytes.len() != 32
        let settings = bytes.as_ptr().cast::<GenericAnalogSpecialConfiguration>();
        unsafe { *settings }
    }

       
}