use crate::sensor_name_from_type_id;

use super::types::*;
use bitfield_struct::bitfield;
use alloc::boxed::Box;
use rtt_target::rprint;
use serde_json::json;
use util::any_as_u8_slice;


pub struct GenericAnalog {
    general_config: SensorDriverGeneralConfiguration,
    special_config: GenericAnalogSpecialConfiguration,
    measured_parameter_values: [f64; 2],
    m: f64,
    b: f64,
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
        self.m = (self.special_config.m as f64) / 1000_f64;
        self.b = (self.special_config.b as f64) / 1000_f64;
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        return 2;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        return Ok(self.measured_parameter_values[index]);
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        return single_raw_or_cal_parameter_identifiers(index, None);
    }


    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        let mut value = 0;
        match self.special_config.settings.adc_select() {
            0 => {
                value = board.query_internal_adc(self.special_config.sensor_port);
            }
            1 => {
                value = board.query_external_adc(self.special_config.sensor_port);
            }
            2_usize.. => todo!("other adcs not implemented"),
        }
        self.measured_parameter_values[0] = value.into();
        self.measured_parameter_values[1] = self.m * value as f64 + self.b;

    }

    fn update_actuators(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        // no actuators
    }
    
    fn clear_calibration(&mut self) {
        self.m = 0_f64;
        self.b = 0_f64;
        self.special_config.m = 0_i32;
        self.special_config.b = 0_i32;
    }
    
    fn fit(&mut self, pairs: &[CalibrationPair]) -> Result<(), ()>{
        let _ = pairs;
        if pairs.len() != 2 {
            return Err(());
        }

        let cal1 = & pairs[0];
        let cal2 = & pairs[1];

        self.m = (cal2.point - cal1.point) / (cal2.values[0] - cal1.values[0]);
        self.b = cal1.point - self.m * cal1.values[0];
        rprint!("calbiration: {} {}", self.m, self.b);
        self.special_config.m = (self.m * 1000_f64) as i32;
        self.special_config.b = (self.b * 1000_f64) as i32;
        Ok(())
    }
    

    
    fn get_configuration_bytes(&self, storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {
        // TODO: this can become a utility or macro function
        let generic_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&self.general_config) };
        let special_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&self.special_config) };

        copy_config_into_partition(0, generic_settings_bytes, storage);
        copy_config_into_partition(1, special_settings_bytes, storage);
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
            m: 0_f64,
            b: 0_f64
        }
    }
}



#[bitfield(u8)]
struct GenericAnalogDriverBitfield {
    #[bits(2)]
    adc_select: usize,

    #[bits(6)]
    unused: usize
}

#[derive(Copy, Clone)]
pub struct GenericAnalogSpecialConfiguration {
    m: i32,          // 4
    b: i32,          // 4
    sensor_port: u8, // 1
    settings: GenericAnalogDriverBitfield, // 1
    empty: [u8; 22], // 22
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


        let mut bitfield = GenericAnalogDriverBitfield::new();
        match &value["adc_select"] {
            serde_json::Value::String(string) => match string.as_str() {
                "internal" => {
                    bitfield.set_adc_select(0);
                }
                "external" => {
                    bitfield.set_adc_select(1);
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
            m: 0_i32,
            b: 0_i32,
            sensor_port: sensor_port,
            settings: bitfield,
            empty: [b'\0'; 22],
        };
    }

    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> GenericAnalogSpecialConfiguration {
        let settings = bytes.as_ptr().cast::<GenericAnalogSpecialConfiguration>();
        unsafe { *settings }
    }

       
}