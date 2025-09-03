use core::time;

use serde_json::json;

use crate::{any_as_u8_slice, sensor_name_from_type_id};

use super::types::*;

#[derive(Copy, Clone)]
pub struct TimedSwitchSpecialConfiguration {
    on_time_s: usize,
    off_time_s: usize,
    empty: [u8; 24],
}

impl TimedSwitchSpecialConfiguration {

    pub fn parse_from_values(value: serde_json::Value) -> Result<TimedSwitchSpecialConfiguration, &'static str> {
        // should we return a Result object here? because we are parsing?  parse_from_values?
        let mut on_time_s: usize = 10;
        match &value["on_time_s"] {
            serde_json::Value::Number(number) => {
                if let Some(number) = number.as_u64() {
                    let number: Result<usize, _> = number.try_into();
                    match number {
                        Ok(number) => {
                            on_time_s = number;
                        }
                        Err(_) => return Err("invalid number"),
                    }
                }
            }
            _ => {
                return Err("missing sensor port")
            }
        }

        let mut off_time_s: usize = 10;
        match &value["off_time_s"] {
            serde_json::Value::Number(number) => {
                if let Some(number) = number.as_u64() {
                    let number: Result<usize, _> = number.try_into();
                    match number {
                        Ok(number) => {
                            off_time_s = number;
                        }
                        Err(_) => todo!("need to handle invalid number"),
                    }
                }
            }
            _ => {
                todo!("need to handle missing sensor port")
            }
        }


      
        Ok( Self {
            on_time_s,
            off_time_s,
            empty: [b'\0'; 24],
        } )
    }


    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> TimedSwitchSpecialConfiguration {
        let settings = bytes.as_ptr().cast::<TimedSwitchSpecialConfiguration>();
        unsafe { *settings }
    }
}

pub struct TimedSwitch {
    general_config: SensorDriverGeneralConfiguration,
    special_config: TimedSwitchSpecialConfiguration,
    state: u8, // 0: off, 1: on, other: invalid for now
    last_state_updated_at: i64
}

impl TimedSwitch {
    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        special_config: TimedSwitchSpecialConfiguration,
    ) -> Self {
        TimedSwitch {
            general_config,
            special_config,
            state: 0,
            last_state_updated_at: 0
        }
    }
}

impl SensorDriver for TimedSwitch {
    fn setup(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        // todo!()
    }

    getters!();
    

    fn get_measured_parameter_count(&mut self) -> usize {
        return 1;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        Ok(self.state as f64)
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        let mut rval = [0u8;16];
        rval[0..7].clone_from_slice("heater\0".as_bytes());
        return rval;
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        //
    }

    fn update_actuators(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        let timestamp = board.timestamp();

        let mut toggle_state = false;
        if self.state == 0 {
            // heater is off
            if timestamp - self.special_config.off_time_s as i64 > self.last_state_updated_at {
                toggle_state = true;
            }
        } else if self.state == 1 {
            // heater is on
            if timestamp - self.special_config.on_time_s as i64 > self.last_state_updated_at {
                toggle_state = true;

            }
        }
        if toggle_state {
            
            board.write_gpio_pin(8, self.state != 0);
            self.state = match self.state {
                0 => 1,
                1 => 0,
                _ => 0,
            };
            self.last_state_updated_at = timestamp;
        }

    }
    
    fn get_configuration_bytes(&self, storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {
        
        let generic_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&self.general_config) };
        let special_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&self.special_config) };

        copy_config_into_partition(0, generic_settings_bytes, storage);
        copy_config_into_partition(1, special_settings_bytes, storage);
    }
    
    fn get_configuration_json(&mut self) -> serde_json::Value {
        
        let sensor_name_bytes = sensor_name_from_type_id(self.get_type_id().into());
        let sensor_name_str = core::str::from_utf8(&sensor_name_bytes).unwrap_or_default();

        json!({
            "id" : self.get_id(),
            "type" : sensor_name_str,
            "on_time_s": self.special_config.on_time_s,
            "off_time_s": self.special_config.off_time_s
        })
    }
    
    fn fit(&mut self, pairs: &[CalibrationPair]) -> Result<(), ()> {
        todo!()
    }
    
    fn clear_calibration(&mut self) {
        todo!()
    }
}