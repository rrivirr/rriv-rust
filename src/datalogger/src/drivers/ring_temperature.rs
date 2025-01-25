// use alloc::fmt::format;

use crate::any_as_u8_slice;

use super::mcp9808::*;

use super::types::*;
use alloc::format;
use alloc::boxed::Box;
use rtt_target::rprint;

// TODO: calibration offsets for all 6 sensors need to be stored and loaded into this driver, and written to EEPROM.


#[derive(Copy, Clone, Debug)]
pub struct RingTemperatureDriverSpecialConfiguration {
    address_offset: u8,
    empty: [u8; 31], // 32
}

impl RingTemperatureDriverSpecialConfiguration {
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

const TEMPERATURE_SENSORS_ON_RING:u8 = 6;

pub struct RingTemperatureDriver {
    general_config: SensorDriverGeneralConfiguration,
    special_config: RingTemperatureDriverSpecialConfiguration,
    measured_parameter_values: [f64; 6],
    sensor_drivers: [MCP9808TemperatureDriver; 6],
}


const INDEX_TO_BYTE_CHAR: [u8; 6] = [b'0',b'1',b'2',b'3',b'4',b'5'];
const NUMBER_OF_SENSORS: u8 = 6;


fn copy_config_into_partition(partition: u8, bytes: &[u8], storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]){
    // let generic_settings_bytes: &[u8] = unsafe { any_as_u8_slice(&self.general_config) };
    // let mut bytes_sized: [u8; EEPROM_SENSOR_SETTINGS_SIZE] = [0; EEPROM_SENSOR_SETTINGS_SIZE];
    let copy_size =
        if bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
            SENSOR_SETTINGS_PARTITION_SIZE
        } else {
            bytes.len()
        };
        let offset = SENSOR_SETTINGS_PARTITION_SIZE * partition;
        storage[offset..offset+copy_size].copy_from_slice(&bytes[0..copy_size]); 
}


impl SensorDriver<RingTemperatureDriverSpecialConfiguration> for RingTemperatureDriver {

    fn new(
        general_config: SensorDriverGeneralConfiguration,
        special_config: RingTemperatureDriverSpecialConfiguration,
    ) -> Self {
        

        let mut addresses: [u8;6] = [0b0011000, 0b0011001, 0b0011010, 0b0011011, 0b0011100, 0b0011101];
        for i in 0..6 {
            addresses[i] = addresses[i] + special_config.address_offset;
        }
        
        RingTemperatureDriver {
            general_config,
            special_config,
            measured_parameter_values: [0.0; 6],
            sensor_drivers: [
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[0]),
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[1]),
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[2]),
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[3]),
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[4]),
                MCP9808TemperatureDriver::new_with_address(SensorDriverGeneralConfiguration::empty(), 
                                                           MCP9808TemperatureDriverSpecialConfiguration::empty(), 
                                                           addresses[5]),
            ],
        }
    }

  
    fn get_configuration_bytes(&self, storage: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE]) {

        // right now this just gets the bytes
        // but the special settings probably should be consisted as member variables and copied back to the storage struct

        let generic_settings_bytes: &[u8] = unsafe {  };
        let special_settings_bytes: &[u8] =  unsafe { any_as_u8_slice(&self.special_config) };

        copy_config_into_partition(0, generic_settings_bytes, storage);
        copy_config_into_partition(1, special_settings_bytes, storage);

        // let mut bytes_sized: [u8; EEPROM_SENSOR_SETTINGS_SIZE] = [0; EEPROM_SENSOR_SETTINGS_SIZE];
        // let copy_size =
        //     if generic_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
        //         SENSOR_SETTINGS_PARTITION_SIZE
        //     } else {
        //         generic_settings_bytes.len()
        //     };
        // bytes[..copy_size].copy_from_slice(&generic_settings_bytes[0..copy_size]);

        // // get the special settings as bytes
        // let copy_size =
        //     if special_settings_bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE {
        //         SENSOR_SETTINGS_PARTITION_SIZE
        //     } else {
        //         special_settings_bytes.len()
        //     };
        // bytes[SENSOR_SETTINGS_PARTITION_SIZE
        //     ..(SENSOR_SETTINGS_PARTITION_SIZE + copy_size)]
        //     .copy_from_slice(&self.special_config[0..copy_size]);
    }

    fn setup(&mut self) {
        
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        6
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        if(self.measured_parameter_values[index] == f64::MAX){
            Err(())
        } else {
            Ok(self.measured_parameter_values[index])
        }    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        let mut buf : [u8;16] = [0u8; 16];
        buf[0] = b'T';
        buf[1] = INDEX_TO_BYTE_CHAR[index];
        buf[2] = b'\0';
        return buf;
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        
        for i in 0..6 {
            self.sensor_drivers[i].take_measurement(board);
            self.measured_parameter_values[i] = match self.sensor_drivers[i].get_measured_parameter_value(0){
                Ok(value) => value,
                Err(_) => { f64::MAX },
            }

        }
        
    }

    fn clear_calibration(&mut self){
        for i in 0..self.sensor_drivers.len() {
            self.sensor_drivers[i].clear_calibration();
        }
    }
    
    fn fit(&mut self, pairs: &[CalibrationPair]) -> Result<(), ()>{
        // validate
        if pairs.len() != 1 {
            return Err(());
        }

        if pairs[0].values.len() < 6 {
            // TODO: check for zeros, not for len().  len is constant
            rprint!("not enough values to calibrate");
            return Err(());
        }

        // fit
        let single = & pairs[0];
        let point = single.point;
        let values = & single.values;
        for i in 0..NUMBER_OF_SENSORS as usize {
            let pairs = CalibrationPair {
                point: point,
                values: Box::new([values[i]])
            };
            let result = self.sensor_drivers[i].fit(&[pairs]);
            match result {
                Ok(_) =>  true,
                Err(_) => return Err(()),
            };
        }
        Ok(())

    }
    
    
        
}


