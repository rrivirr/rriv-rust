
// codes and sensor names mapped to a sensor implementation

use rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;



#[derive(Copy,Clone,Debug)]
pub struct SensorDriverGeneralConfiguration {
    pub tag: [u8;6],
    pub sensor_type_id: u16,
    pub warmup: u16,
    pub burst_length: u8
}

impl SensorDriverGeneralConfiguration {
    pub fn new_from_bytes(bytes: &[u8] ) -> SensorDriverGeneralConfiguration {
        let settings = bytes.as_ptr().cast::<SensorDriverGeneralConfiguration>();
        unsafe {
            *settings
        }
    }
}


pub trait SensorDriver {
    fn setup(&mut self);
}

#[derive(Copy,Clone,Debug)]
pub struct GenericAnalogSpecialConfiguration {
    empty: [u8; 32]
}

impl GenericAnalogSpecialConfiguration {
    pub fn new_from_bytes(bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE] ) -> GenericAnalogSpecialConfiguration {
        // panic if bytes.len() != 32
        let settings = bytes.as_ptr().cast::<GenericAnalogSpecialConfiguration>();
        unsafe {
            *settings
        }
    }
}

pub struct GenericAnalog {
    general_config: SensorDriverGeneralConfiguration,
    special_config: GenericAnalogSpecialConfiguration
}

impl SensorDriver for GenericAnalog {
    fn setup(&mut self) {
        todo!()
    }
}

impl GenericAnalog {
    pub fn new(general_config: SensorDriverGeneralConfiguration, specific_config_bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE]) -> Self {
        
        let special_config = GenericAnalogSpecialConfiguration::new_from_bytes(specific_config_bytes);
        GenericAnalog {
            general_config,
            special_config
        }
    }
}


#[derive(Copy,Clone,Debug)]
pub struct AHT22SpecialConfiguration {
    empty: [u8; 32]
}

impl AHT22SpecialConfiguration {
    pub fn new_from_bytes(bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE] ) -> AHT22SpecialConfiguration {
        let settings = bytes.as_ptr().cast::<AHT22SpecialConfiguration>();
        unsafe {
            *settings
        }
    }
}


pub struct AHT22 {
    general_config: SensorDriverGeneralConfiguration,
    special_config: AHT22SpecialConfiguration
}

impl SensorDriver for AHT22 {
    fn setup(&mut self) {
        todo!()
    }
}

impl AHT22 {
    pub fn new(general_config: SensorDriverGeneralConfiguration, specific_config_bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE]) -> Self {
        
        let special_config = AHT22SpecialConfiguration::new_from_bytes(specific_config_bytes);
        AHT22 {
            general_config,
            special_config
        }
    }
}