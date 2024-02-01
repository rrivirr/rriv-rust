
// codes and sensor names mapped to a sensor implementation

use rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;



#[derive(Copy,Clone,Debug)]
pub struct SensorDriverGeneralConfiguration {
    pub id: [u8;6],
    pub sensor_type_id: u16,
    pub warmup: u16,
    pub burst_repetitions: u8
}

impl SensorDriverGeneralConfiguration {
    pub fn new_from_values ( 
        id: [u8;6],
        sensor_type_id: u16,
     ) -> SensorDriverGeneralConfiguration{
            Self {
                id: id,
                sensor_type_id: sensor_type_id,
                warmup: 0,
                burst_repetitions: 1
            }
        }

    pub fn new_from_bytes(bytes: &[u8] ) -> SensorDriverGeneralConfiguration {
        let settings = bytes.as_ptr().cast::<SensorDriverGeneralConfiguration>();
        unsafe {
            *settings
        }
    }
}


pub trait SensorDriver {
    fn setup(&mut self);
    fn get_id(&mut self) -> [u8;6];
}

pub trait ActuatorDriver {
    fn setup(&mut self);
}

pub trait TelemeterDriver {
    fn setup(&mut self);
}

#[derive(Copy,Clone,Debug)]
pub struct GenericAnalogSpecialConfiguration {
    m: f64,
    b: f64,
    sensor_port: u8,
    empty: [u8; 23]
}

impl GenericAnalogSpecialConfiguration {
    pub fn new(value: serde_json::Value) -> GenericAnalogSpecialConfiguration {
        let mut sensor_port: u8 = 0;
        match value["sensor_port"] {
            serde_json::Value::Number(number) => {
                if let Some(number) = number.as_u64() {
                    let number: Result<u8, _> = number.try_into();
                    match number {
                        Ok(number) => {
                            sensor_port = number;
                        }
                        Err(_) => todo!(),

                    }
                }
            }
            _ => todo!(),
        }

        return Self {
            m: 0.0,
            b: 0.0,
            sensor_port: sensor_port,
            empty: [b'\0'; 23]
        }
    }

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

    fn get_id(&mut self) -> [u8;6] {
        self.general_config.id.clone()
    }
}

impl GenericAnalog {
    pub fn new(general_config: SensorDriverGeneralConfiguration, special_config_bytes: &[u8; rriv_board::EEPROM_SENSOR_SPECIAL_SETTINGS_SIZE]) -> Self {
        
        let special_config = GenericAnalogSpecialConfiguration::new_from_bytes(special_config_bytes);
        GenericAnalog {
            general_config,
            special_config
        }
    }

    pub fn new_from_configs(general_config: SensorDriverGeneralConfiguration, special_config: GenericAnalogSpecialConfiguration) -> Self  {
        GenericAnalog {
            general_config,
            special_config
        }
    }
}


#[derive(Copy,Clone,Debug)]
pub struct AHT22SpecialConfiguration {
    wait_time : usize,
    empty: [u8; 28]
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

    fn get_id(&mut self) -> [u8;6] {
        self.general_config.id.clone()
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