use alloc::boxed::Box;
use bitfield_struct::bitfield;

// codes and sensor names mapped to a sensor implementation

use rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;

pub const SENSOR_SETTINGS_PARTITION_SIZE: usize = 32; // partitioning is part of the driver implemention, and not meaningful at the EEPROM level
pub type SensorGeneralSettingsSlice = [u8; SENSOR_SETTINGS_PARTITION_SIZE];
pub type SensorSpecialSettingsSlice = [u8; SENSOR_SETTINGS_PARTITION_SIZE];

#[derive(Copy, Clone, Debug)]
pub struct SensorDriverGeneralConfiguration {
    pub id: [u8; 6],
    pub sensor_type_id: u16,
    pub warmup: u16,
    pub burst_repetitions: u8,
}

impl SensorDriverGeneralConfiguration {
    pub fn new(id: [u8; 6], sensor_type_id: u16) -> SensorDriverGeneralConfiguration {
        Self {
            id: id,
            sensor_type_id: sensor_type_id,
            warmup: 0,
            burst_repetitions: 1,
        }
    }

    pub fn new_from_bytes(
        bytes: &[u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> SensorDriverGeneralConfiguration {
        let settings = bytes.as_ptr().cast::<SensorDriverGeneralConfiguration>();
        unsafe { *settings }
    }
}

pub trait SensorDriver {
    fn setup(&mut self);
    fn get_id(&mut self) -> [u8; 6];
    fn get_type_id(&mut self) -> u16;
    // fn get_measurement_technology(&mut self) -> usize; // TODO: unnecessary for now, unless we split SensorDriverServices into different types of services collections
    fn get_measured_parameter_count(&mut self) -> usize;
    fn get_measured_parameter_value(&mut self, index: usize) -> f32;
    fn get_measured_parameter_identifier(&mut self, index: usize) -> &str;

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices);
}

pub trait ADCSensorDriver {
    
}

pub trait ActuatorDriver {
    fn setup(&mut self);
}

pub trait TelemeterDriver {
    fn setup(&mut self);
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

macro_rules! getters {
    () => {
        fn get_id(&mut self) -> [u8; 6] {
            self.general_config.id.clone()
        }

        fn get_type_id(&mut self) -> u16 {
            self.general_config.sensor_type_id.clone()
        }
    };
}

#[derive(Copy, Clone, Debug)]
pub struct MCP9808TemperatureDriverSpecialConfiguration {
    empty: [u8; 32], // 32
}

impl MCP9808TemperatureDriverSpecialConfiguration {
    pub fn new_from_values(
        value: serde_json::Value,
    ) -> MCP9808TemperatureDriverSpecialConfiguration {
        Self { empty: [b'\0'; 32] }
    }
    pub fn new_from_bytes(
        bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE],
    ) -> MCP9808TemperatureDriverSpecialConfiguration {
        Self { empty: [b'\0'; 32] }
    }
}

pub struct MCP9808TemperatureDriver {
    general_config: SensorDriverGeneralConfiguration,
    special_config: MCP9808TemperatureDriverSpecialConfiguration,
    measured_parameter_values: [f32; 1],
}

impl SensorDriver for MCP9808TemperatureDriver {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        1
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> f32 {
        self.measured_parameter_values[index]
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> &str {
        return "T";
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        
        let message = [AMBIENT_TEMPERATURE_REGISTER_ADDRESS];
        let mut buffer: [u8; 2] = [0; 2];
        board.ic2_write(self.address_byte(true), &message );
        board.ic2_read(self.address_byte(false), &mut buffer);

        //Convert the temperature data
        //First Check flag bits
        // follows from https://ww1.microchip.com/downloads/en/DeviceDoc/MCP9808-0.5C-Maximum-Accuracy-Digital-Temperature-Sensor-Data-Sheet-DS20005095B.pdf
        let mut upperByte: u16 = buffer[0].into();
        let mut lowerByte: u16 = buffer[1].into();
        if ((upperByte & 0x80) == 0x80){ //T A ≥ TCRIT
        }
        if ((upperByte & 0x40) == 0x40){ //T A > TUPPER
        }
        if ((upperByte & 0x20) == 0x20){ //T A < TLOWER
        }

        upperByte = upperByte & 0x1F; //Clear flag bits
        let mut temperature = 0;
        if ((upperByte & 0x10) == 0x10){ //T A < 0°C
            upperByte = upperByte & 0x0F;//Clear SIGN
            temperature = 256 - (upperByte * 16 + lowerByte / 16);
        } else { //T A ≥ 0°C
            temperature = (upperByte * 16 + lowerByte / 16);
            //Temperature = Ambient Temperature (°C)
        }

        self.measured_parameter_values[0] = temperature.into();
    }
        
}

const AMBIENT_TEMPERATURE_REGISTER_ADDRESS: u8 = 0x05;

impl MCP9808TemperatureDriver {
    pub fn new(
        general_config: SensorDriverGeneralConfiguration,
        special_config: MCP9808TemperatureDriverSpecialConfiguration,
    ) -> Self {
        MCP9808TemperatureDriver {
            general_config,
            special_config,
            measured_parameter_values: [0.0],
        }
    }


    pub fn address_byte(&mut self, write: bool) -> u8 {
        let base_address: u8 = 0b00110000;
        let offset = 0;
        let address: u8 = base_address + offset << 1;
        if write {
            address & 0xFE
        } else {
            address | 0x01
        }
    }
}

pub struct GenericAnalog {
    general_config: SensorDriverGeneralConfiguration,
    special_config: GenericAnalogSpecialConfiguration,
    measured_parameter_values: [f32; 2],
}

impl SensorDriver for GenericAnalog {
    fn setup(&mut self) {
        todo!()
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
            1 => todo!("exadc not implemented"),
            2_usize.. => todo!("other adcs not implemented"),
        }
        self.measured_parameter_values[0] = value.into();
        self.measured_parameter_values[0] = value.into();

    }

    fn get_measured_parameter_count(&mut self) -> usize {
        return 2;
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> f32 {
        return self.measured_parameter_values[index];
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> &str {
        let identifiers = ["raw", "cal"];
        if index > 1 {
            return "invalid";
        }
        return identifiers[index];
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

    fn get_measured_parameter_value(&mut self, index: usize) -> f32 {
        todo!()
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> &str {
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
