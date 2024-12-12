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

    pub fn empty()-> SensorDriverGeneralConfiguration {
        Self {
            id: [0u8;6],
            sensor_type_id: 0,
            warmup: 0,
            burst_repetitions: 0,
        }
    }
}

pub trait SensorDriver {
    fn setup(&mut self);
    fn get_id(&mut self) -> [u8; 6];
    fn get_type_id(&mut self) -> u16;
    // fn get_measurement_technology(&mut self) -> usize; // TODO: unnecessary for now, unless we split SensorDriverServices into different types of services collections
    fn get_measured_parameter_count(&mut self) -> usize;
    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()>;
    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16];

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices);
}


// pub trait ADCSensorDriver {
    
// }

pub trait ActuatorDriver {
    fn setup(&mut self);
}

pub trait TelemeterDriver {
    fn setup(&mut self, board: &mut dyn rriv_board::TelemetryDriverServices);
    fn upload_measurement(&mut self, board: &mut dyn rriv_board::TelemetryDriverServices);
    // error handling
    // busy i.e. don't power down
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

pub(crate) use getters;

