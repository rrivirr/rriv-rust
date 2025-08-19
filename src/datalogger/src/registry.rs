use alloc::boxed::Box;
use rtt_target::rprintln;

use crate::drivers::{ types::{SensorDriver, SensorDriverGeneralConfiguration, SENSOR_SETTINGS_PARTITION_SIZE}};
use util::{any_as_u8_slice};


const SENSOR_NAMES: [&str; 9] = [
    "no_match",
    "generic_analog",
    "atlas_ec",
    "aht20",
    "mcp_9808",
    "ring_temperature",
    "timed_switch",
    "ds18b20",
    "k30_co2",
];

pub fn sensor_type_id_from_name(name: &str) -> u16 {
    for i in 0..SENSOR_NAMES.len() {
        if name == SENSOR_NAMES[i] {
            return u16::try_from(i).ok().unwrap();
        }
    }
    return 0;
}

pub fn sensor_name_from_type_id(id: usize) -> [u8; 16] {
    let mut rval = [b'\0'; 16];
    let name = SENSOR_NAMES[id];
    rval[..name.len()].copy_from_slice(name.as_bytes());
    rval
}


/* start registry WIP */

#[macro_export]
macro_rules! driver_create_functions {
    ($driver:ty, $special_settings_type:ty) => {
        (
            |general_settings: SensorDriverGeneralConfiguration,
             special_settings_values: serde_json::Value|
             -> (Box<dyn SensorDriver>, [u8; SENSOR_SETTINGS_PARTITION_SIZE]) {
                let special_settings =
                    <$special_settings_type>::new_from_values(special_settings_values); // ok
                let driver = <$driver>::new(general_settings, special_settings); // seems ok

                let bytes: &[u8] = unsafe { any_as_u8_slice(&special_settings) }; // must be this one, maybe size comes back wrong
                if bytes.len() != SENSOR_SETTINGS_PARTITION_SIZE {
                    // special_settings_type does not confrm to expected size.  this is a development fault
                    rprintln!("{} is wrong size {}", "<$special_settings_type>", bytes.len());
                    // causes crash later.  other way to gracefully handle?
                    panic!();
                }
                let mut bytes_sized: [u8; SENSOR_SETTINGS_PARTITION_SIZE] =
                    [0; SENSOR_SETTINGS_PARTITION_SIZE];
                let copy_size = if bytes.len() >= SENSOR_SETTINGS_PARTITION_SIZE { // this was supposed to make it safe...
                    SENSOR_SETTINGS_PARTITION_SIZE
                } else {
                    bytes.len()
                };
                bytes_sized[..SENSOR_SETTINGS_PARTITION_SIZE].copy_from_slice(&bytes[0..copy_size]);

                (Box::new(driver), bytes_sized)
            },
            |general_settings: SensorDriverGeneralConfiguration,
             special_settings_slice: &[u8]|
             -> Box<dyn SensorDriver> {
                let mut bytes: [u8; SENSOR_SETTINGS_PARTITION_SIZE] =
                    [0; SENSOR_SETTINGS_PARTITION_SIZE];
                bytes.clone_from_slice(special_settings_slice);
                let special_settings = <$special_settings_type>::new_from_bytes(bytes);
                let driver = <$driver>::new(general_settings, special_settings);
                Box::new(driver)
            },
        )
    };
}

type DriverCreateFunctions = Option<(
    fn(
        SensorDriverGeneralConfiguration,
        serde_json::Value,
    ) -> (Box<dyn SensorDriver>, [u8; SENSOR_SETTINGS_PARTITION_SIZE]),
    fn(SensorDriverGeneralConfiguration, &[u8]) -> Box<dyn SensorDriver>,
)>;

pub fn get_registry() -> [DriverCreateFunctions; 256] {
    const ARRAY_INIT_VALUE: DriverCreateFunctions = None;
    let mut driver_create_functions: [DriverCreateFunctions; 256] = [ARRAY_INIT_VALUE; 256];
    driver_create_functions[0] = None;
    driver_create_functions[1] = Some(driver_create_functions!(
        crate::drivers::generic_analog::GenericAnalog,
        crate::drivers::generic_analog::GenericAnalogSpecialConfiguration
    )); 
    driver_create_functions[2] = None;
    driver_create_functions[3] = Some(driver_create_functions!(
        crate::drivers::aht20::AHT20,
        crate::drivers::aht20::AHT20SpecialConfiguration
    ));
    driver_create_functions[4] = Some(driver_create_functions!(
        crate::drivers::mcp9808::MCP9808TemperatureDriver,
        crate::drivers::mcp9808::MCP9808TemperatureDriverSpecialConfiguration
    )); 
    driver_create_functions[5] = Some(driver_create_functions!(
        crate::drivers::ring_temperature::RingTemperatureDriver,
        crate::drivers::ring_temperature::RingTemperatureDriverSpecialConfiguration
    ));
    driver_create_functions[6] = Some(driver_create_functions!(
        crate::drivers::heater::TimedSwitch, 
        crate::drivers::heater::TimedSwitchSpecialConfiguration
    ));
    driver_create_functions[7] = Some(driver_create_functions!(
        crate::drivers::ds18b20::Ds18b20,
        crate::drivers::ds18b20::Ds18b20SpecialConfiguration
    ));
    driver_create_functions[8] = Some(driver_create_functions!(
        crate::drivers::k30_co2::K30CO2, 
        crate::drivers::k30_co2::K30CO2SpecialConfiguration
    ));

    driver_create_functions
}

/* end registry WIP */