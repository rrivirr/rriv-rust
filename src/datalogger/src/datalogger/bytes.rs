use rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;

use crate::drivers::types::SENSOR_SETTINGS_PARTITION_SIZE;

// these functions create sized byte arrays for use in the datalogger

pub fn empty_sensor_settings() ->  [u8; EEPROM_SENSOR_SETTINGS_SIZE] {
    [0; EEPROM_SENSOR_SETTINGS_SIZE]
}

pub fn empty_sensor_settings_partition() ->  [u8; SENSOR_SETTINGS_PARTITION_SIZE] {
    [0; SENSOR_SETTINGS_PARTITION_SIZE]
}

pub fn empty_sensors_settings() -> [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE* rriv_board::EEPROM_TOTAL_SENSOR_SLOTS] {
    [0; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS]
}