

// codes and sensor names mapped to a sensor implementation

use rriv_board::EEPROM_SENSOR_SETTINGS_SIZE;


pub mod types;
pub mod mcp9808;
pub mod generic_analog;
pub mod ring_temperature;
pub mod heater;
mod AHT22;

