use super::types::*;

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

    pub fn empty()  -> MCP9808TemperatureDriverSpecialConfiguration {
        Self { empty: [b'\0'; 32] }
    }
}

pub struct MCP9808TemperatureDriver {
    general_config: SensorDriverGeneralConfiguration,
    special_config: MCP9808TemperatureDriverSpecialConfiguration,
    measured_parameter_values: [f64; 1],
    address: u8,
}

impl SensorDriver for MCP9808TemperatureDriver {
    fn setup(&mut self) {
        todo!()
    }

    getters!();

    fn get_measured_parameter_count(&mut self) -> usize {
        1
    }

    fn get_measured_parameter_value(&mut self, index: usize) -> Result<f64, ()> {
        if(self.measured_parameter_values[index] == f64::MAX){
            Err(())
        } else {
            Ok(self.measured_parameter_values[index])
        }
    }

    fn get_measured_parameter_identifier(&mut self, index: usize) -> [u8;16] {
        let mut buf = [0u8;16];
        buf[0] = b'T';
        buf[1] = b'\0';
        return buf;
    }

    fn take_measurement(&mut self, board: &mut dyn rriv_board::SensorDriverServices) {
        
        let message = [AMBIENT_TEMPERATURE_REGISTER_ADDRESS];
        let mut buffer: [u8; 2] = [0; 2];
        match board.ic2_write(self.address, &message ) {
            Ok(_) => {
                // continue
            }
            Err(_) =>  {
                // read error, improve this output later by returning a Result<> from take measurement
                self.measured_parameter_values[0] = f64::MAX;
                return;
            },

        }
        match board.ic2_read(self.address, &mut buffer) {
            Ok(_) => {
                // continue
            },
            Err(_) => {
                // read error, improve this output later by returning a Result<> from take measurement
                self.measured_parameter_values[0] = f64::MAX;
                return;
            },
        }

        //Convert the temperature data
        //First Check flag bits
        // follows from https://ww1.microchip.com/downloads/en/DeviceDoc/MCP9808-0.5C-Maximum-Accuracy-Digital-Temperature-Sensor-Data-Sheet-DS20005095B.pdf
        let mut upper_byte: u16 = buffer[0].into();
        let mut lower_byte: u16 = buffer[1].into();
        if ((upper_byte & 0x80) == 0x80){ //T A ≥ TCRIT
        }
        if ((upper_byte & 0x40) == 0x40){ //T A > TUPPER
        }
        if ((upper_byte & 0x20) == 0x20){ //T A < TLOWER
        }

        upper_byte = upper_byte & 0x1F; //Clear flag bits
        let mut temperature: f64 = 0.0;
        if (upper_byte & 0x10) == 0x10 { //T A < 0°Ca
            upper_byte = upper_byte & 0x0F;//Clear SIGN
            let upper_byte: f64 = upper_byte.into();
            let lower_byte: f64 = lower_byte.into();
            temperature = 256.0 - (upper_byte * 16.0 + lower_byte / 16.0);
        } else { //T A ≥ 0°C

            let upper_byte: f64 = upper_byte.into();
            let lower_byte: f64 = lower_byte.into();
            temperature = upper_byte * 16.0 + lower_byte / 16.0;
            //Temperature = Ambient Temperature (°C)
        }

        self.measured_parameter_values[0] = temperature;
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
            address: 0b0011000
        }
    }

    pub fn new_with_address(
        general_config: SensorDriverGeneralConfiguration,
        special_config: MCP9808TemperatureDriverSpecialConfiguration,
        address: u8
    ) -> Self {
        MCP9808TemperatureDriver {
            general_config,
            special_config,
            measured_parameter_values: [0.0],
            address: address
        }
    }

}