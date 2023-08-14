extern crate alloc;
use alloc::format;
use core::ffi::c_char;
use hashbrown::HashMap;

/// NOTE: Since this has a C compatible representation, it could be used in the FFI
/// we use from_str when processing commands from the serial side anyway though..
#[repr(u8)]
#[derive(Debug, Eq, Hash, PartialEq)]
pub enum Command {
    DataloggerSet = 0,
    DataloggerGet = 1,
    DataloggerReset = 2,
    DataloggerSetMode = 3,
    SensorSet = 4,
    SensorGet = 5,
    SensorRemove = 6,
    SensorList = 7,
    SensorCalibratePoint = 8,
    SensorCalibrateFit = 9,
    SensorReset = 10,
    ActuatorSet = 11,
    ActuatorGet = 12,
    ActuatorRemove = 13,
    ActuatorList = 14,
    ActuatorReset = 15,
    TelemeterSet = 16,
    TelemeterGet = 17,
    TelemeterRemove = 18,
    TelemeterList = 19,
    TelemeterReset = 20,
    BoardVersion = 21,
    BoardFirmwareWarranty = 22,
    BoardFirmwareConditions = 23,
    BoardFirmwareLicense = 24,
    BoardRtcSet = 25,
    BoardRtcGet = 26,
    BoardRestart = 27,
    BoardI2cList = 28,
    BoardMemoryCheck = 29,
    BoardMcuStop = 30,
    BoardMcuSleep = 31,
    BoardSignalExAdcHigh = 32,
    BoardSignalExAdcLow = 33,
    BoardSignal3v3BoostHigh = 34,
    BoardSignal3v3BoostLow = 35,
    Unknown = 36, // !!! `Unknown` needs to be the last command, its value is used to get the number of commands see CommandRegistry::new !!!
}
impl Command {
    pub fn from_str(cmd_str: &str) -> Self {
        match cmd_str {
            "datalogger_set" => Command::DataloggerSet,
            "datalogger_get" => Command::DataloggerGet,
            "datalogger_reset" => Command::DataloggerReset,
            "datalogger_set_mode" => Command::DataloggerSetMode,
            "sensor_set" => Command::SensorSet,
            "sensor_get" => Command::SensorGet,
            "sensor_remove" => Command::SensorRemove,
            "sensor_list" => Command::SensorList,
            "sensor_calibrate_point" => Command::SensorCalibratePoint,
            "sensor_calibrate_fit" => Command::SensorCalibrateFit,
            "sensor_reset" => Command::SensorReset,
            "actuator_set" => Command::ActuatorSet,
            "actuator_get" => Command::ActuatorGet,
            "actuator_remove" => Command::ActuatorRemove,
            "actuator_list" => Command::ActuatorList,
            "actuator_reset" => Command::ActuatorReset,
            "telemeter_set" => Command::TelemeterSet,
            "telemeter_get" => Command::TelemeterGet,
            "telemeter_remove" => Command::TelemeterRemove,
            "telemeter_list" => Command::TelemeterList,
            "telemeter_reset" => Command::TelemeterReset,
            "board_version" => Command::BoardVersion,
            "board_firmware_warranty" => Command::BoardFirmwareWarranty,
            "board_firmware_conditions" => Command::BoardFirmwareConditions,
            "board_firmware_license" => Command::BoardFirmwareLicense,
            "board_rtc_set" => Command::BoardRtcSet,
            "board_rtc_get" => Command::BoardRtcGet,
            "board_restart" => Command::BoardRestart,
            "board_i2c_list" => Command::BoardI2cList,
            "board_memory_check" => Command::BoardMemoryCheck,
            "board_mcu_stop" => Command::BoardMcuStop,
            "board_mcu_sleep" => Command::BoardMcuSleep,
            "board_signal_ex_adc_high" => Command::BoardSignalExAdcHigh,
            "board_signal_ex_adc_low" => Command::BoardSignalExAdcLow,
            "board_signal_3v3_boost_high" => Command::BoardSignal3v3BoostHigh,
            "board_signal_3v3_boost_low" => Command::BoardSignal3v3BoostLow,
            _ => Command::Unknown,
        }
    }
}

type CommandMap = HashMap<Command, extern "C" fn(*const c_char)>;

pub struct CommandRegistry {
    command_map: CommandMap,
}

impl CommandRegistry {
    pub fn new() -> Self {
        // NOTE: This is a hack to get the number of commands. There is no better way.
        let num_commands = Command::Unknown as u8 as usize + 1;
        let command_map = HashMap::with_capacity(num_commands);
        CommandRegistry { command_map }
    }
    pub fn register_command(&mut self, command: Command, action_fn: extern "C" fn(*const c_char)) {
        self.command_map.insert(command, action_fn);
    }
    pub fn register_command_str(&mut self, command: &str, action_fn: extern "C" fn(*const c_char)) {
        let command = Command::from_str(command);
        self.register_command(command, action_fn);
    }
    // NOTE: This is not needed since get_action_fn returns an Option, but may be useful for testing.
    pub fn is_registered(&self, command: Command) -> bool {
        self.command_map.contains_key(&command)
    }
    pub fn get_action_fn(&self, command: Command) -> Option<extern "C" fn(*const c_char)> {
        self.command_map.get(&command).copied()
    }
    pub fn get_command_from_parts(&self, object: &str, action: &str) -> Command {
        let command_str = format!("{}_{}", object, action);
        Command::from_str(&command_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_command_registration() {
        let mut command_registry = CommandRegistry::new();
        let command = Command::DataloggerSet;
        extern "C" fn action_fn_1(_: *mut c_void) {}
        command_registry.register_command(command, action_fn_1);
        assert!(command_registry.is_registered(Command::DataloggerSet));
    }
    #[test]
    fn test_command_registration_str() {
        let mut command_registry = CommandRegistry::new();
        let cmd_str = "datalogger_set";
        extern "C" fn action_fn_2(_: *mut c_void) {}
        command_registry.register_command_str(cmd_str, action_fn_2);
        assert!(command_registry.is_registered(Command::DataloggerSet));
    }
    #[test]
    fn test_get_command_from_parts() {
        let command_registry = CommandRegistry::new();
        let command = Command::DataloggerSet;
        let object = "datalogger";
        let action = "set";
        let command_from_parts = command_registry.get_command_from_parts(object, action);
        assert_eq!(command, command_from_parts);
    }
}
