use core::ffi::c_void;
use core::slice;
use core::str::from_utf8_unchecked;
use hashbrown::HashMap;

#[repr(C)]
#[derive(Eq, Hash, PartialEq)]
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
    Unknown = 36,
}
impl Command {
    pub fn from_cstr(cstr: *const i8, len: usize) -> Self {
        let str_slice =
            unsafe { from_utf8_unchecked(slice::from_raw_parts(cstr as *const u8, len)) };
        match str_slice {
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

#[repr(C)]
pub struct CommandRegistration {
    action_fn: extern "C" fn(*mut c_void),
    command: Command,
}

type CommandMap = HashMap<Command, extern "C" fn(*mut c_void)>;

pub struct CommandRegister {
    command_map: CommandMap,
}

impl CommandRegister {
    pub fn new(command_map: CommandMap) -> Self {
        CommandRegister { command_map }
    }
    pub fn register_command(&mut self, command: Command, action_fn: extern "C" fn(*mut c_void)) {
        self.command_map.insert(command, action_fn);
    }
    pub fn is_registered(&self, command: Command) -> bool {
        self.command_map.contains_key(&command)
    }
    pub fn get_action_fn(&self, command: Command) -> Option<extern "C" fn(*mut c_void)> {
        self.command_map.get(&command).copied()
    }
}

/*
Register the command with the CommandRegister
Return a raw pointer to the CommandRegister
*/
#[no_mangle]
pub unsafe extern "C" fn register_command(
    command: Command,
    action_fn: extern "C" fn(*mut c_void),
    command_register: *mut CommandRegister,
) -> *mut CommandRegister {
    (*command_register).register_command(command, action_fn);
    command_register
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_command_registration() {
        static NUM_COMMANDS: usize = 32;
        let command_map = HashMap::with_capacity(NUM_COMMANDS);
        let mut command_register = CommandRegister::new(command_map);
        let command = Command::DataloggerSet;
        extern "C" fn action_fn(_: *mut c_void) {}
        command_register.register_command(command, action_fn);
        assert!(command_register.is_registered(Command::DataloggerSet));
    }
}
