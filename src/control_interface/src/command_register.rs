use core::ffi::CStr;
use core::mem::size_of;
use core::ptr::null_mut;
use core::slice;
use core::str::from_utf8_unchecked;
use core::{alloc::Allocator, ffi::c_void};
use hashbrown::HashMap;
use serde_json::{json, Value};

static NUM_COMMANDS: usize = 32;

#[repr(C)]
pub enum Command {
    DataloggerSet,
    DataloggerGet,
    DataloggerReset,
    DataloggerSetMode,
    SensorSet,
    SensorGet,
    SensorRemove,
    SensorList,
    SensorCalibratePoint,
    SensorCalibrateFit,
    SensorReset,
    ActuatorSet,
    ActuatorGet,
    ActuatorRemove,
    ActuatorList,
    ActuatorReset,
    TelemeterSet,
    TelemeterGet,
    TelemeterRemove,
    TelemeterList,
    TelemeterReset,
    BoardVersion,
    BoardFirmwareWarranty,
    BoardFirmwareConditions,
    BoardFirmwareLicense,
    BoardRtcSet,
    BoardRtcGet,
    BoardRestart,
    BoardI2cList,
    BoardMemoryCheck,
    BoardMcuStop,
    BoardMcuSleep,
    BoardSignalExAdcHigh,
    BoardSignalExAdcLow,
    BoardSignal3v3BoostHigh,
    BoardSignal3v3BoostLow,
    Unknown,
}
impl Command {
    pub fn from_cstr(cstr: *const i8) -> Self {
        let str_slice = unsafe {
            from_utf8_unchecked(slice::from_raw_parts(cstr as *const u8, size_of::<i8>()))
        };
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
    object: Command,
}

type CommandMap = HashMap<Command, extern "C" fn(*mut c_void)>;

pub struct CommandRegister {
    command_map: CommandMap,
}

impl CommandRegister {
    pub fn new(allocator: dyn Allocator + Clone) -> Self {
        let mut map: CommandMap = HashMap::with_capacity_and_hasher_in(
            NUM_COMMANDS,
            ahash::RandomState::new(),
            allocator,
        );
        CommandRegister { command_map: map }
    }

    pub fn register_command(&mut self, command: Command, action_fn: extern "C" fn(*mut c_void)) {
        let registered_command = CommandRegistration {
            action_fn,
            object: command.object,
        };
        self.command_map.insert(command, registered_command);
    }

    fn handle_serial_command(serial_command: &str) {
        // Parse the JSON string into a serde_json::Value
        let command: Value = serde_json::from_str(serial_command).unwrap();

        // Extract the command, object, and parameters from the JSON object
        let action_str = command["command"].as_str().unwrap();
        let object_str = command["object"].as_str().unwrap();
        let command_str = format!("{}_{}", action_str, object_str)
            .to_cstr()
            .to_lowercase()
            .as_cstr();
        let parameters = command
            .as_object()
            .unwrap()
            .iter()
            .filter(|(k, _)| k != "command" && k != "object")
            .map(|(k, v)| (k.to_cstr(), v.to_cstr()))
            .collect();

        // join the command and object strings with and underscore
        // match the command string to the corresponding Command enum
        let command = match command_str {
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
            "board_signal_3v3_boost_high" => Command::BoardSignal3,
        };
        // Execute the command
        execute_command(command, parameters);
    }

    pub fn execute_command(&self, command: Command, object_def: ObjectDefinition) {
        if let Some(registered_command) = self.command_map.get(&command) {
            (registered_command.action_fn)(
                null_mut(),
                &object_def as *const ObjectDefinition as *mut c_void,
            );
        }
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
    use std::ffi::CString;
    #[test]
    fn test_command_register() {
        let mut command_register = CommandRegister::new();
        let mut command = CString::new("test").unwrap();
        let cmd: *mut CString = &mut command;
        let registration = 0xdeadbeef as *mut c_void;
        command_register.register_command(&mut command, registration);
        let registered_command = command_register.command_map.get(&cmd).unwrap();
        assert_eq!(*registered_command, 0xdeadbeef as *mut CommandRegistration);
    }
    #[test]
    fn test_command_deserialize_register_execute() {}
}
