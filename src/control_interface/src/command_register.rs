use core::ffi::c_void;
use std::{collections::HashMap, ffi::CString};
static NUM_COMMANDS: usize = 32;

/// <div rustbindgen nocopy></div>
/// <div rustbindgen opaque></div>
#[repr(C)]
pub struct RegisteredCommand {
    object: *mut c_void,
    action: *mut c_void,
}
type CommandMap = HashMap<*mut CString, *mut RegisteredCommand>;
pub struct CommandRegister {
    command_map: CommandMap,
}

impl CommandRegister {
    fn new() -> Self {
        let mut map = CommandMap::new();
        map.reserve(NUM_COMMANDS);
        CommandRegister { command_map: map }
    }
    fn register_command(&mut self, command: *mut CString, registration: *mut c_void) {
        self.command_map
            .insert(command, registration as *mut RegisteredCommand);
    }
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
        assert_eq!(*registered_command, 0xdeadbeef as *mut RegisteredCommand);
    }
}
