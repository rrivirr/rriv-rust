#![cfg_attr(not(test), no_std)]

use control_interface::command_recognizer::{CommandData, CommandRecognizer};
use control_interface::command_registry::{Command, CommandRegistry};
use rriv_0_4::{Board, RXProcessor};

extern crate alloc;
use alloc::boxed::Box;

use core::cell::RefCell;
use core::ffi::{c_char, CStr};
use core::ops::{Deref, DerefMut};
use cortex_m::interrupt::Mutex;
use serde_json::Value;

static COMMAND_DATA: Mutex<RefCell<Option<CommandData>>> = Mutex::new(RefCell::new(None));
static SETUP_DONE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

pub struct CommandService {
    registry: CommandRegistry,
}

impl CommandService {
    pub fn new() -> Self {
        // set the static, shareable command data
        cortex_m::interrupt::free(|cs| {
            *COMMAND_DATA.borrow(cs).borrow_mut() = Some(CommandData::default());
        });
        CommandService {
            registry: CommandRegistry::new(),
        }
    }

    /// set the global rx processor
    pub fn setup(&mut self, board: &mut Board) {
        // return if already setup
        if self.check_setup() {
            return;
        }
        // create a new CharacterProcessor on the heap and leak it to a static lifetime
        let char_processor = Box::<CharacterProcessor<'static>>::leak(Box::new(
            CharacterProcessor::new(self.get_command_data()),
        ));
        // pass a pointer to the leaked processor to Board::set_rx_processor
        board.set_rx_processor(Box::new(char_processor));
        // set the setup flag to true
        cortex_m::interrupt::free(|cs| {
            SETUP_DONE.borrow(cs).replace(true);
        });
    }

    /// register a command with two &strs, object and action, and a C function pointer that matches registry.register_command's second argument
    /// this calls registry.get_command_from_parts to get a Command object, then calls registry.register_command
    pub fn register_command(
        &mut self,
        object: &str,
        action: &str,
        ffi_cb: extern "C" fn(*const c_char),
    ) {
        let command = self.registry.get_command_from_parts(object, action);
        self.registry.register_command(command, ffi_cb);
    }
    fn check_setup(&self) -> bool {
        cortex_m::interrupt::free(|cs| {
            if !*SETUP_DONE.borrow(cs).borrow() {
                return false;
            } else {
                return true;
            }
        })
    }
    /// get access to the static, shareable command data
    fn get_command_data(&self) -> &'static Mutex<RefCell<Option<CommandData>>> {
        &COMMAND_DATA
    }
    fn pending_message_count(&self) -> usize {
        cortex_m::interrupt::free(|cs| {
            let command_data = self.get_command_data().borrow(cs).borrow();
            if let Some(command_data) = command_data.deref() {
                CommandRecognizer::pending_message_count(&command_data)
            } else {
                0
            }
        })
    }

    fn take_command(&mut self) -> Result<[u8; 100], ()> {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.get_command_data().borrow(cs).borrow_mut();
            if let Some(command_data) = command_data.deref_mut() {
                Ok(CommandRecognizer::take_command(command_data))
            } else {
                Err(())
            }
        })
    }

    pub fn run_loop_iteration(&mut self) {
        if self.pending_message_count() > 0 {
            if let Ok(command) = self.take_command() {
                self.handle_serial_command(command);
            }
        }
    }

    fn handle_serial_command(&self, serial_command_bytes: [u8; 100]) {
        let command_data_str = core::str::from_utf8(&serial_command_bytes).unwrap();
        // Parse the JSON string into a serde_json::Value
        let data_json: Value = serde_json::from_str(command_data_str).unwrap();
        // Extract the command and object strings from the JSON
        let object_str = data_json["object"].as_str().unwrap();
        let action_str = data_json["command"].as_str().unwrap();
        // join the command and object strings with and underscore
        let command = self.registry.get_command_from_parts(object_str, action_str);
        if let Ok(cmd_bytes_cstr) = CStr::from_bytes_with_nul(&serial_command_bytes) {
            self.execute_command(command, cmd_bytes_cstr);
        }
    }

    fn execute_command(&self, command: Command, command_cstr: &CStr) {
        // check if there is a function pointer registered for this command
        if let Some(ffi_cb) = self.registry.get_action_fn(command) {
            // call the registered function pointer and pass ownership of the command bytes to C
            ffi_cb(command_cstr.as_ptr() as *mut c_char);
        } else {
            // TODO: send a message back to the host that the command was not recognized
        }
    }
}

/// I believe the lifetimes here are generic over the lifetime of the CommandData
pub struct CharacterProcessor<'a> {
    command_data: &'a Mutex<RefCell<Option<CommandData>>>,
}

impl<'a> CharacterProcessor<'a> {
    pub fn new(command_data: &Mutex<RefCell<Option<CommandData>>>) -> CharacterProcessor {
        CharacterProcessor { command_data }
    }
}

impl<'a, 'b> RXProcessor for CharacterProcessor<'a> {
    fn process_character(&self, character: u8) {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.command_data.borrow(cs).borrow_mut();
            if let Some(command_data) = command_data.deref_mut() {
                CommandRecognizer::process_character(command_data, character);
            }
        })
    }
}
