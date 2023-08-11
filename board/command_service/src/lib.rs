#![cfg_attr(not(test), no_std)]

use control_interface::command_recognizer::{CommandData, CommandRecognizer};
use control_interface::command_registry::CommandRegistry;
use rriv_0_4::{Board, RXProcessor};

extern crate alloc;
use alloc::boxed::Box;

use core::cell::RefCell;
use core::ffi::c_void;
use core::ops::{Deref, DerefMut};
use cortex_m::interrupt::Mutex;

static COMMAND_DATA: Mutex<RefCell<Option<CommandData>>> = Mutex::new(RefCell::new(None));

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

    pub fn setup(&mut self, board: &mut Board) {
        let rx_processor = Box::new(CharacterProcessor::new(self.get_command_data()));
        // TODO: command data needs to be static??
        // Matty suggests leaking the object?
        // board.set_rx_processor(rx_processor)
    }

    /// register a command with two &strs, object and action, and a C function pointer that matches registry.register_command's second argument
    /// this calls registry.get_command_from_parts to get a Command object, then calls registry.register_command
    pub fn register_command(
        &mut self,
        object: &str,
        action: &str,
        ffi_cb: extern "C" fn(*mut c_void),
    ) {
        let command = self.registry.get_command_from_parts(object, action);
        self.registry.register_command(command, ffi_cb);
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

    fn take_command(&mut self) -> [char; 100] {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.get_command_data().borrow(cs).borrow_mut();
            if let Some(command_data) = command_data.deref_mut() {
                CommandRecognizer::take_command(command_data)
            } else {
                [0 as char; 100]
            }
        })
    }

    pub fn run_loop_iteration(&mut self) {
        if self.pending_message_count() > 0 {
            let command = self.take_command();
            // self.registry.
            // do stuff with the command
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
    fn process_character(&self, character: char) {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.command_data.borrow(cs).borrow_mut();
            if let Some(command_data) = command_data.deref_mut() {
                CommandRecognizer::process_character(command_data, character);
            }
        })
    }
}
