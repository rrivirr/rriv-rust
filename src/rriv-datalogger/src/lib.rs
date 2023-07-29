#![cfg_attr(not(test), no_std)]

// use control_interface::{, command_register::CommandRegistry};
use rriv_0_4::Board;

extern crate alloc;
use alloc::boxed::Box;

use core::{
    borrow::BorrowMut,
    cell::{Ref, RefCell, RefMut},
};
use cortex_m::interrupt::Mutex;

pub struct DataLogger {
    pub board: Board,
    pub command_service: command_service::CommandService,
}

impl DataLogger {
    pub fn new(board: Board) -> Self {
        DataLogger {
            board: board,
            command_service: command_service::CommandService::new(),
        }
    }

    pub fn setup(&mut self) {
        // setup each service
        self.command_service.setup(&mut self.board)
    }

    pub fn run_loop_iteration(&mut self) {
        self.command_service.run_loop_iteration();
    }
}

pub mod command_service {
    use control_interface::command_recognizer::{CommandData, CommandRecognizer};
    use control_interface::command_registry::CommandRegistry;
    use hashbrown::HashMap;
    use rriv_0_4::{Board, RXProcessor};

    extern crate alloc;
    use alloc::boxed::Box;

    use core::ops::{Deref, DerefMut};
    use core::{
        borrow::BorrowMut,
        cell::{Ref, RefCell, RefMut},
    };
    use cortex_m::interrupt::Mutex;

    static COMMAND_DATA: Mutex<RefCell<Option<CommandData>>> = Mutex::new(RefCell::new(None));

    pub struct CommandService {
        registry: CommandRegistry,
        // recognizer: CommandRecognizer,
    }

    impl CommandService {
        pub fn new() -> Self {
            static NUM_COMMANDS: usize = 32;
            let command_map = HashMap::with_capacity(NUM_COMMANDS);
            // set the static, shareable command data
            cortex_m::interrupt::free(|cs| {
                *COMMAND_DATA.borrow(cs).borrow_mut() = Some(CommandData::default());
            });
            CommandService {
                registry: CommandRegistry::new(command_map),
            }
        }

        pub fn setup(&mut self, board: &mut Board) {
            let rx_processor = Box::new(CharacterProcessor{});
            // TODO: command data needs to be static??
            // Matty suggests leaking the object?
            board.set_rx_processor(rx_processor)
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
    pub struct CharacterProcessor {}

    impl RXProcessor for CharacterProcessor {
        fn process_character(&self, character: char) {
            cortex_m::interrupt::free(|cs| {

                let mut command_data = COMMAND_DATA.borrow(cs).borrow_mut();
                if let Some( ref mut command_data) = command_data.deref_mut() {
                    CommandRecognizer::process_character(command_data, character);
                }
            })
        }
    }
}
