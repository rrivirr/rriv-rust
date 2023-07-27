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

    use core::{
        borrow::BorrowMut,
        cell::{Ref, RefCell, RefMut},
    };
    use cortex_m::interrupt::Mutex;

    pub struct CommandService {
        command_data: Mutex<RefCell<CommandData>>,
        registry: CommandRegistry,
        // recognizer: CommandRecognizer,
    }

    impl CommandService {
        pub fn new() -> Self {
            static NUM_COMMANDS: usize = 32;
            let command_map = HashMap::with_capacity(NUM_COMMANDS);
            CommandService {
                command_data: Mutex::new(RefCell::new(CommandData::default())),
                registry: CommandRegistry::new(command_map),
            }
        }

        pub fn setup(&mut self, board: &mut Board) {
            let rxProcessor = Box::new(CharacterProcessor::new(&mut self.command_data)); // TODO: command data needs to be static??
                                                                                         // Matty suggests leaking the object?
            board.set_rx_processor(rxProcessor)
        }

        fn pending_message_count(&mut self) -> usize {
            cortex_m::interrupt::free(|cs| {
                let command_data = self.command_data.borrow(cs).borrow();
                return CommandRecognizer::pending_message_count(command_data);
            })
        }

        fn take_command(&mut self) -> [char; 100] {
            cortex_m::interrupt::free(|cs| {
                let mut command_data = self.command_data.borrow(cs).borrow_mut();
                return CommandRecognizer::take_command(command_data);
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

    pub struct CharacterProcessor<'a> {
        command_data: &'a mut Mutex<RefCell<CommandData>>,
    }

    impl CharacterProcessor<'_> {
        pub fn new(command_data: &mut Mutex<RefCell<CommandData>>) -> CharacterProcessor {
            CharacterProcessor {
                command_data: command_data,
            }
        }

        pub fn function_name() {}
    }

    impl RXProcessor for CharacterProcessor<'_> {
        fn process_character(&mut self, character: char) {
            cortex_m::interrupt::free(|cs| {
                let mut command_data = self.command_data.borrow(cs).borrow_mut();
                CommandRecognizer::process_character(command_data, character);
            })
        }
    }
}
