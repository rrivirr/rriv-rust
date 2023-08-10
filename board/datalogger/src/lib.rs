#![cfg_attr(not(test), no_std)]

extern crate command_service;
use rriv_0_4::Board;
// use control_interface::{, command_register::CommandRegistry};

pub struct DataLogger {
    pub board: Board,
    pub command_service: command_service::CommandService,
}

impl DataLogger {
    pub fn new(board: Board) -> Self {
        DataLogger {
            board,
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
