#![cfg_attr(not(test), no_std)]

use rriv_0_4::Board;
use control_interface::{command_recognizer::CommandRecognizer, command_register::CommandRegister};
use hashbrown::HashMap;


pub struct DataLogger {
  pub board: Board,
  pub command_serivce: CommandService
}

impl DataLogger {
  pub fn new(board: Board) -> Self {
    DataLogger {
      board: board,
      command_serivce: CommandService::new()
    }    
  }

  pub fn run_loop_iteration(&mut self) {
    if self.command_serivce.recognizer.pending_message_count() > 0 {
      // process the message
    }
  }
}

pub struct CommandService {
  registry: CommandRegister,
  recognizer: CommandRecognizer
}

impl CommandService {
  pub fn new() -> Self {
    CommandService {
      registry: CommandRegister::new(),
      recognizer: CommandRecognizer::default()
    }
  }

}