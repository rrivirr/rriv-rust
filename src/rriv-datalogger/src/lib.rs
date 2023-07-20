#![cfg_attr(not(test), no_std)]

use rriv_0_4::Board;

pub struct DataLogger {
  pub board: Board
}

impl DataLogger {
  pub fn new(board: Board) -> Self {
    DataLogger {
      board: board
    }    
  }

  pub fn run_loop_iteration(&mut self) {
    // do stuff
  }
}