#![cfg_attr(not(test), no_std)]
#![allow(clippy::empty_loop)]
#![feature(alloc_error_handler)]
#![feature(prelude_2024)]
#![no_main]

extern crate panic_abort;
extern crate alloc;

use core::prelude::rust_2024::*;
use alloc::boxed::Box;
use cortex_m_rt::entry;
use rtt_target::rtt_init_print;

pub mod prelude;

extern crate rriv_board;

extern crate datalogger;

mod board_select;

// Import only build_board and SelectedBoard, not glob import
use board_select::{build_board, SelectedBoard};
use rriv_board::RRIVBoard;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    prelude::init();
    let mut board = build_board();
    board.start(); // not needed, for debug only
    let mut datalogger = datalogger::DataLogger::new();
    datalogger.setup(&mut board);
    loop {
        board.run_loop_iteration();
        datalogger.run_loop_iteration(&mut board);
    }
}
