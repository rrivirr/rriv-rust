#![cfg_attr(not(test), no_std)]
#![allow(clippy::empty_loop)]
#![feature(alloc_error_handler)]
#![feature(prelude_2024)]
#![no_main]

extern crate panic_abort;

use core::{prelude::rust_2024::*, u8};
use cortex_m_rt::entry;
use rtt_target::rtt_init_print;

pub mod prelude;

extern crate rriv_board;

extern crate rriv_board_0_4_2;
// use rriv_board_0_4_2::{Board,BoardBuilder};

extern crate datalogger;
use datalogger::DataLogger;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    prelude::init();

    let mut board = rriv_board_0_4_2::build();
    board.start(); // not needed, for debug only
    let mut datalogger = DataLogger::new();
    datalogger.setup(&mut board);
    loop {
        datalogger.run_loop_iteration(&mut board);
    }
}
