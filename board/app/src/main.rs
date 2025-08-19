#![cfg_attr(not(test), no_std)]
#![allow(clippy::empty_loop)]
#![feature(alloc_error_handler)]
#![no_main]

// extern crate panic_halt;

use core::prelude::rust_2024::*;
use cortex_m_rt::entry;
use rtt_target::{rtt_init_print};

pub mod prelude;

extern crate rriv_board;

extern crate rriv_board_0_4_2;
use crate::rriv_board::RRIVBoard;

extern crate datalogger;
use datalogger::DataLogger;

use rtt_target::rprintln;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    prelude::init();

    let mut board = rriv_board_0_4_2::build();
    board.start(); // not needed, for debug only
    let mut datalogger = DataLogger::new();
    datalogger.setup(&mut board);
    loop {
        board.run_loop_iteration();
        datalogger.run_loop_iteration(&mut board);
    }
}


use core::panic::PanicInfo;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    rprintln!("Panicked!");
    if let Some(location) = _info.location() {
        rprintln!("at {}", location);
    }
    
    rprintln!("with message: {}", _info.message());

    
    loop {}
}