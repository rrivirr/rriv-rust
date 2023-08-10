#![cfg_attr(not(test), no_std)]
#![feature(alloc_error_handler)] // required for our prelude that sets up the heap
#![feature(prelude_2024)]
#![no_main]

extern crate panic_halt;
use core::{prelude::rust_2024::*, u8};
use cortex_m_rt::entry;
use rtt_target::rtt_init_print;

pub mod prelude;

extern crate rriv_0_4;
use rriv_0_4::Board;

extern crate datalogger;
use datalogger::DataLogger;

#[entry]
fn main() -> ! {
    prelude::init();
    rtt_init_print!();

    let board = Board::new();
    let mut datalogger = DataLogger::new(board);
    loop {
        datalogger.run_loop_iteration();
    }
}
// // TODO:: import the libraries needed and update these FFI functions
// #[no_mangle]
// pub unsafe extern "C" fn rust_serial_interface_new() -> *mut c_void {
//     let board = Board::init();
//     Box::into_raw(Box::new(board)) as *mut c_void
// }

// something like this and make sure the command is the right arg for CString
// #[no_mangle]
// pub extern "C" fn rust_serial_register_command(serial_ptr: *mut c_void, command: *mut c_void, registration: *mut c_void) {
//     let mut board = unsafe { &mut *(serial_ptr as *mut Board) };
//     board.register_command(&mut command, &mut registration as *mut RegisteredCommand as *mut c_void);
// }

// #[no_mangle]
// pub unsafe extern "C" fn rust_serial_read(serial_ptr: *mut c_void) -> u8 {
//     let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
//     // Read the byte that was just sent. Blocks until the read is complete
//     let charr = block!(serial_context.rx.read()).unwrap();
//     Box::into_raw(serial_context);
//     charr
// }

// #[no_mangle]
// pub unsafe extern "C" fn rust_serial_write(serial_ptr: *mut c_void, value: u8) {
//     let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
//     block!(serial_context.tx.write(value)).unwrap_infallible();
//     Box::into_raw(serial_context);
// }
