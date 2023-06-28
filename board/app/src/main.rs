//! This app tests the static rriv board lib

#![allow(clippy::empty_loop)]
#![cfg_attr(not(test), no_std)]
#![no_main]

use core::ffi::c_void;
use core::panic::PanicInfo;
use cortex_m_rt::entry;
// use rtt_target::{rprintln, rtt_init_print};

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

#[link(name = "rriv_0_4", kind = "static")]
extern "C" {
    pub fn rust_serial_interface_new() -> *mut c_void;
}

#[entry]
fn main() -> ! {
    unsafe {
        // let _s = rust_serial_interface_new();
        rust_serial_interface_new();
        loop {}
    }
}
