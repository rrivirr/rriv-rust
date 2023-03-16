//! Tests rust_serial static lib

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use core::ffi::c_void;
use core::panic::PanicInfo;
use cortex_m_rt::entry;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

#[link(name = "rust_serial", kind = "static")]
extern "C" {
    fn rust_serial_interface_new() -> *mut c_void;
    fn rust_serial_write(serial_ptr: *mut c_void, value: u8) -> c_void;
}

#[entry]
fn main() -> ! {
    unsafe {
        let s = rust_serial_interface_new();
        rust_serial_write(s, b'm');
    }
    loop{}
}
