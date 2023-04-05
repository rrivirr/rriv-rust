//! Tests rust_serial static lib

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use core::ffi::c_void;
use core::panic::PanicInfo;

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    loop {}
}

#[link(name = "rust_serial", kind = "static")]
extern "C" {
    pub fn rust_serial_interface_new() -> *mut c_void;
    pub fn rust_serial_read(serial_ptr: *mut ::core::ffi::c_void) -> u8;
    pub fn rust_serial_write(serial_ptr: *mut c_void, value: u8);
}

#[entry]
fn main() -> ! {
    unsafe {
        let s = rust_serial_interface_new();        
        loop{
            match rust_serial_read(s) {
                b'\r' => {
                    rust_serial_write(s, b'\r');
                    rust_serial_write(s, b'\n');
                }
                b'\n' => {
                    rust_serial_write(s, b'\r');
                    rust_serial_write(s, b'\n');
                }
                charr => rust_serial_write(s, charr)
            }
        }
    }
}
