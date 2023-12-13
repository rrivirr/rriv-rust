#![cfg_attr(not(test), no_std)]
#![feature(alloc_error_handler)] // required for our prelude that sets up the heap
#![feature(prelude_2024)]
#![no_main]

extern crate alloc;
extern crate panic_halt;
use alloc::boxed::Box;
use core::ffi::{c_char, c_void, CStr};
use core::{prelude::rust_2024::*, u8};
// use rtt_target::rtt_init_print;

pub mod prelude;

extern crate rriv_0_4;
use rriv_0_4::Board;

extern crate command_service;
use command_service::CommandService;
