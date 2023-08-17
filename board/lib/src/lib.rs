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
use hashbrown::HashMap;


pub mod prelude;

extern crate rriv_0_4;
use rriv_0_4::Board;

extern crate command_service;
use command_service::CommandService;

/// Unsafe FFI method that initializes the `CommandService` and returns a raw pointer to it.
#[no_mangle]
pub unsafe extern "C" fn command_service_init() -> *mut c_void {
    prelude::init();
    // rtt_init_print!();
    let mut board = Board::new();
    let mut command_service = CommandService::new();
    command_service.setup(&mut board);
    Box::into_raw(Box::new(command_service)) as *mut c_void
}

/// Unsafe FFI method that registers a command with the `CommandService` and returns a raw pointer to it.
#[no_mangle]
pub unsafe extern "C" fn command_service_register_command(
    command_service_ptr: *mut c_void,
    object: *const c_char,
    action: *const c_char,
    ffi_cb: extern "C" fn(*const c_char),
) -> *mut c_void {
    let command_service = &mut *(command_service_ptr as *mut CommandService);
    command_service.register_command(
        &CStr::from_ptr(object).to_str().unwrap(),
        &CStr::from_ptr(action).to_str().unwrap(),
        ffi_cb,
    );
    command_service_ptr
}

/// Unsafe FFI method that runs the `CommandService`'s run loop iteration.
/// This should be called in the main loop of the application that is using the `CommandService`.
#[no_mangle]
pub unsafe extern "C" fn command_service_run_loop_iteration(
    command_service_ptr: *mut c_void,
) -> *mut c_void {
    let command_service = &mut *(command_service_ptr as *mut CommandService);
    command_service.run_loop_iteration();
    command_service_ptr
}
