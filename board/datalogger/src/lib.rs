#![cfg_attr(not(test), no_std)]

extern crate command_service;
use rriv_0_4::Board;
// use control_interface::{, command_register::CommandRegistry};

pub struct DataLogger {
    pub board: Board,
    pub command_service: command_service::CommandService,
}

impl DataLogger {
    pub fn new(board: Board) -> Self {
        DataLogger {
            board,
            command_service: command_service::CommandService::new(),
        }
    }

    extern "C" fn test_exec(buf: *const i8) {
        let cmd_str = unsafe { from_c_str(buf) };
        rtt_target::rprintln!("command executed! {}", cmd_str);
    }

    extern "C" fn unknown_command(buf: *const i8) {
        let cmd_str = unsafe { from_c_str(buf) };
        rtt_target::rprintln!("unknown command or invalid json. {}", cmd_str);
    }

    pub fn setup(&mut self) {
        // setup each service
        self.command_service.setup(&mut self.board);
        self.command_service
            .register_command("datalogger", "set", Self::test_exec);
        self.command_service
            .register_command("unknown", "unknown", Self::unknown_command);
    }

    pub fn run_loop_iteration(&mut self) {
        self.command_service.run_loop_iteration();
    }
}

use core::str;

unsafe fn from_c_str<'a>(ptr: *const i8) -> &'a str {
    let mut len = 0;
    while *ptr.offset(len) != 0 {
        len += 1;
    }
    let slice = core::slice::from_raw_parts(ptr as *const u8, len as usize);
    str::from_utf8_unchecked(slice)
}
