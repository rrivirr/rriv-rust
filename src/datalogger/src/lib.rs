#![cfg_attr(not(test), no_std)]

mod command_service;
mod datalogger_commands;
use datalogger_commands::*;
use rriv_board::RRIVBoard;

pub struct DataLogger {
    // pub command_service: command_service::CommandService
}

impl DataLogger {
    pub fn new() -> Self {
        DataLogger {
            // command_service: command_service::CommandService::new()
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

    pub fn setup(&mut self, board: &mut impl RRIVBoard) {
        // setup each service
        command_service::setup(board);
        // self.command_service
        //     .register_command("datalogger", "set", Self::test_exec);
        // self.command_service
        //     .register_command("unknown", "unknown", Self::unknown_command);
    }

    pub fn run_loop_iteration(&mut self, board: & impl RRIVBoard) {
        // todo: refactor to use Result<T,E>
        let pending_command_payload = command_service::get_pending_command(board);
        if let Some(command_payload) = pending_command_payload {
            self.executeCommand(board, command_payload);
        }
    }
    
    
    pub fn executeCommand(&mut self, board: & impl RRIVBoard, command_payload: CommandPayload) {
        match command_payload {
            CommandPayload::SetCommandPayload(payload) => {
                self.update_datalogger_settings(payload);
                board.serial_send("updated datalogger settings\n");
            },
            CommandPayload::GetCommandPayload(_) => todo!(),
            CommandPayload::InvalidPayload() => {
                board.serial_send("invalid payload\n");
            }
            CommandPayload::UnrecognizedCommand() => {
                board.serial_send("unrecognized command\n");
            }
        }
    }

    pub fn update_datalogger_settings(&mut self, set_command_payload: DataloggerSetCommandPayload) {

    }

}

use core::{str, future::pending};

unsafe fn from_c_str<'a>(ptr: *const i8) -> &'a str {
    let mut len = 0;
    while *ptr.offset(len) != 0 {
        len += 1;
    }
    let slice = core::slice::from_raw_parts(ptr as *const u8, len as usize);
    str::from_utf8_unchecked(slice)
}
