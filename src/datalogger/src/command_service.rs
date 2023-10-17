#![cfg_attr(not(test), no_std)]

use control_interface::command_recognizer::{self, CommandData, CommandRecognizer};
use control_interface::command_registry::{CommandRegistry, CommandType};
use rriv_board::{RRIVBoard, RXProcessor};

extern crate alloc;
use alloc::boxed::Box;
use alloc::format;

use core::borrow::BorrowMut;
use core::ffi::{c_char, CStr};
use serde::{Deserialize, Serialize};

use rtt_target::rprintln;

use crate::datalogger_commands::*;

static mut COMMAND_DATA: CommandData = CommandData::default();

// pub struct CommandService {
//     // registry: CommandRegistry,
// }

#[derive(Serialize, Deserialize, Debug)]
struct CLICommand<'a> {
    object: &'a str,
    action: &'a str,
}

// impl CommandService {
// pub fn new() -> Self {
//     // set the static, shareable command data
//     // CommandService {
//     //     registry: CommandRegistry::new(),
//     // }
// }

/// set the global rx processor
pub fn setup(board: &mut impl RRIVBoard) {
    let char_processor = Box::<CharacterProcessor>::leak(Box::new(CharacterProcessor::new()));

    // pass a pointer to the lleaked processor to Board::set_rx_processor
    board.set_rx_processor(Box::new(char_processor));
}

/// register a command with two &strs, object and action, and a C function pointer that matches registry.register_command's second argument
/// this calls registry.get_command_from_parts to get a Command object, then calls registry.register_command
// pub fn register_command(
//     &mut self,
//     object: &str,
//     action: &str,
//     ffi_cb: extern "C" fn(*const c_char),
// ) {
//     let command = self.registry.get_command_from_parts(object, action);
//     self.registry.register_command(command, ffi_cb);
// }

fn pending_message_count(board: &impl RRIVBoard) -> usize {
    let get_pending_message_count = || unsafe {
        let command_data = COMMAND_DATA.borrow_mut();
        CommandRecognizer::pending_message_count(&command_data)
    };

    board.critical_section(get_pending_message_count)
}

fn take_command(board: &impl RRIVBoard) -> Result<[u8; 100], ()> {
    let do_take_command = || unsafe {
        let command_data = COMMAND_DATA.borrow_mut();
        Ok(CommandRecognizer::take_command(command_data))
    };

    board.critical_section(do_take_command)
}

pub fn get_pending_command(board: &impl RRIVBoard) -> Option<Result<CommandPayload, CommandError>> {
    if pending_message_count(board) > 0 {
        if let Ok(command_bytes) = take_command(board) {
            let command_cstr = CStr::from_bytes_until_nul(&command_bytes).unwrap();
            let command_identification_result = identify_serial_command(command_cstr);
            match command_identification_result {
                Ok(command_type) => {
                    let result: Result<CommandPayload, _> =
                        get_command_payload(command_type, command_cstr);
                    return Some(result);
                }
                Err(error) => return Some(Err(error)),
            }
        }
    }
    return None;
}

pub fn get_command_from_parts(object: &str, action: &str) -> CommandType {
    let command_str = format!("{}_{}", object, action);
    CommandType::from_str(&command_str)
}

fn identify_serial_command(command_cstr: &CStr) -> Result<CommandType, CommandError> {
    match parse_command::<CLICommand>(command_cstr) {
        Ok(cli_command) => {
            let command_type = get_command_from_parts(cli_command.object, cli_command.action);
            if command_type == CommandType::Unknown {
                return Err(CommandError::InvalidCommand);
            }
            return Result::Ok(command_type);
        }
        Err(e) => {
            return Result::Err(CommandError::ParseError(e));
        }
    }
}

fn parse_command<'a, T>(command_cstr: &'a CStr) -> Result<T, serde_json::Error>
//Option<T> // use Result with custom error type instead
where
    T: Deserialize<'a>,
{
    let command_data_str = command_cstr.to_str().unwrap();
    rprintln!("{}", command_data_str);

    return serde_json::from_str::<T>(command_data_str);
}

fn get_command_payload(
    command: CommandType,
    command_cstr: &CStr,
) -> Result<CommandPayload, CommandError> {
    match command {
        CommandType::DataloggerSet => {
            let result = parse_command::<DataloggerSetCommandPayload>(command_cstr);
            match result {
                Ok(payload) => return Ok(CommandPayload::SetCommandPayload(payload)),
                Err(error) => return Err(CommandError::InvalidPayload(error)),
            }
        }
        CommandType::DataloggerGet => {
            let result = parse_command::<DataloggerGetCommandPayload>(command_cstr);
            match result {
                Ok(payload) => return Ok(CommandPayload::GetCommandPayload(payload)),
                Err(error) => return Err(CommandError::InvalidPayload(error)),
            }
        }
        CommandType::DataloggerReset => todo!(),
        CommandType::DataloggerSetMode => todo!(),
        CommandType::SensorSet => todo!(),
        CommandType::SensorGet => todo!(),
        CommandType::SensorRemove => todo!(),
        CommandType::SensorList => todo!(),
        CommandType::SensorCalibratePoint => todo!(),
        CommandType::SensorCalibrateFit => todo!(),
        CommandType::SensorReset => todo!(),
        CommandType::ActuatorSet => todo!(),
        CommandType::ActuatorGet => todo!(),
        CommandType::ActuatorRemove => todo!(),
        CommandType::ActuatorList => todo!(),
        CommandType::ActuatorReset => todo!(),
        CommandType::TelemeterSet => todo!(),
        CommandType::TelemeterGet => todo!(),
        CommandType::TelemeterRemove => todo!(),
        CommandType::TelemeterList => todo!(),
        CommandType::TelemeterReset => todo!(),
        CommandType::BoardVersion => todo!(),
        CommandType::BoardFirmwareWarranty => todo!(),
        CommandType::BoardFirmwareConditions => todo!(),
        CommandType::BoardFirmwareLicense => todo!(),
        CommandType::BoardRtcSet => todo!(),
        CommandType::BoardRtcGet => todo!(),
        CommandType::BoardRestart => todo!(),
        CommandType::BoardI2cList => todo!(),
        CommandType::BoardMemoryCheck => todo!(),
        CommandType::BoardMcuStop => todo!(),
        CommandType::BoardMcuSleep => todo!(),
        CommandType::BoardSignalExAdcHigh => todo!(),
        CommandType::BoardSignalExAdcLow => todo!(),
        CommandType::BoardSignal3v3BoostHigh => todo!(),
        CommandType::BoardSignal3v3BoostLow => todo!(),
        // todo: refactor these to be an errors enumeration
        CommandType::Unknown => todo!(),
    }
}

/// I believe the lifetimes here are generic over the lifetime of the CommandData
pub struct CharacterProcessor {}

impl<'a> CharacterProcessor {
    pub fn new() -> CharacterProcessor {
        CharacterProcessor {}
    }
}

impl<'a, 'b> RXProcessor for CharacterProcessor {
    fn process_character(&self, character: u8) {
        unsafe {
            let command_data = COMMAND_DATA.borrow_mut();
            CommandRecognizer::process_character(command_data, character);
        }
    }
}
