#![cfg_attr(not(test), no_std)]

use control_interface::command_recognizer::{self, CommandData, CommandRecognizer};
use control_interface::command_registry::{CommandRegistry, CommandType};
use rriv_board::{RRIVBoard, RXProcessor};

extern crate alloc;
use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use serde_json::Value;

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
    subcommand: Option<Box<str>>
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

fn take_command(board: &impl RRIVBoard) -> Result<[u8; 500], ()> {
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
            rprintln!("{:?}", command_identification_result);
            match command_identification_result {
                Ok(command_type) => {
                    let result: Result<CommandPayload, _> =
                        get_command_payload(command_type, command_cstr);
                    rprintln!("{:?}", result);
                    return Some(result);
                }
                Err(error) => {
                    rprintln!("{:?}", error);
                    return Some(Err(error))
                }
            }
        }
    }
    return None;
}

pub fn get_command_from_parts(object: &str, action: &str, subcommmand: Option<Box<str>>) -> CommandType {
    if let Some(subcommand) = subcommmand {
        let command_str = format!("{}_{}_{}", object, action, subcommand);
        rprintln!("{:?}", command_str);
        CommandType::from_str(&command_str)
    } else {
        let command_str = format!("{}_{}", object, action);
        rprintln!("{:?}", command_str);
        CommandType::from_str(&command_str)
    }
}

fn identify_serial_command(command_cstr: &CStr) -> Result<CommandType, CommandError> {
    match parse_command::<CLICommand>(command_cstr) {
        Ok(cli_command) => {
            let command_type = get_command_from_parts(cli_command.object, cli_command.action, cli_command.subcommand);
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

#[macro_export]
macro_rules!  parse_command_to_payload {
    ($payload_type:ty, $variant:path, $command_cstr:expr ) => {
        let result = parse_command::<$payload_type>($command_cstr);
        match result {
            Ok(payload) => return Ok($variant(payload)),
            Err(error) => return Err(CommandError::InvalidPayload(error)),
        }
    };
}

fn get_command_payload(
    command: CommandType,
    command_cstr: &CStr,
) -> Result<CommandPayload, CommandError> {
    match command {
        CommandType::DataloggerSet => {
            parse_command_to_payload!(DataloggerSetCommandPayload, CommandPayload::DataloggerSetCommandPayload, command_cstr);
        }
        CommandType::DataloggerGet => {
            parse_command_to_payload!(DataloggerGetCommandPayload, CommandPayload::DataloggerGetCommandPayload, command_cstr);
        }
        CommandType::DataloggerReset => todo!(),
        CommandType::DataloggerSetMode => {
            parse_command_to_payload!(DataloggerSetModeCommandPayload, CommandPayload::DataloggerSetModeCommandPayload, command_cstr);
        },
        CommandType::SensorSet => {

            // return serde_json::from_str::<T>(command_data_str);

            let command_data_str = command_cstr.to_str().unwrap();

            let mut raw_value: Value = serde_json::from_str(command_data_str).unwrap(); // use hashbrown HashMap?


            let result = parse_command::<SensorSetCommandPayload>(command_cstr);
            match result {
                Ok(payload) => return Ok(CommandPayload::SensorSetCommandPayload(payload, raw_value)),
                Err(error) => return Err(CommandError::InvalidPayload(error)),
            }
            // let mut map: HashMap<String, serde_json::Value> = HashMap::new();
            // for key in keys {
            //     let (k, v) = lookup.remove_entry (key).unwrap();
            //     map.insert(k, v);
            // }

            // parse_command_to_payload!(SensorSetCommandPayload, CommandPayload::SensorSetCommandPayload, command_cstr);
        },
        CommandType::SensorGet => {
            parse_command_to_payload!(SensorGetCommandPayload, CommandPayload::SensorGetCommandPayload, command_cstr);
        },
        CommandType::SensorRemove => {
            parse_command_to_payload!(SensorRemoveCommandPayload, CommandPayload::SensorRemoveCommandPayload, command_cstr);
        },
        CommandType::SensorList => {
            parse_command_to_payload!(SensorListCommandPayload, CommandPayload::SensorListCommandPayload, command_cstr);
        },
        CommandType::SensorCalibratePoint => {
            rprintln!("parsing SensorCalibratePoint");
            parse_command_to_payload!(SensorCalibratePointPayload, CommandPayload::SensorCalibratePointPayload, command_cstr);
        },
        CommandType::SensorCalibrateList => todo!("SensorCalibrateList not built"),
        CommandType::SensorCalibrateRemove => todo!(),
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
        CommandType::BoardRtcSet => {
            parse_command_to_payload!(BoardRtcSetPayload, CommandPayload::BoardRtcSetPayload, command_cstr);
        },
        CommandType::BoardGet => {
            parse_command_to_payload!(BoardGetPayload, CommandPayload::BoardGetPayload, command_cstr);
        }
        CommandType::BoardRestart => todo!(),
        CommandType::BoardI2cList => todo!(),
        CommandType::BoardMemoryCheck => todo!(),
        CommandType::BoardMcuStop => todo!(),
        CommandType::BoardMcuSleep => todo!(),
        CommandType::BoardSignalExAdcHigh => todo!(),
        CommandType::BoardSignalExAdcLow => todo!(),
        CommandType::BoardSignal3v3BoostHigh => todo!(),
        CommandType::BoardSignal3v3BoostLow => todo!(),
        // todo: refactor these to be an errors enumerationget_command_payload
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
