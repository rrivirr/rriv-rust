#![cfg_attr(not(test), no_std)]

use control_interface::command_recognizer::{CommandData, CommandRecognizer, self};
use control_interface::command_registry::{CommandType, CommandRegistry};
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

    pub fn get_pending_command(board: & impl RRIVBoard) ->  Option<CommandPayload> {
        if pending_message_count(board) > 0 {
            if let Ok(command_bytes) = take_command(board) {
                let command_cstr = CStr::from_bytes_until_nul(&command_bytes).unwrap();
                let command  = identify_serial_command(command_cstr);
                // return the command to run, run it in the datalogger
                let command_payload = get_command_payload(command, command_cstr);
                return Some(command_payload);
            }
        }
        return None;
    }

    pub fn get_command_from_parts(object: &str, action: &str) -> CommandType {
        let command_str = format!("{}_{}", object, action);
        CommandType::from_str(&command_str)
    }

    fn identify_serial_command(command_cstr :&CStr) -> CommandType {
        // let command_data_str = command_data_cstr.to_str().unwrap();
        // Parse the JSON string into a serde_json::Value

        // Next
        // 2. port the EEPROM
        // 3. add and remove configurations from EEPROM

        let command_object: Option<CLICommand> = parse_command(command_cstr);
        if let Some(command_object) = command_object {
            let command = 
                    get_command_from_parts(command_object.object, command_object.action);
            return command;
        } else {
            CommandType::Unrecognized
        }
    }

    fn parse_command<'a, T>(command_cstr: &'a CStr) -> Option<T> // use Result with custom error type instead
        where T: Deserialize<'a> {
        let command_data_str = command_cstr.to_str().unwrap();
        rprintln!("{}", command_data_str);
        match serde_json::from_str::<T>(command_data_str) {
            Ok(command_payload) => {
                Some(command_payload)
            }
            Err(e) => {
                rprintln!("{:?}",e);
                return None
            }
        }
    }

    fn get_command_payload(command: CommandType, command_cstr: &CStr) -> CommandPayload {
            match command {
                CommandType::DataloggerSet => {
                    let payload = parse_command::<DataloggerSetCommandPayload>(command_cstr);
                    if let Some(command_payload_object) = payload {
                        return CommandPayload::SetCommandPayload(command_payload_object)
                    } 
                }
                CommandType::DataloggerGet => {
                    if let Some(command_payload_object) = parse_command::<DataloggerGetCommandPayload>(command_cstr) {
                        return CommandPayload::GetCommandPayload(command_payload_object)
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
                CommandType::Unrecognized => {
                    return CommandPayload::UnrecognizedCommand()
                },
                CommandType::Unknown => {
                    return CommandPayload::UnrecognizedCommand() 
                }
            }

            return CommandPayload::InvalidPayload()

            
        // } else if let Some(unknown_cb) = self.registry.get_action_fn(&CommandType::Unknown) {
        //     // send a message back to the host that the command was not recognized
        //     unknown_cb(command_cstr.as_ptr() as *mut c_char);
        // }
    }
// }

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
