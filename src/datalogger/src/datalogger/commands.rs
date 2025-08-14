use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde::{Deserialize, Serialize};
use serde_json::{json, Number, Value};
use alloc::fmt::Debug;
use alloc::format;


use crate::protocol::responses;
use crate::DataLoggerMode;

const LOGGER_NAME_LENGTH : usize = 8;
const SITE_NAME_LENGTH : usize = 8;
const DEPLOYMENT_IDENTIFIER_LENGTH : usize = 16;

#[derive(Default)]
pub struct DataloggerSettingsValues {
    pub deployment_identifier: Option<[u8; 16]>,
    pub logger_name: Option<[u8; 8]>,
    pub site_name: Option<[u8; 8]>,
    pub deployment_timestamp: Option<u64>,
    pub interval: Option<u16>,
    pub start_up_delay: Option<u16>,
    pub delay_between_bursts: Option<u16>,
    pub bursts_per_measurement_cycle: Option<u8>,
    pub mode: Option<u8>,
}

#[derive(Serialize, Deserialize)]
pub struct DataloggerSetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub logger_name: Option<Value>,
    pub site_name: Option<Value>,
    pub deployment_identifier: Option<Value>,
    pub interval: Option<u16>,
    pub bursts_per_cycle: Option<u8>,
    pub start_up_delay: Option<u16>
    // pub user_note: Option<Value>, // not implemented for now
    // pub user_value: Option<i16>
}

fn value_length( target: &[u8], value: &[u8]) -> usize {
    if value.len() > target.len() { target.len() } else { value.len() }
}

// fn match_and_set(source: Option<Value>, target: &mut [u8]) -> Option<[u8]> {
//     if let Some(value) = source {
//             match value {
//                 serde_json::Value::String(value) => {
//                     let value = value.as_bytes();
//                     target.clone_from_slice(&value[0..value_length(target, value)]);
//                     // has to set here somehow
//                 },
//                 _ => {

//                 },
//             }
//     }
// }

impl DataloggerSetCommandPayload {
    pub fn values(self) -> DataloggerSettingsValues {

        let mut datalogger_settings_values = DataloggerSettingsValues::default();

        if let Some(value) = self.logger_name {
            match value {
                serde_json::Value::String(value) => {
                    let value = value.as_bytes();
                    let mut target = [0u8; LOGGER_NAME_LENGTH];
                    let len = value_length(&target, value);
                    target[0..len].clone_from_slice(&value[0..len]);
                    datalogger_settings_values.logger_name = Some(target);
                },
                _ => {},
            }
        }

        if let Some(value) = self.site_name {
            match value {
                serde_json::Value::String(value) => {
                    let value = value.as_bytes();
                    let mut target = [0u8; SITE_NAME_LENGTH];
                    let len = value_length(&target, value);
                    target[0..len].clone_from_slice(&value[0..len]);
                    datalogger_settings_values.site_name = Some(target);
                },
                _ => {},
            }
        }
        
        if let Some(value) = self.deployment_identifier {
            match value {
                serde_json::Value::String(value) => {
                    let value = value.as_bytes();
                    let mut target = [0u8; DEPLOYMENT_IDENTIFIER_LENGTH];
                    let len = value_length(&target, value);
                    target[0..len].clone_from_slice(&value[0..len]);
                    datalogger_settings_values.deployment_identifier = Some(target);
                },
                _ => {},
            }
        }

        if let Some(interval) = self.interval {
            datalogger_settings_values.interval = Some(interval);
        }

        if let Some(bursts_per_cycle) = self.bursts_per_cycle {
            datalogger_settings_values.bursts_per_measurement_cycle = Some(bursts_per_cycle);
        }

        if let Some(start_up_delay) = self.start_up_delay {
            datalogger_settings_values.start_up_delay = Some(start_up_delay);
        }

        datalogger_settings_values

    }
}


#[derive(Serialize, Deserialize)]
pub struct DataloggerGetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub propery: Option<Value>
}

#[derive(Serialize, Deserialize)]
pub struct DataloggerSetModeCommandPayload {
    pub object: Value,
    pub action: Value,
    pub mode: Option<Value>
}


#[derive(Serialize, Deserialize)]
pub struct SensorSetCommandPayload {
    pub object: Value,
    pub action: Value, 
    pub id: Option<Value>, // option
    pub r#type: Value, // option
}

#[derive(Serialize, Deserialize)]
pub struct SensorGetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
}

#[derive(Serialize, Deserialize)]
pub struct SensorRemoveCommandPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
}

#[derive(Serialize, Deserialize)]
pub struct SensorListCommandPayload {
    pub object: Value,
    pub action: Value,
}

#[derive(Serialize, Deserialize)]
pub struct BoardRtcSetPayload {
    pub object: Value,
    pub action: Value,
    pub epoch: Value
}


#[derive(Serialize, Deserialize)]
pub struct BoardGetPayload {
    pub object: Value,
    pub action: Value,
    pub parameter: Option<Value>
}



#[derive(Serialize, Deserialize)]
pub struct BoardSerialSendPayload {
    pub object: Value,
    pub action: Value,
    pub message: Value
}

pub struct BoardSerialSendCommandPayload {
    pub message: [u8;20],
    pub message_len: u8, 
}

impl BoardSerialSendPayload {
    pub fn convert(&self) -> Result<BoardSerialSendCommandPayload, &'static str> { // TODO: this static lifetime is not good.  Need to pass in some storage or use a box.
        let message = match self.message {
            serde_json::Value::String(ref message) => {
                message
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad message");
            }
        };

        let mut message_bytes = [0u8;20];
        message_bytes[0..message.len()].clone_from_slice(&message.as_bytes()[0..message.len()]);
        
        Ok(
        BoardSerialSendCommandPayload {
            message: message_bytes,
            message_len: message.len() as u8,
        }
        )
    }
}


#[derive(Serialize, Deserialize)]
pub struct SensorCalibratePointPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
    pub subcommand: Value,
    pub point: Number,
    pub tag: Option<Value>
}


#[derive(Serialize, Deserialize)]
pub struct SensorCalibrateListPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
    pub subcommand: Value,
}


#[derive(Serialize, Deserialize)]
pub struct SensorCalibrateRemovePayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
    pub subcommand: Value,
    pub tag: Value
}

#[derive(Serialize, Deserialize)]
pub struct SensorCalibrateFitPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
    pub subcommand: Value,
}

#[derive(Serialize, Deserialize)]
pub struct SensorCalibrateClearPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
    pub subcommand: Value,
}

pub struct SensorCalibrateSubcommand<'a> {
    pub object: &'a str,
    pub action: &'a str,
    pub id: &'a str,
    pub subcommand: &'a str
}

// TODO: these impls should be derived or ??

impl SensorCalibrateFitPayload {
    pub fn convert(&self) -> Result<SensorCalibrateSubcommand, &'static str> {
        let id = match self.id {
            serde_json::Value::String(ref payload_id) => {
                payload_id
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad sensor id");
            }
        };

        return Ok(SensorCalibrateSubcommand {
            object: "sensor", // TODO: what's the most ideal way to handle these?  no real reason to convert them again
            action: "calibrate",
            id,
            subcommand: "fit"
        })
    }
}

impl SensorCalibrateClearPayload {
    pub fn convert(&self) -> Result<SensorCalibrateSubcommand, &'static str> {
        let id = match self.id {
            serde_json::Value::String(ref payload_id) => {
                payload_id
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad sensor id");
            }
        };

        return Ok(SensorCalibrateSubcommand {
            object: "sensor", // TODO: what's the most ideal way to handle these?  no real reason to convert them again
            action: "calibrate",
            id,
            subcommand: "clear"
        })
    }
}



impl SensorCalibrateListPayload {
    pub fn convert(&self) -> Result<SensorCalibrateSubcommand, &'static str> {
        let id = match self.id {
            serde_json::Value::String(ref payload_id) => {
                payload_id
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad sensor id");
            }
        };

        return Ok(SensorCalibrateSubcommand {
            object: "sensor", // TODO: what's the most ideal way to handle these?  no real reason to convert them again
            action: "calibrate",
            id,
            subcommand: "list"
        })
    }
}


pub struct SensorCalibrateRemove<'a> {
    pub object: &'a str,
    pub action: &'a str,
    pub id: &'a str,
    pub subcommand: &'a str,
    pub tag: &'a str,
}

impl SensorCalibrateRemovePayload {
    pub fn convert(&self) -> Result<SensorCalibrateRemove, &'static str> {
        let id = match self.id {
            serde_json::Value::String(ref payload_id) => {
                payload_id
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad sensor id");
            }
        };

        let tag = match self.tag {
            serde_json::Value::String(ref tag) => {
                tag
            },
            _ => {
                // board.usb_serial_send("bad sensor id\n");
                return Err("bad calibration point tag");
            }
        };

        return Ok(SensorCalibrateRemove {
            object: "sensor", // TODO: what's the most ideal way to handle these?  no real reason to convert them again
            action: "calibrate",
            id,
            subcommand: "remove",
            tag

        })
    }
}

#[derive(Serialize, Deserialize)]
pub enum CommandPayload {
    DataloggerSetCommandPayload(DataloggerSetCommandPayload),
    DataloggerGetCommandPayload(DataloggerGetCommandPayload),
    DataloggerSetModeCommandPayload(DataloggerSetModeCommandPayload),
    SensorSetCommandPayload(SensorSetCommandPayload, Value),
    SensorGetCommandPayload(SensorGetCommandPayload),
    SensorRemoveCommandPayload(SensorRemoveCommandPayload),
    SensorListCommandPayload(SensorListCommandPayload),
    SensorCalibratePointPayload(SensorCalibratePointPayload),
    SensorCalibrateListPayload(SensorCalibrateListPayload),
    SensorCalibrateRemovePayload(SensorCalibrateRemovePayload),
    SensorCalibrateFitPayload(SensorCalibrateFitPayload),
    SensorCalibrateClearPayload(SensorCalibrateClearPayload),
    BoardRtcSetPayload(BoardRtcSetPayload),
    BoardGetPayload(BoardGetPayload),
    BoardSerialSendPayload(BoardSerialSendPayload),
    TelemeterGet
}


// errors to use in refactor
pub enum CommandError {
    ParseError(serde_json::Error), // can we store a string with the error string in the the enum class?
    InvalidCommand,
    InvalidPayload(serde_json::Error)
}

impl Debug for CommandError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::ParseError(arg0) => f.debug_tuple("ParseError").field(arg0).finish(),
            Self::InvalidCommand => write!(f, "InvalidCommand"),
            Self::InvalidPayload(arg0) => f.debug_tuple("InvalidPayload").field(arg0).finish(),
        }
    }
}

pub fn get_id(id: &serde_json::Value) -> Result<(&alloc::string::String), (&str)> {
    match id {
        serde_json::Value::String(ref payload_id) => {
            return Ok(payload_id)
        },
        _ => {
            // board.usb_serial_send("bad sensor id\n");
            return Err("bad sensor id")
        }
    };

}


pub fn mode_text(mode: &DataLoggerMode) -> &'static str {
    match mode {
        DataLoggerMode::Interactive => "interactive",
        DataLoggerMode::Watch => "watch",
        DataLoggerMode::Quiet => "quiet",
        DataLoggerMode::Field => "field",
        DataLoggerMode::HibernateUntil => "hibernate",
    }
}


// TODO: this is a command implementation, probably doesn't below in this file
pub fn get_board(board: &mut impl RRIVBoard, payload: BoardGetPayload){

    // let mut board_response = BoardResponse::new();
    
    if let Some(param) = payload.parameter {
        match param {
            serde_json::Value::String(param) => {
                rprintln!("{:?}", param.as_str());
                match param.as_str() {
                    "epoch" => {
                        let epoch = board.epoch_timestamp();
                        board.usb_serial_send(format!("{:}\n", epoch).as_str());
                    }
                    "version" => {
                        let mut branch = "none";
                        if let Some(found_branch) = option_env!("GIT_BRANCH") {
                            branch = found_branch;
                        }
                         let mut gitref = "none";
                        if let Some(found_ref) = option_env!("GIT_REF") {
                            gitref = found_ref;
                        }

                        let response = json!({
                            "hv":"0.4.2",
                            "fv":"0.5.0",
                            "br":branch,
                            "ref":gitref
                        });
                        responses::send_command_response_message(board, format!("{}\n", response).as_str());
                    }
                    "eeprom" => {
                        board.dump_eeprom();
                    },
                    _ => {
                        responses::send_command_response_message(board, "Unsupported param in command");
                    }
                }
            }
            err => {
                responses::send_command_response_message(board, "Bad param in command");
                rprintln!("Bad epoch {:?}", err);
                return;
            }

        }
    } else {
        let epoch = board.epoch_timestamp();
        board.usb_serial_send(format!("{:}", epoch).as_str());                
    }

}