use serde::{Deserialize, Serialize};
use serde_json::{Number, Value};
use alloc::fmt::Debug;
use alloc::boxed::Box;


#[derive(Serialize, Deserialize)]
pub struct DataloggerSetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub logger_name: Option<Value>,
    pub site_name: Option<Value>,
    pub deployment_identifier: Option<Value>,
    pub interval: Option<u16>,
    pub burst_repetitions: Option<u8>,
    pub start_up_delay: Option<u16>
    // pub user_note: Option<Value>, // not implemented for now
    // pub user_value: Option<i16>
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
    BoardSerialSendPayload(BoardSerialSendPayload)
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