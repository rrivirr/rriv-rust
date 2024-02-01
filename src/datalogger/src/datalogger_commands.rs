use serde::{Deserialize, Serialize};
use serde_json::Value;
use alloc::fmt::Debug;

#[derive(Serialize, Deserialize, Debug)]
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


#[derive(Serialize, Deserialize, Debug)]
pub struct DataloggerGetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub propery: Option<Value>
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SensorSetCommandPayload {
    pub object: Value,
    pub action: Value, 
    pub id: [u8;6], // option
    pub r#type: [u8;16], // option
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SensorGetCommandPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SensorRemoveCommandPayload {
    pub object: Value,
    pub action: Value,
    pub id: Value,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct SensorListCommandPayload {
    pub object: Value,
    pub action: Value,
}

pub enum CommandPayload {
    DataloggerSetCommandPayload(DataloggerSetCommandPayload),
    DataloggerGetCommandPayload(DataloggerGetCommandPayload),
    SensorSetCommandPayload(SensorSetCommandPayload, Value),
    SensorGetCommandPayload(SensorGetCommandPayload),
    SensorRemoveCommandPayload(SensorRemoveCommandPayload),
    SensorListCommandPayload(SensorListCommandPayload)
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