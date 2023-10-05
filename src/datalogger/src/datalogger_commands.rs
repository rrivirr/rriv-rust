use serde::{Deserialize, Serialize};
use serde_json::Value;

#[derive(Serialize, Deserialize, Debug)]
pub struct DataloggerSetCommandPayload {
    object: Value,
    action: Value,
    logger_name: Option<Value>,
    site_name: Option<Value>,
    deployment_identifier: Option<Value>,
    burst_number: Option<u8>,
    start_up_delay: Option<u16>,
    user_note: Option<Value>,
    user_value: Option<i16>
}


#[derive(Serialize, Deserialize, Debug)]
pub struct DataloggerGetCommandPayload {
    object: Value,
    action: Value,
    propery: Option<Value>
}

pub enum CommandPayload {
    SetCommandPayload(DataloggerSetCommandPayload),
    GetCommandPayload(DataloggerGetCommandPayload),
    UnrecognizedCommand(),
    InvalidPayload()
}


// errors to use in refactor
pub enum CommandErrors {
    UnrecognizedCommand,
    InvalidPayload
}
