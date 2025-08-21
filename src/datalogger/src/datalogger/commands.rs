use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde_json::json;
use alloc::format;
extern crate alloc;
use alloc::boxed::Box;
use crate::{
    alloc::string::ToString
};



use crate::{datalogger::payloads::BoardGetPayload, drivers::types::SensorDriver};
use crate::protocol::responses;
use crate::registry::sensor_name_from_type_id;



// Command implementations

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

pub fn set_sensor() {

}



pub fn list_sensors(board: &mut impl RRIVBoard, drivers: &[Option<Box<dyn SensorDriver>>; rriv_board::EEPROM_TOTAL_SENSOR_SLOTS]){
        board.usb_serial_send("{\"sensors\":[");
                let mut first = true;
                for i in 0..drivers.len() {
                    // create json and output it
                    if let Some(driver) = & drivers[i] {
                        if first {
                            first = false
                        } else {
                            board.usb_serial_send(",");
                        }

                        let mut id_bytes = driver.get_id();
                        let id_str = match util::str_from_utf8(&mut id_bytes) {
                            Ok(str) => str,
                            Err(_) => "error",
                        };

                        let type_id = driver.get_type_id();
                        let mut sensor_name_bytes = sensor_name_from_type_id(type_id.into());
                        let sensor_name_str = match util::str_from_utf8(&mut sensor_name_bytes) {
                            Ok(str) => str,
                            Err(_) => "error",
                        };

                        let json = json!({
                            "id": id_str,
                            "type": sensor_name_str
                        });
                        let string = json.to_string();
                        let str = string.as_str();
                        board.usb_serial_send(str);
                    }
                }
                board.usb_serial_send("]}");
                board.usb_serial_send("\n");
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