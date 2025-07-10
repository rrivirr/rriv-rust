use alloc::format;

use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde_json::json;

use crate::BoardGetPayload;

// use super::board_response::BoardResponse;


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
                        board.usb_serial_send(format!("{}\n", response).as_str());
                    }
                    _ => {
                        board.usb_serial_send("Unsupported param in command\n");
                    }
                }
            }
            err => {
                board.usb_serial_send("Bad param in command\n");
                rprintln!("Bad epoch {:?}", err);
                return;
            }

        }
    } else {
        let epoch = board.epoch_timestamp();
        board.usb_serial_send(format!("{:}", epoch).as_str());                
    }

}