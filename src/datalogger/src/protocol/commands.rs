use alloc::format;

use rriv_board::RRIVBoard;
use rtt_target::rprintln;

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
                        board.serial_send(format!("{:}\n", epoch).as_str());
                    },
                    "eeprom" => {
                        board.dump_eeprom();
                    },
                    _ => {
                        board.serial_send("Unsupported param in command\n");
                    }
                }
            }
            err => {
                board.serial_send("Bad param in command\n");
                rprintln!("Bad epoch {:?}", err);
                return;
            }

        }
    } else {
        let epoch = board.epoch_timestamp();
        board.serial_send(format!("{:}", epoch).as_str());                
    }

}