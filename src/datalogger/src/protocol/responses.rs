use alloc::boxed::Box;
use alloc::format;
use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde_json::{json, Value};

use crate::{alloc::string::ToString, drivers::types::CalibrationPair};

pub fn send_command_response_message(board: &mut impl RRIVBoard, message: &str) {
    rprintln!("{}", message);
    board.usb_serial_send(json!({"message":message}).to_string().as_str());
    board.usb_serial_send("\n");
}

pub fn send_command_response_error(board: &mut impl RRIVBoard, message: &str, error: &str) {
    board.usb_serial_send(
        json!({"message":message, "error": error})
            .to_string()
            .as_str(),
    );
    board.usb_serial_send("\n");
}

pub fn send_json(board: &mut impl RRIVBoard, json: Value) {
    board.usb_serial_send(json.to_string().as_str());
    board.usb_serial_send("\n");
}

pub fn calibration_point_list(board: &mut impl RRIVBoard, pairs: &Option<Box<[CalibrationPair]>>) {
    board.usb_serial_send("{ pairs: [");

    if let Some(pairs) = pairs {
        for i in 0..pairs.len() {
            rprintln!("calib pair{:?}", i);
            let pair = &pairs[i];
            board.usb_serial_send(format!("{{'point': {}, 'values': [", pair.point).as_str());
            for i in 0..pair.values.len() {
                board.usb_serial_send(format!("{}", pair.values[i]).as_str());
                if i < pair.values.len() - 1 {
                    board.usb_serial_send(",");
                }
            }

            board.usb_serial_send("] }");
            if i < pairs.len() - 1 {
                board.usb_serial_send(",");
            }
        }
    }

    board.usb_serial_send("]}\n");
}
