use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde_json::{json, Value};

use crate::alloc::string::ToString;

pub fn send_command_response_message(board: &mut impl RRIVBoard, message: &str) {
    rprintln!("{}", message);
    board.usb_serial_send(json!({"message":message}).to_string().as_str());
    board.usb_serial_send("\n");
}

pub fn send_command_response_error(board: &mut impl RRIVBoard, message: &str, error: &str) {
    board.usb_serial_send(json!({"message":message, "error": error}).to_string().as_str());
    board.usb_serial_send("\n");
}

pub fn send_json(board: &mut impl RRIVBoard, json: Value){
    board.usb_serial_send(json.to_string().as_str());
    board.usb_serial_send("\n");
}