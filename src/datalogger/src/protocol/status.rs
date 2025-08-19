use rriv_board::RRIVBoard;
use rtt_target::rprintln;
use serde_json::json;

use crate::alloc::string::ToString;

pub fn send_ready_status(board: &mut impl RRIVBoard) {
    board.usb_serial_send(json!({"status":"datalogger-ready"}).to_string().as_str());
    board.usb_serial_send("\n");
}