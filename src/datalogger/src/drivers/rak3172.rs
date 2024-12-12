use super::types::*;

pub struct RAK3172 {

}

impl TelemeterDriver for RAK3172 {
    fn setup(&mut self, board: &mut dyn rriv_board::TelemetryDriverServices) {
        board.serial_send("ATE");
    }

    fn upload_measurement(&mut self, board: &mut dyn rriv_board::TelemetryDriverServices) {
        board.serial_send("AT+BAND?");
    }
}