use core::fmt::Write;

use rriv_board::RRIVBoard;
use rtt_target::rprintln;

use crate::services::usart_service;
use alloc::string::String;

use alloc::format;



#[derive(Clone, Copy)]
enum RakWireless3172Step {
    Begin = 0,
    StopJoinConfirm = 1,
    SetBand = 2,
    SetBandConfirm = 3,
    SetMask = 4,
    SetMaskConfirm = 5,
    StartJoin = 6,
    StartJoinConfirm = 7,
    CheckJoined = 8,
    Joined = 9,
    Undefined = 255
}

impl RakWireless3172Step {
    fn from_integer(v: u8) -> Self {
        match v {
            0 => Self::Begin,
            1 => Self::StopJoinConfirm,
            2 => Self::SetBand,
            3 => Self::SetBandConfirm,
            4 => Self::SetMask,
            5 => Self::SetMaskConfirm,
            6 => Self::StartJoin,
            7 => Self::StartJoinConfirm,
            8 => Self::CheckJoined,
            9 => Self::Joined,
            _ => Self::Undefined
        }
    }

    fn next(self) -> Self {
        Self::from_integer((self as u8) + 1)
    }

    fn status(&self) -> String {
        match self {
            RakWireless3172Step::Joined => {
                return String::from("Joined");
            },
            _ => {
                return String::from("Not Joined");
            }
        }
    }
}

pub struct RakWireless3172 {
    telemetry_step: RakWireless3172Step,
    usart_send_time: i64,
    last_transmission: i64
}

impl RakWireless3172 {
    pub fn new() -> RakWireless3172 {
        return Self {
            telemetry_step: RakWireless3172Step::Begin,
            usart_send_time: 0,
            last_transmission: 0
        }
    }

    pub fn status(&self) -> String {
        self.telemetry_step.status()
    }

    fn send_and_increment_step(&mut self, board: &mut impl RRIVBoard, message: &str) {
        let prepared_message = format!("{}\r\n", message);
        board.usart_send(&prepared_message);
        self.usart_send_time = board.timestamp();
        self.telemetry_step = self.telemetry_step.next();
        rprintln!("trying telemetry step {}", self.telemetry_step as u8);
    }

    fn check_ok_or_restart(&mut self, board: &mut impl RRIVBoard) {
        match usart_service::take_command(board) {
            Ok(message) => {
                let mut message = message;
                let message = util::str_from_utf8(&mut message);
                match message {
                    Ok(message) => match message.find("OK") {
                        Some(index) => {
                            if index == 0 {
                                self.telemetry_step = self.telemetry_step.next();
                                rprintln!("trying telemetry step {}", self.telemetry_step as u8);
                                return;
                            }
                        }
                        None => {
                            rprintln!("telem not ok: {}", message);
                            self.telemetry_step = RakWireless3172Step::Begin;
                            return;
                        }
                    },
                    Err(_) => {
                        self.telemetry_step = RakWireless3172Step::Begin; // bad message
                        return;
                    }
                }
            }
            Err(_) => {} // no command ready, check timeout
        }

        // need to check for a timeout here
        // rprintln!("no message, checking timeout");
        if board.timestamp() - self.usart_send_time > 2 {
            rprintln!("timed out, going to step 0");
            self.telemetry_step = RakWireless3172Step::Begin;
        }
    }

    // TODO: need a command recieved queue, just like for USB
    // AT_BUSY_ERROR
    // Restricted_Wait_158785
    fn check_joined(&mut self, board: &mut impl RRIVBoard) {
        match usart_service::take_command(board) {
            Ok(message) => {
                let mut message = message;
                let message = util::str_from_utf8(&mut message);
                match message {
                    Ok(message) => match message.find("+EVT:JOINED") {
                        Some(index) => {
                            if index == 0 {
                                self.telemetry_step = self.telemetry_step.next();
                                rprintln!("Joined!!");  
                                return;
                            }
                        }
                        None => {
                            rprintln!("{}", message);
                        }
                    },
                    Err(_) => {} // bad message
                }
            }
            Err(_) => {} // keep waiting
        }

        // checking timeout if we didn't return above
        if board.timestamp() - self.usart_send_time > 2 {
            // just let it keep trying
        }

    }

    pub fn run_loop_iteration(&mut self, board: &mut impl RRIVBoard) {
        match self.telemetry_step {
            RakWireless3172Step::Begin => {
                rprintln!("trying telemetry step {}", self.telemetry_step as u8);
                self.send_and_increment_step(board, "AT+JOIN=0");
            }
            RakWireless3172Step::StopJoinConfirm => {
                self.check_ok_or_restart(board);
            }
            RakWireless3172Step::SetBand => {
                self.send_and_increment_step(board, "AT+BAND=5");
            }
            RakWireless3172Step::SetBandConfirm => {
                self.check_ok_or_restart(board);
            }
            RakWireless3172Step::SetMask => {
                self.send_and_increment_step(board, "AT+MASK=0002");
            }
            RakWireless3172Step::SetMaskConfirm => {
                self.check_ok_or_restart(board);
            }
            RakWireless3172Step::StartJoin => {
                self.send_and_increment_step(board, "AT+JOIN=1:0:15:100");
            }
            RakWireless3172Step::StartJoinConfirm => {
                self.check_ok_or_restart(board);
            }
            RakWireless3172Step::CheckJoined => {
                self.check_joined(board);
            }
            _ => {}
        }

        // rprintln!("done setting up lorawan")
    }

    pub fn transmit(&mut self, board: &mut impl RRIVBoard, payload: &[u8]){
        // AT+SEND=14:696E746572727570743
        
        let len = payload.len();

        let mut s = String::with_capacity(payload.len() * 2);
        for byte in payload {
            write!(&mut s, "{:02X}", byte);
        }
        
        let command = format!("AT+SEND={}:{}\r\n", payload.len(), s.as_str());
        board.usart_send(command.as_str());
        self.last_transmission = board.timestamp();
  
    }

    pub fn ready_to_transmit(&mut self, board: &mut impl RRIVBoard) -> bool{
        if self.status() != "Joined" {
            return false;
        }

        if board.timestamp() < self.last_transmission + 10 {
            false
        } else {
            true
        }
    }

}