use core::{borrow::BorrowMut, fmt::Error};
use alloc::boxed::Box;
use rriv_board::{RRIVBoard, RXProcessor};


const USART_BUFFER_NUM: usize = 3; // Includes an extra empty cell for end marker
const USART_BUFFER_SIZE: usize = 40;

static mut MESSAGE_DATA: MessageData = MessageData::default();

pub struct MessageData {
    buffer: [[u8; USART_BUFFER_SIZE]; USART_BUFFER_NUM],
    cur: usize,
    end: usize,
    command_pos: usize,
}

impl MessageData {
    pub const fn default() -> Self {
        Self {
            cur: 0,
            end: USART_BUFFER_NUM - 1,
            command_pos: 0,
            buffer: [[b'\0'; USART_BUFFER_SIZE]; USART_BUFFER_NUM],
        }
    }
}

pub struct USARTCharacterProcessor {}

impl<'a> USARTCharacterProcessor {
    pub fn new() -> USARTCharacterProcessor {
        USARTCharacterProcessor {}
    }
}

impl<'a, 'b> RXProcessor for USARTCharacterProcessor {
    fn process_character(&self, character: u8) {
        unsafe {
            let message_data = MESSAGE_DATA.borrow_mut();
            process_character(message_data, character);
        }
    }
}


pub fn setup(board: &mut impl RRIVBoard) {
    let char_processor = Box::<USARTCharacterProcessor>::leak(Box::new(USARTCharacterProcessor::new()));

    // pass a pointer to the leaked processor to Board::set_rx_processor
    board.set_usart_rx_processor(Box::new(char_processor));
}


pub fn process_character(command_data: &mut MessageData, character: u8) {
    let cur = command_data.cur;
    let pos: usize = command_data.command_pos;

    if command_data.cur == command_data.end {
        // circular buffer is full
        return;
    }

    if character == b'\n' && pos > 0 && command_data.buffer[cur][pos - 1] == b'\r' {
        // command is done
        command_data.buffer[cur][pos - 1] = 0; // remove the carriage return
        command_data.command_pos = 0;
        command_data.cur = (command_data.cur + 1) % USART_BUFFER_NUM;
    }

    command_data.buffer[cur][pos] = character;
    if pos < USART_BUFFER_SIZE - 1 {
        command_data.command_pos = command_data.command_pos + 1;
    }
}

fn _pending_message_count(command_data: &MessageData) -> usize {
    if command_data.cur >= (command_data.end + 1) % USART_BUFFER_NUM {
        return command_data.cur - (command_data.end + 1) % USART_BUFFER_NUM;
    } else {
        return command_data.cur + USART_BUFFER_NUM - (command_data.end + 1) % USART_BUFFER_NUM;
    }
}

fn _take_command(command_data: &mut MessageData) -> [u8; USART_BUFFER_SIZE] {
    // clone the command bytes buffer so the caller isn't borrowing the command_data buffer
    let buffer_index = (command_data.end + 1) % USART_BUFFER_NUM;
    let command = command_data.buffer[buffer_index].clone();
    // null the USART_BUFFER_SIZE
    command_data.buffer[buffer_index] = [b'\0'; USART_BUFFER_SIZE];
    // move the end marker, effectively marking the buffer as ready for use again
    command_data.end = buffer_index;

    return command;
}

pub fn pending_message_count(board: &impl RRIVBoard) -> usize {
    let get_pending_message_count = || unsafe {
        let command_data = MESSAGE_DATA.borrow_mut();
        _pending_message_count(&command_data)
    };

    board.critical_section(get_pending_message_count)
}

pub fn take_command(board: &impl RRIVBoard) -> Result<[u8; USART_BUFFER_SIZE], ()> {
    if pending_message_count(board) < 1 {
        (());
    }

    let do_take_command = || unsafe {
        let command_data = MESSAGE_DATA.borrow_mut();
        Ok(_take_command(command_data))
    };

    board.critical_section(do_take_command)
}
