// #![cfg_attr(not(test), no_std)]

// https://ferrous-systems.com/blog/test-embedded-app/
use core::{
    borrow::BorrowMut,
    cell::{RefCell, RefMut},
    ops::DerefMut,
};
use cortex_m::interrupt::Mutex;
use rriv_0_4::RXProcessor; // create separate module that doesn't have embedded dependencies to import this

mod command_utility {
  use core::{
    cell::{Ref, RefMut},
  };

    pub const BUFFER_NUM: usize = 11; // Includes an extra empty cell for end marker
    pub const BUFFER_SIZE: usize = 100;

    pub struct CommandData {
        receiving: bool,
        message_ready: bool,
        buffer: [[char; BUFFER_SIZE]; BUFFER_NUM],
        cur: usize,
        end: usize,
        command_pos: usize,
    }

    impl CommandData {
        pub fn default() -> Self {
            Self {
                receiving: false,
                cur: 0,
                end: BUFFER_NUM - 1,
                message_ready: false,
                command_pos: 0,
                buffer: [['\0'; BUFFER_SIZE]; BUFFER_NUM],
            }
        }
    }
    pub struct CommandUtility {}
    impl CommandUtility {
        pub fn process_character(mut command_data: RefMut<CommandData>, character: char) {
            let receiving = command_data.receiving;
            let starting = character == '{';

            if receiving && !starting {
                // meaningless character
                return;
            }

            if receiving && character == '\r' {
                command_data.receiving = false;
                command_data.cur = (command_data.cur + 1) % BUFFER_NUM;
                command_data.message_ready = true;
                return;
            }

            if starting {
                if command_data.cur == command_data.end {
                    // circular buffer is full
                    return;
                }
                command_data.receiving = true;
                command_data.command_pos = 0;
            }

            let cur = command_data.cur;
            let pos = command_data.command_pos;
            command_data.buffer[cur][pos] = character;
            command_data.command_pos = command_data.command_pos + 1;
        }

        pub fn pending_message_count(mut command_data: Ref<CommandData>) -> usize {
            return command_data.cur - (command_data.end + 1) % BUFFER_NUM;
        }
      
  
        pub fn take_command(mut command_data: RefMut<CommandData>) -> [char; 100] {
            let command = command_data.buffer[(command_data.end + 1) % BUFFER_NUM];
            command_data.end = (command_data.end + 1) % BUFFER_NUM; // review this in a sec
            return command;
        }  


    }
}

pub struct CharacterProcessor<'a> {
    command_data: &'a mut Mutex<RefCell<command_utility::CommandData>>,
}

impl CharacterProcessor<'_> {
    pub fn new(command_data: &mut Mutex<RefCell<command_utility::CommandData>>) -> CharacterProcessor {
        CharacterProcessor {
            command_data: command_data,
        }
    }

    pub fn function_name() {}
}

impl RXProcessor for CharacterProcessor<'_> {
    fn process_character(&mut self, character: char) {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.command_data.borrow(cs).borrow_mut();
            command_utility::CommandUtility::process_character(command_data, character);
        })
    }
}

// refactor this so the data struct is separate and can be shared
pub struct CommandRecognizer {
    command_data: Mutex<RefCell<command_utility::CommandData>>,
}

impl CommandRecognizer {
    pub fn default() -> Self {
        Self {
            command_data: Mutex::new(RefCell::new(command_utility::CommandData::default())),
        }
    }

    pub fn getRXProcessor(&mut self) -> CharacterProcessor {
        CharacterProcessor::new(&mut self.command_data)
    }

    pub fn pending_message_count(&mut self) -> usize {
        cortex_m::interrupt::free(|cs| {
          let command_data = self.command_data.borrow(cs).borrow();
          return command_utility::CommandUtility::pending_message_count(command_data);
        })
    }

    pub fn take_command(&mut self) -> [char; 100] {
        cortex_m::interrupt::free(|cs| {
            let mut command_data = self.command_data.borrow(cs).borrow_mut();
            return command_utility::CommandUtility::take_command(command_data);
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_receiving() {
        let mut command_recognizer = CommandRecognizer::default();
        command_recognizer.process_character('{');

        assert_eq!(true, command_recognizer.receiving)
    }

    #[test]
    fn test_receiving_done() {
        let mut command_recognizer = CommandRecognizer::default();
        command_recognizer.process_character('{');
        command_recognizer.process_character('\r');

        assert_eq!(false, command_recognizer.receiving)
    }

    #[test]
    fn test_message_ready() {
        let mut command_recognizer = CommandRecognizer::default();
        command_recognizer.process_character('{');
        command_recognizer.process_character('\r');

        assert_eq!(true, command_recognizer.message_ready)
    }

    #[test]
    fn test_message_saved() {
        let mut command_recognizer = CommandRecognizer::default();
        let command = "{\"cmd\":\"set\",\"object\":\"sensor\"}\r";
        for c in command.chars() {
            command_recognizer.process_character(c);
        }

        assert_eq!(true, command_recognizer.message_ready);

        println!("{}", command);
        println!(
            "{}",
            command_recognizer.buffer[0]
                .iter()
                .cloned()
                .collect::<String>()
        );

        let mut matching = true;
        for (i, c) in command.chars().enumerate() {
            if c == '\r' {
                break;
            }
            if command_recognizer.buffer[0][i] != c {
                println!("// {} {}", c, command_recognizer.buffer[0][i]);
                matching = false;
            }
        }
        assert_eq!(true, matching);
    }

    #[test]
    fn test_multiple_messages() {
        let mut command_recognizer: CommandRecognizer = CommandRecognizer::default();
        let command = "{\"cmd\":\"set\",\"object\":\"sensor\"}\r";
        let command2 = "{\"cmd\":\"set\",\"object\":\"actuator\"}\r";
    }

    #[test]
    fn test_many_messages() {
        let mut command_recognizer: CommandRecognizer = CommandRecognizer::default();
        let command = "{\"cmd\":\"set\",\"object\":\"sensor\"}\r";

        for _i in 0..10 {
            for c in command.chars() {
                command_recognizer.process_character(c);
            }
        }
        println!("count {}", command_recognizer.pending_message_count());
        assert_eq!(10, command_recognizer.pending_message_count())
    }
}
