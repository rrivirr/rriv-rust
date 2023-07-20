//#![cfg_attr(not(test), no_std)]
// https://ferrous-systems.com/blog/test-embedded-app/

const BUFFER_NUM: usize = 11;  // Includes an extra empty cell for end marker
const BUFFER_SIZE: usize = 100;

// #[derive(Default)]
struct CommandRecognizer {
  receiving: bool,
  message_ready: bool,
  buffer: [[char; BUFFER_SIZE]; BUFFER_NUM],
  cur: usize,
  end: usize,
  command_pos: usize,
}

impl CommandRecognizer {

  fn default() -> Self {
    Self {
      receiving: false,
      cur: 0,
      end: BUFFER_NUM - 1,
      message_ready: false,
      command_pos: 0,
      buffer: [ [ '\0'; BUFFER_SIZE]; BUFFER_NUM]
    }
 }

  fn process_character(&mut self, character: char) {

    if self.receiving == true {
      if character == '\r' {
        self.receiving = false;
        self.cur = (self.cur + 1) % BUFFER_NUM; 
        self.message_ready = true;
      }

      self.add_character_to_buffer(character);
      return
    }

    if character == '{' {
      if self.cur == self.end {
        // circular buffer is full
        return
      }
      self.receiving = true;
      self.command_pos = 0;
      self.add_character_to_buffer(character)
    } 

  }

  fn add_character_to_buffer(&mut self, character: char) {
    self.buffer[self.cur][self.command_pos] = character;
    self.command_pos = self.command_pos + 1;
  }

  fn pending_message_count(&mut self) -> usize {
    return self.cur - (self.end + 1) % BUFFER_NUM
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
    println!("{}", command_recognizer.buffer[0].iter().cloned().collect::<String>());

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

    for c in command.chars() { 
      command_recognizer.process_character(c);
    }
    for c in command2.chars() { 
      command_recognizer.process_character(c);
    }

    assert_eq!(true, command_recognizer.message_ready);

    println!("{}", command2);
    println!("{}", command_recognizer.buffer[1].iter().cloned().collect::<String>());

    let mut matching = true;
    for (i, c) in command2.chars().enumerate() {
      if c == '\r' {
        break;
      }
      if command_recognizer.buffer[1][i] != c {
        println!("// {} {}", c, command_recognizer.buffer[1][i]);
        matching = false;
      }
    }
    assert_eq!(true, matching);

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