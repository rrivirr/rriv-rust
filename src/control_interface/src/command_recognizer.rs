//#![cfg_attr(not(test), no_std)]
// https://ferrous-systems.com/blog/test-embedded-app/

const BUFFER_NUM: usize = 10;

// #[derive(Default)]
struct CommandRecognizer {
  receiving: bool,
  pendingMessages: usize,
  messageReady: bool,
  buffer: [[char; 100]; BUFFER_NUM],
  firstBuffer: usize,
  currentBuffer: usize,
  commandPos: usize,
}

impl CommandRecognizer {

  fn default() -> Self {
    Self {
      receiving: false,
      pendingMessages: 0,
      firstBuffer: 0,
      currentBuffer: 0,
      messageReady: false,
      commandPos: 0,
      buffer: [ [ '\0'; 100]; BUFFER_NUM]
    }
 }

  fn processCharacter(&mut self, character: char) {

    if character == '\r' {
      self.receiving = false;
      self.pendingMessages = self.pendingMessages + 1;
      self.currentBuffer = self.currentBuffer + 1;
      self.messageReady = true;
      return
    }

    if self.receiving == true {
      return
    }
    _ready
    if  self.currentBuffer - self.firstBuffer >= BUFFER_NUM - 1 {
      // circular buffer is full
      return
    }

    self.receiving = true;
    self.addCharacterToBuffer(character);

  }

  fn addCharacterToBuffer(&mut self, character: char) {
    self.buffer[self.currentBuffer][self.commandPos] = character;
    self.commandPos = self.commandPos + 1;
  }
}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_receiving() {
    let mut commandRecognizer = CommandRecognizer::default();
    commandRecognizer.processCharacter('a');

    assert_eq!(true, commandRecognizer.receiving)
  }

  #[test]
  fn test_receiving_done() {
    let mut commandRecognizer = CommandRecognizer::default();
    commandRecognizer.processCharacter('a');
    commandRecognizer.processCharacter('\r');

    assert_eq!(false, commandRecognizer.receiving)
  }

  fn test_message_ready() {
    let mut commandRecognizer = CommandRecognizer::default();
    commandRecognizer.processCharacter('a');
    commandRecognizer.processCharacter('\r');

    assert_eq!(false, commandRecognizer.messageReady)
  }
}