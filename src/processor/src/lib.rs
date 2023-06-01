// #![cfg_attr(not(test), no_std)]
// https://ferrous-systems.com/blog/test-embedded-app/

const BUFFER_NUM: usize = 10;

// #[derive(Default)]
struct CommandRecognizer {
  receiving: bool,
  pendingMessages: usize,
  messageReady: bool,
  // buffer: [[char; 100]; BUFFER_NUM],
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
      commandPos: 0
    }
 }

  fn processCharacter(&self, character: char) {

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

    if  self.currentBuffer - self.firstBuffer >= BUFFER_NUM - 1 {
      // circular buffer is full
      return
    }

    self.receiving = true;
    self.addCharacterToBuffer(character);

  }

  fn addCharacterToBuffer(&self, character: char) {
    // self.buffer[self.currentBuffer][self.commandPos] = character;
    self.commandPos = self.commandPos + 1;
  }
}


// #[cfg(test)]
// mod tests {
//   use super::*;

//   #[test]
//   fn test_result() {

//     assert_eq!(true, false)
//   }
// }