#![no_std]
// https://ferrous-systems.com/blog/test-embedded-app/

const BUFFER_NUM: i32 = BUFFER_NUM;

#[derive(Default)]
struct CommandRecognizer {
  receiving: bool,
  pendingMessages: u16,
  messageReady: bool,
  // buffer: [[char; 100]; ::BUFFER_NUM],
  firstBuffer: u16,
  currentBuffer: u16,
  commandPos: u16,
}


impl Default for CommandRecognizer {
  fn default() -> Self {
      CommandRecognizer {
        receiving: false,
        pendingMessages: 0,
        firstBuffer: 0,
        currentBuffer: 0,
        messageReady: true,
        commandPos: 0,
        messageReady: false//,
        // buffer :
      }
   }
}

impl CommandRecognizer {

  fn processCharacter(character: char) -> char {

    if( character == "\r" ){
      receiving = false;
      pendingMessages = pendingMessages + 1;
      currentBuffer = currentBuffer + 1;
      return
    }

    if( receiving == true ){
      return
    }

    if( currentBuffer - firstBuffer >= BUFFER_NUM - 1 ){
      // circular buffer is full
      return
    }

    receiving = true;
    addCharacterToBuffer(character);

  }

  fn addCharacterToBuffer(character: char) {
    // buffer[currentBuffer][commandPos] = character;
    commandPos = commandPos + 1;
  }
}

extern crate std;
#[cfg(test)]
extern crate std;
mod tests {
  use super::*;

  #[test]
  fn test_result() {

    assert_eq!(true, false)
  }
}