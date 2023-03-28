#include <Arduino.h>
#include "rust_serial.h"


void * rustSerial;

void setup(void) {
  pinMode(PA5, OUTPUT);
  digitalWrite(PA5, HIGH);
  delay(500);
  digitalWrite(PA5, LOW);
  delay(500);
  digitalWrite(PA5, HIGH);
  delay(500);
  rustSerial = rust_serial_interface_new();
  digitalWrite(PA5, LOW);
  delay(250);
  digitalWrite(PA5, HIGH);
  delay(250);
  digitalWrite(PA5, LOW);
  delay(250);
  digitalWrite(PA5, HIGH);
  delay(250);
}

void loop(void)
{
  rust_serial_write(rustSerial, 'a');
  digitalWrite(PA5, LOW);
  delay(500);
  digitalWrite(PA5, HIGH);
  delay(500);
  uint8_t aChar = rust_serial_read(rustSerial);
  rust_serial_write(rustSerial, aChar);
  
  
  digitalWrite(PA5, HIGH);
  delay(500);
  digitalWrite(PA5, LOW);
  delay(500);
}