use crate::*;

pub struct PowerControl {
  pins: PowerPins
}

impl PowerControl {
  pub fn new(pins: PowerPins) -> Self {
    return PowerControl {
      pins
    }
  }

  pub fn cycle_3v(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32> ) {
    self.pins.enable_3v.set_low();
    delay.delay_ms(250_u32);
    self.pins.enable_3v.set_high();
    delay.delay_ms(250_u32);
  }

  pub fn cycle_5v(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32> ) {
    self.pins.enable_5v.set_low();
    delay.delay_ms(250_u32);
    self.pins.enable_5v.set_high();
    delay.delay_ms(250_u32);
  }

  pub fn cycle_all(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32> ) {
    self.pins.enable_5v.set_low();
    self.pins.enable_3v.set_low();
    delay.delay_ms(250_u32);
    self.pins.enable_3v.set_high();
    self.pins.enable_5v.set_high();
  }
}