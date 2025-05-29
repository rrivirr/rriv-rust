use crate::*;

pub struct ExternalAdc {
  pins: pin_groups::ExternalAdcPins
}

impl ExternalAdc {
  pub fn new(pins: pin_groups::ExternalAdcPins) -> Self {
    return ExternalAdc {
      pins
    }
  }

  



  pub fn start(&mut self,  delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8> ) {

    self.enable(delay);
    self.reset(delay);
  }

  pub fn enable(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {

    self.pins.enable.set_low();
    delay.delay_ms(250); // let the chip get power
    
  }

  pub fn disable(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {

    self.pins.enable.set_high();
    delay.delay_ms(250); // let the chip power down
    
  }

  pub fn reset(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {

    delay.delay_ms(1_u8);  // delay > 50ns before applying ADC reset
    self.pins.reset.set_low();
    delay.delay_ms(1_u8);  // delay > 10ns after starting ADC reset
    self.pins.reset.set_high();
    delay.delay_ms(100_u8);  // let the chip start up

  }

  pub fn shutdown(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>){

    self.pins.enable.set_high();

  }
 
}

