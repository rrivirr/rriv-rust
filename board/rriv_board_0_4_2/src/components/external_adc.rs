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



  pub fn start(&mut self,  delay: &mut SysDelay) {
    self.enable(delay);
    self.reset(delay);
  }

  pub fn enable(&mut self, delay: &mut SysDelay) {

    self.pins.enable.set_low();
    // delay.delay_ms(20); // let the chip get power, maybe not needed?
    
  }

  pub fn disable(&mut self, delay: &mut SysDelay) {

    self.pins.enable.set_high();
    // delay.delay_ms(20); // let the chip power down, maybe not needed?
    
  }


  pub fn reset(&mut self, delay: &mut SysDelay) {

    delay.delay_ms(1_u8);  // delay > 50ns before applying ADC reset
    self.pins.reset.set_low();
    delay.delay_ms(1_u8);  // delay > 10ns after starting ADC reset
    self.pins.reset.set_high();
    delay.delay_ms(100_u8);  // let the chip start up

  }

  // pub fn shutdown(&mut self, delay: &mut SysDelay){

  //   self.pins.enable.set_high();

  // }
 
}

