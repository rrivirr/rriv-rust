use crate::*;

pub struct OscillatorControl {
  pins: pin_groups::OscillatorControlPins
}

impl OscillatorControl {
  pub fn new(pins: pin_groups::OscillatorControlPins) -> Self {
    return OscillatorControl {
      pins
    }
  }

  pub fn enable_hse(&mut self, delay: &mut SysDelay) {

    self.pins.enable_hse.set_high();

  }

  pub fn disable_hse(&mut self, delay: &mut SysDelay) {

    self.pins.enable_hse.set_low();

  }
}

