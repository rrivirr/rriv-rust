use stm32f1xx_hal::gpio::*;
use crate::pins::*;

pub struct RgbLedPins {
  pub red: Pin<'A', 8, Alternate<OpenDrain>>,
  pub green: Pin<'A', 9, Alternate<OpenDrain>>,
  pub blue: Pin<'A', 10, Alternate<OpenDrain>>,
}

impl RgbLedPins { // And button?
  pub fn build(
      red: Pin<'A', 8>,
      green: Pin<'A', 9>,
      blue: Pin<'A', 10>,
      cr: &mut GpioCr,
  ) -> Self {
      return RgbLedPins {
          red: red.into_alternate_open_drain(&mut cr.gpioa_crh),
          green: green.into_alternate_open_drain(&mut cr.gpioa_crh),
          blue: blue.into_alternate_open_drain(&mut cr.gpioa_crh),
      };
  }
}