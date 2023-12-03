use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct RgbLedPin {
  pub red: Pin<'A', 8, Output<OpenDrain>>,
  pub green: Pin<'A', 9, Output<OpenDrain>>,
  pub blue: Pin<'A', 10, Output<OpenDrain>>,
}

impl RgbLedPin { // And button?
  pub fn build(
      red: Pin<'A', 8>,
      green: Pin<'A', 9>,
      blue: Pin<'A', 10>,
      cr: &mut GpioCr,
  ) -> Self {
      return RgbLedPin {
          red: red.into_open_drain_output_with_state(&mut cr.gpioa_crh, PinState::High),
          green: green.into_open_drain_output_with_state(&mut cr.gpioa_crh, PinState::High),
          blue: blue.into_open_drain_output_with_state(&mut cr.gpioa_crh, PinState::High),
      };
  }
}