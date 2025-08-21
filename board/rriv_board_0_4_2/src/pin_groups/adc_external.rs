use stm32f1xx_hal::gpio::*;
use crate::pins::*;

pub struct ExternalAdcPins {
  pub enable: Pin<'C', 6, Output>,
  pub reset: Pin<'A', 4, Output>,
}

impl ExternalAdcPins {
  pub fn build(
    enable: Pin<'C', 6>, 
    reset: Pin<'A', 4>,
    cr: &mut GpioCr) -> Self {
      Self {
        enable: enable.into_push_pull_output(&mut cr.gpioc_crl),
        reset: reset.into_push_pull_output(&mut cr.gpioa_crl),
      }
  }
}