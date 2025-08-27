use stm32f1xx_hal::gpio::*;
use crate::pins::*;

pub struct PowerPins {
  pub enable_3v: Pin<'C', 5, Output>,
  pub enable_5v: Pin<'B', 12, Output>,
}

impl PowerPins {
  pub fn build(
      enable_3v: Pin<'C', 5>,
      enable_5v: Pin<'B', 12>,
      cr: &mut GpioCr,
  ) -> Self {
      return PowerPins {
          enable_3v: enable_3v.into_push_pull_output(&mut cr.gpioc_crl),
          enable_5v: enable_5v.into_push_pull_output(&mut cr.gpiob_crh),
       };
  }
}