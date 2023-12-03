use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct Power {
  pub enable_3v: Pin<'C', 5, Output>,
  pub enable_5v: Pin<'B', 12, Output>,
}

impl Power {
  pub fn build(
      enable_3v: Pin<'C', 5>,
      enable_5v: Pin<'B', 12>,
      cr: &mut GpioCr,
  ) -> Self {
      return Power {
          enable_3v: enable_3v.into_push_pull_output(&mut cr.gpioc_crl),
          enable_5v: enable_5v.into_push_pull_output(&mut cr.gpiob_crh),
       };
  }
}