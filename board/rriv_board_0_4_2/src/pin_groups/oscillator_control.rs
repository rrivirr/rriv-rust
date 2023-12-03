use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct OscillatorControl {
  pub enable_hse: Pin<'C', 13, Output>,
}

impl OscillatorControl {
  pub fn build(enable_hse: Pin<'C', 13>, cr: &mut GpioCr) -> Self {
      Self {
          enable_hse: enable_hse.into_push_pull_output(&mut cr.gpioc_crh),
      }
  }
}