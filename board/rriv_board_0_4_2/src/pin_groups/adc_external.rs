use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct ExternalAdc {
  enable_adc: Pin<'C', 6, Output>,
  enable_avdd: Pin<'A', 1, Output>,
  reset: Pin<'A', 4, Output>,
}

impl ExternalAdc {
  pub fn build(
    enable_adc: Pin<'C', 6>, 
    enable_avdd: Pin<'A', 1>,
    reset: Pin<'A', 4>,
    cr: &mut GpioCr) -> Self {
      Self {
        enable_adc: enable_adc.into_push_pull_output(&mut cr.gpioc_crl),
        enable_avdd: enable_avdd.into_push_pull_output(&mut cr.gpioa_crl),
        reset: reset.into_push_pull_output(&mut cr.gpioa_crl),
      }
  }
}