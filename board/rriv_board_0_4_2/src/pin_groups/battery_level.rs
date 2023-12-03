use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;


pub struct BatteryLevelPins {
  pub vin_measure: Pin<'B', 0, Input>,
  pub enable_vin_measure: Pin<'B', 1, Output>,
}

impl BatteryLevelPins {
  pub fn build(
    vin_measure: Pin<'B', 0>,
    enable_vin_measure: Pin<'B', 1>,
    cr: &mut GpioCr
  ) -> Self {
    return BatteryLevelPins {
      vin_measure: vin_measure.into_floating_input(&mut cr.gpiob_crl),
      enable_vin_measure: enable_vin_measure.into_push_pull_output(&mut cr.gpiob_crl),
    }
  }

}
