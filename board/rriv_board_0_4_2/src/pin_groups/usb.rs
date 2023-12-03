use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct Usb {
  pub usb_dp: Pin<'A', 12, Dynamic>,
  pub usb_dm: Pin<'A', 11>,
}

impl Usb {
  pub fn build(
    usb_dp: Pin<'A', 12>,
    usb_dm: Pin<'A', 11>,
    cr: &mut GpioCr
  ) -> Self {

    return Usb {
      usb_dp: usb_dp.into_dynamic(&mut cr.gpioa_crh),
      usb_dm: usb_dm,
    }
  }

}
