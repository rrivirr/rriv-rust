use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct Spi2Pins {
  pub sck: Pin<'B', 13, Alternate>,
  pub miso: Pin<'B', 14>,
  pub mosi: Pin<'B', 15, Alternate>,
}

impl Spi2Pins {
  pub fn build(
    sck: Pin<'B', 13>,
    miso: Pin<'B', 14>,
    mosi: Pin<'B', 15>,
    cr: &mut GpioCr
  ) -> Self {

    Spi2Pins {
      sck: sck.into_alternate_push_pull(&mut cr.gpiob_crh),
      miso,
      mosi: mosi.into_alternate_push_pull(&mut cr.gpiob_crh),
    }
  }

}