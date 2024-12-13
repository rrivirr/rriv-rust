use stm32f1xx_hal::{gpio::*, afio::MAPR};
use crate::pins::*;

pub struct Spi1Pins {
  pub sck: Pin<'A', 5, Alternate>,
  pub miso: Pin<'A', 6>,
  pub mosi: Pin<'A', 7, Alternate>,
  // pub sd_card_chip_select: Pin<'C', 8, Output>
}

impl Spi1Pins {
  pub fn build(
    sck: Pin<'A', 5>,
    miso: Pin<'A', 6>,
    mosi: Pin<'A', 7>,
    // sd_card_chip_select: Pin<'C', 8>,
    cr: &mut GpioCr
  ) -> Self {

    Spi1Pins {
      sck: sck.into_alternate_push_pull(&mut cr.gpioa_crl),
      miso,
      mosi: mosi.into_alternate_push_pull(&mut cr.gpioa_crl),
      // sd_card_chip_select: sd_card_chip_select.into_push_pull_output(&mut cr.gpioc_crh),
    }
  }

}