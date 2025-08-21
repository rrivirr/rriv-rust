use stm32f1xx_hal::gpio::*;
use crate::pins::*;

pub struct SerialPins {
  pub tx: Pin<'A', 2, Alternate>,
  pub rx: Pin<'A', 3>,
}

impl SerialPins {
  pub fn build(
    tx: Pin<'A', 2>,
    rx: Pin<'A', 3>,
    cr: &mut GpioCr
  ) -> Self {

    return SerialPins {
      tx: tx.into_alternate_push_pull(&mut cr.gpioa_crl), // USART2,
      rx: rx,
    }
  }

}
