use stm32f1xx_hal::gpio::*;
use crate::pins::*;


pub type I2c1Scl = Pin<'B', 6, Alternate<OpenDrain>>;
pub type I2c1Sda = Pin<'B', 7, Alternate<OpenDrain>>;

pub struct I2c1Pins {
  pub i2c1_scl: I2c1Scl,
  pub i2c1_sda: I2c1Sda,
}

impl I2c1Pins {
  pub fn build(
    i2c1_scl: Pin<'B', 6>,
    i2c1_sda: Pin<'B', 7>,
    cr: &mut GpioCr
  ) -> Self {

    return I2c1Pins {
      i2c1_scl: i2c1_scl.into_alternate_open_drain(&mut cr.gpiob_crl), 
      i2c1_sda: i2c1_sda.into_alternate_open_drain(&mut cr.gpiob_crl),
    }
  }

 pub fn rebuild(
    i2c1_scl: Pin<'B', 6, Output<OpenDrain>>,
    i2c1_sda: Pin<'B', 7, Output<OpenDrain>>,
    cr: &mut GpioCr
  ) -> Self {

    return I2c1Pins {
      i2c1_scl: i2c1_scl.into_alternate_open_drain(&mut cr.gpiob_crl), 
      i2c1_sda: i2c1_sda.into_alternate_open_drain(&mut cr.gpiob_crl),
    }
  }


}
