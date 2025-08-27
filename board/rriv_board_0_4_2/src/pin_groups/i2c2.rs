use stm32f1xx_hal::gpio::*;
use crate::pins::*;

pub type I2c2Scl = Pin<'B', 10, Alternate<OpenDrain>>;
pub type I2c2Sda = Pin<'B', 11, Alternate<OpenDrain>>;

pub struct I2c2Pins {
  pub i2c2_scl: I2c2Scl,
  pub i2c2_sda: I2c2Sda,
}

impl I2c2Pins {
  pub fn build(
    i2c2_scl: Pin<'B', 10>,
    i2c2_sda: Pin<'B', 11>,
    cr: &mut GpioCr
  ) -> Self {

    return I2c2Pins {
      i2c2_scl: i2c2_scl.into_alternate_open_drain(&mut cr.gpiob_crh), 
      i2c2_sda: i2c2_sda.into_alternate_open_drain(&mut cr.gpiob_crh),
    }
  }

   pub fn rebuild(
    i2c2_scl: Pin<'B', 10, Output<OpenDrain>>,
    i2c2_sda: Pin<'B', 11, Output<OpenDrain>>,
    cr: &mut GpioCr
  ) -> Self {

    return I2c2Pins {
      i2c2_scl: i2c2_scl.into_alternate_open_drain(&mut cr.gpiob_crh), 
      i2c2_sda: i2c2_sda.into_alternate_open_drain(&mut cr.gpiob_crh),
    }
  }
}
