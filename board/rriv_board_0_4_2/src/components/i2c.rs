use stm32f1xx_hal::{i2c::{BlockingI2c, Mode}, pac::{afio::MAPR, I2C1, I2C2}, rcc::Clocks};

use crate::{pin_groups, pins::GpioCr};

use stm32f1xx_hal::prelude::*;

// type aliases to make things tenable
pub type BoardI2c1 = BlockingI2c<I2C1>;
pub type BoardI2c2 = BlockingI2c<I2C2>;

pub fn setup_i2c1(
  pins: pin_groups::I2c1Pins,
  cr: &mut GpioCr,
  i2c1: I2C1,
  clocks: &Clocks,
) -> BoardI2c1 {
  let scl1 = pins.i2c1_scl.into_alternate_open_drain(&mut cr.gpiob_crl); // i2c
  let sda1 = pins.i2c1_sda.into_alternate_open_drain(&mut cr.gpiob_crl); // i2c
  BlockingI2c::new(
      i2c1,
      (scl1, sda1),
      Mode::Standard {
          frequency: 100.kHz(), 
      },
      clocks,
      1000,
      10,
      1000,
      1000,
  )
}

pub fn setup_i2c2(
  pins: pin_groups::I2c2Pins,
  cr: &mut GpioCr,
  i2c2: I2C2,
  clocks: &Clocks,
) -> BoardI2c2 {
  let scl2 = pins.i2c2_scl.into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
  let sda2 = pins.i2c2_sda.into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
  let x = BlockingI2c::new(
      i2c2,
      (scl2, sda2),
      Mode::Standard {
          frequency: 100.kHz(), // slower to same some energy?
      },
      clocks,
      1000,
      10,
      1000,
      1000,
  );

  return x;
}
