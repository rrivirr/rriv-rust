use crate::*;

pub struct ExternalAdc {
  pins: pin_groups::ExternalAdcPins
}


// impl<SPI, CS, DELAYER> SdCard<SPI, CS, DELAYER>
// where
//     SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
//     CS: embedded_hal::digital::v2::OutputPin,
//     <SPI as embedded_hal::blocking::spi::Transfer<u8>>::Error: core::fmt::Debug,
//     <SPI as embedded_hal::blocking::spi::Write<u8>>::Error: core::fmt::Debug,
//     DELAYER: embedded_hal::blocking::delay::DelayUs<u8>,
// {
//     /// Create a new SD/MMC Card driver using a raw SPI interface.
//     ///
//     /// The card will not be initialised at this time. Initialisation is
//     /// deferred until a method is called on the object.
//     ///
//     /// Uses the default options.
//     pub fn new(spi: SPI, cs: CS, delayer: DELAYER) -> SdCard<SPI, CS, DELAYER> {
//         Self::new_with_options(spi, cs, delayer, AcquireOpts::default())
//     }


impl ExternalAdc {
  pub fn new(pins: pin_groups::ExternalAdcPins) -> Self {
    return ExternalAdc {
      pins
    }
  }



  pub fn start(&mut self,  delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8> ) {
    self.enable(delay);
    self.reset(delay);
  }

  pub fn enable(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8> ) {

    self.pins.enable.set_low();
    // delay.delay_ms(20); // let the chip get power, maybe not needed?
    
  }

  pub fn reset(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8> ) {

    delay.delay_ms(1_u8);  // delay > 50ns before applying ADC reset
    self.pins.reset.set_low();
    delay.delay_ms(1_u8);  // delay > 10ns after starting ADC reset
    self.pins.reset.set_high();
    delay.delay_ms(100_u8);  // let the chip start up

  }

  pub fn shutdown(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>){

    self.pins.enable.set_high();

  }
 
}

