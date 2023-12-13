use embedded_hal::spi::{Mode, Phase, Polarity};
use embedded_sdmmc::SdCard;
use stm32f1xx_hal::{spi::{Spi, Spi1NoRemap}, pac::SPI1};

use crate::*;

pub const MODE: Mode = Mode {
  phase: Phase::CaptureOnSecondTransition,
  polarity: Polarity::IdleHigh,
};

pub fn build(pins: Spi1Pins, spi: SPI1, mapr: &mut MAPR, clocks: Clocks, delay: SysDelay) {
  let spi = Spi::spi1(
    spi,
    (pins.sck, pins.miso, pins.mosi),
    mapr,
    MODE,
    1.MHz(),
    clocks,
  );

  let sdcard = embedded_sdmmc::SdCard::new(spi, pins.sd_card_chip_select, delay);

}

type RrivSdCard = SdCard<Spi<SPI1, Spi1NoRemap, (Pin<'A', 5, Alternate>, Pin<'A', 6>, Pin<'A', 7, Alternate>), u8>, Pin<'C', 8, Output>, SysDelay>;
struct Storage {
  sd_card: RrivSdCard,
}


impl Storage {

  pub fn new(sd_card: RrivSdCard) -> Self {
    Storage {
      sd_card
    }
  }

  pub fn write(&mut self){

  }

}
