use alloc::format;

use embedded_hal::spi::{Mode, Phase, Polarity};
use embedded_sdmmc::{SdCard, TimeSource, Timestamp, Volume, VolumeManager};
use pac::SPI2;
use stm32f1xx_hal::{pac::SPI1, spi::{Spi, Spi1NoRemap, Spi2NoRemap}};

use crate::*;

pub const MODE: Mode = Mode {
  phase: Phase::CaptureOnSecondTransition,
  polarity: Polarity::IdleHigh,
};



 pub fn build(pins: Spi2Pins, spi_dev: SPI2, clocks: Clocks, delay: Delay<TIM2, 1000000>) -> Storage {

  let spi2 = Spi::spi2(
    spi_dev,
    (pins.sck, pins.miso, pins.mosi),
    MODE,
    1.MHz(),
    clocks,
);

  rprintln!("set up sdcard");
  // let sdmmc_spi = embedded_hal_bus::spi::RefCellDevice::new(&spi_bus, DummyCsPin, delay).unwrap();
  // only one SPI device on this bus, can we avoid using embedded_hal_bus?

  let sd_card: RrivSdCard = embedded_sdmmc::SdCard::new(spi2, delay);
  rprintln!("set up sdcard");

  return Storage::new(sd_card);

}

type RrivSdCard = SdCard<Spi<SPI2, Spi2NoRemap, (Pin<'B', 13, Alternate>, Pin<'B', 14>, Pin<'B', 15, Alternate>), u8>, Delay<TIM2, 1000000>>;
// SdCard<Spi<SPI2, Spi2NoRemap, (Pin<'B', 13, Alternate>, Pin<'B', 14>, Pin<'B', 15, Alternate>), u8>, Pin<'C', 8, Output>>;
pub struct RrivTimeSource {}

impl RrivTimeSource {
  pub fn new () -> Self {
    RrivTimeSource {}
  }
}

impl TimeSource for RrivTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
      match Timestamp::from_calendar(2024, 10, 7, 00, 00, 00) {
          Ok(timestamp) => timestamp,
          Err(_) => Timestamp::from_calendar(0, 0, 0, 0, 0, 0).unwrap()
        }
    }
}

pub struct Storage {
  // volume_manager: VolumeManager<RrivSdCard, RrivTimeSource>
}


impl Storage {

  pub fn new(sd_card: RrivSdCard) -> Self {

    let time_source = RrivTimeSource::new();
    rprintln!("set up sdcard");
  
    let mut volume_manager  = embedded_sdmmc::VolumeManager::new(sd_card, time_source);
    // Try and access Volume 0 (i.e. the first partition).
    // The volume object holds information about the filesystem on that volume.
  
    rprintln!("set up volume");
    // let result = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0));
    // match result {
    //   Ok(volume0) =>   rprintln!("Volume 0 Success: {:?}", volume0),
    //   Err(error) => rprintln!("Volume 0 error: {:?}", error),
    // }
  
    let volume: Volume<'_, _, _, _, _, _> = volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    rprintln!("Volume 0 Success: {:?}", volume);
    // Open the root directory (mutably borrows from the volume).
 
  
  
    // let (sd_card, time_source) = volume_mgr.free();
    

    Storage {
      // volume_manager
    }
  }

  // look at the c code to copy the structure of these calls
  pub fn create_file(&mut self, timestamp: i64){

    rprintln!("set up root dir");

    let root_dir = self.volume_manager.open_root_dir(self.volume).unwrap();
    // Open a file called "MY_FILE.TXT" in the root directory
    // This mutably borrows the directory.
    rprintln!("Root Dir: {:?}", root_dir);

    // let timestamp = timestamp - 1704067200; // the RRIV epoch starts on Jan 1 2024, necessary to support short file names
    let output = format!("{}.csv", timestamp); 
    rprintln!("file: {:?}", output);

    // let my_file = self.volume_manager.open_file_in_dir(
    //   root_dir, output.as_str(), embedded_sdmmc::Mode::ReadWriteCreateOrAppend).unwrap();

    let my_file = match self.volume_manager.open_file_in_dir(
        root_dir, output.as_str(), embedded_sdmmc::Mode::ReadWriteCreateOrAppend){
            Ok(my_file) => my_file,
            Err(error) => {
              rprintln!("{:?}", error);
              panic!();
            },
        };

      
    rprintln!("File: {:?}", my_file);
    
    match self.volume_manager.write(my_file, b"hello\n") {
      Ok(ret) => rprintln!("Success: {:?}", ret),
      Err(err) => rprintln!("Err: {:?}", err),
    }
    match self.volume_manager.write(my_file, b"hello\n") {
      Ok(ret) => rprintln!("Success: {:?}", ret),
      Err(err) => rprintln!("Err: {:?}", err),
    }
    match self.volume_manager.write(my_file, b"hello\n") {
      Ok(ret) => rprintln!("Success: {:?}", ret),
      Err(err) => rprintln!("Err: {:?}", err),
    }
    match self.volume_manager.write(my_file, b"hello\n") {
      Ok(ret) => rprintln!("Success: {:?}", ret),
      Err(err) => rprintln!("Err: {:?}", err),
    }
    match self.volume_manager.write(my_file, b"hello\n") {
      Ok(ret) => rprintln!("Success: {:?}", ret),
      Err(err) => rprintln!("Err: {:?}", err),
    }
    self.volume_manager.close_file(my_file);
    
  }


  pub fn write(&mut self, data: &str){
    
    let time_source = RrivTimeSource::new();

    // let mut volume_mgr = embedded_sdmmc::VolumeManager::new(self.sd_card, time_source);
    // // Try and access Volume 0 (i.e. the first partition).
    // // The volume object holds information about the filesystem on that volume.
    // let volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    // rprintln!("Volume 0: {:?}", volume0);
    // // Open the root directory (mutably borrows from the volume).
    // let root_dir = volume_mgr.open_root_dir(volume0).unwrap();
    // // Open a file called "MY_FILE.TXT" in the root directory
    // // This mutably borrows the directory.
    // rprintln!("Volume 0: {:?}", root_dir);

    // let my_file = volume_mgr.open_file_in_dir(
    // root_dir, "MY_FILE.TXT", embedded_sdmmc::Mode::ReadOnly).unwrap();
    // // let my_file = root_dir.open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)?;
    
    // rprintln!("Volume 0: {:?}", my_file);

    // Print the contents of the file, assuming it's in ISO-8859-1 encoding
    // while !my_file.is_eof() {
    //   let mut buffer = [0u8; 32];
    //   let num_read = my_file.read(&mut buffer)?;
    //   for b in &buffer[0..num_read] {
    //       rprintln!("{}", *b as char);
    //   }
    // }
  }

}
