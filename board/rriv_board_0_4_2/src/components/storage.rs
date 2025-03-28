use core::{ffi::CStr, time};

use alloc::format;

use ds323x::{Datelike, Timelike};
use embedded_hal::{
    delay::DelayNs,
    spi::{self, ErrorType, SpiBus, SpiDevice},
};

use stm32f1xx_hal::spi::{Mode, Phase, Polarity, Spi, SpiReadWrite};

use embedded_sdmmc::{
    BlockDevice, Directory, File, RawDirectory, RawFile, SdCard, SdCardError, TimeSource,
    Timestamp, Volume, VolumeManager,
};
use pac::SPI2;

use crate::*;

pub static mut STORAGE: Option<Storage> = None;


pub struct RrivTimeSource {}

impl RrivTimeSource {
    pub fn new() -> Self {
        RrivTimeSource {}
    }
}

// the Timesource trait below is a fraught way for us pass timestamps into the sd card library
// this cloud be refactored in the library to allow the caller to simply pass the timestamp
// into each write call, from whatever source it wants to use for a timestmap.
// for the time being, this static variable provides a way to transmit the timestamp into the library
// Additinally, in the case of the RRIVBoard, we do not want to query the I2C RTC every time we write
// therefore, this time should be coming form the internal RTC, which needs to be set to match that I2C RTC

static mut EPOCH_TIMESTAMP: i64 = 0;

pub fn update_time_source(timestamp: i64) {
    unsafe {
        EPOCH_TIMESTAMP = timestamp;
    }
}

impl TimeSource for RrivTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        let mut naive = NaiveDateTime::default();
        unsafe {
            naive = NaiveDateTime::from_timestamp(EPOCH_TIMESTAMP, 0);
        }
        let time = naive.time();
        let timestamp = embedded_sdmmc::Timestamp::from_calendar(
            naive.year().try_into().unwrap(),
            naive.month().try_into().unwrap(),
            naive.day().try_into().unwrap(),
            time.hour().try_into().unwrap(),
            time.minute().try_into().unwrap(),
            time.second().try_into().unwrap(),
        );
        match timestamp {
            Ok(timestamp) => timestamp,
            Err(err) => Timestamp::from_calendar(0, 0, 0, 0, 0, 0).unwrap(),
        }
    }
}

pub const MODE: Mode = Mode {
    polarity: Polarity::IdleHigh,
    phase: Phase::CaptureOnSecondTransition,
};

pub struct MainSpi {
    spi: Spi<SPI2, u8>,
    cs: Pin<'C', 8, Output>,
}
//     type Error = PinModeError;

impl ErrorType for MainSpi {
    type Error = stm32f1xx_hal::spi::Error;
}

impl MainSpi {
    fn read_spi(&mut self, buf: &mut [u8]) -> Result<(), stm32f1xx_hal::spi::Error> {
        for i in 0..buf.len() {
            loop {
                match self.spi.read_nonblocking() {
                    Ok(word) => {
                        buf[i] = word;
                        break;
                    }
                    Err(error) => match error {
                        nb::Error::Other(other_error) => {
                            rprintln!("{:?}", other_error);
                            return Err(other_error);
                        }
                        nb::Error::WouldBlock => {
                            rprintln!("{:?}", error);
                            continue;
                        }
                    },
                }
            }
        }

        Ok(())
    }

    fn write_spi(&mut self, buf: &[u8], ignore: bool) -> Result<(), stm32f1xx_hal::spi::Error> {
        for i in 0..buf.len() {
            loop {
                match self.spi.write_nonblocking(buf[i]) {
                    Ok(_) => {
                        while !self.spi.is_tx_empty() {}
                        if ignore {
                            while self.spi.is_rx_not_empty() {
                                let _ = self.spi.read_data_reg(); // ignore what comes back and clear the rx register to prevent an overrun
                            }
                        }
                        break;
                    }

                    Err(error) => match error {
                        nb::Error::Other(other_error) => {
                            rprintln!("{:?}", other_error);
                            return Err(other_error);
                        }
                        nb::Error::WouldBlock => {
                            rprintln!("{:?}", error);
                            continue;
                        }
                    },
                }
            }
        }

        Ok(())
    }
}

impl SpiDevice for MainSpi {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        // assert chip select pin
        // do the operations
        // assert chip select pin

        // self.cs.set_low().map_err(SpiDeviceError::Cs)?;

        // self.cs.set_low();

        let op_res = operations.iter_mut().try_for_each(|op| match op {
            embedded_hal::spi::Operation::Read(buf) => {
                // while self.spi.is_busy() {}
                // rprintln!("{:?}", "spi read");

                return match self.read_spi(buf) {
                    Ok(_) => Ok(()),
                    Err(error) => Err(error),
                };
            }
            embedded_hal::spi::Operation::Write(buf) => {
                // while self.spi.is_busy() {}
                // while self.spi.is_rx_not_empty() {
                //     self.spi.read_data_reg();
                // }
                // rprintln!("{:?}", "spi write");

                return match self.write_spi(buf, true) {
                    Ok(_) => Ok(()),
                    Err(error) => {
                        rprintln!("{:?}", error);
                        Err(error)
                    }
                };
            }
            embedded_hal::spi::Operation::Transfer(read, write) => {

                // rprintln!("{:?}", "spi transfer");

                // while self.spi.is_busy() {}
                match self.write_spi(write, false) {
                    Ok(_) => {}
                    Err(error) => return Err(error),
                }

                // while self.spi.is_busy() {}
                let returnval = match self.read_spi(read) {
                    Ok(_) => Ok(()),
                    Err(error) => Err(error),
                };

                return returnval;
            }
            embedded_hal::spi::Operation::TransferInPlace(buf) => {
                // rprintln!("{:?}", "spi transfer in place");

                for i in 0..buf.len() {
                    let mut place : [u8; 1] = [buf[i]; 1];
                    match self.write_spi(&place, false) {
                        Ok(_) => {}
                        Err(error) => return Err(error),
                    }

                    match self.read_spi(&mut place) {
                        Ok(_) => {
                            buf[i] = place[0];
                        },
                        Err(error) => return Err(error),
                    };
                }
                Ok(())
            }

            #[cfg(not(feature = "time"))]
            embedded_hal::spi::Operation::DelayNs(_) => unreachable!(),

            #[cfg(feature = "time")]
            Operation::DelayNs(ns) => {
                embassy_time::block_for(embassy_time::Duration::from_nanos(*ns as _));
                Ok(())
            }
        });


        // rprintln!("{:?}", "spi op complete");

        // self.cs.set_high();

        // On failure, it's important to still flush and deassert CS.
        //  let flush_res = bus.flush();
        //  let cs_res = self.cs.set_high();

        // let op_res = op_res.map_err(SpiDeviceError::Spi)?;
        // flush_res.map_err(SpiDeviceError::Spi)?;
        // cs_res.map_err(SpiDeviceError::Cs)?;

        op_res
    }

    fn read(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::Read(buf)])
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::Write(buf)])
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::Transfer(read, write)])
    }

    fn transfer_in_place(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.transaction(&mut [embedded_hal::spi::Operation::TransferInPlace(buf)])
    }
}

pub struct StorageDelay {
    delay: DelayUs<TIM2>,
}

impl DelayNs for StorageDelay {
    fn delay_ns(&mut self, ns: u32) {
        embedded_hal::delay::DelayNs::delay_us(&mut self.delay, 10);
    }
}

pub fn build(
    pins: Spi2Pins,
    spi_peripheral: SPI2,
    clocks: Clocks,
    mut delay: DelayUs<TIM2>,
) -> components::storage::Storage {
    let spi2 = spi_peripheral.spi(
        (Some(pins.sck), Some(pins.miso), Some(pins.mosi)),
        MODE,
        1.MHz(),
        &clocks,
    );

    // Spi::new(SPI2, pins, mode, freq, &clocks);

    let mut cs = pins.sd_card_chip_select;
    cs.set_low();

    let mut myspi = MainSpi { spi: spi2, cs: cs };

    // spi2.

    // let spi2 = Spi::new_blocking(SPI2, pins.sck, pins.mosi, pins.miso, Config::default());
    // let spi = SpiDevice::new(&spi2, cs);

    // ExclusiveDevice::new(spi2, DummyCsPin, delay);

    rprintln!("set up sdcard");
    // let sdmmc_spi = embedded_hal_bus::spi::RefCellDevice::new(&spi_bus, DummyCsPin, delay).unwrap();
    // only one SPI device on this bus, can we avoid using embedded_hal_bus?

    embedded_hal::delay::DelayNs::delay_us(&mut delay, 10);
    let storage_delay = StorageDelay { delay };

    while myspi.spi.is_rx_not_empty() {
        myspi.spi.read_data_reg();
    }

    let sdcard = embedded_sdmmc::SdCard::new(myspi, storage_delay);

    rprintln!("set up sdcard");

    let time_source = RrivTimeSource::new(); // unsafe access to the board
                                             // or global time var via interrupt
                                             // or copy into a global variable at the top of the run loop
    rprintln!("set up sdcard");

    let volume_manager = embedded_sdmmc::VolumeManager::new(sdcard, time_source);

    Storage {
        volume_manager: volume_manager,
        filename: [b'\0'; 11],
        file: None,
        // dir: None,
        root_dir: None,
        cache: [b'\0'; CACHE_SIZE],
        next_position: 0,
    }

    // return Storage::new(sdcard);
}

pub trait OutputDevice {
    fn write(data: &[u8]);
}

const CACHE_SIZE: usize = 50;

pub struct Storage {
    pub volume_manager: VolumeManager<SdCard<MainSpi, StorageDelay>, RrivTimeSource>,
    // volume: Option<Volume<'a, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1>>,
    filename: [u8; 11],
    // file: Option<File<'a, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1>>,
    file: Option<RawFile>,
    // dir: Option<RawDirectory>,
    // root_dir: Option<Directory<'a, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1>>,
    root_dir: Option<RawDirectory>,
    cache: [u8; CACHE_SIZE],
    next_position: usize,
}

impl Storage {
    pub fn new(
        // , time_source: impl TimeSource //  a timesource passed in here could use unsafe access to internal RTC or i2c bus
        sd_card: SdCard<MainSpi, StorageDelay>,
    ) -> Self {
        let time_source = RrivTimeSource::new(); // unsafe access to the board
                                                 //  or global time var via interrupt
                                                 // or copy into a global variable at the top of the run loop
        rprintln!("set up sdcard");

        // let mut volume_manager = embedded_sdmmc::VolumeManager::new(sd_card, time_source);

        // rprintln!("set up volume");

        // let mut volume: Volume<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
        //     match volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0)) {
        //         Ok(volume0) => {
        //             rprintln!("Volume 0 Success: {:?}", volume0);
        //             volume0
        //         }
        //         Err(error) => panic!("Volume 0 error: {:?}", error),
        //     };

        // let mut root_dir: Directory<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
        //     match volume.open_root_dir() {
        //         Ok(root_dir) => root_dir,
        //         Err(error) => panic!("root dir error: {:?}", error),
        //     };

        // let filename = match core::str::from_utf8("hellO".as_bytes()) {
        //     Ok(filename) => filename,
        //     Err(err) => {
        //         rprintln!("{:?}", err);
        //         panic!();
        //     }
        // };

        // rprintln!("Filename: {:?}", filename);

        // let file = root_dir.open_file_in_dir(filename, embedded_sdmmc::Mode::ReadWriteCreateOrAppend);
        // let the_file = match file {
        //     Ok(file) => file,
        //     Err(_) => todo!(),
        // };

        // // let volume = volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
        // // rprintln!("Volume 0 Success: {:?}", volume);

        // // Open the root directory (mutably borrows from the volume).
        // rprintln!("set up root dir");
        // //  Result<Directory<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1>,embedded_sdmmc::Error<SdCardError>>

        // let root_dir = root_dir.to_raw_directory();
        // root_dir.to_raw_directory()

        let mut volume_manager = embedded_sdmmc::VolumeManager::new(sd_card, time_source);


        // sd_card.num_bytes() //the size
        // volume_manager.device().read(blocks, start_block_idx, reason)

        // sd_card.num_blocks();
        // sd_card.read(blocks, start_block_idx, reason);
        // sd_card.write(blocks, start_block_idx);

        // rprintln!("set up volume");

        // let result = volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0));
        // let mut volume: Volume<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
        //     match result {
        //         Ok(volume0) => {
        //             rprintln!("Volume 0 Success: {:?}", volume0);
        //             volume0
        //         }
        //         Err(error) => panic!("Volume 0 error: {:?}", error),
        //     };

        Storage {
            volume_manager: volume_manager,
            // volume: None,
            filename: [b'\0'; 11],
            file: None,
            // dir: None,
            root_dir: None,
            cache: [b'\0'; CACHE_SIZE],
            next_position: 0,
        }
    }

    // pub fn open_file(&mut self){
    //     // The volume object holds information about the filesystem on that volume.

    //     // let adir = root_dir.to_raw_directory();
    //     // let bdir = adir.to_directory(&mut self.volume_manager);

    // }

    pub fn setup<'a>(&'a mut self) {
        // let time_source = RrivTimeSource::new(); // unsafe access to the board
        // or global time var via interrupt
        // or copy into a global variable at the top of the run loop
        rprintln!("set up sdcard");

        let v = &mut self.volume_manager;

        // self.volume_manager = Some(volume_manager);

        rprintln!("set up volume");

        let result = v.open_volume(embedded_sdmmc::VolumeIdx(0));
        let mut volume: Volume<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
            match result {
                Ok(volume0) => {
                    rprintln!("Volume 0 Success: {:?}", volume0);
                    volume0
                }
                Err(error) => {
                    rprintln!("Volume 0 error: {:?}", error);
                    panic!("Volume 0 error: {:?}", error);
                }
            };

        // self.volume = Some(volume);

        let mut root_dir: Directory<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
            match volume.open_root_dir() {
                Ok(root_dir) => root_dir,
                Err(error) => panic!("root dir error: {:?}", error),
            };

        self.root_dir = Some(root_dir.to_raw_directory());
    }

    pub fn reopen_file(&mut self) {
        // let file = match dir.open_file_in_dir("a", embedded_sdmmc::Mode::ReadWriteCreateOrAppend) {
        //     Ok(file) => file,
        //     Err(error) => panic!("Open file error: {:?}", error),
        // };

        // self.file = Some(file.to_raw_file());
        // self.dir = Some(dir.to_raw_directory());

        let filename = match core::str::from_utf8(&self.filename) {
            Ok(filename) => filename,
            Err(err) => {
                rprintln!("{:?}", err);
                panic!();
            }
        };

        rprintln!("Filename: {:?}", filename);

        let root_dir = self.root_dir.unwrap();
        let file = self.volume_manager.open_file_in_dir(
            root_dir,
            filename,
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        );

        let my_file = match file {
            Ok(file) => file,
            Err(error) => {
                rprintln!("{:?}", error);
                panic!();
            }
        };
        self.file = Some(my_file);
        self.root_dir = Some(root_dir);

        // let mut dir = match root_dir.open_dir("data") {
        //     Ok(dir) => dir,
        //     Err(error) => panic!("Open dir error: {:?}", error),
        // };

        // let my_file = match self.volume_manager.open_file_in_dir(
        //     self.root_dir,
        //     filename,
        //     embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        // ) {
        //     Ok(my_file) => my_file,
        //     Err(error) => {
        //         rprintln!("{:?}", error);
        //         panic!();
        //     }
        // };

        // rprintln!("File: {:?}", my_file);

        // self.file = Some(my_file);
    }

    // look at the c code to copy the structure of these calls
    pub fn create_file(&mut self, timestamp: i64) {
        // let mut volume: Volume<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
        //     match self
        //         .volume_manager
        //         .open_volume(embedded_sdmmc::VolumeIdx(0))
        //     {
        //         Ok(volume0) => {
        //             rprintln!("Volume 0 Success: {:?}", volume0);
        //             volume0
        //         }
        //         Err(error) => panic!("Volume 0 error: {:?}", error),
        //     };
        // rprintln!("Volume 0 Success: {:?}", volume);

        // let root_dir: Directory<'_, SdCard<MainSpi, StorageDelay>, RrivTimeSource, 4, 4, 1> =
        //     match volume.open_root_dir() {
        //         Ok(root_dir) => root_dir,
        //         Err(error) => panic!("root dir error: {:?}", error),
        //     };

        // self.root_dir = Some(root_dir);

        let timestamp = if timestamp > 9999999 {
            timestamp & 9999999
        } else {
            timestamp
        };
        // let timestamp = timestamp > 1704067200 ? timestamp - 1704067200 : timestamp; // the RRIV epoch starts on Jan 1 2024, necessary to support short file names
        let filename = format!("{:0>7}.csv", timestamp);
        let filename = filename.as_bytes();
        rprintln!("file: {:?}", core::str::from_utf8(filename));
        let mut filename_bytes: [u8; 11] = [b'\0'; 11];
        filename_bytes[0..filename.len()].clone_from_slice(filename);
        self.filename = filename_bytes;

        // let my_file = self.volume_manager.open_file_in_dir(
        //   root_dir, output.as_str(), embedded_sdmmc::Mode::ReadWriteCreateOrAppend).unwrap();

        self.reopen_file();
    }

    pub fn flush(&mut self) {
        if let Some(file) = self.file {
            let cache_data = &self.cache[0..self.next_position];
            match self.volume_manager.write(file, cache_data) {
                Ok(ret) => rprintln!("Success: {:?}", ret),
                Err(err) => rprintln!("Err: {:?}", err),
            }

            let _ = self.volume_manager.close_file(file);
            self.reopen_file();
            self.next_position = 0;
        }
    }

    pub fn write(&mut self, data: &[u8], timestamp: i64) {
        //-> Result<Ok, Error<Error>>{

        unsafe {
            EPOCH_TIMESTAMP = timestamp; // or function set_write_timestamp
        }

        if let Some(file) = self.file {
            if self.next_position + data.len() > CACHE_SIZE {
                // flush cache
                self.flush();
            }

            self.cache[self.next_position..self.next_position + data.len()].copy_from_slice(data);
            self.next_position = self.next_position + data.len();
        } else {
            // return an error
            rprintln!("File not open");
        }
    }

   

    // pub fn write(&mut self, data: &[u8]){

    //   let root_dir = self.volume_manager.open_root_dir(self.volume).unwrap();

    //   let filename_str: &str = match core::str::from_utf8(&self.filename) {
    //       Ok(str) => str,
    //       Err(err) => {
    //         rprintln!("{:?}", err);
    //         panic!();
    //       }
    //   };

    //   let my_file = match self.volume_manager.open_file_in_dir(
    //     root_dir, filename_str, embedded_sdmmc::Mode::ReadWriteCreateOrAppend){
    //         Ok(my_file) => my_file,
    //         Err(error) => {
    //           rprintln!("{:?}", error);
    //           panic!();
    //         },
    //     };

    //   match self.volume_manager.write(my_file, data.as_bytes()) {
    //     Ok(ret) => rprintln!("Success: {:?}", ret),
    //     Err(err) => rprintln!("Err: {:?}", err),
    //   }

    //   self.volume_manager.close_file(my_file); // this is how you flush the file.

    //   // let mut volume_mgr = embedded_sdmmc::VolumeManager::new(self.sd_card, time_source);
    //   // // Try and access Volume 0 (i.e. the first partition).
    //   // // The volume object holds information about the filesystem on that volume.
    //   // let volume0 = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)).unwrap();
    //   // rprintln!("Volume 0: {:?}", volume0);
    //   // // Open the root directory (mutably borrows from the volume).
    //   // let root_dir = volume_mgr.open_root_dir(volume0).unwrap();
    //   // // Open a file called "MY_FILE.TXT" in the root directory
    //   // // This mutably borrows the directory.
    //   // rprintln!("Volume 0: {:?}", root_dir);root_dir

    //   // let my_file = volume_mgr.open_file_in_dir(
    //   // root_dir, "MY_FILE.TXT", embedded_sdmmc::Mode::ReadOnly).unwrap();
    //   // // let my_file = root_dir.open_file_in_dir("MY_FILE.TXT", embedded_sdmmc::Mode::ReadWriteCreateOrAppend)?;

    //   // rprintln!("Volume 0: {:?}", my_file);

    //   // Print the contents of the file, assuming it's in ISO-8859-1 encoding
    //   // while !my_file.is_eof() {
    //   //   let mut buffer = [0u8; 32];
    //   //   let num_read = my_file.read(&mut buffer)?;
    //   //   for b in &buffer[0..num_read] {
    //   //       rprintln!("{}", *b as char);
    //   //   }
    //   // }
    // }
}
