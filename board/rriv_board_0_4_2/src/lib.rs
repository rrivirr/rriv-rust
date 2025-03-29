#![cfg_attr(not(test), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use embedded_sdmmc::{Block, BlockDevice};

use core::{
    cell::RefCell, concat, default::Default, format_args, mem::MaybeUninit, num, ops::DerefMut, option::Option::{self, *}, result::Result::*
};
use core::{mem};
use cortex_m::{
    asm::{delay, dmb, dsb},
    interrupt::{CriticalSection, Mutex},
    peripheral::NVIC,
};
use embedded_hal::digital::OutputPin;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::gpio::{Alternate, Pin};
use stm32f1xx_hal::pac::{I2C1, I2C2, TIM2, USART2, USB};

use rtt_target::{rprintln};
use stm32f1xx_hal::rcc::{Clocks, CFGR};
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output},
    i2c::{BlockingI2c, Mode},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
    timer::delay::*,
};

use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use usbd_storage::subclass::ufi::{Ufi, UfiCommand};
use usbd_storage::subclass::Command;
use usbd_storage::transport::bbb::{BulkOnly};





use rriv_board::{
    ActuatorDriverServices, RRIVBoard, RXProcessor, SensorDriverServices,
    TelemetryDriverServices,
};

use ds323x::{DateTimeAccess, Ds323x, NaiveDateTime};
use stm32f1xx_hal::rtc::Rtc;

mod builder;
use builder::*;
mod util;
use util::*;
mod components;
use components::*;
mod pins;
use pins::{GpioCr, Pins};
mod pin_groups;
use pin_groups::*;



pub static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));


#[repr(C)]
pub struct Serial {
    tx: &'static Mutex<RefCell<Option<Tx<pac::USART2>>>>,
}



pub struct Board {
    pub delay: SysDelay,
    // // pub power_control: PowerControl,
    pub gpio: DynamicGpioPins,
    pub internal_adc: InternalAdc,
    pub external_adc: ExternalAdc,
    pub battery_level: BatteryLevel,
    pub rgb_led: RgbLed,
    pub oscillator_control: OscillatorControl,
    pub i2c1: Option<BoardI2c1>,
    pub i2c2: BoardI2c2,
    pub internal_rtc: Rtc,
    // pub storage: Storage,
    pub debug: bool,
    pub file_epoch: i64,
}

impl Board {
    pub fn start(&mut self) {
        rprintln!("starting board");
        // self.power_control.cycle_3v(&mut self.delay);

        // self.internal_adc.enable(&mut self.delay);
        // let timestamp: i64 = rriv_board::RRIVBoard::epoch_timestamp(self);
        
        // let storage: &mut Storage = unsafe { STORAGE.as_mut().unwrap() };
        // storage.create_file(timestamp);

    }
}

impl RRIVBoard for Board {
    fn run_loop_iteration(&mut self) {
        self.file_epoch = self.epoch_timestamp();
       
    }

    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>) {
        cortex_m::interrupt::free(|cs| {
            let mut global_rx_binding = RX_PROCESSOR.borrow(cs).borrow_mut();
            *global_rx_binding = Some(processor);
        });
    }

    fn critical_section<T, F>(&self, f: F) -> T
    where
        F: Fn() -> T,
    {
        cortex_m::interrupt::free(|cs| f())
    }

    fn serial_send(&self, string: &str) {
        cortex_m::interrupt::free(|cs| {
            // USART
            let bytes = string.as_bytes();
            for char in bytes.iter() {
                let t = TX.borrow(cs);
                if let Some(tx) = t.borrow_mut().deref_mut() {
                    _ = nb::block!(tx.write(char.clone()));
                }
            }

            // USB
            let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
            serial.write(string.as_bytes()).ok();
        });
    }

    fn serial_debug(&self, string: &str) {
        if self.debug {
            rriv_board::RRIVBoard::serial_send(self, string);
            rriv_board::RRIVBoard::serial_send(self, "\n");
        }
    }

    fn store_datalogger_settings(
        &mut self,
        bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        // who knows the eeprom bytes to use? - the board doeas
        eeprom::write_datalogger_settings_to_eeprom(self, bytes);
    }

    fn retrieve_datalogger_settings(
        // let timestamp: i64 = rriv_board::RRIVBoard::epoch_timestamp(self);
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        eeprom::read_datalogger_settings_from_eeprom(self, buffer);
    }

    fn retrieve_sensor_settings(
        // retrieve_all_sensor_configurations
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE
                 * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    ) {
        for slot in 0..rriv_board::EEPROM_TOTAL_SENSOR_SLOTS {
            let slice = &mut buffer[slot * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE
                ..(slot + 1) * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];
            read_sensor_configuration_from_eeprom(self, slot.try_into().unwrap(), slice)
        }
    }

    fn store_sensor_settings(
        &mut self,
        slot: u8,
        bytes: &[u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE],
    ) {
        // sensor_configuration
        write_sensor_configuration_to_eeprom(self, slot, bytes);
    }

    fn delay_ms(&mut self, ms: u16) {
        self.delay.delay_ms(ms);
    }

    fn set_epoch(&mut self, epoch: i64) {
        let i2c1 = mem::replace(&mut self.i2c1, None);
        let mut ds3231 = Ds323x::new_ds3231(i2c1.unwrap());
        let millis = epoch * 1000;
        // DateTime::from_timestamp_millis(micros);
        let datetime = NaiveDateTime::from_timestamp_millis(millis);
        rprintln!("{:?}", datetime);
        if let Some(datetime) = datetime {
            match ds3231.set_datetime(&datetime) {
                Ok(_) => {}
                Err(err) => rprintln!("Error {:?}", err),
            }
        }
        let result = ds3231.datetime();
        self.i2c1 = Some(ds3231.destroy_ds3231());
    }

    fn epoch_timestamp(&mut self) -> i64 {
        let i2c1 = mem::replace(&mut self.i2c1, None);
        let mut ds3231 = Ds323x::new_ds3231(i2c1.unwrap());
        let result = ds3231.datetime();
        self.i2c1 = Some(ds3231.destroy_ds3231());

        match result {
            Ok(date_time) => {
                // rprintln!("got DS3231 time {:?}", date_time.and_utc().timestamp());
                date_time.and_utc().timestamp()
            }
            Err(err) => {
                rprintln!("DS3231 error {:?}", err);
                return 0; // this could fail back to some other clock
            }
        }
    }

    // crystal time, systick
    fn timestamp(&mut self) -> i64 {
        return self.internal_rtc.current_time().into(); // internal RTC
    }

    fn get_sensor_driver_services(&mut self) -> &mut dyn SensorDriverServices {
        return self;
    }

    fn get_actuator_driver_services(&mut self) -> &mut dyn ActuatorDriverServices {
        return self;
    }

    fn get_telemetry_driver_services(&mut self) -> &mut dyn TelemetryDriverServices {
        return self;
    }

    fn set_debug(&mut self, debug: bool) {
        self.debug = debug;
    }

    fn write_log_file(&mut self, data: &str) {
        let storage: &mut Storage = unsafe { STORAGE.as_mut().unwrap() };
        // rprint!("skip write log file");
        // storage.write(data.as_bytes(), self.file_epoch);
    }

    fn flush_log_file(&mut self) {
        todo!("flush_log_file");
    }
}

macro_rules! control_services_impl {
    () => {
        fn serial_send(&self, string: &str) {
            rriv_board::RRIVBoard::serial_send(self, string);
        }

        fn serial_debug(&self, string: &str) {
            rriv_board::RRIVBoard::serial_debug(self, string);
        }

        fn delay_ms(&mut self, ms: u16) {
            rriv_board::RRIVBoard::delay_ms(self, ms);
        }
        fn timestamp(&mut self) -> i64 {
            rriv_board::RRIVBoard::timestamp(self)
        }
    };
}

impl SensorDriverServices for Board {
    fn query_internal_adc(&mut self, channel: u8) -> u16 {
        match self.internal_adc.read(channel) {
            Ok(value) => return value,
            Err(error) => {
                let mut errorString = "unhandled error";
                match error {
                    AdcError::NBError(_) => errorString = "ADC NBError",
                    AdcError::NotConfigured => errorString = "ADC Not Configured",
                    AdcError::ReadError => errorString = "ADC Read Error",
                }
                rriv_board::RRIVBoard::serial_send(self, &errorString);
                return 0;
            }
        }
    }

    fn query_external_adc(&mut self, channel: u8) -> u32 {
        // return self.external_adc.read(1);
        return 0; // not implemented
    }

    control_services_impl!();

    fn ic2_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), ()> {
        match self.i2c2.read(addr, buffer) {
            Ok(_) => return Ok(()),
            Err(e) => {
                rprintln!("{:?}", e);
                // rprintln!(&format!("Problem reading I2C2 {}\n", addr))
                // let error_msg  = &format!("Problem reading I2C2 {}\n", addr);
                // rprintln!(error_msg);
                rriv_board::RRIVBoard::serial_debug(
                    self,
                    &format!("Problem reading I2C2 {}\n", addr),
                );
                for i in 0..buffer.len() {
                    buffer[i] = 0b11111111; // error value
                }
                return Err(());
            }
        }
    }

    fn ic2_write(&mut self, addr: u8, message: &[u8]) -> Result<(), ()> {
        match self.i2c2.write(addr, message) {
            Ok(_) => return Ok(()),
            Err(e) => {
                rprintln!("{:?}", e);
                rriv_board::RRIVBoard::serial_debug(
                    self,
                    &format!("Problem writing I2C2 {}", addr),
                );
                return Err(());
            }
        }
    }
}

impl ActuatorDriverServices for Board {
    control_services_impl!();
}

impl TelemetryDriverServices for Board {
    control_services_impl!();
}

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut rx) = RX.borrow(cs).borrow_mut().deref_mut() {
            if rx.is_rx_not_empty() {
                if let Ok(c) = nb::block!(rx.read()) {
                    rprintln!("serial rx byte: {}", c);
                    let r = RX_PROCESSOR.borrow(cs);

                    if let Some(processor) = r.borrow_mut().deref_mut() {
                        processor.process_character(c);
                    }
                    let t = TX.borrow(cs);
                    if let Some(tx) = t.borrow_mut().deref_mut() {
                        _ = nb::block!(tx.write(c.clone())); // need to make a blocking call to TX
                    }
                }
                // use PA9 to flash RGB led
                // if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                //     if led.is_low() {
                //         led.set_high();
                //     } else {
                //         led.set_low();
                //     }
                // }
            }
        }
    })
}

#[interrupt]
fn USB_HP_CAN_TX() {
    cortex_m::interrupt::free(|cs| {
        dsb();
        usb_interrupt(cs);
        dmb();
    });
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    cortex_m::interrupt::free(|cs| {
        dsb();
        usb_interrupt(cs);
        dmb();
    });
}

fn usb_interrupt(cs: &CriticalSection) {
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
    let ufi = unsafe { UFI.as_mut().unwrap() };

    //     if let Some(usb_device) = &mut USB_DEVICE {
    //         if matches!(usb_device.state(), UsbDeviceState::Default) {
    //             unsafe {
    //                 STATE.reset();
    //             };
    //         }
    //     }

    if !usb_dev.poll(&mut [serial, ufi]) {
        return;
    }

    let _ = ufi.poll(|command| {
        components::block_device::process_ufi_command(command);
    });

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter() {
                let r = RX_PROCESSOR.borrow(cs);
                if let Some(processor) = r.borrow_mut().deref_mut() {
                    processor.process_character(c.clone());
                }
                // use PA9 to flash RGB led
                // if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                //     if led.is_low() {
                //         led.set_high();
                //     } else {
                //         led.set_low();
                //     }
                // }
            }
            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}

pub fn build() -> Board {
    let mut board_builder = BoardBuilder::new();
    board_builder.setup();
    let board = board_builder.build();
    board
}

