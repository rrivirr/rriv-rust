#![cfg_attr(not(test), no_std)]

extern crate alloc;
extern crate panic_halt;

use alloc::boxed::Box;
use cortex_m::interrupt::enable;
use core::borrow::BorrowMut;
use core::default;
use core::fmt::Write;
use core::marker::{Send, Sync};
use core::{
    cell::RefCell,
    concat,
    default::Default,
    format_args,
    ops::DerefMut,
    option::{Option, Option::*},
    result::Result::*,
};
use cortex_m::peripheral::DWT;
use cortex_m::{
    asm::{delay, dmb, dsb},
    interrupt::{CriticalSection, Mutex},
    peripheral::NVIC,
};
use embedded_hal::blocking::i2c;
use stm32f1xx_hal::gpio::{Alternate, Pin};
use stm32f1xx_hal::i2c::I2c;
use stm32f1xx_hal::pac::{I2C1, I2C2, RCC};

use rtt_target::rprintln;
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output, PinState},
    i2c::{BlockingI2c, DutyCycle, Mode, Instance},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
    timer::delay::*,
};
use stm32f1xx_hal::rcc::{BusClock, Clocks, Enable, Reset};


use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use rriv_board::{RRIVBoard, RXProcessor};

mod eeprom;

type RedLed = gpio::Pin<'A', 9, Output<OpenDrain>>;

static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

static RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
static TX: Mutex<RefCell<Option<Tx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));

#[repr(C)]
pub struct Serial {
    tx: &'static Mutex<RefCell<Option<Tx<pac::USART2>>>>,
}

// type aliases to make things tenable
type BoardI2c = BlockingI2c<
    I2C1,
    (
        Pin<'B', 6, Alternate<OpenDrain>>,
        Pin<'B', 7, Alternate<OpenDrain>>,
    ),
>;
// type sensor  = BlockingI2c<I2C1, (Pin<'B', 6, Alternate<OpenDrain>>, Pin<'B', 7, Alternate<OpenDrain>>)>;

pub struct Board {
    pub i2c1: Option<BoardI2c>,
    pub delay: Option<SysDelay>,
}

impl Board {
    pub fn new() -> Self {
        Board { i2c1: None, delay: None}
    }
}


impl RRIVBoard for Board {
    fn setup(&mut self) {
        rprintln!("board new");

        let mut core_peripherals = cortex_m::Peripherals::take().unwrap();

        // Get access to the device specific peripherals from the peripheral access crate
        let device_peripherals = pac::Peripherals::take().unwrap();
        // rprintln!("{:?}",device_peripherals.RCC.apb2enr); //.i2c1en();  //enabled();
        // device_peripherals.RCC.apb2enr.

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = device_peripherals.FLASH.constrain();
        let rcc = device_peripherals.RCC.constrain();
        let mut afio = device_peripherals.AFIO.constrain();     // Prepare the alternate function I/O registers


        // Prepare the GPIO
        let mut gpioa = device_peripherals.GPIOA.split();
        let mut gpiob = device_peripherals.GPIOB.split();
        let mut gpioc = device_peripherals.GPIOC.split();

        // Set up pins
        let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl); // USART2
        let rx_pin = gpioa.pa3; // USART2
        let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl); // i2c
        let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl); // i2c
        let mut switched_pow = gpioc.pc6.into_push_pull_output(&mut gpioc.crl);
        let mut hse_pin = gpioc.pc13.into_push_pull_output(&mut gpioc.crh); // high speed external on/off
        

        // Turn on the HSE
        hse_pin.set_high();

        // Freeze the configuration of all the clocks in the system
        // and store the frozen frequencies in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz()) // was 24, set to 6 following i2c code
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

  

        // Set up and initialize switched power bus
        self.delay = Some(core_peripherals.SYST.delay(&clocks));
        switched_pow.set_high();
        self.delay_ms(250_u16);
        switched_pow.set_low();
        self.delay_ms(250_u16);
        switched_pow.set_high();
        self.delay_ms(250_u16);
        switched_pow.set_low();
        self.delay_ms(250_u16);
    

   



        // Init Serial
        // rprintln!("initializing serial");
        let mut serial = Hal_Serial::new(
            device_peripherals.USART2,
            (tx_pin, rx_pin),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            &clocks,
        );

        rprintln!("serial rx.listen()");

        serial.rx.listen();

        let led = gpioa
            .pa9
            .into_open_drain_output_with_state(&mut gpioa.crh, PinState::Low);
        cortex_m::interrupt::free(|cs| {
            RX.borrow(cs).replace(Some(serial.rx));
            TX.borrow(cs).replace(Some(serial.tx));
            WAKE_LED.borrow(cs).replace(Some(led));
        });
        // rprintln!("unmasking USART2 interrupt");
        unsafe {
            NVIC::unmask(pac::Interrupt::USART2);
        }

        // USB Serial
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: device_peripherals.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        // Unsafe to allow access to static variables
        unsafe {
            let bus = UsbBus::new(usb);

            USB_BUS = Some(bus);

            USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

            let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x0483, 0x29))
                .manufacturer("RRIV")
                .product("RRIV Data Logger")
                .serial_number("_rriv")
                .device_class(USB_CLASS_CDC)
                .build();

            USB_DEVICE = Some(usb_dev);
        }

        unsafe {
            NVIC::unmask(pac::Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(pac::Interrupt::USB_LP_CAN_RX0);
        }



        rprintln!("starting i2c");
        core_peripherals.DWT.enable_cycle_counter(); // BlockingI2c says this is required

        // let rcc = unsafe { &(*RCC::ptr()) };

        self.i2c1 = Some(BlockingI2c::i2c1(
            device_peripherals.I2C1,
            (scl, sda),
            &mut afio.mapr,
            Mode::Standard {
                frequency: 100.kHz(), // slower to same some energy?
            },
            // Mode::Fast {
            //     frequency: 400.kHz(),
            //     duty_cycle: DutyCycle::Ratio16to9,
            // },
            clocks,
            1000,
            10,
            1000,
            1000,
        ));
        rprintln!("started i2c");

        // I2C1::disable(rcc);
        // delay.delay_ms(50_u16);
        // I2C1::enable(rcc);
        // delay.delay_ms(50_u16);
        // I2C1::reset(rcc);
        // delay.delay_ms(50_u16);


        rprintln!("Start i2c scanning...");
        rprintln!();

        for addr in 0x00_u8..0x7F {
            // Write the empty array and check the slave response.
            // rprintln!("trying {:02x}", addr);
            let mut buf = [b'\0'; 1];
            if let Some(i2c) = &mut self.i2c1 {
                if i2c.read(addr, &mut buf).is_ok() {
                    rprintln!("{:02x} good", addr);
                }
            }
            self.delay_ms(10_u16);
        }
        rprintln!("scan is done")
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

    fn store_datalogger_settings(
        &mut self,
        bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        // who knows the eeprom bytes to use? - the board doeas
        eeprom::write_datalogger_settings_to_eeprom(self, bytes);
    }

    fn retrieve_datalogger_settings(
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        eeprom::read_datalogger_settings_from_eeprom(self, buffer);
    }

    fn delay_ms(&mut self, ms: u16) {
        if let Some(delay) = &mut self.delay {
            delay.delay_ms(ms);
        }
    }
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
                if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                    if led.is_low() {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }
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

    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter() {
                let r = RX_PROCESSOR.borrow(cs);
                if let Some(processor) = r.borrow_mut().deref_mut() {
                    processor.process_character(c.clone());
                }
            }
            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}
