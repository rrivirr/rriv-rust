#![cfg_attr(not(test), no_std)]

extern crate alloc;
extern crate panic_halt;

use alloc::boxed::Box;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::flash::ACR;
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
use cortex_m::interrupt::{enable, free};
use cortex_m::peripheral::DWT;
use cortex_m::Peripherals;
use cortex_m::{
    asm::{delay, dmb, dsb},
    interrupt::{CriticalSection, Mutex},
    peripheral::NVIC,
};
use embedded_hal::blocking::i2c;
use stm32f1xx_hal::gpio::{gpioa, Alternate, Pin};
use stm32f1xx_hal::i2c::I2c;
use stm32f1xx_hal::pac::can1::tx;
use stm32f1xx_hal::pac::{I2C1, I2C2, RCC, USART2, USB};

use rtt_target::rprintln;
use stm32f1xx_hal::rcc::{BusClock, Clocks, Enable, Reset, CFGR};
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output, PinState},
    i2c::{BlockingI2c, DutyCycle, Instance, Mode},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
    timer::delay::*,
};

use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use rriv_board::{RRIVBoard, RRIVBoardBuilder, RXProcessor};

mod eeprom;
mod pins;
use pins::{Pins, GpioCr};

mod pin_groups;
use pin_groups::*;

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

// // Set up pins
// let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl); // USART2
// let rx_pin = gpioa.pa3; // USART2
// let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl); // i2c
// let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl); // i2c
// let mut switched_pow = gpioc.pc6.into_push_pull_output(&mut gpioc.crl);
// let mut hse_pin = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

// type aliases to make things tenable
type BoardI2c1 = BlockingI2c<I2C1, (pin_groups::I2c1Scl, pin_groups::I2c1Sda)>;

type BoardI2c2 = BlockingI2c<I2C2, (pin_groups::I2c2Scl, pin_groups::I2c2Sda)>;

pub struct Board {
    pub delay: SysDelay,
    pub power_control: Power,
    pub gpio: DynamicGpio, //DynamicGpio ??
    pub internal_adc: InternalAdc,
    pub rgb_led: RgbLed,
    pub oscillator_control: OscillatorControl,
    pub i2c1: BoardI2c1,
    pub i2c2: BoardI2c2,
}

impl Board {
    pub fn start(&mut self) {
        let pc = &mut self.power_control;
        let s = &mut pc.enable_3v;
        s.set_high();
        let delay_ref = &mut self.delay;
        delay_ref.delay_ms(250_u16);
        s.set_low();
        delay_ref.delay_ms(250_u16);
        s.set_high();
        delay_ref.delay_ms(250_u16);
        s.set_low();
        delay_ref.delay_ms(250_u16);
    }
}


pub struct BoardBuilder {

    // chip features
    pub delay: Option<SysDelay>,

    // pins groups
    pub power_control: Option<Power>,
    pub gpio: Option<DynamicGpio>,
    pub internal_adc: Option<InternalAdc>,
    pub rgb_led: Option<RgbLed>,
    pub oscillator_control: Option<OscillatorControl>,

    // board features
    pub i2c1: Option<BoardI2c1>,
    pub i2c2: Option<BoardI2c2>,
}

impl BoardBuilder {
    pub fn new() -> Self {
        BoardBuilder {
            i2c1: None,
            i2c2: None,
            delay: None,
            power_control: None,
            gpio: None,
            internal_adc: None,
            rgb_led: None,
            oscillator_control: None,
        }
    }

    pub fn build(self) -> Board {
        Board {
            i2c1: self.i2c1.unwrap(),
            i2c2: self.i2c2.unwrap(),
            delay: self.delay.unwrap(),
            power_control: self.power_control.unwrap(),
            gpio: self.gpio.unwrap(),
            internal_adc: self.internal_adc.unwrap(),
            rgb_led: self.rgb_led.unwrap(),
            oscillator_control: self.oscillator_control.unwrap(),
        }
    }

    fn setup_clocks( oscillator_control: &mut OscillatorControl, cfgr: CFGR, flash_acr: &mut ACR ) -> Clocks {
        oscillator_control.enable_hse.set_high();

        // Freeze the configuration of all the clocks in the system
        // and store the frozen frequencies in `clocks`
        let clocks = cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz()) // was 24, set to 6 following i2c code
            .freeze(flash_acr);

        assert!(clocks.usbclk_valid());

        clocks
    }

    fn setup_serial(pins: pin_groups::Serial, cr: &mut GpioCr, mapr: &mut MAPR, usart: USART2, clocks: &Clocks) {
        // rprintln!("initializing serial");

        let mut serial = Hal_Serial::new(
            usart,
            (pins.tx, pins.rx),
            mapr,
            Config::default().baudrate(115200.bps()),
            &clocks,
        );

        rprintln!("serial rx.listen()");

        serial.rx.listen();

        cortex_m::interrupt::free(|cs| {
            RX.borrow(cs).replace(Some(serial.rx));
            TX.borrow(cs).replace(Some(serial.tx));
            // WAKE_LED.borrow(cs).replace(Some(led));
        });
        // rprintln!("unmasking USART2 interrupt");
        unsafe {
            NVIC::unmask(pac::Interrupt::USART2);
        }

    }

    fn setup_usb(pins: pin_groups::Usb, cr: &mut GpioCr, usb: USB, clocks: &Clocks) {
        // USB Serial
        let mut usb_dp = pins.usb_dp; // take ownership
        usb_dp.make_push_pull_output(&mut cr.gpioa_crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        let usb_dm = pins.usb_dm;
        let usb_dp = usb_dp.into_floating_input(&mut cr.gpioa_crh);

        let usb = Peripheral {
            usb: usb,
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
    }

    pub fn setup_i2c1(pins: pin_groups::I2c1, cr: &mut GpioCr, i2c1: I2C1, mapr: &mut MAPR,  clocks: &Clocks) -> BoardI2c1 {
        let scl1 = pins.i2c1_scl.into_alternate_open_drain(&mut cr.gpiob_crl); // i2c
        let sda1 = pins.i2c1_sda.into_alternate_open_drain(&mut cr.gpiob_crl); // i2c
        BlockingI2c::i2c1(
            i2c1,
            (scl1, sda1),
            mapr,
            Mode::Standard {
                frequency: 100.kHz(), // slower to same some energy?
            },
            *clocks,
            1000,
            10,
            1000,
            1000,
        ) 
    }


    pub fn setup_i2c2(pins: pin_groups::I2c2, cr: &mut GpioCr, i2c2: I2C2, clocks: &Clocks) -> BoardI2c2 {

        let scl2 = pins
            .i2c2_scl
            .into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
        let sda2 = pins
            .i2c2_sda
            .into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
        BlockingI2c::i2c2(
            i2c2,
            (scl2, sda2),
            Mode::Standard {
                frequency: 100.kHz(), // slower to same some energy?
            },
            *clocks,
            1000,
            10,
            1000,
            1000,
        )
    }
     
    fn setup(mut self) -> Self {
        rprintln!("board new");

        let mut core_peripherals = cortex_m::Peripherals::take().unwrap();
        let device_peripherals = pac::Peripherals::take().unwrap();

        // mcu device registers
        let rcc = device_peripherals.RCC.constrain();
        let mut flash = device_peripherals.FLASH.constrain();
        let mut afio = device_peripherals.AFIO.constrain(); // Prepare the alternate function I/O registers



        // Prepare the GPIO
        let gpioa = device_peripherals.GPIOA.split();
        let gpiob = device_peripherals.GPIOB.split();
        let gpioc = device_peripherals.GPIOC.split();
        let gpiod = device_peripherals.GPIOD.split();

        // Set up pins
        let (mut pins, mut gpio_cr) = Pins::build(gpioa, gpiob, gpioc, gpiod, &mut afio.mapr);
        let (
            external_adc_pins,
            internal_adc_pins,
            battery_level_pins,
            dynamic_gpio_pins,
            i2c1_pins,
            i2c2_pins,
            oscillator_control_pins,
            power_pins,
            rgb_led_pinsS,
            serial_pins,
            usb_pins
        ) = pin_groups::build(pins, &mut gpio_cr);
  

        let clocks = BoardBuilder::setup_clocks(&mut self.oscillator_control.as_mut().unwrap(), rcc.cfgr, &mut flash.acr);
        self.delay = Some(core_peripherals.SYST.delay(&clocks));
        BoardBuilder::setup_serial(serial_pins, &mut gpio_cr, &mut afio.mapr, device_peripherals.USART2, &clocks);
        BoardBuilder::setup_usb(usb_pins, &mut gpio_cr,  device_peripherals.USB, &clocks);

        // rprintln!("starting i2c");
        core_peripherals.DWT.enable_cycle_counter(); // BlockingI2c says this is required
        let i2c1 = BoardBuilder::setup_i2c1(i2c1_pins, &mut gpio_cr, device_peripherals.I2C1, &mut afio.mapr, &clocks);
        self.i2c1 = Some(i2c1);
        rprintln!("set up i2c1");

        let i2c2 = BoardBuilder::setup_i2c2(i2c2_pins, &mut gpio_cr, device_peripherals.I2C2, &clocks);
        self.i2c2 = Some(i2c2);
        rprintln!("set up i2c2");



        // // I2C1::disable(rcc);
        // // delay.delay_ms(50_u16);
        // // I2C1::enable(rcc);
        // // delay.delay_ms(50_u16);
        // // I2C1::reset(rcc);
        // // delay.delay_ms(50_u16);

        rprintln!("Start i2c scanning...");
        rprintln!();

        // for addr in 0x00_u8..0x7F {
        //     // Write the empty array and check the slave response.
        //     // rprintln!("trying {:02x}", addr);
        //     let mut buf = [b'\0'; 1];
        //     if let Some(i2c) = &mut self.i2c1 {
        //         if i2c.read(addr, &mut buf).is_ok() {
        //             rprintln!("{:02x} good", addr);
        //         }
        //     }
        //     self.delay.as_ref().unwrap().delay_ms(10_u16);
        // }
        // rprintln!("scan is done");

        self

    }
}

impl RRIVBoard for Board {
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
        self.delay.delay_ms(ms);
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

pub fn build() -> Board {
    let mut board_builder = BoardBuilder::new();
    board_builder = board_builder.setup();
    let board = board_builder.build();
    board
}
