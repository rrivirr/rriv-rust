#![cfg_attr(not(test), no_std)]

extern crate alloc;
extern crate panic_halt;

use alloc::boxed::Box;
use core::marker::{Send, Sync};
use core::{
    cell::RefCell,
    concat,
    ops::DerefMut,
    option::{Option, Option::*},
    result::Result::*,
};

use rtt_target::rprintln;

use cortex_m::asm::{delay, dmb, dsb, wfi};
use cortex_m::interrupt::{CriticalSection, Mutex};
use stm32f1xx_hal::gpio::{self, OpenDrain, Output, PinState};
use stm32f1xx_hal::pac::{self, interrupt, Interrupt, NVIC};
use stm32f1xx_hal::prelude::*;
// use stm32f1xx_hal::timer::Timer;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

type RedLed = gpio::Pin<'C', 10, Output<OpenDrain>>;

static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));

pub trait RXProcessor: Send + Sync {
    fn process_character(&'static self, character: u8);
}

static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));

pub struct Board {}

impl Board {
    pub fn new() -> Self {
        rprintln!("board new");
        // Get access to the core peripherals from the cortex-m crate
        // let core_pac = cortex_m::Peripherals::take().unwrap();
        // Get access to the device specific peripherals from the peripheral access crate
        let p = pac::Peripherals::take().unwrap();

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = p.FLASH.constrain();
        let rcc = p.RCC.constrain();

        // Freeze the configuration of all the clocks in the system
        // and store the frozen frequencies in `clocks`
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .freeze(&mut flash.acr);

        assert!(clocks.usbclk_valid());

        // Prepare the peripherals
        let mut gpioa = p.GPIOA.split();
        let mut gpioc = p.GPIOC.split();

        // USB Serial
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low();
        delay(clocks.sysclk().raw() / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = Peripheral {
            usb: p.USB,
            pin_dm: usb_dm,
            pin_dp: usb_dp,
        };

        // Unsafe to allow access to static variables
        unsafe {
            let bus = UsbBus::new(usb);

            USB_BUS = Some(bus);

            USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

            let usb_dev =
                UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                    .manufacturer("RRIV")
                    .product("RRIV Data Logger")
                    .serial_number("_rriv")
                    .device_class(USB_CLASS_CDC)
                    .build();

            USB_DEVICE = Some(usb_dev);
        }

        unsafe {
            NVIC::unmask(Interrupt::USB_HP_CAN_TX);
            NVIC::unmask(Interrupt::USB_LP_CAN_RX0);
        }

        wfi();

        // Configure the syst timer to trigger an update every second
        // let mut timer = Timer::syst(core_pac.SYST, &clocks).counter_hz();
        // timer.start(1.Hz()).unwrap();

        let led = gpioc
            .pc10
            .into_open_drain_output_with_state(&mut gpioc.crh, PinState::High);

        // Wait for the timer to trigger an update and change the state of the LED
        // nb::block!(timer.wait()).unwrap();

        cortex_m::interrupt::free(|cs| {
            WAKE_LED.borrow(cs).replace(Some(led));
        });

        Board {}
    }

    pub fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>) {
        cortex_m::interrupt::free(|cs| {
            let mut global_rx_binding = RX_PROCESSOR.borrow(cs).borrow_mut();
            *global_rx_binding = Some(processor);
        });
    }
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
