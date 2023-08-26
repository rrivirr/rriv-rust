#![cfg_attr(not(test), no_std)]

extern crate alloc;
extern crate panic_halt;

use alloc::boxed::Box;
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
use cortex_m::delay;
use cortex_m::{
    asm::{dmb, dsb},
    interrupt::Mutex,
    peripheral::NVIC,
};
use rtt_target::rprintln;
use stm32f1xx_hal::timer::Timer;

// use rtt_target::rprintln;
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output, PinState},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
};

type RedLed = gpio::Pin<'C', 10, Output<OpenDrain>>;

static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));

pub trait RXProcessor: Send + Sync {
    fn process_character(&'static self, character: u8);
}

static RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
static TX: Mutex<RefCell<Option<Tx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));

#[repr(C)]
pub struct Serial {
    tx: &'static Mutex<RefCell<Option<Tx<pac::USART2>>>>,
}

pub struct Board {
    pub serial: Serial,
}

impl Board {
    pub fn new() -> Self {
        rprintln!("board new");
        // Get access to the core peripherals from the cortex-m crate
        let core_pac = cortex_m::Peripherals::take().unwrap();
        // Get access to the device specific peripherals from the peripheral access crate
        let p = pac::Peripherals::take().unwrap();

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = p.FLASH.constrain();
        let rcc = p.RCC.constrain();

        // Freeze the configuration of all the clocks in the system
        // and store the frozen frequencies in `clocks`
        let clocks = rcc.cfgr.freeze(&mut flash.acr);

        // Prepare the alternate function I/O registers
        let mut afio = p.AFIO.constrain();

        // Prepare the peripherals
        let mut gpioa = p.GPIOA.split();
        let mut gpioc = p.GPIOC.split();

        // USART2
        let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let rx_pin = gpioa.pa3;

        // Init Serial
        rprintln!("initializing serial");
        let mut serial = Hal_Serial::new(
            p.USART2,
            (tx_pin, rx_pin),
            &mut afio.mapr,
            Config::default().baudrate(115200.bps()),
            &clocks,
        );
        // Configure the syst timer to trigger an update every second
        let mut timer = Timer::syst(core_pac.SYST, &clocks).counter_hz();
        timer.start(1.Hz()).unwrap();

        serial.rx.listen();
        serial.rx.listen_idle();
        rprintln!("listening for serial rx interrupt");

        let led = gpioc
            .pc10
            .into_open_drain_output_with_state(&mut gpioc.crh, PinState::Low);

        // Wait for the timer to trigger an update and change the state of the LED
        // test sending serial data
        nb::block!(timer.wait()).unwrap();
        serial.tx.write_str("hello serial listeners").unwrap();
        nb::block!(timer.wait()).unwrap();

        cortex_m::interrupt::free(|cs| {
            RX.borrow(cs).replace(Some(serial.rx));
            TX.borrow(cs).replace(Some(serial.tx));
            WAKE_LED.borrow(cs).replace(Some(led));
        });

        rprintln!("unmasking USART2 interrupt");
        unsafe {
            NVIC::unmask(pac::Interrupt::USART2);
        }

        Board {
            serial: Serial { tx: &TX },
        }
    }

    pub fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>) {
        cortex_m::interrupt::free(|cs| {
            let mut global_rx_binding = RX_PROCESSOR.borrow(cs).borrow_mut();
            *global_rx_binding = Some(processor);
        });
    }
}

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut rx) = RX.borrow(cs).borrow_mut().deref_mut() {
            if rx.is_rx_not_empty() {
                // use PA10 to flash RGB led
                if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                    led.set_high();
                    // wait 500ms to see the led
                }
                dsb();
                if let Ok(c) = nb::block!(rx.read()) {
                    rprintln!("serial rx byte: {}", c);
                    let r = RX_PROCESSOR.borrow(cs);

                    if let Some(processor) = r.borrow_mut().deref_mut() {
                        processor.process_character(c);
                    }

                    rx.listen_idle();

                    // let t = TX.borrow(cs);
                    // if let Some(tx) = t.borrow_mut().deref_mut() {
                    //     _ = tx.write(c);
                    // }

                    rx.unlisten_idle();
                }
                dmb();
                if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                    led.set_low();
                }
            }
        }
    })
}
//
