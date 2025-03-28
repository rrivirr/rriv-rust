use stm32f1xx_hal::{afio::MAPR, pac::NVIC, prelude::*, serial::Config};

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;
use rtt_target::rprintln;
use stm32f1xx_hal::{pac::{self, USART2}, rcc::Clocks, serial::{Rx, SerialExt, Tx}};

use crate::{pin_groups, pins::GpioCr};



pub static RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
pub static TX: Mutex<RefCell<Option<Tx<pac::USART2>>>> = Mutex::new(RefCell::new(None));

pub fn setup_serial(
    pins: pin_groups::SerialPins,
    cr: &mut GpioCr,
    mapr: &mut MAPR,
    usart: USART2,
    clocks: &Clocks,
) {
    // rprintln!("initializing serial");
    rprintln!("serial rx.listen()");
    let mut serial = usart.serial(
        (pins.tx, pins.rx),
        Config::default().baudrate(115200.bps()),
        clocks,
    );
    serial.rx.listen();

    cortex_m::interrupt::free(|cs| {
        RX.borrow(cs).replace(Some(serial.rx));
        TX.borrow(cs).replace(Some(serial.tx));
        // WAKE_LED.borrow(cs).replace(Some(led)); // TODO: this needs to be updated.  entire rgb_led object needs to be shared.
    });
    // rprintln!("unmasking USART2 interrupt");
    unsafe {
        NVIC::unmask(pac::Interrupt::USART2);
    }
}