#![allow(clippy::empty_loop)]
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::{alloc::Layout, boxed::Box};
use core::{cell::RefCell, default::Default, ffi::c_void, mem::MaybeUninit, ops::DerefMut, u8};
use cortex_m::{
    asm::{dmb, dsb},
    interrupt::Mutex,
    peripheral::NVIC,
};
use embedded_alloc::Heap;
use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output, PinState},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
};

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 128;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

// Initialize the allocator BEFORE you use it
fn alloc_heap() {
    {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}

// type RedLed = gpio::Pin<'C', 7, Output<OpenDrain>>;

// static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));

static RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));

#[repr(C)]
pub(crate) struct Serial {
    tx: Tx<pac::USART2>,
}

/// <div rustbindgen nocopy></div>
/// <div rustbindgen opaque></div>
pub(crate) struct Board {
    pub serial: Serial,
}

impl Board {
    fn init() -> Self {
        // Initialize the allocator
        alloc_heap();
        rtt_init_print!();
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
        // let mut gpioc = p.GPIOC.split();

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

        rprintln!("serial rx.listen()");
        serial.rx.listen();
        serial.rx.listen_idle();

        // let led = gpioc
        //     .pc7
        //     .into_open_drain_output_with_state(&mut gpioc.crl, PinState::Low);

        cortex_m::interrupt::free(|cs| {
            RX.borrow(cs).replace(Some(serial.rx));
            // WAKE_LED.borrow(cs).replace(Some(led));
        });

        rprintln!("unmasking USART2 interrupt");
        unsafe {
            NVIC::unmask(pac::Interrupt::USART2);
        }

        Board {
            serial: Serial { tx: serial.tx },
        }
    }
}

// TODO:: Move interface_new and register_command to App
#[no_mangle]
pub unsafe extern "C" fn rust_serial_interface_new() -> *mut c_void {
    let board = Board::init();
    Box::into_raw(Box::new(board)) as *mut c_void
}
// something like this and make sure the command is the right arg for CString
// #[no_mangle]
// pub extern "C" fn rust_serial_register_command(serial_ptr: *mut c_void, command: *mut c_void, registration: *mut c_void) {
//     let mut board = unsafe { &mut *(serial_ptr as *mut Board) };
//     board.register_command(&mut command, &mut registration as *mut RegisteredCommand as *mut c_void);
// }

// #[no_mangle]
// pub unsafe extern "C" fn rust_serial_read(serial_ptr: *mut c_void) -> u8 {
//     let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
//     // Read the byte that was just sent. Blocks until the read is complete
//     let charr = block!(serial_context.rx.read()).unwrap();
//     Box::into_raw(serial_context);
//     charr
// }

// #[no_mangle]
// pub unsafe extern "C" fn rust_serial_write(serial_ptr: *mut c_void, value: u8) {
//     let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
//     block!(serial_context.tx.write(value)).unwrap_infallible();
//     Box::into_raw(serial_context);
// }

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut rx) = RX.borrow(cs).borrow_mut().deref_mut() {
            if rx.is_rx_not_empty() {
                // if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                //     led.is_set_low();
                // }
                dsb();
                if let Ok(char) = nb::block!(rx.read()) {
                    // process_character(char);
                    rprintln!("serial rx char: {}", char);
                }
                dmb();
                // if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                //     led.is_set_high();
                // }
                rx.listen_idle();
            }
        }
    })
}
