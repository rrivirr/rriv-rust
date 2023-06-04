#![allow(clippy::empty_loop)]
#![cfg_attr(not(test), no_std)]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::{alloc::Layout, boxed::Box};
use core::{default::Default, ffi::c_void, mem::MaybeUninit, u8};
use embedded_alloc::Heap;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{
    pac,
    pac::interrupt,
    prelude::*,
    serial::*,
    serial::{Config, Rx, Serial as Hal_Serial},
};
use unwrap_infallible::UnwrapInfallible;

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

/// <div rustbindgen nocopy></div>
/// <div rustbindgen opaque></div>
#[repr(C)]
pub(crate) struct Serial {
    tx: Tx<pac::USART2>,
}

static mut RX: Option<Rx<pac::USART2>> = None;

pub(crate) struct Board {
    pub serial: Serial,
}

#[no_mangle]
pub unsafe extern "C" fn rust_serial_interface_new() -> *mut c_void {
    alloc_heap();

    // Get access to the device specific peripherals from the peripheral access crate
    let p = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // Prepare the alternate function I/O registers
    let mut afio = p.AFIO.constrain();
    // Prepare the GPIOB peripheral
    let mut gpioa = p.GPIOA.split();
    // USART2
    let tx_pin = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let rx_pin = gpioa.pa3;
    // Init Serial
    let mut serial = Hal_Serial::new(
        p.USART2,
        (tx_pin, rx_pin),
        &mut afio.mapr,
        Config::default().baudrate(115200.bps()),
        &clocks,
    );
    serial.rx.listen();
    let context = Serial { tx: serial.tx };
    let serial_ptr = Box::into_raw(Box::new(context)) as *mut c_void;
    let serial_context = Box::from_raw(serial_ptr as *mut Serial);
    cortex_m::interrupt::free(|_| unsafe {
        RX = Some(serial.rx);
    });
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::USART2);
    Box::into_raw(serial_context) as *mut c_void
}

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
    cortex_m::interrupt::free(|_| {
        if let Some(rx) = RX.as_mut() {
            if rx.is_rx_not_empty() {
                if let Ok(char) = nb::block!(rx.read()) {
                    // process_character(char);
                }
                rx.listen_idle();
            }
        }
    })
}
