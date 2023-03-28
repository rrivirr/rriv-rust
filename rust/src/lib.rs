//! Serial interface loopback test
//!
//! You have to short the TX and RX pins to make this program work

#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use core::{default::Default, u8, ffi::c_void, mem::MaybeUninit};
use alloc::{boxed::Box, alloc::Layout};
use panic_halt as _;
use nb::block;
use embedded_alloc::Heap;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    serial::*
};
use unwrap_infallible::UnwrapInfallible;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const CR: u8 = b'\r';
const LF: u8 = b'\n';
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
pub(crate) struct SerialInterfaceContext {
    rx: Rx<pac::USART2>,
    tx: Tx<pac::USART2>,
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

    // USART1
    // let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    // let rx = gpioa.pa10;

    // USART1
    // let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    // let rx = gpiob.pb7;

    // USART2
    let tx = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
    let rx = gpioa.pa3;

    // // USART3
    // // Configure pb10 as a push_pull output, this will be the tx pin
    // let tx = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    // // Take ownp.USART2usart device. Take ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    // let mut SERIAL:Serial<stm32f1xx_hal::pac::USART2, (stm32f1xx_hal::gpio::Pin<'A', 2, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 3>)> = Serial::new(
    let serial = Serial::new(
        p.USART2,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        &clocks,
    );
    let mut context = SerialInterfaceContext {
        rx: serial.rx,
        tx: serial.tx,
    };
    context.tx.write(b'\r').unwrap();
    let serial_ptr = Box::into_raw(Box::new(context)) as *mut c_void;
    let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
    block!(serial_context.tx.write(b'S')).unwrap_infallible();
    block!(serial_context.tx.write(b'T')).unwrap_infallible();
    block!(serial_context.tx.write(b'A')).unwrap_infallible();
    block!(serial_context.tx.write(b'R')).unwrap_infallible();
    block!(serial_context.tx.write(b'T')).unwrap_infallible();
    block!(serial_context.tx.write(CR)).unwrap_infallible();
    block!(serial_context.tx.write(LF)).unwrap_infallible();
    Box::into_raw(serial_context) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn rust_serial_read(serial_ptr: *mut c_void) -> u8 {
    
    let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
    // Read the byte that was just sent. Blocks until the read is complete
    let charr = block!(serial_context.rx.read()).unwrap();
    Box::into_raw(serial_context);
    charr
}

#[no_mangle]
pub unsafe extern "C" fn rust_serial_write(serial_ptr: *mut c_void, value: u8) {
    let mut serial_context = Box::from_raw(serial_ptr as *mut SerialInterfaceContext);
    block!(serial_context.tx.write(value)).unwrap_infallible();
    Box::into_raw(serial_context);
}
