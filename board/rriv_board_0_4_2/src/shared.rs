extern crate alloc;

use alloc::boxed::Box;

use core::{cell::RefCell, mem::MaybeUninit};

use cortex_m::interrupt::Mutex;
use rriv_board::RXProcessor;
use stm32f1xx_hal::{pac, serial::{Rx, Tx}, usb::{Peripheral, UsbBus, UsbBusType}};
use usb_device::{bus::UsbBusAllocator, device::UsbDevice};
use usbd_storage::{subclass::ufi::Ufi, transport::bbb::BulkOnly};

use crate::components::Storage;



// type RedLed = gpio::Pin<'A', 9, Output<OpenDrain>>;
// static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));
pub static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
pub static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;
pub static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; 512]> = MaybeUninit::uninit();

pub static mut UFI: Option<Ufi<BulkOnly<'_, UsbBus<Peripheral>, &mut [u8]>>> = None;
pub static mut STORAGE: Option<Storage> = None;

pub static RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
pub static TX: Mutex<RefCell<Option<Tx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
pub static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));
