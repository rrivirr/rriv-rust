
use core::mem::MaybeUninit;

use cortex_m::asm::delay;
use embedded_hal::digital::OutputPin;
use stm32f1xx_hal::{pac::{self, NVIC, USB}, rcc::Clocks, serial::{Rx, Tx}, usb::{Peripheral, UsbBus, UsbBusType}};
use usb_device::{bus::UsbBusAllocator, device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid}};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use usbd_storage::{subclass::ufi::Ufi, transport::bbb::BulkOnly};

use crate::{pin_groups, pins::GpioCr};

use stm32f1xx_hal::prelude::*;


pub static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
pub static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
pub static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;
pub static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; 512]> = MaybeUninit::uninit();
pub static mut UFI: Option<Ufi<BulkOnly<'_, UsbBus<Peripheral>, &mut [u8]>>> = None;

const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64


pub fn setup_usb(pins: pin_groups::UsbPins, cr: &mut GpioCr, usb: USB, clocks: &Clocks) {
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

        let ufi = usbd_storage::subclass::ufi::Ufi::new(
            USB_BUS.as_ref().unwrap(),
            USB_PACKET_SIZE,
            unsafe { USB_TRANSPORT_BUF.assume_init_mut().as_mut_slice() },
        )
        .unwrap();
        UFI = Some(ufi);

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x0483, 0x29))
            .device_class(USB_CLASS_CDC)
            .self_powered(false)
            .strings(&[StringDescriptors::default()
                .manufacturer("RRIV")
                .product("RRIV Data Logger")
                .serial_number("_rriv")])
            .unwrap()
            .build();

        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        NVIC::unmask(pac::Interrupt::USB_HP_CAN_TX);
        NVIC::unmask(pac::Interrupt::USB_LP_CAN_RX0);
    }
}
