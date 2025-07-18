#![cfg_attr(not(test), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use i2c_hung_fix::try_unhang_i2c;

use core::{
    any::Any,
    cell::RefCell,
    concat,
    default::Default,
    format_args,
    ops::DerefMut,
    option::Option::{self, *},
    result::Result::*,
};
use core::{mem, result};
use cortex_m::{
    asm::{delay, dmb, dsb},
    interrupt::{CriticalSection, Mutex},
    peripheral::NVIC,
};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use stm32f1xx_hal::{afio::MAPR, gpio::{Cr, Dynamic, PinModeError}, pac::TIM3, time::MilliSeconds, watchdog::IndependentWatchdog};
use stm32f1xx_hal::{serial::StopBits};
use stm32f1xx_hal::flash::ACR;
use stm32f1xx_hal::gpio::{Alternate, Pin};
use stm32f1xx_hal::pac::{I2C1, I2C2, TIM2, USART2, USB};
use stm32f1xx_hal::spi::Spi;

use rtt_target::rprintln;
use stm32f1xx_hal::rcc::{Clocks, CFGR};
use stm32f1xx_hal::{
    gpio::{self, OpenDrain, Output},
    i2c::{BlockingI2c, Mode},
    pac,
    pac::interrupt,
    prelude::*,
    serial::{Config, Rx, Serial as Hal_Serial, Tx},
    timer::delay::*,
};

use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use rriv_board::{
    ActuatorDriverServices, OneWireBusInterface, RRIVBoard, RRIVBoardBuilder, RXProcessor,
    SensorDriverServices, TelemetryDriverServices, EEPROM_TOTAL_SENSOR_SLOTS,
};

use ds323x::{DateTimeAccess, Ds323x, NaiveDate, NaiveDateTime, NaiveTime};
use stm32f1xx_hal::rtc::Rtc;

use one_wire_bus::{crc::check_crc8, Address, OneWire, SearchState};

mod components;
use components::*;
mod pins;
use pins::{GpioCr, Pins};

mod pin_groups;
use pin_groups::*;

type RedLed = gpio::Pin<'A', 9, Output<OpenDrain>>;

static WAKE_LED: Mutex<RefCell<Option<RedLed>>> = Mutex::new(RefCell::new(None));
static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

static USART_RX: Mutex<RefCell<Option<Rx<pac::USART2>>>> = Mutex::new(RefCell::new(None));
static USART_TX: Mutex<RefCell<Option<Tx<pac::USART2>>>> = Mutex::new(RefCell::new(None));

static RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));
static USART_RX_PROCESSOR: Mutex<RefCell<Option<Box<&dyn RXProcessor>>>> = Mutex::new(RefCell::new(None));

#[repr(C)]
pub struct Usart {
    tx: &'static Mutex<RefCell<Option<Tx<pac::USART2>>>>,
}

// type aliases to make things tenable
type BoardI2c1 = BlockingI2c<I2C1, (pin_groups::I2c1Scl, pin_groups::I2c1Sda)>;
type BoardI2c2 = BlockingI2c<I2C2, (pin_groups::I2c2Scl, pin_groups::I2c2Sda)>;

pub struct Board {
    pub delay: DelayUs<TIM3>,
    // // pub power_control: PowerControl,
    pub gpio: DynamicGpioPins,
    pub gpio_cr: GpioCr,
    pub internal_adc: InternalAdc,
    pub external_adc: ExternalAdc,
    pub battery_level: BatteryLevel,
    pub rgb_led: RgbLed,
    pub oscillator_control: OscillatorControl,
    pub i2c1: Option<BoardI2c1>,
    pub i2c2: BoardI2c2,
    pub internal_rtc: Rtc,
    pub storage: Storage,
    pub debug: bool,
    pub file_epoch: i64,
    pub one_wire_bus: OneWire<OneWirePin>,
    one_wire_search_state: Option<SearchState>,
    pub watchdog: IndependentWatchdog
}

impl Board {
    pub fn start(&mut self) {
        rprintln!("starting board");
        // self.power_control.cycle_3v(&mut self.delay);

        // self.internal_adc.enable(&mut self.delay);
        let timestamp: i64 = rriv_board::RRIVBoard::epoch_timestamp(self);
        self.storage.create_file(timestamp);


        // TODO: this is for the NOX sensor, needs to be configured by drivers
        self.gpio.gpio6.make_push_pull_output(&mut self.gpio_cr.gpioc_crh);

    }
}

impl RRIVBoard for Board {
    fn run_loop_iteration(&mut self) {
        self.watchdog.feed();

        self.file_epoch = self.epoch_timestamp();
        // TODO: implement a peek rprint here with a peeked bool
        // TODO: don't interfere with other code that needs async usart
        // if rriv_board::RRIVBoard::unread_usart_message(self) {
        //     let mut message = [0u8; 40];
        //     rriv_board::RRIVBoard::get_usart_response(self, &mut message);
        //     util::remove_invalid_utf8(&mut message);
        //     match core::str::from_utf8(&message){
        //         Ok(message) => rprintln!("got usart message: {}", message),
        //         Err(e) => rprintln!("{:?}", e),
        //     }
        // }
    }

    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>) {
        cortex_m::interrupt::free(|cs| {
            let mut global_rx_binding = RX_PROCESSOR.borrow(cs).borrow_mut();
            *global_rx_binding = Some(processor);
        });
    }

    fn set_usart_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>) {
        cortex_m::interrupt::free(|cs| {
            let mut global_rx_binding = USART_RX_PROCESSOR.borrow(cs).borrow_mut();
            *global_rx_binding = Some(processor);
        });
    }

    fn critical_section<T, F>(&self, f: F) -> T
    where
        F: Fn() -> T,
    {
        cortex_m::interrupt::free(|cs| f())
    }

    // // use this to talk out on serial to other UART modules, RS 485, etc
    fn usart_send(&mut self, string: &str){

        // set control bit for sending
        let _ = self.gpio.gpio6.set_high(); // origi
        // self.gpio.gpio6.set_low();
        // rriv_board::RRIVBoard::delay_ms(self, 100);

        cortex_m::interrupt::free(|cs| {
            
            // USART
            let bytes = string.as_bytes();
            for char in bytes.iter() {
                // rprintln!("char {}", char);
                let t: &RefCell<Option<Tx<USART2>>> = USART_TX.borrow(cs);
                if let Some(tx) = t.borrow_mut().deref_mut() {
                    _ = nb::block!(tx.write(char.clone()));
                }
            }

            // let c = 0b00001101;
            // let t: &RefCell<Option<Tx<USART2>>> = USART_TX.borrow(cs);
            // if let Some(tx) = t.borrow_mut().deref_mut() {
            //     _ = nb::block!(tx.write(c.clone()));
            // }
        });

        rriv_board::RRIVBoard::delay_ms(self, 2);
        self.gpio.gpio6.set_low(); // origi
        // self.gpio.gpio6.set_high();


        // set control bit for receiving
        // rriv_board::RRIVBoard::delay_ms(self, 70);
    }

  
    fn usb_serial_send(&self, string: &str) {
        cortex_m::interrupt::free(|cs| {
            // USB
            let serial = unsafe { USB_SERIAL.as_mut().unwrap() };
            serial.write(string.as_bytes()).ok();
        });
    }

    // outputs to serial and also echos to rtt
    fn serial_debug(&self, string: &str) {
        rprintln!("{:?}", string);
        if self.debug {
            rriv_board::RRIVBoard::usb_serial_send(self, string);
            rriv_board::RRIVBoard::usb_serial_send(self, "\n");
        }
    }

    fn store_datalogger_settings(
        &mut self,
        bytes: &[u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        // who knows the eeprom bytes to use? - the board doeas
        eeprom::write_datalogger_settings_to_eeprom(self, bytes);
    }

    fn retrieve_datalogger_settings(
        // let timestamp: i64 = rriv_board::RRIVBoard::epoch_timestamp(self);
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_DATALOGGER_SETTINGS_SIZE],
    ) {
        eeprom::read_datalogger_settings_from_eeprom(self, buffer);
        util::remove_invalid_utf8(buffer);
    }

    fn retrieve_sensor_settings(
        // retrieve_all_sensor_configurations
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE
                 * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS],
    ) {
        for slot in 0..rriv_board::EEPROM_TOTAL_SENSOR_SLOTS {
            let slice = &mut buffer[slot * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE
                ..(slot + 1) * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];
            read_sensor_configuration_from_eeprom(self, slot.try_into().unwrap(), slice);
            util::remove_invalid_utf8(slice);
        }
    }


    fn store_sensor_settings(
        &mut self,
        slot: u8,
        bytes: &[u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE],
    ) {
        // sensor_configuration
        write_sensor_configuration_to_eeprom(self, slot, bytes);
    }

    fn delay_ms(&mut self, ms: u16) {
        self.delay.delay_ms(ms);
    }

    fn set_epoch(&mut self, epoch: i64) {
        let i2c1 = mem::replace(&mut self.i2c1, None);
        let mut ds3231 = Ds323x::new_ds3231(i2c1.unwrap());
        let millis = epoch * 1000;
        // DateTime::from_timestamp_millis(micros);
        let datetime = NaiveDateTime::from_timestamp_millis(millis);
        // rprintln!("{:?}", datetime);
        if let Some(datetime) = datetime {
            match ds3231.set_datetime(&datetime) {
                Ok(_) => {}
                Err(err) => rprintln!("Error {:?}", err),
            }
        }
        let result = ds3231.datetime();
        self.i2c1 = Some(ds3231.destroy_ds3231());
    }

    fn epoch_timestamp(&mut self) -> i64 {
        let i2c1 = mem::replace(&mut self.i2c1, None);
        let mut ds3231 = Ds323x::new_ds3231(i2c1.unwrap());
        let result = ds3231.datetime();
        self.i2c1 = Some(ds3231.destroy_ds3231());

        match result {
            Ok(date_time) => {
                // rprintln!("got DS3231 time {:?}", date_time.and_utc().timestamp());
                date_time.and_utc().timestamp()
            }
            Err(err) => {
                rprintln!("DS3231 error {:?}", err);
                return 0; // this could fail back to some other clock
            }
        }
    }

    // crystal time, systick
    fn timestamp(&mut self) -> i64 {
        return self.internal_rtc.current_time().into(); // internal RTC
    }

    fn get_sensor_driver_services(&mut self) -> &mut dyn SensorDriverServices {
        return self;
    }

    fn get_actuator_driver_services(&mut self) -> &mut dyn ActuatorDriverServices {
        return self;
    }

    fn get_telemetry_driver_services(&mut self) -> &mut dyn TelemetryDriverServices {
        return self;
    }

    fn set_debug(&mut self, debug: bool) {
        self.debug = debug;
    }

    fn write_log_file(&mut self, data: &str) {
        self.storage.write(data.as_bytes(), self.file_epoch);
    }

    fn flush_log_file(&mut self) {
        todo!("flush_log_file");
    }

    fn dump_eeprom(&mut self) {
        
        let mut buffer: [u8; EEPROM_TOTAL_SENSOR_SLOTS * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE] =
            [0; EEPROM_TOTAL_SENSOR_SLOTS * rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];
        self.retrieve_sensor_settings(&mut buffer);

        for i in 0..buffer.len(){
            if i % rriv_board::EEPROM_SENSOR_SETTINGS_SIZE == 0 {
              rriv_board::RRIVBoard::usb_serial_send(self, format!("\n{}:", i / rriv_board::EEPROM_SENSOR_SETTINGS_SIZE ).as_str());  
            }
           rriv_board::RRIVBoard::usb_serial_send(self,format!("{}", &buffer[i]).as_str()); 
        }
        rriv_board::RRIVBoard::usb_serial_send(self,"}\n"); // } ends the transmissions
    }
}

macro_rules! control_services_impl {
    () => {
        fn usb_serial_send(&self, string: &str) {
            rriv_board::RRIVBoard::usb_serial_send(self, string);
        }

        fn usart_send(&mut self, string: &str) {
            rriv_board::RRIVBoard::usart_send(self, string);
        }

        fn serial_debug(&self, string: &str) {
            rriv_board::RRIVBoard::serial_debug(self, string);
        }

        fn delay_ms(&mut self, ms: u16) {
            rriv_board::RRIVBoard::delay_ms(self, ms);
        }

        fn timestamp(&mut self) -> i64 {
            rriv_board::RRIVBoard::timestamp(self)
        }
    };
}

type OneWireGpio1 = OneWire<Pin<'B', 8, Dynamic>>;

impl SensorDriverServices for Board {
    fn query_internal_adc(&mut self, channel: u8) -> u16 {
        match self.internal_adc.read(channel) {
            Ok(value) => return value,
            Err(error) => {
                let mut error_string = "unhandled error";
                match error {
                    AdcError::NBError(_) => error_string = "Internal ADC NBError",
                    AdcError::NotConfigured => error_string = "Internal ADC Not Configured",
                    AdcError::ReadError => error_string = "Internal ADC Read Error",
                }
                rriv_board::RRIVBoard::usb_serial_send(self, &error_string);
                return 0;
            }
        }
    }

    fn query_external_adc(&mut self, channel: u8) -> u16 {
        let i2c1 = mem::replace(&mut self.i2c1, None);
        let mut i2c1 = i2c1.unwrap();
        let value = self.external_adc.read_single_channel(&mut i2c1, channel);
        self.i2c1 = Some(i2c1);
        return value; 
    }

    control_services_impl!();

    fn ic2_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), ()> {
        match self.i2c2.read(addr, buffer) {
            Ok(_) => return Ok(()),
            Err(e) => {
                rprintln!("{:?}", e);
                // rprintln!(&format!("Problem reading I2C2 {}\n", addr))
                // let error_msg  = &format!("Problem reading I2C2 {}\n", addr);
                // rprintln!(error_msg);
                rriv_board::RRIVBoard::serial_debug(
                    self,
                    &format!("Problem reading I2C2 {}\n", addr),
                );
                for i in 0..buffer.len() {
                    buffer[i] = 0b11111111; // error value
                }
                return Err(());
            }
        }
    }

    fn ic2_write(&mut self, addr: u8, message: &[u8]) -> Result<(), ()> {
        match self.i2c2.write(addr, message) {
            Ok(_) => return Ok(()),
            Err(e) => {
                let kind = match e {
                    stm32f1xx_hal::i2c::Error::Bus => "bus",
                    stm32f1xx_hal::i2c::Error::Arbitration => "arb",
                    stm32f1xx_hal::i2c::Error::Acknowledge => "ack",
                    stm32f1xx_hal::i2c::Error::Overrun => "ovr",
                    stm32f1xx_hal::i2c::Error::Timeout => "tout",
                    _ => "none",
                };
                rriv_board::RRIVBoard::serial_debug(self, &format!("Problem writing I2C2 {} {}", addr, kind));
                return Err(());
            }
        }
    }

    // fn borrow_one_wire_bus(&mut self) -> &mut dyn rriv_board::OneWireBusInterface  {

    //     return &mut self.one_wire_bus;

    // }

    fn one_wire_send_command(&mut self, command: u8, address: u64) {
        let address = Address(address);

        match self
            .one_wire_bus
            .send_command(command, Some(&address), &mut self.delay)
        {
            Ok(_) => rprintln!("sent command ok"),
            Err(e) => rprintln!("{:?}", e),
        }
    }

    fn write_gpio_pin(&mut self, pin: u8, value: bool) {
        let gpio = match pin {
            0 => {
                // self.gpio.gpio1.make_push_pull_output(&mut self.gpio_cr.gpiob_crh);
                if (value) {
                    self.gpio.gpio1.set_high();
                } else {
                    self.gpio.gpio1.set_low();
                }
            }
            // 1 => &mut self.gpio.gpio2,
            // 2 => &mut self.gpio.gpio3,
            // 3 => &mut self.gpio.gpio4,
            // 4 => &mut self.gpio.gpio5,
            // 5 => &mut self.gpio.gpio6,
            // 6 => &mut self.gpio.gpio7,
            // 7 => {
            //     self.gpio.gpio1.make_push_pull_output(&mut self.gpio_cr.gpiob_crh);
            //     if(value){
            //         self.gpio.gpio1.set_high();
            //     } else {
            //         self.gpio.gpio1.set_low();
            //     }
            // }
            _ => {
                if value {
                    self.gpio.gpio6.set_high();
                } else {
                    self.gpio.gpio6.set_low();
                }
            }
        };
    }

    fn one_wire_reset(&mut self) {
        let _ = self.one_wire_bus.reset(&mut self.delay);
    }

    fn one_wire_skip_address(&mut self) {
        let _ = self.one_wire_bus.skip_address(&mut self.delay);
    }

    fn one_wire_write_byte(&mut self, byte: u8) {
        let _ = self.one_wire_bus.write_byte(byte, &mut self.delay);
    }

    fn one_wire_match_address(&mut self, address: u64) {
        let address = Address(address);
        self.one_wire_bus.match_address(&address, &mut self.delay);
    }

    fn one_wire_read_bytes(&mut self, output: &mut [u8]) {
        self.one_wire_bus.read_bytes(output, &mut self.delay);
        // TODO
        match check_crc8::<one_wire_bus::OneWireError<OneWireGpio1>>(output) {
            Ok(_) => return,
            Err(_) => rprintln!("1wire crc error"),
        }
    }

    fn one_wire_bus_start_search(&mut self) {
        self.one_wire_search_state = None;
    }

    fn one_wire_bus_search(&mut self) -> Option<u64> {
        match self.one_wire_bus.device_search(
            self.one_wire_search_state.as_ref(),
            false,
            &mut self.delay,
        ) {
            Ok(Some((device_address, state))) => {
                self.one_wire_search_state = Some(state);
                return Some(device_address.0);
            }
            Ok(None) => {
                rprintln!("no devices 1wire");
                return None;              
            },
            Err(e) => {
                rprintln!("1wire error{:?}", e);          
                return None;  
            },
        } 
    }
}

impl ActuatorDriverServices for Board {
    control_services_impl!();
}

impl TelemetryDriverServices for Board {
    control_services_impl!();
}

#[interrupt]
unsafe fn USART2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut rx) = USART_RX.borrow(cs).borrow_mut().deref_mut() {
            if rx.is_rx_not_empty() {
                if let Ok(c) = nb::block!(rx.read()) {
                    rprintln!("serial rx byte: {}", c);
                    // USART_UNREAD_MESSAGE = true;
                    // if USART_RECEIVE_INDEX < USART_RECEIVE_SIZE - 1 {
                    //     USART_RECEIVE[USART_RECEIVE_INDEX] = c;
                    //     USART_RECEIVE_INDEX = USART_RECEIVE_INDEX + 1;
                    // }
                    

                    let r = USART_RX_PROCESSOR.borrow(cs);
                    if let Some(processor) = r.borrow_mut().deref_mut() {
                        processor.process_character(c.clone());
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
                // use PA9 to flash RGB led
                if let Some(led) = WAKE_LED.borrow(cs).borrow_mut().deref_mut() {
                    if led.is_low() {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }
            }
            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}

pub fn build() -> Board {
    let mut board_builder = BoardBuilder::new();
    board_builder.setup();
    let board = board_builder.build();
    board
}

pub struct OneWirePin {
    pin: Pin<'D', 2, Dynamic>,
}

impl InputPin for OneWirePin {
    type Error = PinModeError;

    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|b| !b)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        // unsafely access the pin state of GPIO B8
        // because the hal doesn't currently implement a IO pin type later change to that
        // this is safe because this is a one wire protocol
        // and we don't need the mode of the pin to be checked.
        unsafe { Ok((*crate::pac::GPIOD::ptr()).idr.read().bits() & (1 << 2) == 0) }
    }
}

impl OutputPin for OneWirePin {
    type Error = PinModeError;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        let result = self.pin.set_low();
        return result;
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        let result = self.pin.set_high();
        return result;
    }
}

pub struct BoardBuilder {
    // chip features
    pub delay: Option<DelayUs<TIM3>>,

    // pins groups
    pub gpio: Option<DynamicGpioPins>,
    pub gpio_cr: Option<GpioCr>,

    // board features
    pub internal_adc: Option<InternalAdc>,
    pub external_adc: Option<ExternalAdc>,
    pub power_control: Option<PowerControl>,
    pub oscillator_control: Option<OscillatorControl>,
    pub battery_level: Option<BatteryLevel>,
    pub rgb_led: Option<RgbLed>,
    pub i2c1: Option<BoardI2c1>,
    pub i2c2: Option<BoardI2c2>,
    pub internal_rtc: Option<Rtc>,
    pub storage: Option<Storage>,
    pub watchdog: Option<IndependentWatchdog>
}

impl BoardBuilder {
    pub fn new() -> Self {
        BoardBuilder {
            i2c1: None,
            i2c2: None,
            delay: None,
            gpio: None,
            gpio_cr: None,
            internal_adc: None,
            external_adc: None,
            power_control: None,
            battery_level: None,
            rgb_led: None,
            oscillator_control: None,
            internal_rtc: None,
            storage: None,
            watchdog: None
        }
    }

    pub fn build(self) -> Board {
        let mut one_wire_option = None;
        let mut gpio_cr = self.gpio_cr.unwrap();
        // steal the gpio5 pin to build a one wire
        // this is probably how we want to build a one wire in general
        // we don't need to worry about the unsafeness, just get the pin we want
        // the board logic can ensure the safeness, or it can be the operators responsibility
        unsafe {
            let device_peripherals = pac::Peripherals::steal();
            let gpiod = device_peripherals.GPIOD.split();

            let mut gpio5 = gpiod.pd2;
            let mut gpio5 = gpio5.into_dynamic(&mut gpio_cr.gpiod_crl);
            gpio5.make_open_drain_output(&mut gpio_cr.gpiod_crl);

            let gpio5 = OneWirePin { pin: gpio5 };

            one_wire_option = match OneWire::new(gpio5) {
                Ok(one_wire) => Some(one_wire),
                Err(e) => {
                    rprintln!("{:?} bad one wire bus", e);
                    panic!("bad one wire bus");
                }
            };
        }

        if (one_wire_option.is_none()) {
            rprintln!("bad one wire creation");
        }
        let one_wire = one_wire_option.unwrap();

        // mcu device registers

        // TODO: just one GPIO pin for the moment
        let mut gpio = self.gpio.unwrap();
        gpio.gpio6.make_push_pull_output(&mut gpio_cr.gpioc_crh);

        // let one_wire_bus_rriv = OneWireGpio1 {
        //     one_wire
        // };

        Board {
            i2c1: self.i2c1,
            i2c2: self.i2c2.unwrap(),
            delay: self.delay.unwrap(),
            gpio: gpio,
            gpio_cr: gpio_cr,
            // // power_control: self.power_control.unwrap(),
            internal_adc: self.internal_adc.unwrap(),
            external_adc: self.external_adc.unwrap(),
            battery_level: self.battery_level.unwrap(),
            rgb_led: self.rgb_led.unwrap(),
            oscillator_control: self.oscillator_control.unwrap(),
            internal_rtc: self.internal_rtc.unwrap(),
            storage: self.storage.unwrap(),
            debug: true,
            file_epoch: 0,
            one_wire_bus: one_wire,
            one_wire_search_state: None,
            watchdog: self.watchdog.unwrap()
        }
    }

    fn setup_clocks(
        oscillator_control: &mut OscillatorControlPins,
        cfgr: CFGR,
        flash_acr: &mut ACR,
    ) -> Clocks {
        oscillator_control.enable_hse.set_high();

        // Freeze the configuration of all the clocks in the system
        // and store the frozen frequencies in `clocks`
        let clocks = cfgr
            .use_hse(8.MHz())
            .sysclk(48.MHz())
            .pclk1(24.MHz())
            .adcclk(14.MHz())
            .freeze(flash_acr);

        assert!(clocks.usbclk_valid());

        rprintln!("{:?}", clocks);

        clocks
    }

    fn setup_serial(
        pins: pin_groups::SerialPins,
        cr: &mut GpioCr,
        mapr: &mut MAPR,
        usart: USART2,
        clocks: &Clocks,
    ) {
        // rprintln!("initializing serial");

        let mut serial = Hal_Serial::new(
            usart,
            (pins.tx, pins.rx),
            mapr,
            // Config::default().baudrate(38400.bps()).wordlength_8bits().parity_none().stopbits(StopBits::STOP1), // this worked for the nox sensor
            Config::default().baudrate(115200.bps()).wordlength_8bits().parity_none().stopbits(StopBits::STOP1), // this appears to be right for the RAK 3172
            &clocks,
        );

        // rprintln!("serial rx.listen()");

        serial.rx.listen();

        cortex_m::interrupt::free(|cs| {
            USART_RX.borrow(cs).replace(Some(serial.rx));
            USART_TX.borrow(cs).replace(Some(serial.tx));
            // WAKE_LED.borrow(cs).replace(Some(led)); // TODO: this needs to be updated.  entire rgb_led object needs to be shared.
        });
        // rprintln!("unmasking USART2 interrupt");
        unsafe {
            NVIC::unmask(pac::Interrupt::USART2);
        }
    }

    fn setup_usb(pins: pin_groups::UsbPins, cr: &mut GpioCr, usb: USB, clocks: &Clocks) {
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

    pub fn setup_i2c1 (
        pins: pin_groups::I2c1Pins,
        cr: &mut GpioCr,
        i2c1: I2C1,
        mapr: &mut MAPR,
        clocks: &Clocks
    ) -> BoardI2c1 {
        let scl1 = pins.i2c1_scl; 
        let sda1 = pins.i2c1_sda; 
           

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

    pub fn setup_i2c2(
        pins: pin_groups::I2c2Pins,
        cr: &mut GpioCr,
        i2c2: I2C2,
        clocks: &Clocks,
    ) -> BoardI2c2 {
        let scl2 = pins.i2c2_scl.into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
        let sda2 = pins.i2c2_sda.into_alternate_open_drain(&mut cr.gpiob_crh); // i2c
        let mut x = BlockingI2c::i2c2(
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
        );

        // this works, so moving out and putting back could conceivably work.
        // let mut ds3231 = Ds323x::new_ds3231( x);
        // let result = ds3231.datetime();
        // x = ds3231.destroy_ds3231();

        // let mut ds3231 = Ds323x::new_ds3231( x);
        // let result = ds3231.datetime();
        // x = ds3231.destroy_ds3231();

        return x;
    }

    fn setup(&mut self) {
        rprintln!("board new");

        let mut core_peripherals = cortex_m::Peripherals::take().unwrap();
        let device_peripherals = pac::Peripherals::take().unwrap();

        // mcu device registers
        let rcc = device_peripherals.RCC.constrain();
        let mut flash = device_peripherals.FLASH.constrain();
        let mut afio = device_peripherals.AFIO.constrain(); // Prepare the alternate function I/O registers

        let mut pwr = device_peripherals.PWR;
        let mut backup_domain = rcc.bkp.constrain(device_peripherals.BKP, &mut pwr);
        self.internal_rtc = Some(Rtc::new(device_peripherals.RTC, &mut backup_domain)); // TODO: make sure LSE on and running?

        // Prepare the GPIO
        let gpioa: gpio::gpioa::Parts = device_peripherals.GPIOA.split();
        let gpiob = device_peripherals.GPIOB.split();
        let gpioc = device_peripherals.GPIOC.split();
        let gpiod = device_peripherals.GPIOD.split();

        let delay = cortex_m::delay::Delay::new(core_peripherals.SYST, 1000000);

        // Set up pins
        let (mut pins, mut gpio_cr) = Pins::build(gpioa, gpiob, gpioc, gpiod, &mut afio.mapr);
        let (
            mut external_adc_pins,
            internal_adc_pins,
            battery_level_pins,
            dynamic_gpio_pins,
            i2c1_pins,
            i2c2_pins,
            mut oscillator_control_pins,
            mut power_pins,
            rgb_led_pins,
            serial_pins,
            spi1_pins,
            spi2_pins,
            usb_pins,
        ) = pin_groups::build(pins, &mut gpio_cr, delay);

        let clocks =
            BoardBuilder::setup_clocks(&mut oscillator_control_pins, rcc.cfgr, &mut flash.acr);

        // let mut delay: Option<SysDelay> = None;
        // unsafe {
        //     let core_peripherals = cortex_m::Peripherals::steal();
        //     delay = Some(core_peripherals.SYST.delay(&clocks));
        // }
        // let mut delay = delay.unwrap();

        let mut delay: DelayUs<TIM3> = device_peripherals.TIM3.delay(&clocks);

        BoardBuilder::setup_serial(
            serial_pins,
            &mut gpio_cr,
            &mut afio.mapr,
            device_peripherals.USART2,
            &clocks,
        );
        BoardBuilder::setup_usb(usb_pins, &mut gpio_cr, device_peripherals.USB, &clocks);

        self.external_adc = Some(ExternalAdc::new(external_adc_pins));
        self.external_adc.as_mut().unwrap().disable(&mut delay);

        power_pins.enable_3v.set_high();
        delay.delay_ms(500_u32);
        power_pins.enable_3v.set_low();
        delay.delay_ms(500_u32);
        power_pins.enable_3v.set_high();
        delay.delay_ms(500_u32);

        // external adc and i2c stability require these steps
        power_pins.enable_5v.set_high();
        delay.delay_ms(250_u32);
        self.external_adc.as_mut().unwrap().enable(&mut delay);
        self.external_adc.as_mut().unwrap().reset(&mut delay);


        let mut watchdog = IndependentWatchdog::new(device_peripherals.IWDG);
        watchdog.stop_on_debug(&device_peripherals.DBGMCU, true);
        
        watchdog.start(MilliSeconds::secs(6));
        watchdog.feed();

        rprintln!("unhang I2C1 if hung");

        let mut scl1= i2c1_pins.i2c1_scl.into_open_drain_output(&mut gpio_cr.gpiob_crl);
        let mut sda1 = i2c1_pins.i2c1_sda.into_open_drain_output(&mut gpio_cr.gpiob_crl);
        sda1.set_high(); // remove signal from the master

        match  try_unhang_i2c(&mut scl1, &sda1, &mut delay, i2c_hung_fix::FALLBACK_I2C_FREQUENCY, 30){
            Ok(_) => {},
            Err(e) => {
                rprintln!("Couln't reset i2c1");
                loop{}
            }, // wait for IDWP to reset.   actually we can just hardware reset here?
        }

        let i2c1_pins = I2c1Pins::rebuild(scl1, sda1, &mut gpio_cr);

        // rprintln!("starting i2c");
        core_peripherals.DWT.enable_cycle_counter(); // BlockingI2c says this is required
        let mut i2c1 = BoardBuilder::setup_i2c1(
            i2c1_pins,
            &mut gpio_cr,
            device_peripherals.I2C1,
            &mut afio.mapr,
            &clocks
        );
        rprintln!("set up i2c1 done");

        rprintln!("unhang I2C2 if hung");
        
        let mut scl2 = i2c2_pins.i2c2_scl.into_open_drain_output(&mut gpio_cr.gpiob_crh);
        let mut sda2= i2c2_pins.i2c2_sda.into_open_drain_output(&mut gpio_cr.gpiob_crh);
        sda2.set_high(); // remove signal from the master

        match  try_unhang_i2c(&mut scl2, &sda2, &mut delay, 100_000, i2c_hung_fix::RECOMMENDED_MAX_CLOCK_CYCLES){
            Ok(_) => {},
            Err(_) => loop{}, // wiat for IDWP to reset.   actually we can just hardware reset here?
        }

        let i2c2_pins = I2c2Pins::rebuild(scl2, sda2, &mut gpio_cr);

        let mut i2c2 =
            BoardBuilder::setup_i2c2(i2c2_pins, &mut gpio_cr, device_peripherals.I2C2, &clocks);
        rprintln!("set up i2c2 done");

        rprintln!("i2c1 scanning...");

        for addr in 0x00_u8..0x7F {
            // Write the empty array and check the slave response.
            // rprintln!("trying {:02x}", addr);
            let mut buf = [b'\0'; 1];
            if i2c1.read(addr, &mut buf).is_ok() {
                rprintln!("{:02x} good", addr);
            }

            delay.delay_ms(10_u32);
        }
        rprintln!("scan is done");

        watchdog.feed();

        rprintln!("i2c2 scanning...");
        rprintln!();
        for addr in 0x00_u8..0x7F {
            // Write the empty array and check the slave response.
            // rprintln!("trying {:02x}", addr);
            let mut buf = [b'\0'; 1];
            if i2c2.read(addr, &mut buf).is_ok() {
                rprintln!("{:02x} good", addr);
            }
            delay.delay_ms(10_u32);
        }
        rprintln!("scan is done");


        watchdog.feed();

        // configure external ADC
        self.external_adc.as_mut().unwrap().configure(&mut i2c1);

        self.i2c1 = Some(i2c1);
        self.i2c2 = Some(i2c2);

        // a basic idea is to have the struct for a given periphal take ownership of the register block that controls stuff there
        // then Board would have ownership of the feature object, and make changes to the the registers (say through shutdown) through the interface of that struct

        // build the power control
        let mut power_control = Some(PowerControl::new(power_pins)).unwrap();
        power_control.cycle_5v(&mut delay);

        // build the internal adc
        let internal_adc_configuration =
            InternalAdcConfiguration::new(internal_adc_pins, device_peripherals.ADC1);
        let mut internal_adc = internal_adc_configuration.build(&clocks);
        internal_adc.enable(&mut delay);
        self.internal_adc = Some(internal_adc);

        self.rgb_led = Some(build_rgb_led(
            rgb_led_pins,
            device_peripherals.TIM1,
            &mut afio.mapr,
            &clocks,
        ));

        self.battery_level = Some(BatteryLevel::new(battery_level_pins));

        self.oscillator_control = Some(OscillatorControl::new(oscillator_control_pins));

        self.gpio = Some(dynamic_gpio_pins);
        self.gpio_cr = Some(gpio_cr);

        let delay2: DelayUs<TIM2> = device_peripherals.TIM2.delay(&clocks);
        // delay2.delay(2);
        // rprintln!("{:?}", clocks);
        
        // let mut storage = storage::build(
        //     spi1_pins,
        //     device_peripherals.SPI1,
        //     &mut afio.mapr,
        //     clocks,
        //     delay2,
        // );

        let mut storage = storage::build(spi2_pins, device_peripherals.SPI2, clocks, delay2);
        // for SPI SD https://github.com/rust-embedded-community/embedded-sdmmc-rs
        // rprintln!("{:?}", clocks);
   
        self.storage = Some(storage);

        // // let spi_mode = Mode {
        // //     polarity: Polarity::IdleLow,
        // //     phase: Phase::CaptureOnFirstTransition,
        // // };
        // let spi2 = Spi::spi2(
        //     device_peripherals.SPI2,
        //     (spi2_pins.sck, spi2_pins.miso, spi2_pins.mosi),
        //     MODE,
        //     1.MHz(),
        //     clocks,
        // );

        self.delay = Some(delay);

        watchdog.feed();

        self.watchdog = Some(watchdog);

        rprintln!("done with setup");
    }
}

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts((p as *const T) as *const u8, ::core::mem::size_of::<T>())
}
