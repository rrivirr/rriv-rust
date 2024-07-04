#![cfg_attr(not(test), no_std)]

extern crate alloc;

use alloc::boxed::Box;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::afio::MAPR;
use stm32f1xx_hal::flash::ACR;
use stm32f1xx_hal::spi::{Spi};
use core::{
    cell::RefCell,
    concat,
    default::Default,
    format_args,
    ops::DerefMut,
    option::{Option, Option::*},
    result::Result::*,
};
use cortex_m::{
    asm::{delay, dmb, dsb},
    interrupt::{CriticalSection, Mutex},
    peripheral::NVIC,
};
use stm32f1xx_hal::gpio::{Alternate, Pin};
use stm32f1xx_hal::pac::{I2C1, I2C2, USART2, USB, TIM2};

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

use rriv_board::{ActuatorDriverServices, RRIVBoard, RRIVBoardBuilder, RXProcessor, SensorDriverServices, TelemetryDriverServices};

use stm32f1xx_hal::rtc::Rtc;

mod components;
use components::*;
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


// type aliases to make things tenable
type BoardI2c1 = BlockingI2c<I2C1, (pin_groups::I2c1Scl, pin_groups::I2c1Sda)>;
type BoardI2c2 = BlockingI2c<I2C2, (pin_groups::I2c2Scl, pin_groups::I2c2Sda)>;

pub struct Board {
    pub delay: SysDelay,
    // // pub power_control: PowerControl,
    pub gpio: DynamicGpioPins, 
    pub internal_adc: InternalAdc,
    pub external_adc: ExternalAdc,
    pub battery_level: BatteryLevel,
    pub rgb_led: RgbLed,
    pub oscillator_control: OscillatorControl,
    pub i2c1: BoardI2c1,
    pub i2c2: BoardI2c2,
    pub rtc: Rtc,
}

impl Board {
    pub fn start(&mut self) {
        // self.power_control.cycle_3v(&mut self.delay);

        // self.internal_adc.enable(&mut self.delay); 
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

    fn retrieve_sensor_settings( // retrieve_all_sensor_configurations
        &mut self,
        buffer: &mut [u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE * rriv_board::EEPROM_TOTAL_SENSOR_SLOTS]
    ) {
        for slot in 0..rriv_board::EEPROM_TOTAL_SENSOR_SLOTS {
            let slice = &mut buffer[slot*rriv_board::EEPROM_SENSOR_SETTINGS_SIZE..(slot+1)*rriv_board::EEPROM_SENSOR_SETTINGS_SIZE];
            read_sensor_configuration_from_eeprom(self, slot.try_into().unwrap(), slice) 
        }
    }

    fn store_sensor_settings(&mut self, slot: u8, bytes: &[u8; rriv_board::EEPROM_SENSOR_SETTINGS_SIZE] ) { // sensor_configuration
        write_sensor_configuration_to_eeprom(self, slot, bytes);
    }

    fn delay_ms(&mut self, ms: u16) {
        self.delay.delay_ms(ms);
    }

    fn timestamp(&mut self) -> u32 {
        return self.rtc.current_time();
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

}

macro_rules! control_services_impl {
    () => {
        fn serial_send(&self, string: &str) {
            rriv_board::RRIVBoard::serial_send(self, string);
        }    

        fn delay_ms(&mut self, ms: u16){
            rriv_board::RRIVBoard::delay_ms(self, ms);
        }
        fn timestamp(&mut self) -> u32 {
            rriv_board::RRIVBoard::timestamp(self)
        }
    };
}



impl SensorDriverServices for Board {

    fn query_internal_adc(&mut self, channel: u8) -> u16 {
        match self.internal_adc.read(channel) {
            Ok(value) => return value,
            Err(error) => {

                let mut errorString = "unhandled error";
                match error {
                    AdcError::NBError(_) => errorString = "ADC NBError",
                    AdcError::NotConfigured => errorString = "ADC Not Configured",
                    AdcError::ReadError => errorString = "ADC Read Error",
                }
                rriv_board::RRIVBoard::serial_send(self, &errorString);
                return 0;
            }
        }
    }

    fn query_external_adc(&mut self, channel: u8) -> u32 {
        // return self.external_adc.read(1);
        return 0;  // not implemented
    }


    control_services_impl!();
    
    fn ic2_read(&mut self, addr: u8, buffer: &mut [u8]) {
        match self.i2c2.read(addr, buffer){
            Ok(_) => return,
            Err(_) => todo!(),
        }
    }
    
    fn ic2_write(&mut self, addr: u8, message: &[u8]) {
        match self.i2c2.write(addr, message){
            Ok(_) => return,
            Err(e) => {
                rprintln!("{:?}", e);
                rriv_board::RRIVBoard::serial_send(self, "Problem writing I2C2");
            }
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
    board_builder.setup();
    let board = board_builder.build();
    board
}


pub struct BoardBuilder {

    // chip features
    pub delay: Option<SysDelay>,

    // pins groups
    pub gpio: Option<DynamicGpioPins>,

    // board features
    pub internal_adc: Option<InternalAdc>,
    pub external_adc: Option<ExternalAdc>,
    pub power_control: Option<PowerControl>,
    pub oscillator_control: Option<OscillatorControl>,
    pub battery_level: Option<BatteryLevel>,
    pub rgb_led: Option<RgbLed>,
    pub i2c1: Option<BoardI2c1>,
    pub i2c2: Option<BoardI2c2>,
    pub rtc: Option<Rtc>
}

impl BoardBuilder {
    pub fn new() -> Self {
        BoardBuilder {
            i2c1: None,
            i2c2: None,
            delay: None,
            gpio: None,
            internal_adc: None,
            external_adc: None,
            power_control: None,
            battery_level: None,
            rgb_led: None,
            oscillator_control: None,
            rtc: None
        }
    }

    pub fn build(self) -> Board {

        // loop{}
        Board {
            i2c1: self.i2c1.unwrap(),
            i2c2: self.i2c2.unwrap(),
            delay: self.delay.unwrap(),
            // // power_control: self.power_control.unwrap(),
            gpio: self.gpio.unwrap(),
            internal_adc: self.internal_adc.unwrap(),
            external_adc: self.external_adc.unwrap(),
            battery_level: self.battery_level.unwrap(),
            rgb_led: self.rgb_led.unwrap(),
            oscillator_control: self.oscillator_control.unwrap(),
            rtc: self.rtc.unwrap()
        }
    }

    fn setup_clocks( oscillator_control: &mut OscillatorControlPins, cfgr: CFGR, flash_acr: &mut ACR ) -> Clocks {
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

        clocks
    }

    fn setup_serial(pins: pin_groups::SerialPins, cr: &mut GpioCr, mapr: &mut MAPR, usart: USART2, clocks: &Clocks) {
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

    pub fn setup_i2c1(pins: pin_groups::I2c1Pins, cr: &mut GpioCr, i2c1: I2C1, mapr: &mut MAPR,  clocks: &Clocks) -> BoardI2c1 {
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


    pub fn setup_i2c2(pins: pin_groups::I2c2Pins, cr: &mut GpioCr, i2c2: I2C2, clocks: &Clocks) -> BoardI2c2 {

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
        self.rtc = Some(Rtc::new(device_peripherals.RTC, &mut backup_domain)); // TODO: make sure LSE on and running?

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
            usb_pins
        ) = pin_groups::build(pins, &mut gpio_cr, delay);

        let clocks = BoardBuilder::setup_clocks(&mut oscillator_control_pins, rcc.cfgr, &mut flash.acr);
        
        let mut delay : Option<SysDelay> = None;
        unsafe {
            let core_peripherals = cortex_m::Peripherals::steal();
            delay = Some(core_peripherals.SYST.delay(&clocks));
        }
        let mut delay = delay.unwrap();
        
        BoardBuilder::setup_serial(serial_pins, &mut gpio_cr, &mut afio.mapr, device_peripherals.USART2, &clocks);
        BoardBuilder::setup_usb(usb_pins, &mut gpio_cr,  device_peripherals.USB, &clocks);


        self.external_adc = Some(ExternalAdc::new(external_adc_pins));


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


        // rprintln!("starting i2c");
        core_peripherals.DWT.enable_cycle_counter(); // BlockingI2c says this is required
        let i2c1 = BoardBuilder::setup_i2c1(i2c1_pins, &mut gpio_cr, device_peripherals.I2C1, &mut afio.mapr, &clocks);
        self.i2c1 = Some(i2c1);
        rprintln!("set up i2c1");

        let i2c2 = BoardBuilder::setup_i2c2(i2c2_pins, &mut gpio_cr, device_peripherals.I2C2, &clocks);
        self.i2c2 = Some(i2c2);
        rprintln!("set up i2c2");

        // loop {
        rprintln!("Start i2c1 scanning...");
        rprintln!();

        for addr in 0x00_u8..0x7F {
            // Write the empty array and check the slave response.
            // rprintln!("trying {:02x}", addr);
            let mut buf = [b'\0'; 1];
            if let Some(i2c) = &mut self.i2c1 {
                if i2c.read(addr, &mut buf).is_ok() {
                    rprintln!("{:02x} good", addr);
                }
            }

            delay.delay_ms(10_u32);
        }
        rprintln!("scan is done");

        rprintln!("Start i2c2 scanning...");
        rprintln!();
        for addr in 0x00_u8..0x7F {
            // Write the empty array and check the slave response.
            // rprintln!("trying {:02x}", addr);
            let mut buf = [b'\0'; 1];
            if let Some(i2c) = &mut self.i2c2 {
                if i2c.read(addr, &mut buf).is_ok() {
                    rprintln!("{:02x} good", addr);
                }
            }
            delay.delay_ms(10_u32);
        }
        rprintln!("scan is done");

        // }

        // a basic idea is to have the struct for a given periphal take ownership of the register block that controls stuff there
        // then Board would have ownership of the feature object, and make changes to the the registers (say through shutdown) through the interface of that struct

        // build the power control
        // self.power_control = Some(PowerControl::new(power_pins));
    

        // build the internal adc
        let internal_adc_configuration = InternalAdcConfiguration::new(internal_adc_pins, device_peripherals.ADC1);
        let internal_adc = internal_adc_configuration.build(&clocks);
        self.internal_adc = Some(internal_adc);
 
        self.rgb_led = Some(build_rgb_led(rgb_led_pins, device_peripherals.TIM1, &mut afio.mapr, &clocks));
        
        self.battery_level = Some(BatteryLevel::new(battery_level_pins));
        
        self.oscillator_control = Some(OscillatorControl::new(oscillator_control_pins));


        self.gpio = Some(dynamic_gpio_pins);

        let delay2: DelayUs<TIM2> =device_peripherals.TIM2.delay(&clocks);
        storage::build(spi1_pins, device_peripherals.SPI1, &mut afio.mapr, clocks, delay2);
        // for SPI SD https://github.com/rust-embedded-community/embedded-sdmmc-rs


        // // let spi_mode = Mode {
        // //     polarity: Polarity::IdleLow,
        // //     phase: Phase::CaptureOnFirstTransition,
        // // };
        let spi2 = Spi::spi2(
            device_peripherals.SPI2,
            (spi2_pins.sck, spi2_pins.miso, spi2_pins.mosi),
            MODE,
            1.MHz(),
            clocks,
          );

        self.delay = Some(delay);


        // we can unsafely .steal on device peripherals to get rcc again, or not?
        // unsafe {
        // let rcc_block = pac::Peripherals::steal().RCC;
        // I2C1::disable(rcc_block);
        // // delay.delay_ms(50_u16);
        // // I2C1::enable(rcc);
        // // delay.delay_ms(50_u16);
        // // I2C1::reset(rcc);
        // // delay.delay_ms(50_u16);

        rprintln!("done with setup");

    }
}

