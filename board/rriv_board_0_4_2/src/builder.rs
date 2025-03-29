use rtt_target::rprintln;
use stm32f1xx_hal::gpio;
use stm32f1xx_hal::pac;
use stm32f1xx_hal::pac::TIM2;
use stm32f1xx_hal::rtc::Rtc;
use stm32f1xx_hal::timer::DelayUs;
use stm32f1xx_hal::timer::SysDelay;

use crate::components;
use crate::components::*;
use crate::pin_groups;
use crate::pin_groups::*;
use crate::pins::Pins;
use crate::Board;

use stm32f1xx_hal::prelude::*;


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
    pub internal_rtc: Option<Rtc>,
    // pub storage: Option<Storage>,
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
            internal_rtc: None,
            // storage: None,
        }
    }

    pub fn build(self) -> Board {
        // loop{}
        Board {
            i2c1: self.i2c1,
            i2c2: self.i2c2.unwrap(),
            delay: self.delay.unwrap(),
            // // power_control: self.power_control.unwrap(),
            gpio: self.gpio.unwrap(),
            internal_adc: self.internal_adc.unwrap(),
            external_adc: self.external_adc.unwrap(),
            battery_level: self.battery_level.unwrap(),
            rgb_led: self.rgb_led.unwrap(),
            oscillator_control: self.oscillator_control.unwrap(),
            internal_rtc: self.internal_rtc.unwrap(),
            debug: true,
            file_epoch: 0,
        }
    }

    

    pub fn setup(&mut self) {
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
            setup_clocks(&mut oscillator_control_pins, rcc.cfgr, &mut flash.acr);

        let mut delay: Option<SysDelay> = None;
        unsafe {
            let core_peripherals = cortex_m::Peripherals::steal();
            delay = Some(core_peripherals.SYST.delay(&clocks));
        }
        let mut delay = delay.unwrap();

        let mut external_adc = ExternalAdc::new(external_adc_pins);
        external_adc.disable(&mut delay);
        self.external_adc = Some(external_adc);


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
        let i2c1 = setup_i2c1(
            i2c1_pins,
            &mut gpio_cr,
            device_peripherals.I2C1,
            &clocks,
        );
        self.i2c1 = Some(i2c1);
        rprintln!("set up i2c1");

        let i2c2 =
            setup_i2c2(i2c2_pins, &mut gpio_cr, device_peripherals.I2C2, &clocks);
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

        // a basic idea is to have the struct for a given periphal take ownership of the register block that controls stuff there
        // then Board would have ownership of the feature object, and make changes to the the registers (say through shutdown) through the interface of that struct

        // build the power control
        // self.power_control = Some(PowerControl::new(power_pins));

        // build the internal adc
        let internal_adc_configuration =
            InternalAdcConfiguration::new(internal_adc_pins, device_peripherals.ADC1);
        let internal_adc = internal_adc_configuration.build(&clocks);
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

        let delay2: DelayUs<TIM2> = device_peripherals.TIM2.delay(&clocks);
        rprintln!("{:?}", clocks);

        storage::build(spi2_pins, device_peripherals.SPI2, clocks, delay2);

        components::serial::setup_serial(
            serial_pins,
            &mut gpio_cr,
            &mut afio.mapr,
            device_peripherals.USART2,
            &clocks,
        );

        components::usb::setup_usb(
            usb_pins, 
            &mut gpio_cr, 
            device_peripherals.USB, 
            &clocks
        );

        self.delay = Some(delay);

        rprintln!("done with setup");
    }
}






