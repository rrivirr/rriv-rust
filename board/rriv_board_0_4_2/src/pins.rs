use stm32f1xx_hal::gpio::{Debugger, Output, Input, OpenDrain, PullUp, PullDown, Floating, PushPull, Dynamic, PinState};

pub struct GPIO {
  pub gpio1: stm32f1xx_hal::gpio::Pin<'B', 8, Dynamic>,
  pub gpio2: stm32f1xx_hal::gpio::Pin<'B', 5, Dynamic>,
  pub gpio3: stm32f1xx_hal::gpio::Pin<'B', 4, Dynamic>,
  pub gpio4: stm32f1xx_hal::gpio::Pin<'B', 3, Dynamic>,
  pub gpio5: stm32f1xx_hal::gpio::Pin<'D', 2, Dynamic>,
  pub gpio6: stm32f1xx_hal::gpio::Pin<'C', 12, Dynamic>,
  pub gpio7: stm32f1xx_hal::gpio::Pin<'C', 11, Dynamic>,
  pub gpio8: stm32f1xx_hal::gpio::Pin<'C', 10, Dynamic>,
}

impl GPIO {
  pub fn build (
    gpio1: stm32f1xx_hal::gpio::Pin<'B', 8>,
    gpio2: stm32f1xx_hal::gpio::Pin<'B', 5>,
    gpio3: stm32f1xx_hal::gpio::Pin<'B', 4>,
    gpio4: stm32f1xx_hal::gpio::Pin<'B', 3>,
    gpio5: stm32f1xx_hal::gpio::Pin<'D', 2>,
    gpio6: stm32f1xx_hal::gpio::Pin<'C', 12>,
    gpio7: stm32f1xx_hal::gpio::Pin<'C', 11>,
    gpio8: stm32f1xx_hal::gpio::Pin<'C', 10>,
    gpio_raw: &mut GpioRaw,
  ) -> Self {
    return GPIO {
      gpio1: gpio1.into_dynamic(&mut gpio_raw.gpiob.crh), 
      gpio2: gpio2.into_dynamic(&mut gpio_raw.gpiob.crl),  
      gpio3: gpio3.into_dynamic(&mut gpio_raw.gpiob.crl),
      gpio4: gpio4.into_dynamic(&mut gpio_raw.gpiob.crl), 
      gpio5: gpio5.into_dynamic(&mut gpio_raw.gpiod.crl), 
      gpio6: gpio6.into_dynamic(&mut gpio_raw.gpioc.crh), 
      gpio7: gpio7.into_dynamic(&mut gpio_raw.gpioc.crh),
      gpio8: gpio8.into_dynamic(&mut gpio_raw.gpioc.crh),
    }
  }

}

pub struct PowerControl {
  pub enable_3v: stm32f1xx_hal::gpio::Pin<'C', 5, Output>,
  pub enable_5v: stm32f1xx_hal::gpio::Pin<'B', 12, Output>,
  pub enable_vin_measure: stm32f1xx_hal::gpio::Pin<'B', 1, Output>,
  pub enable_external_adc: stm32f1xx_hal::gpio::Pin<'C', 6, Output>,
  pub enable_avdd: stm32f1xx_hal::gpio::Pin<'A', 1, Output>,
}

impl PowerControl {
  pub fn build(
    enable_3v: stm32f1xx_hal::gpio::Pin<'C', 5>,
    enable_5v: stm32f1xx_hal::gpio::Pin<'B', 12>,
    enable_vin_measure: stm32f1xx_hal::gpio::Pin<'B', 1>,
    enable_external_adc: stm32f1xx_hal::gpio::Pin<'C', 6>,
    enable_avdd: stm32f1xx_hal::gpio::Pin<'A', 1>,
    gpio_raw: &mut GpioRaw,
  ) -> Self {
    return PowerControl {
      enable_3v: enable_3v.into_push_pull_output(&mut gpio_raw.gpioc.crl),
      enable_5v: enable_5v.into_push_pull_output(&mut gpio_raw.gpiob.crh),
      enable_vin_measure: enable_vin_measure.into_push_pull_output(&mut gpio_raw.gpiob.crl),
      enable_avdd: enable_avdd.into_push_pull_output(&mut gpio_raw.gpioa.crl),
      enable_external_adc: enable_external_adc.into_push_pull_output(&mut gpio_raw.gpioc.crl)
    }
  }
}

pub struct InternalAdc {
  pub vin_measure: stm32f1xx_hal::gpio::Pin<'B', 0, Input>,
  pub channel1: stm32f1xx_hal::gpio::Pin<'A', 0, Input>,
  pub channel2: stm32f1xx_hal::gpio::Pin<'C', 3, Input>,
  pub channel3: stm32f1xx_hal::gpio::Pin<'C', 2, Input>,
  pub channel4: stm32f1xx_hal::gpio::Pin<'C', 1, Input>,
  pub channel5: stm32f1xx_hal::gpio::Pin<'C', 0, Input>,
}

impl InternalAdc {
  pub fn build(
    vin_measure: stm32f1xx_hal::gpio::Pin<'B', 0>,
    channel1: stm32f1xx_hal::gpio::Pin<'A', 0>,
    channel2: stm32f1xx_hal::gpio::Pin<'C', 3>,
    channel3: stm32f1xx_hal::gpio::Pin<'C', 2>,
    channel4: stm32f1xx_hal::gpio::Pin<'C', 1>,
    channel5: stm32f1xx_hal::gpio::Pin<'C', 0>,
    gpio_raw: &mut GpioRaw,
  ) -> Self {
    return InternalAdc { 
      vin_measure: vin_measure.into_floating_input(&mut gpio_raw.gpiob.crl), 
      channel1: channel1.into_floating_input(&mut gpio_raw.gpioa.crl), 
      channel2: channel2.into_floating_input(&mut gpio_raw.gpioc.crl), 
      channel3: channel3.into_floating_input(&mut gpio_raw.gpioc.crl), 
      channel4: channel4.into_floating_input(&mut gpio_raw.gpioc.crl), 
      channel5: channel5.into_floating_input(&mut gpio_raw.gpioc.crl)
    }
  }
}

pub struct RgbLed { 
  pub red: stm32f1xx_hal::gpio::Pin<'A', 8, Output<OpenDrain>>,
  pub green: stm32f1xx_hal::gpio::Pin<'A', 9, Output<OpenDrain>>,
  pub blue: stm32f1xx_hal::gpio::Pin<'A', 10, Output<OpenDrain>>
}

impl RgbLed {
  pub fn build(
    red: stm32f1xx_hal::gpio::Pin<'A', 8>,
    green: stm32f1xx_hal::gpio::Pin<'A', 9>,
    blue: stm32f1xx_hal::gpio::Pin<'A', 10>,
    gpio_raw: &mut GpioRaw,
  ) -> Self {
    return RgbLed { 
      red: red.into_open_drain_output_with_state(&mut gpio_raw.gpioa.crh, PinState::High), 
      green: green.into_open_drain_output_with_state(&mut gpio_raw.gpioa.crh, PinState::High), 
      blue: blue.into_open_drain_output_with_state(&mut gpio_raw.gpioa.crh, PinState::High), 
    }
  }
}


// pub external_adc_reset: stm32f1xx_hal::gpio::Pin<'A', 4>,
// pub sd_card_chip_select: stm32f1xx_hal::gpio::Pin<'C', 8>,

pub struct OscillatorControl {
  pub enable_hse: stm32f1xx_hal::gpio::Pin<'C', 13, Output>
}

impl OscillatorControl {
  pub fn build (
    enable_hse: stm32f1xx_hal::gpio::Pin<'C', 13>,
    gpio_raw: &mut GpioRaw,
  ) -> Self {
    Self {
      enable_hse: enable_hse.into_push_pull_output(&mut gpio_raw.gpioc.crh)
    }
  }
}

pub struct Button {
  pub button: stm32f1xx_hal::gpio::Pin<'A', 8, PullUp>,
}


pub struct FreedDebugPins {
  pub pb3: stm32f1xx_hal::gpio::Pin<'B', 3>,
  pub pb4: stm32f1xx_hal::gpio::Pin<'B', 4>,
  pub pa15: stm32f1xx_hal::gpio::Pin<'A', 15>,
}

impl FreedDebugPins {
  pub fn build( pins: (
    stm32f1xx_hal::gpio::Pin<'A', 15, Input<Floating>>, 
    stm32f1xx_hal::gpio::Pin<'B', 3, Input<Floating>>,
    stm32f1xx_hal::gpio::Pin<'B', 4, Input<Floating>>,
    ) ) -> Self {
      FreedDebugPins { pb3: pins.1, pb4: pins.2, pa15: pins.0 }
  }
}

pub struct GpioRaw {
  pub gpioa: stm32f1xx_hal::gpio::gpioa::Parts, 
  pub gpiob: stm32f1xx_hal::gpio::gpiob::Parts, 
  pub gpioc: stm32f1xx_hal::gpio::gpioc::Parts,
  pub gpiod: stm32f1xx_hal::gpio::gpiod::Parts,
  pub freed_debug_pins: FreedDebugPins,
}

pub struct Pins {
  pub internal_adc1: stm32f1xx_hal::gpio::Pin<'A', 0>,
  pub enable_avdd: stm32f1xx_hal::gpio::Pin<'A', 1>,
  pub tx: stm32f1xx_hal::gpio::Pin<'A', 2>,
  pub rx: stm32f1xx_hal::gpio::Pin<'A', 3>,
  pub external_adc_reset: stm32f1xx_hal::gpio::Pin<'A', 4>,
  pub spi1_sck: stm32f1xx_hal::gpio::Pin<'A', 5>,
  pub spi1_mosi: stm32f1xx_hal::gpio::Pin<'A', 6>,
  pub spi1_miso: stm32f1xx_hal::gpio::Pin<'A', 7>,
  pub rgb_red_and_wake_button: stm32f1xx_hal::gpio::Pin<'A', 8>,    // multi-use pin
  pub rgb_green_and_spi2_chip_select: stm32f1xx_hal::gpio::Pin<'A', 9>,  // multi-use pin
  pub rgb_blue: stm32f1xx_hal::gpio::Pin<'A', 10>,
  pub usb_n: stm32f1xx_hal::gpio::Pin<'A', 11>,
  pub usb_p: stm32f1xx_hal::gpio::Pin<'A', 12>,
  // unused: stm32f1xx_hal::gpio::Pin<'A', 13>, // swdio
  // unused: stm32f1xx_hal::gpio::Pin<'A', 14>, // swclk
  // unused: stm32f1xx_hal::gpio::Pin<'A', 15>,

  pub vin_measure: stm32f1xx_hal::gpio::Pin<'B', 0>,
  pub enable_vin_measure: stm32f1xx_hal::gpio::Pin<'B', 1>,
  //unused: stm32f1xx_hal::gpio::Pin<'B', 2>, // boot1
  pub gpio4: stm32f1xx_hal::gpio::Pin<'B', 3>,
  pub gpio3: stm32f1xx_hal::gpio::Pin<'B', 4>,
  pub gpio2: stm32f1xx_hal::gpio::Pin<'B', 5>,
  pub i2c1_scl: stm32f1xx_hal::gpio::Pin<'B', 6>,
  pub i2c1_sda: stm32f1xx_hal::gpio::Pin<'B', 7>,
  pub gpio1: stm32f1xx_hal::gpio::Pin<'B', 8>,
  // unused: tm32f1xx_hal::gpio::Pin<'B', 9>,
  pub i2c2_scl: stm32f1xx_hal::gpio::Pin<'B', 10>,
  pub i2c2_sda: stm32f1xx_hal::gpio::Pin<'B', 11>,
  pub enable_5v: stm32f1xx_hal::gpio::Pin<'B', 12>,
  pub spi2_sck: stm32f1xx_hal::gpio::Pin<'B', 13>,
  pub spi2_mosi: stm32f1xx_hal::gpio::Pin<'B', 14>,
  pub spi2_miso: stm32f1xx_hal::gpio::Pin<'B', 15>,

  pub int_adc5: stm32f1xx_hal::gpio::Pin<'C', 0>,
  pub int_adc4: stm32f1xx_hal::gpio::Pin<'C', 1>,
  pub int_adc3: stm32f1xx_hal::gpio::Pin<'C', 2>,
  pub int_adc2: stm32f1xx_hal::gpio::Pin<'C', 3>,
  // unused: stm32f1xx_hal::gpio::Pin<'C', 4>,
  pub enable_3v: stm32f1xx_hal::gpio::Pin<'C', 5>,
  pub enable_external_adc: stm32f1xx_hal::gpio::Pin<'C', 6>,
  // rgb_red_and_wake_duplicate: stm32f1xx_hal::gpio::Pin<'C', 7>,  // trace accidentally duplicated on PCB
  pub sd_card_chip_select: stm32f1xx_hal::gpio::Pin<'C', 8>,
  // unused: stm32f1xx_hal::gpio::Pin<'C', 9>,
  pub gpio8: stm32f1xx_hal::gpio::Pin<'C', 10>,
  pub gpio7: stm32f1xx_hal::gpio::Pin<'C', 11>,
  pub gpio6: stm32f1xx_hal::gpio::Pin<'C', 12>,
  pub enable_hse: stm32f1xx_hal::gpio::Pin<'C', 13>,  // high speed external oscillator on/off
  // unused: stm32f1xx_hal::gpio::Pin<'C', 14>,
  // unused: stm32f1xx_hal::gpio::Pin<'C', 15>,

  // unused: stm32f1xx_hal::gpio::Pin<'D', 0>, // osc in
  // unused: stm32f1xx_hal::gpio::Pin<'D', 1>, // osc out
  pub gpio5: stm32f1xx_hal::gpio::Pin<'D', 2>,

}

impl Pins {
  pub fn build(
      gpio_raw: GpioRaw,
  ) -> Self {

      return Pins { 
          internal_adc1: gpio_raw.gpioa.pa0,
          enable_avdd: gpio_raw.gpioa.pa1,
          tx: gpio_raw.gpioa.pa2, 
          rx: gpio_raw.gpioa.pa3,
          external_adc_reset: gpio_raw.gpioa.pa4,
          spi1_sck: gpio_raw.gpioa.pa5,
          spi1_mosi: gpio_raw.gpioa.pa6,
          spi1_miso: gpio_raw.gpioa.pa7,
          rgb_red_and_wake_button: gpio_raw.gpioa.pa8,
          // wake_button: gpio_raw.gpioa.pa8, // multi-use pin
          rgb_green_and_spi2_chip_select: gpio_raw.gpioa.pa9,
          //spi2_chip_select: gpio_raw.gpioa.pa9, // multi-use pin
          rgb_blue: gpio_raw.gpioa.pa10,
          usb_n: gpio_raw.gpioa.pa11,
          usb_p: gpio_raw.gpioa.pa12,

          vin_measure: gpio_raw.gpiob.pb0,
          enable_vin_measure: gpio_raw.gpiob.pb1,
          gpio4: gpio_raw.freed_debug_pins.pb3,
          gpio3: gpio_raw.freed_debug_pins.pb4,
          gpio2: gpio_raw.gpiob.pb5,
          i2c1_scl: gpio_raw.gpiob.pb6,
          i2c1_sda: gpio_raw.gpiob.pb7,
          gpio1: gpio_raw.gpiob.pb8,
          i2c2_scl: gpio_raw.gpiob.pb10,
          i2c2_sda: gpio_raw.gpiob.pb11,
          enable_5v: gpio_raw.gpiob.pb12,
          spi2_sck: gpio_raw.gpiob.pb13,
          spi2_mosi: gpio_raw.gpiob.pb14,
          spi2_miso: gpio_raw.gpiob.pb15,

          int_adc5: gpio_raw.gpioc.pc0,
          int_adc4: gpio_raw.gpioc.pc1,
          int_adc3: gpio_raw.gpioc.pc2,
          int_adc2: gpio_raw.gpioc.pc3,
          enable_3v: gpio_raw.gpioc.pc5,
          enable_external_adc: gpio_raw.gpioc.pc6,
          sd_card_chip_select: gpio_raw.gpioc.pc8,
          gpio8: gpio_raw.gpioc.pc10,
          gpio7: gpio_raw.gpioc.pc11,
          gpio6: gpio_raw.gpioc.pc12,
          enable_hse: gpio_raw.gpioc.pc13,

          gpio5: gpio_raw.gpiod.pd2,




      }
  }

}
