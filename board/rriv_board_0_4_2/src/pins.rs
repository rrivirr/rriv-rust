use stm32f1xx_hal::gpio::Debugger;

pub struct Pins {
  pub gpioa: stm32f1xx_hal::gpio::gpioa::Parts, 
  pub gpiob: stm32f1xx_hal::gpio::gpiob::Parts, 
  pub gpioc: stm32f1xx_hal::gpio::gpioc::Parts,
  pub gpiod: stm32f1xx_hal::gpio::gpiod::Parts,

  pub int_adc1: stm32f1xx_hal::gpio::Pin<'A', 0>,
  pub enable_avdd: stm32f1xx_hal::gpio::Pin<'A', 1>,
  pub tx: stm32f1xx_hal::gpio::Pin<'A', 2>,
  pub rx: stm32f1xx_hal::gpio::Pin<'A', 3>,
  pub ex_adc_reset: stm32f1xx_hal::gpio::Pin<'A', 4>,
  pub spi1_sck: stm32f1xx_hal::gpio::Pin<'A', 5>,
  pub spi1_mosi: stm32f1xx_hal::gpio::Pin<'A', 6>,
  pub spi1_miso: stm32f1xx_hal::gpio::Pin<'A', 7>,
  pub rgb_red: stm32f1xx_hal::gpio::Pin<'A', 8>,    // multi-use pin
  // pub wake_button: &'a stm32f1xx_hal::gpio::Pin<'A', 8>,
  pub rgb_green: stm32f1xx_hal::gpio::Pin<'A', 9>,  // multi-use pin
  // pub spi2_chip_select: &'a stm32f1xx_hal::gpio::Pin<'A', 9>,
  pub rgb_blue: stm32f1xx_hal::gpio::Pin<'A', 10>,
  pub usb_n: stm32f1xx_hal::gpio::Pin<'A', 11>,
  pub usb_p: stm32f1xx_hal::gpio::Pin<'A', 12>,
  // unused: stm32f1xx_hal::gpio::Pin<'A', 13>, // swdio
  // unused: stm32f1xx_hal::gpio::Pin<'A', 14>, // swclk
  // unused: stm32f1xx_hal::gpio::Pin<'A', 15>,

  pub vin_measure: stm32f1xx_hal::gpio::Pin<'B', 0>,
  pub enable_vin_measure: stm32f1xx_hal::gpio::Pin<'B', 1>,
  //unused: stm32f1xx_hal::gpio::Pin<'B', 2>, // boot1
  pub gpio4: stm32f1xx_hal::gpio::Pin<'B', 3, Debugger>,
  pub gpio3: stm32f1xx_hal::gpio::Pin<'B', 4, Debugger>,
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
  pub enable_ex_adc: stm32f1xx_hal::gpio::Pin<'C', 6>,
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
  pub gpio5: stm32f1xx_hal::gpio::Pin<'D', 2>, // osc out

  // enable_3v_cr: &mut gpio::Cr<'C', false>

}

impl Pins {
  pub fn build(
      gpioa: stm32f1xx_hal::gpio::gpioa::Parts, 
      gpiob: stm32f1xx_hal::gpio::gpiob::Parts, 
      gpioc: stm32f1xx_hal::gpio::gpioc::Parts,
      gpiod: stm32f1xx_hal::gpio::gpiod::Parts,
  ) -> Self {


      return Pins { 
          gpioa: gpioa,
          gpiob: gpiob,
          gpioc: gpioc,
          gpiod: gpiod,
   
          int_adc1: gpioa.pa0,
          enable_avdd: gpioa.pa1,
          tx: gpioa.pa2, 
          rx: gpioa.pa3,
          ex_adc_reset: gpioa.pa4,
          spi1_sck: gpioa.pa5,
          spi1_mosi: gpioa.pa6,
          spi1_miso: gpioa.pa7,
          rgb_red: gpioa.pa8,
          // wake_button: &gpioa.pa8,
          rgb_green: gpioa.pa9,
          // spi2_chip_select: &gpioa.pa9,
          rgb_blue: gpioa.pa10,
          usb_n: gpioa.pa11,
          usb_p: gpioa.pa12,

          vin_measure: gpiob.pb0,
          enable_vin_measure: gpiob.pb1,
          gpio4: gpiob.pb3,
          gpio3: gpiob.pb4,
          gpio2: gpiob.pb5,
          i2c1_scl: gpiob.pb6,
          i2c1_sda: gpiob.pb7,
          gpio1: gpiob.pb8,
          i2c2_scl: gpiob.pb10,
          i2c2_sda: gpiob.pb11,
          enable_5v: gpiob.pb12,
          spi2_sck: gpiob.pb13,
          spi2_mosi: gpiob.pb14,
          spi2_miso: gpiob.pb15,

          int_adc5: gpioc.pc0,
          int_adc4: gpioc.pc1,
          int_adc3: gpioc.pc2,
          int_adc2: gpioc.pc3,
          enable_3v: gpioc.pc5,
          enable_ex_adc: gpioc.pc6,
          sd_card_chip_select: gpioc.pc8,
          gpio8: gpioc.pc10,
          gpio7: gpioc.pc11,
          gpio6: gpioc.pc12,
          enable_hse: gpioc.pc13,

          gpio5: gpiod.pd2,

          // enable_3v_cr: &mut gpioc.crl



      }
  }

  pub fn wake_button<'a>(self) -> &'a stm32f1xx_hal::gpio::Pin<'A', 8> {
    return & self.rgb_red;
  }

  pub fn spi2_chip_select<'a>(self) -> &'a stm32f1xx_hal::gpio::Pin<'A', 9> {
    return & self.rgb_green;
  }
}
