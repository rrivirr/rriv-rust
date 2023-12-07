use cortex_m::asm;
use stm32f1xx_hal::{timer::{Tim1NoRemap, Channel, PwmHz, Ch}, time::ms, pac::TIM1};

use crate::*;

pub fn build_rgb_led(pins: RgbLedPins, tim1: TIM1, mapr: &mut MAPR, clocks: &Clocks) -> RgbLed{

  let pins_array = (pins.red, pins.green, pins.blue);
  let mut pwm = tim1
    .pwm_hz::<Tim1NoRemap, _, _>(pins_array, mapr, 1.kHz(), clocks);

  pwm.enable(Channel::C1);
  pwm.enable(Channel::C2);
  pwm.enable(Channel::C3);

  pwm.set_period(ms(500).into_rate());
  asm::bkpt();

  let max = pwm.get_max_duty();

  RgbLed::new(pwm, max)

}

pub type RgbPwm  = PwmHz<TIM1, Tim1NoRemap, (Ch<0>, Ch<1>, Ch<2>), (Pin<'A', 8, Alternate<OpenDrain>>, Pin<'A', 9, Alternate<OpenDrain>>, Pin<'A', 10, Alternate<OpenDrain>>)>;

pub struct RgbLed {
  pwm: RgbPwm,
  max: u16,
}

impl RgbLed {
  pub fn new(pwm: RgbPwm, max: u16) -> Self {

    RgbLed {
      pwm,
      max
    }

  }

  fn convert(&self, x: u8) -> u16 {
    let x1 = self.max as u32 * (x as u32);
    let x2 = x1 / 255;
    x2 as u16
  }

  pub fn set_color(&mut self, red: u8, green: u8, blue: u8) {

    let r = self.max as u32 * (red as u32);

    self.pwm.set_duty(Channel::C1, self.convert(red) );
    self.pwm.set_duty(Channel::C2, self.convert(green) );
    self.pwm.set_duty(Channel::C3, self.convert(blue) );
    asm::bkpt();

  }

  pub fn test_colors_loop(&mut self, delay: & SysDelay) {

    let mut blue = 0;
    let mut red = 100;
    let mut green = 200;
    let factor = 2;

    loop {

      self.set_color(red, green, blue);

      blue = (blue + 1*factor) % 256;
      red = (red + 2*factor) % 256;
      green = (green + 3*factor) % 256;
      delay.delay_ms(50_u16);

    }
  }
}