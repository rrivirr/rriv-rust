use crate::*;
use stm32f1xx_hal::{adc};
use stm32f1xx_hal::pac::ADC1;


pub struct InternalAdc {
  pins: pin_groups::InternalAdcPins,
  adc_device: Option<ADC1>,
  adc: Option<adc::Adc<ADC1>>,
}

impl InternalAdc {
  pub fn new(pins: pin_groups::InternalAdcPins, adc1: ADC1) -> Self {

    // adc.save_cfg()

    return InternalAdc {
      pins,
      adc_device: Some(adc1),
      adc: None
    }
  }

  pub fn enable(&mut self, clocks: &Clocks) {

    let adc1 = self.adc_device.unwrap(); // take ownership
    self.adc_device = None;
    self.adc = Some(adc::Adc::adc1(adc1, *clocks));

  }

  pub fn disable(&mut self) {

    let adc_device = self.adc.unwrap().release();
    self.adc_device = Some(adc_device)

  }

  pub fn read(&mut self, channel: u8) -> u16 {

    // let channels[&mut ] = [
    //   &mut self.pins.channel1,
    //   &mut self.pins.channel2
    // ];

    // if let Some(adc) = self.adc {
    //   let res: Result<u16, nb::Error<()>> = adc.read(channels[channel + 1]);
    // }
    // return 0;

    if let Some(adc) = self.adc {
      let res: Result<u16, nb::Error<()>> = match channel{
        1 => adc.read(&mut self.pins.channel1),
        2 => adc.read(&mut self.pins.channel2),
        3 => adc.read(&mut self.pins.channel3),
        4 => adc.read(&mut self.pins.channel4),
        5 => adc.read(&mut self.pins.channel5),
      }
      match res {

      } 
    } else {
      return 0;
    }

    // what to do if we get an error?
    // what to return in this case?
  }

}

// trait Read {
//   fn read_chan(&mut self) -> i32;
// }

// impl Read for adc::Adc<pac::ADC1> {

//   fn read_chan(&mut self) -> i32 {
//       /// According to section 5.3.18 "Temperature sensor characteristics"
//       /// from STM32F1xx datasheets, TS constants values are as follows:
//       ///   AVG_SLOPE - average slope
//       ///   V_25 - temperature sensor ADC voltage at 25°C
//       const AVG_SLOPE: i32 = 43;
//       const V_25: i32 = 1430;

//       let prev_cfg = self.save_cfg();

//       // recommended ADC sampling for temperature sensor is 17.1 usec,
//       // so use the following approximate settings
//       // to support all ADC frequencies

//       // let sample_time = match self.clocks.adcclk().raw() {
//       //     0..=1_200_000 => SampleTime::T_1,
//       //     1_200_001..=1_500_000 => SampleTime::T_7,
//       //     1_500_001..=2_400_000 => SampleTime::T_13,
//       //     2_400_001..=3_100_000 => SampleTime::T_28,
//       //     3_100_001..=4_000_000 => SampleTime::T_41,
//       //     4_000_001..=5_000_000 => SampleTime::T_55,
//       //     5_000_001..=14_000_000 => SampleTime::T_71,
//       //     _ => SampleTime::T_239,
//       // };

//       // self.set_sample_time(sample_time);

//       let val_temp: i32 = self.read_aux(16u8).into();
//       let val_vref: i32 = self.read_aux(17u8).into();
//       let v_sense = val_temp * 1200 / val_vref;

//       self.restore_cfg(prev_cfg);

//       (V_25 - v_sense) * 10 / AVG_SLOPE + 25
//   }
// }