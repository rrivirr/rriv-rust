use rtt_target::rprintln;
use stm32f1xx_hal::{flash::ACR, rcc::{Clocks, CFGR}};
use crate::pin_groups::OscillatorControlPins;
use stm32f1xx_hal::prelude::*;  // need this for fugit


pub fn setup_clocks(
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

