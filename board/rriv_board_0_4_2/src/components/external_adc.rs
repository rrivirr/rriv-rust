use crate::*;
use bitfield_struct::bitfield;

const ADC_I2C_ADDRESS: u8 = 0x2F;
const ADC_CONVERSION_RESULT_REGISTER_ADDRESS: u8 = 0x00;
const ADC_CHANNEL_REGISTER_ADDRESS: u8 = 0x01;
const ADC_CONFIGURATION_REGISTER_ADDRESS: u8 = 0x02;

#[bitfield(u16)]
struct ConversionResultRegister {
    #[bits(12)]
    conv_result: u16,

    #[bits(1)]
    alert: bool,

    #[bits(2)]
    channel: u8,

    #[bits(1)]
    reserved: u8,
}

impl ConversionResultRegister {
    fn from_bytes(bytes: [u8; 2]) -> ConversionResultRegister {
        let u16_value: u16 = ((bytes[1] as u16) << 8) | bytes[0] as u16;
        ConversionResultRegister::from_bits(u16_value)
    }
}

#[bitfield(u8)] // bit order is opposite on MCU vs ADC chip
struct ChannelRegister {
    #[bits(1, default = false)]
    channel_0: bool,

    #[bits(1, default = false)]
    channel_1: bool,

    #[bits(1, default = false)]
    channel_2: bool,

    #[bits(1, default = false)]
    channel_3: bool,

    #[bits(4, access = RO, default = 0)]
    reserved: u8,
}

fn build_channel_register_set_command(register: ChannelRegister) -> [u8; 2] {
    let register_bytes = unsafe { any_as_u8_slice(&register) };
    let data: [u8; 2] = [ADC_CHANNEL_REGISTER_ADDRESS, register_bytes[0]];
    return data;
}

#[bitfield(u16)]
struct ConfigurationRegister {
    #[bits(2, default = 0)]
    p_down: u8,

    #[bits(1, default = false)]
    gpo1: bool,

    #[bits(1, default = false)]
    alert_pol_or_gpo0: bool,

    #[bits(1, default = false)]
    alert_en_or_gpoo: bool,

    #[bits(1, default = false)]
    busy: bool,

    #[bits(2, default = 3)]
    cycle_timer: u8,

    #[bits(1, default = false)]
    auto: bool,

    #[bits(1, default = false)]
    swrs: bool,

    #[bits(1, default = true)]
    cmd: bool,

    #[bits(1, default = false)]
    fltr: bool,

    #[bits(2, access = RO, default = 0)]
    reserved: u8,

    #[bits(1, default = false)]
    gpo2: bool,

    #[bits(1, default = false)]
    alert_drive_type: bool,
}

fn build_configuration_register_set_command(register: ConfigurationRegister) -> [u8; 3] {
    let register_bytes = unsafe { any_as_u8_slice(&register) };
    let data: [u8; 3] = [
        ADC_CONFIGURATION_REGISTER_ADDRESS,
        register_bytes[1], // MSB (I2C is Big Ending Byte Order)
        register_bytes[0], // LSB
    ];
    return data;
}

pub struct ExternalAdc {
    pins: pin_groups::ExternalAdcPins,
}

impl ExternalAdc {
    pub fn new(pins: pin_groups::ExternalAdcPins) -> Self {
        return ExternalAdc { pins };
    }

    pub fn start(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {
        self.enable(delay);
        self.reset(delay);
    }

    pub fn reset(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {
        delay.delay_ms(1_u8); // delay > 50ns before applying ADC reset
        self.pins.reset.set_low();
        delay.delay_ms(1_u8); // delay > 10ns after starting ADC reset
        self.pins.reset.set_high();
        delay.delay_ms(100_u8); // let the chip start up
    }

    // read a single channel
    pub fn read_single_channel(&mut self, i2c: &mut BoardI2c1, channel: u8) -> u16 {
        let mut channel_register = ChannelRegister::new();
        match channel {
            1 => channel_register.set_channel_0(true),
            2 => channel_register.set_channel_1(true),
            3 => channel_register.set_channel_2(true),
            4 => channel_register.set_channel_3(true),
            _ => {}
        }

        // channel_register.set_channel_0(true);
        // channel_register.set_channel_1(true);
        // channel_register.set_channel_2(true);
        // channel_register.set_channel_3(true);
        let channel_command = build_channel_register_set_command(channel_register);
        rprintln!("set channel reg: {}", channel_command[1]);
        self.send_i2c(i2c, &channel_command); // writing to the channel register restarts the channel cycle.

        let mut buffer: [u8; 1] = [0; 1];
        self.read_i2c(i2c, ADC_CHANNEL_REGISTER_ADDRESS, &mut buffer);
        rprintln!("channel reg: {}", buffer[0]);

        // // the old code always sets the config register here.  is this necessary to make it work??
        // let configuration_register = ConfigurationRegister::new();
        // let configuration_command =
        //     build_configuration_register_set_command(configuration_register);
        // self.send_i2c(i2c, &configuration_command);

        // let mut buffer: [u8; 2] = [0; 2];
        // self.read_i2c(i2c, ADC_CONFIGURATION_REGISTER_ADDRESS, &mut buffer);
        // rprintln!("config reg: {:X?}", buffer);

        let mut value: u16 = core::u16::MAX;
        let mut buffer: [u8; 2] = [0; 2];
        self.read_i2c(i2c, ADC_CONVERSION_RESULT_REGISTER_ADDRESS, &mut buffer);
        // MSB and LSB need to be swapped in order to change to little endian
        let buffer_le: [u8; 2] = [buffer[1], buffer[0]]; //
        let conversion_result = ConversionResultRegister::from_bytes(buffer_le);
        value = conversion_result.conv_result();
        rprintln!(
            "EXADC: conversion result {} : {}   {:?}",
            conversion_result.channel(),
            value,
            buffer
        );
        value
    }

    pub fn enable(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {
        self.pins.enable.set_low();
        delay.delay_ms(250); // let the chip get power
    }

    pub fn disable(&mut self, delay: &mut impl embedded_hal::blocking::delay::DelayMs<u8>) {
        self.pins.enable.set_high();
        delay.delay_ms(250); // let the chip power down
    }

    pub fn configure(&mut self, i2c: &mut BoardI2c1) {
        let configuration_register = ConfigurationRegister::new();
        let configuration_command =
            build_configuration_register_set_command(configuration_register);
        self.send_i2c(i2c, &configuration_command);
        rprintln!("send config reg: {:X?}", configuration_command);

        let mut buffer: [u8; 2] = [0; 2];
        self.read_i2c(i2c, ADC_CONFIGURATION_REGISTER_ADDRESS, &mut buffer);
        rprintln!("config reg: {:X?}", buffer);

        let channel_register = ChannelRegister::new();
        let channel_command = build_channel_register_set_command(channel_register);
        self.send_i2c(i2c, &channel_command);

        /// TESTS BELOW... delete later
        rprintln!("TEST REGISTERs");

        let configuration_register = ConfigurationRegister::new();
        let configuration_command =
            build_configuration_register_set_command(configuration_register);
        self.send_i2c(i2c, &configuration_command);
        rprintln!(
            "send config reg: {:X?} {:08b} {:08b} {:08b}",
            configuration_command,
            configuration_command[0],
            configuration_command[1],
            configuration_command[2]
        );

        let mut buffer: [u8; 2] = [0; 2];
        self.read_i2c(i2c, ADC_CONFIGURATION_REGISTER_ADDRESS, &mut buffer);
        rprintln!(
            "config reg: {:X?} {:08b} {:08b}",
            buffer,
            buffer[0],
            buffer[1]
        ); // in this case MSB is in buffer[0]

        let mut channel_register = ChannelRegister::new();
        channel_register.set_channel_2(true);
        let channel_command = build_channel_register_set_command(channel_register);
        rprintln!(
            "write channel reg: {:X?} {:08b} {:08b}",
            channel_command,
            channel_command[0],
            channel_command[1]
        );
        self.send_i2c(i2c, &channel_command);

        let mut buffer: [u8; 1] = [0; 1];
        self.read_i2c(i2c, ADC_CHANNEL_REGISTER_ADDRESS, &mut buffer);
        rprintln!("channel reg: {:X?} {:08b}", buffer, buffer[0]);

        let mut value: u16 = core::u16::MAX;
        for i in 0..4 {
            let mut buffer: [u8; 2] = [0; 2];
            self.read_i2c(i2c, ADC_CONVERSION_RESULT_REGISTER_ADDRESS, &mut buffer);
            rprintln!(
                "read conv result: {:X?} {:08b} {:08b}",
                buffer,
                buffer[0],
                buffer[1]
            ); // in this case MSB is in buffer[0]
               // MSB and LSB need to be swapped
            let butter_le: [u8; 2] = [buffer[1], buffer[0]];
            let conversion_result = ConversionResultRegister::from_bytes(butter_le);
            value = conversion_result.conv_result();
            rprintln!(
                "conversion result {} : {}   {:?}",
                conversion_result.channel(),
                value,
                butter_le
            );
        }

        rprintln!("TEST DONE");
    }

    fn send_i2c(&mut self, i2c: &mut BoardI2c1, bytes: &[u8]) {
        rprintln!("send i2c {:X?}", bytes);
        match i2c.write(ADC_I2C_ADDRESS, bytes) {
            Ok(_) => {}
            Err(err) => rprintln!("{:?}", err),
        }
    }

    fn read_i2c(&mut self, i2c: &mut BoardI2c1, register_address: u8, buffer: &mut [u8]) {
        let address_bytes: [u8; 1] = [register_address];
        rprintln!("read i2c {:X?}", address_bytes);
        match i2c.write_read(ADC_I2C_ADDRESS, &address_bytes, buffer) {
            Ok(_) => {}
            Err(err) => rprintln!("{:?}", err),
        }
    }
}
