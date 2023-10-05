#![cfg_attr(not(test), no_std)]
extern crate alloc;
use alloc::boxed::Box;

pub trait RXProcessor: Send + Sync {
    fn process_character(&'static self, character: u8);
}

pub trait RRIVBoard: Send + Sync {
    fn setup(&mut self);
    fn set_rx_processor(&mut self, processor: Box<&'static dyn RXProcessor>);
    fn serial_send(&self, string: &str);
    fn critical_section<T, F>(&self, f: F) -> T where F: Fn() -> T;
}
