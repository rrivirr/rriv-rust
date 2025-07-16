use core::time;

use rtt_target::rprintln;

use alloc::boxed::Box;


pub fn encode( timestamp: i64, 
               values: &[f64]
             ) -> Box<[u8]>{

    rprintln!("{}", "codec not implemented");
    let mut bytes: [u8; 22] = [0; 22];
    let timestamp_bytes = timestamp.to_le_bytes();
    let value_bytes = values[0].to_le_bytes();
    bytes[0..7].copy_from_slice(&timestamp_bytes);
    bytes[8..16].copy_from_slice(&value_bytes);
    return Box::new(bytes);
}