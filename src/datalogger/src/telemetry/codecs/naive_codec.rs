
use rtt_target::rprintln;

use alloc::boxed::Box;


pub fn encode( timestamp: i64, 
               values: &[f32]
             ) -> Box<[u8]>{
    
    rprintln!("encode {} {}", timestamp, values[0]);
    // rprintln!("{} values", values.len());
    let mut bytes: [u8; 22] = [0; 22];
    // let timestamp_bytes = timestamp.to_le_bytes();
    // rprintln!("{:X?}", timestamp_bytes);
    let timestamp_bytes = timestamp.to_be_bytes();
    rprintln!("{:X?}", timestamp_bytes);
    bytes[0..8].copy_from_slice(&timestamp_bytes);
    for i in 0..values.len(){
      let value = (values[i] * 100.0) as u32;
      let value_bytes = value.to_be_bytes();
      rprintln!("{:?}", (i * 4 + 8)..(i * 4 + 12));
      bytes[(i * 4 + 8)..(i * 4 + 12)].copy_from_slice(&value_bytes);
      rprintln!("{:X?}", value_bytes);
      if i == 2 { break }; // send up to 3 values
    }
    
    rprintln!("{:X?}", bytes);
    return Box::new(bytes);
}