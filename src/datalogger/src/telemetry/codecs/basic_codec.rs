


use rtt_target::rprintln;








// consider templating this
// can this happen?  i think it can't.
// pub fn encode<T> ( timestamp: long, 
//                values: [double; T],
//                bits: [u8; T]
//              ) -> [u8; T]{

// }

pub fn encode( timestamp: long, 
               values: &[double],
               bits: &[u8]
             ) -> Box<[u8]>{

    rprintln("{}", "debug message while running on board");
}