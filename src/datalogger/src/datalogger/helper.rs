extern crate alloc;
use crate::alloc::string::{String, ToString};

pub fn get_prefix(sensor_name_bytes: &mut [u8; 6]) -> String {
    let prefix = match util::str_from_utf8(&mut sensor_name_bytes.clone()) {
        Ok(str) => (str.to_string() + "_").clone(),
        Err(_) => "".to_string(),
    };
    prefix
}
