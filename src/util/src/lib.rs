#![no_std]

use core::str::Utf8Error;

pub fn remove_invalid_utf8(buffer: &mut [u8]) {
    // make sure all bytes are utf8 compliant
    for i in 0..buffer.len() {
        if buffer[i] > 0x7F {
            buffer[i] = 42; // change to star character.
        }
    }
}

pub fn str_from_utf8( buffer: &mut [u8] )-> Result<&str, Utf8Error> {
    remove_invalid_utf8(buffer);
    core::str::from_utf8(buffer)
}



pub fn check_alphanumeric(array: &[u8]) -> bool {
    let checks = array.iter();
    let checks = checks.map(|x| (*x as char).is_alphanumeric());
    checks.fold(true, |acc, check| if !check || !acc { false } else { true })
}

pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts((p as *const T) as *const u8, ::core::mem::size_of::<T>())
}
