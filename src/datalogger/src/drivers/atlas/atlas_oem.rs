// Constants for Atlas device types
pub const UNKNOWN_DEVICE: u8 = 0;
pub const EC_OEM_DEVICE: u8 = 4;
pub const PH_OEM_DEVICE: u8 = 1;
pub const DO_OEM_DEVICE: u8 = 3;
pub const ORP_OEM_DEVICE: u8 = 2;

pub const UNKNOWN_VERSION: u8 = 0;

// Constants for interrupt control
pub const DISABLED_INTERRUPT: u8 = 0;
pub const HIGH_ON_INTERRUPT: u8 = 2;
pub const LOW_ON_INTERRUPT: u8 = 4;
pub const CHANGE_ON_INTERRUPT: u8 = 8;

// Constants for address unlock
pub const ADDRESS_UNLOCK_A: u8 = 0x55;
pub const ADDRESS_UNLOCK_B: u8 = 0xAA;

// Constants for EC_OEM register data
pub const DEVICE_TYPE: u8 = 0x00;
pub const FIRMWARE_VERSION: u8 = 0x01;
pub const ADDR_LOCK: u8 = 0x02;
pub const NEW_ADDR_REGISTER: u8 = 0x03;
pub const INT_CTRL: u8 = 0x04;
pub const LED_CTRL: u8 = 0x05;
pub const SLEEP_CTRL: u8 = 0x06;
pub const DATA_AVAILABLE: u8 = 0x07;

// NOP instruction - in Rust we'd typically use a different approach for delays
// For embedded Rust, you might use cortex_m::asm::nop() or similar

// Data handler union equivalent - Rust doesn't have unions in the same way as C,
// but we can use a struct with methods to handle different interpretations
pub struct DataHandler {
    bytes: [u8; 4],
}

impl DataHandler {
    pub fn new() -> Self {
        DataHandler { bytes: [0; 4] }
    }
    
    pub fn as_bytes_mut(&mut self) -> &mut [u8; 4] {
        &mut self.bytes
    }
    
    pub fn as_unsigned_long(&self) -> u32 {
        u32::from_le_bytes(self.bytes)
    }
    
    pub fn as_unsigned_int(&self) -> u16 {
        u16::from_le_bytes([self.bytes[0], self.bytes[1]])
    }
    
    pub fn set_from_float(&mut self, value: f32) {
        let scaled = (value * 100.0) as u32;
        self.bytes = scaled.to_le_bytes();
    }
    
    pub fn to_float(&self) -> f32 {
        self.as_unsigned_long() as f32 / 100.0
    }
}

// Parameter structs
pub struct ParamOemEc {
    pub salinity: f32,
    pub conductivity: f32,
    pub tds: f32,
}

pub struct ParamOemDo {
    pub in_milligrams: f32,
    pub in_saturation: f32,
}

pub struct ParamOemDoCompensation {
    pub temperature: f32,
    pub pressure: f32,
    pub salinity: f32,
}

// Device address finder function
// Note: This would need to be adapted to whatever I2C library you're using in Rust
pub fn get_address_device<T: Into<u8>>(device_type: T) -> u8 {
    let device_type_byte = device_type.into();
    
    if device_type_byte < 1 || device_type_byte > 4 {
        return UNKNOWN_DEVICE;
    }
    
    // In Rust, you would typically use an I2C library here
    // This is a placeholder implementation
    for addr in 1..128 {
        // Pseudocode for I2C operations - replace with actual I2C library calls
        /*
        if let Ok(_) = i2c.write(addr, &[DEVICE_TYPE]) {
            if let Ok(data) = i2c.read(addr, 1) {
                if data[0] == device_type_byte {
                    return addr;
                }
            }
        }
        */
    }
    
    UNKNOWN_DEVICE
}