use embedded_hal::blocking::i2c::{Write, WriteRead};
use core::marker::PhantomData;
use core::time::Duration;

use super::atlas_oem::*;


// Constants from AtlasOEM_basic.h (already converted in previous example)
// Additional constants from EC_OEM.h
pub const NONE_INT: u8 = 255;
pub const EC_I2C_ID: u8 = 0x64;

pub const ON_EC: bool = true;
pub const OFF_EC: bool = false;

pub const EC_HIBERNATES: bool = true;
pub const EC_ACTIVES: bool = false;

// Calibration constants
pub const NO_CALIBRATION: u8 = 0;
pub const DRY_CALIBRATION: u8 = 1;
pub const SINGLE_POINT_CALIBRATION: u8 = 2;
pub const LOW_POINT_CALIBRATION: u8 = 3;
pub const HIGH_POINT_CALIBRATION: u8 = 4;

// Register addresses
pub const PROBE_TYPE: u8 = 0x08;
pub const CALIB_REG: u8 = 0x0A;
pub const CALIB_REQUEST: u8 = 0x0E;
pub const CALIB_CONFIRM: u8 = 0x0F;
pub const TEMP_COMPEN: u8 = 0x10;
pub const TEMP_CONFIRM: u8 = 0x14;
pub const EC_READ_REG: u8 = 0x18;
pub const TDS_READ_REG: u8 = 0x1C;
pub const PSS_READ_REG: u8 = 0x20;

// Parameter struct
pub struct ParamOemEc {
    pub salinity: f32,
    pub conductivity: f32,
    pub tds: f32,
}

impl Default for ParamOemEc {
    fn default() -> Self {
        ParamOemEc {
            salinity: 0.0,
            conductivity: 0.0,
            tds: 0.0,
        }
    }
}

// Main EC_OEM struct
pub struct EcOem {
    one_byte_read: u8,
    two_byte_read: u8,
    four_byte_read: u8,
    device_addr: u8,
    int_pin: Option<u8>,
    move_data: DataHandler,
    param: ParamOemEc,
    new_reading_available: bool,
    hibernation_status: bool,
    status_presence: bool,
    firm_version: u8,
    int_control: u8,
    type_device: u8,
    // SalinityStability would need to be implemented separately
    // salinity_stability: StabilityDetector,
}

impl<I2C, E> EcOem
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(int_pin: Option<u8>, device_addr: u8) -> Self {
        EcOem {
            device_addr,
            int_pin,
            move_data: DataHandler::new(),
            param: ParamOemEc::default(),
            new_reading_available: false,
            hibernation_status: false,
            status_presence: false,
            firm_version: UNKNOWN_VERSION,
            int_control: 0,
            type_device: UNKNOWN_DEVICE,
            one_byte_read: 0x01,
            two_byte_read: 0x02,
            four_byte_read: todo!(),
            // salinity_stability: StabilityDetector::new(0.02),
        }
    }

    // Delay function
    pub fn delay_for_millis(&self, timeout: u32) {
        // Implementation depends on your hardware platform
        // Typically you'd use a hardware timer or RTOS delay
    }

    // I2C operations
    //
    // TODO: should we pull from the board functions for I2C here, or graph the I2C and replace it with a NONE ?
    //
    //
    fn i2c_write_long<I2C, E>(&mut self, i2c: &mut I2C, reg: u8, data: u32) -> Result<(), E> {
        let bytes = data.to_be_bytes();
        i2c.write(self.device_addr, &[reg, bytes[3], bytes[2], bytes[1], bytes[0]])
    }

    fn i2c_write_byte<I2C, E>(&mut self, i2c: &mut I2C, reg: u8, data: u8) -> Result<(), E> {
        i2c.write(self.device_addr, &[reg, data])
    }

    fn i2c_read<I2C, E>(&mut self, i2c: &mut I2C, reg: u8, count: usize) -> Result<(), E> {
        let mut buf = [0u8; 4];
        i2c.write_read(self.device_addr, &[reg], &mut buf[..count])?;
        
        // Store in reverse order to match C++ implementation
        for i in 0..count {
            self.move_data.as_bytes_mut()[count - 1 - i] = buf[i];
        }
        
        Ok(())
    }

    // Public methods
    // TODO: need to pass i2c to everything
    pub fn init<I2C, E>(&mut self, i2c: &mut I2C, led: bool, hibernate: bool, int_ctrl: u8) -> Result<(), E> {
        // Check device presence
        if self.i2c_write_byte(i2c, DEVICE_TYPE, 0).is_ok() {
            if self.get_device_type()? == EC_OEM_DEVICE {
                self.status_presence = true;
            } else {
                // Implement get_address_device if needed
                self.status_presence = false;
                return Ok(());
            }
        }

        if self.status_presence {
            if led { self.set_led_on(true)?; }
            if hibernate {
                if !self.is_hibernate()? { self.set_hibernate()?; }
            } else {
                if self.is_hibernate()? { self.wake_up()?; }
            }
            self.set_interrupt_available(int_ctrl)?;
            self.clear_new_data_register()?;
        }
        
        Ok(())
    }

    pub fn get_device_type<I2C, E>(&mut self, i2c: &mut I2C) -> Result<u8, E> {
        self.type_device = UNKNOWN_DEVICE;
        self.i2c_read(DEVICE_TYPE, 1)?;
        self.type_device = self.move_data.as_bytes_mut()[0];
        Ok(self.type_device)
    }

    pub fn get_firmware_version<I2C, E>(&mut self, i2c: &mut I2C) -> Result<u8, E> {
        self.firm_version = UNKNOWN_VERSION;
        self.i2c_read(FIRMWARE_VERSION, 1)?;
        self.firm_version = self.move_data.as_bytes_mut()[0];
        Ok(self.firm_version)
    }

    pub fn is_locked_address<I2C, E>(&mut self, i2c: &mut I2C) -> Result<bool, E> {
        self.i2c_read(ADDR_LOCK, 1)?;
        Ok(self.move_data.as_bytes_mut()[0] > 0)
    }

    pub fn set_locked_address<I2C, E>(&mut self, i2c: &mut I2C, lock: bool) -> Result<bool, E> {
        if lock {
            self.i2c_write_byte(ADDR_LOCK, 0x00)?;
            self.i2c_read(ADDR_LOCK, 1)?;
            Ok(self.move_data.as_bytes_mut()[0] > 0)
        } else {
            self.i2c_write_byte(ADDR_LOCK, ADDRESS_UNLOCK_A)?;
            self.i2c_write_byte(ADDR_LOCK, ADDRESS_UNLOCK_B)?;
            self.i2c_read(ADDR_LOCK, 1)?;
            Ok(self.move_data.as_bytes_mut()[0] == 0)
        }
    }

    pub fn set_device_address<I2C, E>(&mut self, i2c: &mut I2C, new_address: u8) -> bool{
        if new_address < 1 || new_address > 127 { return false };

        i2c.read()
    }
    

    // Additional methods would follow the same pattern...
    // Implementations for:
    // - set_device_address
    // - is_interrupt_available
    // - set_interrupt_available
    // - is_led_on
    // - set_led_on
    // - is_hibernate
    // - set_hibernate
    // - wake_up
    // - is_new_data_available
    // - clear_new_data_register
    // - get_probe_type
    // - set_probe_type
    // - get_status_calibration
    // - clear_calibration_data
    // - set_calibration
    // - get_temp_compensation_value
    // - set_temp_compensation
    // - single_reading
    // - get_salinity
    // - get_conductivity
    // - get_tds
    // - get_all_param

    // Example of one of the more complex methods:
    pub fn single_reading(&mut self) -> Result<bool, E> {
        if self.is_hibernate()? {
            self.wake_up()?;
            if self.is_hibernate()? {
                return Ok(false);
            }
        }

        if self.is_new_data_available()? {
            self.i2c_read(EC_READ_REG, 4)?;
            self.param.conductivity = self.move_data.to_float();

            self.i2c_read(PSS_READ_REG, 4)?;
            self.param.salinity = self.move_data.to_float();
            // self.salinity_stability.push_to_buffer(self.param.salinity);

            self.i2c_read(TDS_READ_REG, 4)?;
            self.param.tds = self.move_data.to_float();

            self.clear_new_data_register()?;
            Ok(true)
        } else {
            Ok(false)
        }
    }
}