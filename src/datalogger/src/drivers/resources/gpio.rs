
pub struct GpioRequest {
    gpio1 : bool,
    gpio2 : bool,
    gpio3 : bool,
    gpio4 : bool,
    gpio5 : bool,
    gpio6 : bool,
    gpio7 : bool,
    gpio8 : bool,
}


#[macro_export]
macro_rules! check_gpio {
    ($self:ident, $gpio:ident, $request:ident) => {
        if $self.$gpio && $request.$gpio {
            return Err(concat!(stringify!($gpio), " already requested"))
        } else if $request.$gpio {
            $self.$gpio = true;
        }
    }
}

macro_rules! release_gpio {
    ($self:ident, $gpio:ident, $request:ident) => {
        if $self.$gpio && $request.$gpio {
            $self.$gpio = false;
        }
    }
}


#[macro_export]
macro_rules! use_pin {
    ($pin:expr, $pin_value:ident, $self:ident, $gpio:ident, $mode:ident) => {
        if $pin_value == $pin {
            $self.$gpio = true;        }
    };
}

impl GpioRequest {
    pub fn none() -> GpioRequest {
        GpioRequest { 
            gpio1: false, 
            gpio2: false, 
            gpio3: false, 
            gpio4: false, 
            gpio5: false, 
            gpio6: false, 
            gpio7: false, 
            gpio8: false, 
        }
    }

    pub fn differs(&mut self, request: GpioRequest) -> bool {
        if self.gpio1 != request.gpio1 { return true }
        if self.gpio2 != request.gpio2 { return true }
        if self.gpio3 != request.gpio3 { return true }
        if self.gpio4 != request.gpio4 { return true }
        if self.gpio5 != request.gpio5 { return true }
        if self.gpio6 != request.gpio6 { return true }
        if self.gpio7 != request.gpio7 { return true }
        if self.gpio8 != request.gpio8 { return true }
        return false;
    }

    pub fn update_or_conflict(&mut self, request: GpioRequest) -> Result<(), &'static str>{
        check_gpio!(self, gpio1, request);
        check_gpio!(self, gpio2, request);
        check_gpio!(self, gpio3, request);
        check_gpio!(self, gpio4, request);
        check_gpio!(self, gpio5, request);
        check_gpio!(self, gpio6, request);
        check_gpio!(self, gpio7, request);
        check_gpio!(self, gpio8, request);
        Ok(())
    }

    pub fn use_pin(&mut self, pin: u8){
        use_pin!(1, pin, self, gpio1, mode1);
        use_pin!(2, pin, self, gpio2, mode2);
        use_pin!(3, pin, self, gpio3, mode3);
        use_pin!(4, pin, self, gpio4, mode4);
        use_pin!(5, pin, self, gpio5, mode5);
        use_pin!(6, pin, self, gpio6, mode6);
        use_pin!(7, pin, self, gpio7, mode7);
        use_pin!(8, pin, self, gpio8, mode8);       
    }

    pub fn release(&mut self, request: GpioRequest) {
        release_gpio!(self, gpio1, request);
        release_gpio!(self, gpio2, request);
        release_gpio!(self, gpio3, request);
        release_gpio!(self, gpio4, request);
        release_gpio!(self, gpio5, request);
        release_gpio!(self, gpio6, request);
        release_gpio!(self, gpio7, request);
        release_gpio!(self, gpio8, request);
    }
}
