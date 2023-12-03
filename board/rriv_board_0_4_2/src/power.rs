
struct Power {
  pins: pin_groups::Power
}

impl Power {
  pub fn new(pins: pin_groups::Power) -> Self {
    return Power {
      pins
    }
  }

  pub fn cycle_3v(board: &Board) {
    pins.enable_3v.set_high();
    board.delay_ms(250_u16);
    pins.enable_3v.set_low();
    board.delay_ms(250_u16);
    pins.enable_3v.set_low();
    board.delay_ms(250_u16);
  }
}