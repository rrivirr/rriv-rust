use std::{thread, time::Duration};

// Led Driver struct
pub struct LedDriver {
    gpio_pin: u8, // GPIO pin controlling the LED
}

impl LedDriver {
    pub fn new(gpio_pin: u8) -> Self {
        LedDriver { gpio_pin }
    }

    // Method to turn the LED on
    pub fn turn_on(&self) {
        println!("LED on GPIO {} is ON", self.gpio_pin);
    }

    // Method to turn the LED off
    pub fn turn_off(&self) {
        println!("LED on GPIO {} is OFF", self.gpio_pin);
    }

    // Method to run the loop that controls the LED
    pub fn run(&self) {
        loop {
            // Turn the LED on
            self.turn_on();
            // Wait for 30 seconds
            thread::sleep(Duration::from_secs(30));

            // Turn the LED off
            self.turn_off();
            // Wait for 240 seconds
            thread::sleep(Duration::from_secs(240));
        }
    }
}

fn main() {
    // Create a new LedDriver instance for a GPIO pin.
    let led_driver = LedDriver::new(17);

    // Run the driver loop
    led_driver.run();
}
