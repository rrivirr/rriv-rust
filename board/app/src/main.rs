#![cfg_attr(not(test), no_std)]
//! This app tests the static rriv board lib

#![allow(clippy::empty_loop)]
#![feature(alloc_error_handler)]
#![feature(prelude_2024)]
#![no_main]

// use core::panic::PanicInfo;
// use core::ptr::null_mut;
use cortex_m_rt::entry;
// use rtt_target::{rprintln, rtt_init_print};

use rtt_target::{rprintln, rtt_init_print};
extern crate alloc;
use alloc::{alloc::Layout, boxed::Box};
use core::{cell::RefCell, default::Default, ffi::c_void, option::{Option, Option::*}, result::Result::*, mem::MaybeUninit, ops::DerefMut, u8, prelude::rust_2024::*, *};
// use alloc::{alloc::Layout, boxed::Box};
use embedded_alloc::Heap;
use panic_halt as _;

extern crate rriv_0_4;
use rriv_0_4::Board;

extern crate rriv_datalogger;
use rriv_datalogger::DataLogger;


// #[panic_handler]
// fn panic(_: &PanicInfo) -> ! {
//     loop {}
// }

// #[link(name = "rriv_0_4", kind = "static")]
// extern "C" {
//     pub fn rust_serial_interface_new() -> *mut c_void;
// }

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 128;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

// Initialize the allocator BEFORE you use it
fn alloc_heap() {
    {
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}


#[entry]
fn main() -> ! {

    alloc_heap();
    // Initialize the allocator
    rtt_init_print!();

    let board = Board::new();
    let mut datalogger = DataLogger::new(board);
    loop {
        datalogger.run_loop_iteration();
    }

}

//// I think this is for the services crate..
// use core::ffi::{c_void, CStr};
// use serde_json::{json, Value};

// fn handle_serial_command(serial_command: &str) {
//     // Parse the JSON string into a serde_json::Value
//     let command: Value = serde_json::from_str(serial_command).unwrap();

//     // Extract the command, object, and parameters from the JSON object
//     let action_str = command["command"].as_str().unwrap();
//     let object_str = command["object"].as_str().unwrap();
//     let command_str = format!("{}_{}", action_str, object_str)
//         .to_cstr()
//         .to_lowercase()
//         .as_cstr();
//     let parameters = command
//         .as_object()
//         .unwrap()
//         .iter()
//         .filter(|(k, _)| k != "command" && k != "object")
//         .map(|(k, v)| (k.to_cstr(), v.to_cstr()))
//         .collect();

//     // join the command and object strings with and underscore
//     // match the command string to the corresponding Command enum
//     let command = match command_str {
//         "datalogger_set" => Command::DataloggerSet,
//         "datalogger_get" => Command::DataloggerGet,
//         "datalogger_reset" => Command::DataloggerReset,
//         "datalogger_set_mode" => Command::DataloggerSetMode,
//         "sensor_set" => Command::SensorSet,
//         "sensor_get" => Command::SensorGet,
//         "sensor_remove" => Command::SensorRemove,
//         "sensor_list" => Command::SensorList,
//         "sensor_calibrate_point" => Command::SensorCalibratePoint,
//         "sensor_calibrate_fit" => Command::SensorCalibrateFit,
//         "sensor_reset" => Command::SensorReset,
//         "actuator_set" => Command::ActuatorSet,
//         "actuator_get" => Command::ActuatorGet,
//         "actuator_remove" => Command::ActuatorRemove,
//         "actuator_list" => Command::ActuatorList,
//         "actuator_reset" => Command::ActuatorReset,
//         "telemeter_set" => Command::TelemeterSet,
//         "telemeter_get" => Command::TelemeterGet,
//         "telemeter_remove" => Command::TelemeterRemove,
//         "telemeter_list" => Command::TelemeterList,
//         "telemeter_reset" => Command::TelemeterReset,
//         "board_version" => Command::BoardVersion,
//         "board_firmware_warranty" => Command::BoardFirmwareWarranty,
//         "board_firmware_conditions" => Command::BoardFirmwa        alloc_heap();

//         "board_memory_check" => Command::BoardMemoryCheck,
//         "board_mcu_stop" => Command::BoardMcuStop,
//         "board_mcu_sleep" => Command::BoardMcuSleep,
//         "board_signal_ex_adc_high" => Command::BoardSignalExAdcHigh,
//         "board_signal_ex_adc_low" => Command::BoardSignalExAdcLow,
//         "board_signal_3v3_boost_high" => Command::BoardSignal3,
//     };
//     execute_command(command, parameters);
// }
