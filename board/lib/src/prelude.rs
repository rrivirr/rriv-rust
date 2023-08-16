#![allow(clippy::empty_loop)]

extern crate alloc;

use alloc::alloc::Layout;
use core::{mem::MaybeUninit, prelude::rust_2024::*, u8, *};
use embedded_alloc::Heap;

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

pub(crate) fn init() {
    // Initialize the allocator
    alloc_heap();
}
