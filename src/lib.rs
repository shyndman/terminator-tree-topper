#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    async_closure,
    const_mut_refs,
    const_trait_impl,
    custom_test_frameworks,
    error_in_core,
    exclusive_range_pattern,
    try_blocks,
    type_alias_impl_trait
)]
#![test_runner(run_tests)]
#![allow(incomplete_features)]
#![allow(unused)]

extern crate alloc;

mod _global_alloc;
pub mod stepper;
pub mod time;
pub mod uart;
pub mod ws2812;

pub fn run_tests(_: &[u8; 0]) {}
