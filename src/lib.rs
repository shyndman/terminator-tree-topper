#![no_std]
#![no_main]
#![feature(
    asm_experimental_arch,
    async_closure,
    const_mut_refs,
    const_trait_impl,
    coroutines,
    error_in_core,
    exclusive_range_pattern,
    try_blocks,
    type_alias_impl_trait,
    proc_macro_hygiene,
    stmt_expr_attributes
)]
#![allow(internal_features)]
#![allow(incomplete_features)]
#![allow(unused)]

extern crate alloc;
use defmt_rtt as _;
use panic_probe as _;

mod _global_alloc;

pub mod dac;
pub mod debug;
pub mod gpio;
pub mod led;
pub mod stepper;
pub mod time;
pub mod uart;
