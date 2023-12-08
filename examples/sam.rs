//! Attempts to generate software automated mouth PCM data on the Pi Pico
//!
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use defmt::{println, *};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{clocks::ClockConfig, config};
use embassy_time::Instant;
use panic_probe as _;
use rustsam::{parser, reciter, renderer};
use t800 as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");

    let _peripherals =
        embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let text = "test";
    let phrase = reciter::text_to_phonemes(text).expect("Could not recite text");
    let phonemes = parser::parse_phonemes(&phrase).expect("Could not parse phonemes");

    for (i, p) in phonemes.iter().rev().enumerate() {
        let ts = Instant::now();
        let single_phoneme = alloc::vec![p.clone()];
        let output = renderer::render(&single_phoneme, 100, 127, 127, 60, false);
        let us_elapsed = (Instant::now() - ts).as_micros();
        println!(
            "SAM phoneme[{}], sample_count={} duration={}µs ({}us/sample)",
            i,
            output.len(),
            us_elapsed,
            us_elapsed as f32 / output.len() as f32,
        );
    }
}
