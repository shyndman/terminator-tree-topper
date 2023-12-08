//! Attempts to generate software automated mouth PCM data on the Pi Pico
//!
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use core::f32::consts::PI;

use defmt::{println, *};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::{self, ClockConfig},
    config,
    gpio::{self, Input},
    peripherals::PIO0,
    pio::{self, Pio},
};
use embassy_time::{Duration, Instant, Ticker};
use micromath::F32Ext;
use panic_probe as _;
use t800::{
    self as _,
    dac::ad5626::{u12, Ad5626},
    debug::StopWatch,
};

const SAMPLE_RATE: Duration = Duration::from_hz(20_050);

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");

    let peripherals = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let mut start_button = Input::new(peripherals.PIN_15, gpio::Pull::Down);

    let Pio {
        mut common, sm0, ..
    } = Pio::new(peripherals.PIO0, Irqs);
    let mut dac = Ad5626::new(
        &mut common,
        sm0,
        peripherals.DMA_CH0,
        peripherals.PIN_19,
        peripherals.PIN_18,
        peripherals.PIN_16,
        peripherals.PIN_20,
    )
    .await;

    println!("START A440");
    const A440_HZ: f32 = 440.0;
    const US_PER_WAVE: f32 = 1_000_000.0 / A440_HZ;

    // Wait for a button press to begin
    println!("WAIT FOR PRESS");
    start_button.wait_for_rising_edge().await;

    let mut sample_ticker = Ticker::every(SAMPLE_RATE);
    let start_ts = Instant::now();

    #[inline]
    fn elapsed_us_to_radians(us: f32) -> f32 {
        (us / US_PER_WAVE) * 2.0 * PI
    }

    loop {
        // Determine the next sample to output
        let elapsed_wave_us = (Instant::now() - start_ts).as_micros() as f32 % US_PER_WAVE;
        let elapsed_radians = elapsed_us_to_radians(elapsed_wave_us);

        let amplitude = elapsed_radians.sin();
        let sample_u8 = (((amplitude + 1.0) / 2.0) * 255.0) as u8;
        let sample_u12 = u12::new((sample_u8 as u16) << 4);

        // println!("DATA {}\n     {}", amplitude, sample_u12);

        // Wait until the next sample's start time
        sample_ticker.next().await;

        // Instruct the DAC to convert the data in its input register

        dac.write(sample_u12).await;
    }
}
