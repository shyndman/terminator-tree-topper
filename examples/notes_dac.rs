//! Attempts to generate software automated mouth PCM data on the Pi Pico
//!
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use dasp::{
    sample::U12,
    signal::{Phase, Step},
    Sample, Signal,
};
use defmt::{println, *};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::ClockConfig,
    config,
    gpio::{self, Input},
    peripherals::PIO0,
    pio::{self, Pio},
};
use embassy_time::{Duration, Instant, Ticker};
use panic_probe as _;
use rand::prelude::*;
use t800::{self as _, dac::ad5626::Ad5626};

const SAMPLE_HZ: f64 = 22050.0;
const SAMPLE_RATE: Duration = Duration::from_hz(SAMPLE_HZ as u64);

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
        peripherals.PIN_16,
        peripherals.PIN_17,
        peripherals.PIN_20,
        peripherals.PIN_18,
    )
    .await;

    let mut rng = embassy_rp::clocks::RoscRng;

    loop {
        // Wait for a button press to begin
        println!("WAIT FOR PRESS");
        start_button.wait_for_rising_edge().await;

        println!("CLEARING DAC");
        dac.clear().await;

        let note_hz = *NOTE_FREQUENCIES.iter().choose(&mut rng).unwrap();
        println!("NOTE {}hz", note_hz);

        let ts = Instant::now();
        let mut sample_ticker = Ticker::every(SAMPLE_RATE);
        let mut signal = native_sine(dasp::signal::rate(SAMPLE_HZ).const_hz(note_hz).phase());

        while Instant::now() - ts < Duration::from_secs(2) {
            let signal_sample = signal.next();
            let sample_u12 = signal_sample.to_sample::<U12>();

            trace!(
                "DATA {} {}",
                Debug2Format(&signal_sample),
                Debug2Format(&sample_u12)
            );

            // Wait until the next sample's start time
            sample_ticker.next().await;

            // Instruct the DAC to convert the data in its input register
            dac.write(sample_u12).await;
        }

        println!("DONE");
    }
}

pub fn native_sine<S: Step>(phase: Phase<S>) -> NativeSine<S> {
    NativeSine { phase: phase }
}

#[derive(Clone)]
pub struct NativeSine<S>
where
    S: Step,
{
    phase: Phase<S>,
}

impl<S> Signal for NativeSine<S>
where
    S: Step,
{
    type Frame = f64;

    #[inline]
    fn next(&mut self) -> Self::Frame {
        use embassy_rp::rom_data::double_funcs::dsin;

        const PI_2: f64 = core::f64::consts::PI * 2.0;
        let phase = self.phase.next_phase();
        dsin(PI_2 * phase)
    }
}

static NOTE_FREQUENCIES: [f64; 25] = [
    261.626,  // C  4
    277.183,  // C# 4
    293.665,  // D  4
    311.127,  // D# 4
    329.628,  // E  4
    349.228,  // F  4
    369.994,  // F# 4
    391.995,  // G  4
    415.305,  // G# 4
    440.0,    // A  4
    466.164,  // A# 4
    493.883,  // B  4
    523.251,  // C  5
    554.365,  // C# 5
    587.33,   // D  5
    622.254,  // D# 5
    659.255,  // E  5
    698.456,  // F  5
    739.989,  // F# 5
    783.991,  // G  5
    830.609,  // G# 5
    880.0,    // A  5
    932.328,  // A# 5
    987.767,  // B  5
    1046.502, // C  6
];
