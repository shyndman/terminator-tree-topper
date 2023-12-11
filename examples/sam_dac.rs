//! Attempts to generate software automated mouth PCM data on the Pi Pico
//
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use dasp::{sample::U12, Sample};
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
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Ticker};
use futures_util::{pin_mut, select_biased, FutureExt};
use panic_probe as _;
use rustsam::{parser, reciter, renderer};
use static_cell::make_static;
use t800 as _;
use t800::{self as _, dac::ad5626::Ad5626};

const SAMPLE_HZ: f64 = 24050.0;
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

    let sample_gen_channel: &'static mut Channel<NoopRawMutex, u8, 1> =
        make_static!(Channel::new());
    let (sample_gen_sender, sample_gen_receiver) =
        (sample_gen_channel.sender(), sample_gen_channel.receiver());

    loop {
        // Wait for a button press to begin
        println!("WAIT FOR PRESS");
        start_button.wait_for_rising_edge().await;

        println!("Preparing speech");

        let text = "Ho Ho Ho Merry X mas.";
        let phrase = reciter::text_to_phonemes(text).expect("Could not recite text");
        let phonemes = parser::parse_phonemes(&phrase).expect("Could not parse phonemes");
        let render_task =
            renderer::render(&phonemes, 50, 190, 190, 55, false, sample_gen_sender).fuse();
        pin_mut!(render_task);

        println!("CLEARING DAC");
        dac.clear().await;

        println!("RUNNING SAMPLE GEN LOOP");

        let mut sample_ticker = Ticker::every(SAMPLE_RATE);
        let ts = Instant::now();
        let mut count = 0;
        loop {
            let sample = select_biased! {
                sample = sample_gen_receiver.receive().fuse() => sample,
                _ = render_task => break,
                complete => break,
            };

            let sample_u12 = sample.to_sample::<U12>();

            count += 1;

            trace!("DATA {}", Debug2Format(&sample_u12),);

            // Wait until the next sample's start time
            sample_ticker.next().await;

            // Instruct the DAC to convert the data in its input register
            dac.write(sample_u12).await;
        }

        println!("VOICE UTTERANCE COMPLETE");
        let elapsed = Instant::now() - ts;
        println!(
            "elapsed: {}s, rate was {} /s",
            elapsed.as_micros() as f32 / 1_000_000.0,
            1_000_000.0 * count as f32 / elapsed.as_micros() as f32
        );
    }
}
