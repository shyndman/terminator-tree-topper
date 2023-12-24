//! Attempts to generate software automated mouth PCM data on the Pi Pico
//!
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::ClockConfig,
    config,
    peripherals::{DMA_CH0, PIN_15, PIO0, PIO1},
    pio::{self, Pio},
};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use rand::prelude::*;
use rgb::RGB8;
use t800::{
    self as _,
    led::{Ws2812Chain, Ws2812FrameProvider},
};

bind_interrupts!(struct Irqs {
    PIO1_IRQ_0 => pio::InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");

    let p = embassy_rp::init(config::Config::default());
    let pio0 = Pio::new(p.PIO1, Irqs);

    spawner
        .spawn(manage_eye_leds(pio0, p.DMA_CH0, p.PIN_15))
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(60 * 60)).await;
    }
}

const WHITE_EYE_COLOR: RGB8 = RGB8::new(0xFF, 0xFF, 0xFF);
const RED_EYE_COLOR: RGB8 = RGB8::new(0xD6, 0x00, 0x1C);
const GREEN_EYE_COLOR: RGB8 = RGB8::new(0x00, 0x87, 0x3E);
const EYE_COLORS: [RGB8; 3] = [WHITE_EYE_COLOR, RED_EYE_COLOR, GREEN_EYE_COLOR];
const LED_N: usize = 2;

#[embassy_executor::task]
async fn manage_eye_leds(pio0: Pio<'static, PIO1>, dma0: DMA_CH0, led_data_pin: PIN_15) {
    info!("Starting eye LEDs");

    // Number of LEDs in the string
    let mut eye_frames = EyeLightFrameSource::new(SmallRng::seed_from_u64(0xdeadbee7));

    let Pio {
        mut common, sm0, ..
    } = pio0;
    let mut eye_led_chain =
        Ws2812Chain::<PIO1, 0, LED_N>::new(&mut common, sm0, dma0, led_data_pin, &eye_frames)
            .await;
    eye_led_chain.brightness = 0x60;

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        eye_frames.advance();
        eye_led_chain.draw(&eye_frames).await;
        ticker.next().await;
    }
}

struct EyeLightFrameSource {
    rng: SmallRng,
    eye_colors: [RGB8; LED_N],
}
impl EyeLightFrameSource {
    pub fn new(rng: SmallRng) -> Self {
        Self {
            rng,
            eye_colors: [RED_EYE_COLOR, RED_EYE_COLOR],
        }
    }
    pub fn advance(&mut self) {
        self.eye_colors = [
            *EYE_COLORS.choose(&mut self.rng).unwrap(),
            *EYE_COLORS.choose(&mut self.rng).unwrap(),
        ];
    }
}
impl Ws2812FrameProvider<LED_N> for EyeLightFrameSource {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        self.eye_colors
    }
}
