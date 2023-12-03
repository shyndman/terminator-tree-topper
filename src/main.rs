//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(custom_test_frameworks)]
#![test_runner(t800::run_tests)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{self, DMA_CH0, PIN_16, PIO0, UART1},
    pio::{InterruptHandler, Pio},
    uart::{self, BufferedInterruptHandler, BufferedUart},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use rgb::RGB8;
use static_cell::make_static;
use t800::{
    uart::bus::UartDevice,
    ws2812::{Ws2812Chain, Ws2812FrameProvider},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());
    let pio0 = Pio::new(p.PIO0, Irqs);

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    Timer::after(Duration::from_secs(1)).await;

    spawner
        .spawn(manage_eye_leds(pio0, p.DMA_CH0, p.PIN_16))
        .unwrap();

    spawner
        .spawn(manage_eye_stepper(p.PIN_14, p.PIN_13))
        .unwrap();

    loop {
        Timer::after(Duration::from_secs(60 * 60 * 24)).await;
    }
}

#[embassy_executor::task]
async fn manage_eye_leds(pio0: Pio<'static, PIO0>, dma0: DMA_CH0, led_data_pin: PIN_16) -> ! {
    info!("Starting eye LEDs");

    // Number of LEDs in the string
    const LED_N: usize = 1;
    let mut eye_frames = EyeLightFrameSource::<LED_N>::new(RGB8::new(0xff, 0xff, 0xff));

    let Pio {
        mut common, sm0, ..
    } = pio0;
    let mut eye_led_chain =
        Ws2812Chain::<PIO0, 0, LED_N>::new(&mut common, sm0, dma0, led_data_pin, &eye_frames)
            .await;

    let mut ticker = Ticker::every(Duration::from_hz(10));
    loop {
        eye_frames.advance();
        eye_led_chain.draw(&eye_frames).await;
        ticker.next().await;
    }
}

struct EyeLightFrameSource<const LED_N: usize> {
    color: RGB8,
}
impl<const LED_N: usize> EyeLightFrameSource<LED_N> {
    pub fn new(color: RGB8) -> Self {
        Self { color }
    }
    pub fn advance(&mut self) {}
}
impl<const LED_N: usize> Ws2812FrameProvider<LED_N> for EyeLightFrameSource<LED_N> {
    fn frame_colors(&self) -> [RGB8; LED_N] {
        [self.color; LED_N]
    }
}

#[embassy_executor::task]
async fn manage_eye_stepper(
    x_step_pin: peripherals::PIN_14,
    x_direction_pin: peripherals::PIN_13,
) -> ! {
    info!("Starting stepper management");

    let mut x_step_pin = Output::new(x_step_pin, Level::Low);
    let mut x_direction_pin = Output::new(x_direction_pin, Level::Low);

    const STEP_PULSE_WIDTH: Duration = Duration::from_micros(400);
    const MAX_STEP: u16 = (360 / 18) * 9;
    let mut current_microstep: u16 = 0; // 0..320

    let mut step_ticker = Ticker::every(Duration::from_hz(420));
    loop {
        if current_microstep % 5 == 0 {
            info!("STEP ms={}", current_microstep);
        }

        x_step_pin.set_high();
        Timer::after(STEP_PULSE_WIDTH).await;

        x_step_pin.set_low();

        Timer::after(STEP_PULSE_WIDTH).await;

        current_microstep = (current_microstep + 1) % MAX_STEP;

        if current_microstep == 0 {
            info!("...reversing direction");
            x_direction_pin.toggle();
        }

        step_ticker.next().await;
    }
}
