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
    peripherals::{DMA_CH0, PIN_16, PIO0, UART1},
    pio::{InterruptHandler, Pio},
    uart::{self, BufferedInterruptHandler, BufferedUart},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use rgb::RGB8;
use static_cell::make_static;
use t800::{
    stepper::{
        motor_constants::MINI_8PM020S1_02001_CONSTANTS,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
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
        .spawn(drive_eye_steppers(
            BufferedUart::new(
                p.UART1,
                Irqs,
                p.PIN_8,
                p.PIN_9,
                &mut make_static!([0u8; 256])[..],
                &mut make_static!([0u8; 256])[..],
                {
                    let mut cfg = uart::Config::default();
                    cfg.baudrate = UART_BAUD_RATE;
                    cfg
                },
            ),
            p.PIN_7,
        ))
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
async fn drive_eye_steppers(
    uart1: BufferedUart<'static, embassy_rp::peripherals::UART1>,
    enable_pin: embassy_rp::peripherals::PIN_7,
) -> ! {
    info!("Starting stepper management");
    info!("Creating UART0 bus");
    let uart0_bus: &'static Mutex<
        CriticalSectionRawMutex,
        BufferedUart<'static, embassy_rp::peripherals::UART1>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(uart1) });

    info!("Disabling power stage");
    let mut enable_power_stage_pin = Output::new(enable_pin, Level::High);

    info!("Creating connection to X axis stepper driver");
    let mut x_driver = Tmc2209UartConnection::connect(UartDevice::new(uart0_bus), 0x00)
        .await
        .unwrap();
    info!("Tuning X axis stepper driver");
    tune_driver(&mut x_driver, MINI_8PM020S1_02001_CONSTANTS)
        .await
        .unwrap();

    // let mut y_driver = Tmc2209UartConnection::connect(UartDevice::new(uart0_bus), 0x01)
    //     .await
    //     .unwrap();
    // tune_driver(&mut y_driver, MINI_8PM020S1_02001_CONSTANTS)
    //     .await
    //     .unwrap();

    info!("Re-enabling power stage");
    enable_power_stage_pin.set_low();

    info!("Setting X axis stepper driver to static velocity");
    let mut vactual = tmc2209::reg::VACTUAL::default();
    vactual.set(1);
    x_driver.write_register(vactual).await.unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(1));
    loop {
        ticker.next().await;
    }
}
