//! Driver testing example for the Arducam HM0360

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::{debug, println};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::{self, ClockConfig},
    config,
    gpio::Input,
    i2c,
    peripherals::{I2C1, PIO0},
    pio::{self, Pio},
    pwm,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};
use panic_probe as _;
use static_cell::make_static;
use t800 as _;
use t800::camera::hm0360::Hm0360;

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let i2c1 = i2c::I2c::new_async(p.I2C1, p.PIN_15, p.PIN_14, Irqs, {
        let mut c = i2c::Config::default();
        // c.frequency = 200_000;
        c
    });
    let i2c1_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        i2c::I2c<'static, I2C1, i2c::Async>,
    > = make_static!({
        Mutex::<CriticalSectionRawMutex, i2c::I2c<'static, I2C1, i2c::Async>>::new(i2c1)
    });

    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let mut cam = Hm0360::new(
        I2cDevice::new(i2c1_bus),
        &mut common,
        sm0,
        p.DMA_CH2,
        p.PIN_11, // D0
        p.PIN_12, // HSYNC
        p.PIN_13, // PCLK
        p.PIN_10, // VSYNC
        p.PIN_9,  // RESET
    )
    .await;

    let mut long_tick = Ticker::every(Duration::from_secs(2));
    loop {
        let image_bytes = cam.capture_frame().await;
        println!("{:#x}", image_bytes);
        long_tick.next().await;
    }
}
