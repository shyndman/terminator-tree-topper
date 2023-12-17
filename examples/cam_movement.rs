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
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use static_cell::make_static;
use t800::{
    self as _,
    camera::hm0360::{
        reg::{InterruptClear, InterruptIndicator, RegisterAddress as Addr},
        Hm0360,
    },
};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
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

    let mut camera_interrupt = Input::new(p.PIN_8, embassy_rp::gpio::Pull::None);

    cam.set_motion_detection_threshold(0xff / 2 +10)
        .await
        .expect("Unable to set detection threshold");
    cam.enable_motion_detection()
        .await
        .expect("Unable to enable motion detection");

    loop {
        camera_interrupt.wait_for_high().await;
        println!("Interrupt!!");

        let interrupt_indicator = cam
            .read::<InterruptIndicator>()
            .await
            .expect("Failed to read IntIndic register");
        println!("{}", interrupt_indicator);

        if interrupt_indicator.motion_detected {
            let motion_map = cam
                .get_motion_map()
                .await
                .expect("Could not fetch motion bits");
            println!("motion map:\n{}", motion_map);

            cam.clear_motion_detection()
                .await
                .expect("Failed to clear motion interrupt bit");
            println!("...reset");
        } else {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}
