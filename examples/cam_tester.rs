//! Driver testing example for the Arducam HM0360

#![no_std]
#![no_main]
#![feature(try_blocks, type_alias_impl_trait)]
#[allow(dead_code)]
extern crate alloc;

use defmt::{self, info, println, trace};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    clocks::ClockConfig,
    config,
    gpio::{self, Input},
    i2c,
    peripherals::{I2C1, UART0},
    uart,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_1::i2c::SevenBitAddress;
use panic_probe as _;
use static_cell::make_static;
use t800::{
    self as _,
    camera::{
        self,
        hm0360::{Hm0360, BUS_ADDRESS_ALT1, BUS_ADDRESS_DEFAULT},
    },
    debug::DebugUart,
};

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

#[allow(dead_code)]
const LEFT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_DEFAULT;
#[allow(dead_code)]
const RIGHT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_ALT1;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    info!("Initializing");
    Timer::after(Duration::from_secs(1)).await;

    let i2c1_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        i2c::I2c<'static, I2C1, i2c::Async>,
    > = make_static!({
        let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_15, p.PIN_14, Irqs, {
            let mut c = i2c::Config::default();
            c.frequency = 64_000;
            c
        });
        Mutex::<CriticalSectionRawMutex, i2c::I2c<'static, I2C1, i2c::Async>>::new(i2c)
    });

    let mut cam = setup_camera(i2c1_bus, LEFT_CAM_ADDRESS, p.PIN_13)
        // let mut cam = setup_camera(i2c1_bus, RIGHT_CAM_ADDRESS, p.PIN_12)
        .await
        .unwrap();
    let mut camera_interrupt = Input::new(p.PIN_11, embassy_rp::gpio::Pull::None);

    info!("Camera ready");

    loop {
        let _: Result<(), camera::hm0360::error::ErrorKind> = try {
            camera_interrupt.wait_for_high().await;
            trace!("Interrupt!!");

            if cam.is_motion_detected().await? {
                let motion_map = cam.get_motion_map().await?;

                println!("\n{}", motion_map);

                cam.clear_motion_detection().await?;
            } else {
                Timer::after(Duration::from_secs(1)).await;
            }
        };
    }
}

async fn setup_camera<M: RawMutex + 'static, B: i2c::Instance + 'static, Reset: gpio::Pin>(
    i2c_bus: &'static Mutex<M, i2c::I2c<'static, B, i2c::Async>>,
    i2c_address: SevenBitAddress,
    reset_pin: Reset,
) -> Result<Hm0360<'static, M, B, SevenBitAddress, Reset>, camera::hm0360::error::ErrorKind> {
    let mut cam = Hm0360::new(I2cDevice::new(i2c_bus), i2c_address, reset_pin).await;
    cam.set_motion_detection_threshold(0xff / 2 + 20).await?;
    cam.enable_motion_detection().await?;
    Ok(cam)
}

#[embassy_executor::task]
pub async fn start_debug_uart(uart: uart::BufferedUart<'static, UART0>) {
    info!("Starting debug UART");

    let mut debug_uart = DebugUart::new(uart);
    let mut tick = Ticker::every(Duration::from_secs(1));
    loop {
        debug_uart.render().await.unwrap();
        tick.next().await;
    }
}
