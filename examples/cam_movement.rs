//! Driver testing example for the Arducam HM0360

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(try_blocks)]

use anyhow::Result;
use defmt::println;
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts, clocks::ClockConfig, config, gpio::Input, i2c, peripherals::I2C1,
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Timer};
use embedded_hal_1::i2c::SevenBitAddress;
use panic_probe as _;
use static_cell::make_static;
use t800::{
    self as _,
    camera::hm0360::{
        reg::InterruptIndicator, Hm0360, BUS_ADDRESS_ALT1, BUS_ADDRESS_DEFAULT,
    },
};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

const LEFT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_DEFAULT;
const RIGHT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_ALT1;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let i2c1_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        i2c::I2c<'static, I2C1, i2c::Async>,
    > = make_static!({
        let i2c =
            i2c::I2c::new_async(p.I2C1, p.PIN_15, p.PIN_14, Irqs, i2c::Config::default());
        Mutex::<CriticalSectionRawMutex, i2c::I2c<'static, I2C1, i2c::Async>>::new(i2c)
    });

    let mut camera_interrupt = Input::new(p.PIN_8, embassy_rp::gpio::Pull::None);
    let (mut cam_left, mut cam_right) = match join(
        setup_camera(i2c1_bus, LEFT_CAM_ADDRESS),
        setup_camera(i2c1_bus, RIGHT_CAM_ADDRESS),
    )
    .await
    {
        (Ok(cl), Ok(cr)) => (cl, cr),
        _ => panic!(),
    };

    loop {
        let _: anyhow::Result<()> = try {
            camera_interrupt.wait_for_high().await;
            println!("Interrupt!!");

            if cam_left.is_motion_detected().await? || cam_right.is_motion_detected().await? {
                let _motion_map = cam_left.get_motion_map().await?;
                let _motion_map = cam_right.get_motion_map().await?;
                // println!("motion map:\n{}", motion_map);

                cam_left.clear_motion_detection().await?;
                cam_right.clear_motion_detection().await?;
                println!("...reset");
            } else {
                Timer::after(Duration::from_secs(1)).await;
            }
        };
    }
}

async fn setup_camera<M: RawMutex + 'static, B: i2c::Instance + 'static>(
    i2c_bus: &'static Mutex<M, i2c::I2c<'static, B, i2c::Async>>,
    i2c_address: SevenBitAddress,
) -> Result<Hm0360<'static, M, B, SevenBitAddress>> {
    let mut cam = Hm0360::new(I2cDevice::new(i2c_bus), i2c_address).await;
    cam.set_motion_detection_threshold(0xff / 2 + 20).await?;
    cam.enable_motion_detection().await?;
    Ok(cam)
}
