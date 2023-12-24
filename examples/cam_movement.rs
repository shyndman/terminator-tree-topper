//! Driver testing example for the Arducam HM0360

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(try_blocks)]

extern crate alloc;

use core::ops::Index;

#[allow(unused_imports)]
use defmt::{self, debug, info, println, trace, unwrap, Format, Formatter};
use defmt_rtt as _;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::{
    bind_interrupts,
    clocks::{self, ClockConfig},
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
        hm0360::{Hm0360, MotionMap, BUS_ADDRESS_ALT1, BUS_ADDRESS_DEFAULT},
    },
    debug::DebugUart,
};

bind_interrupts!(struct Irqs {
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

const LEFT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_DEFAULT;
const RIGHT_CAM_ADDRESS: SevenBitAddress = BUS_ADDRESS_ALT1;

const UART_PERIPHERAL_CLOCK_DIVIDER: u32 = 1600;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    info!("Initializing");
    Timer::after(Duration::from_secs(1)).await;

    let uart_cfg = {
        let mut c = uart::Config::default();
        // Use a baud rate that divides more easily if we've overclocked
        c.baudrate = clocks::clk_peri_freq() / UART_PERIPHERAL_CLOCK_DIVIDER;
        c
    };
    info!("Setting debug UART baud rate to {} hz", uart_cfg.baudrate);
    let uart0 = uart::BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_0,
        p.PIN_1,
        make_static!([0u8; 32]),
        make_static!([0u8; 32]),
        uart_cfg,
    );
    unwrap!(spawner.spawn(start_debug_uart(uart0)));

    let i2c1_bus: &'static mut Mutex<
        CriticalSectionRawMutex,
        i2c::I2c<'static, I2C1, i2c::Async>,
    > = make_static!({
        let i2c =
            i2c::I2c::new_async(p.I2C1, p.PIN_11, p.PIN_10, Irqs, i2c::Config::default());
        Mutex::<CriticalSectionRawMutex, i2c::I2c<'static, I2C1, i2c::Async>>::new(i2c)
    });

    let mut camera_interrupt = Input::new(p.PIN_12, embassy_rp::gpio::Pull::None);
    let (mut cam_left, mut cam_right) = match join(
        setup_camera(i2c1_bus, LEFT_CAM_ADDRESS, p.PIN_13),
        setup_camera(i2c1_bus, RIGHT_CAM_ADDRESS, p.PIN_14),
    )
    .await
    {
        (Ok(cl), Ok(cr)) => (cl, cr),
        _ => defmt::panic!(),
    };

    loop {
        let _: Result<(), camera::hm0360::error::ErrorKind> = try {
            camera_interrupt.wait_for_high().await;
            // debug!("Interrupt!!");

            if cam_left.is_motion_detected().await? || cam_right.is_motion_detected().await? {
                let left = cam_left.get_motion_map().await?;
                let right = cam_right.get_motion_map().await?;
                let motion_map = BinocularMotionMap::new(left, right);

                println!("\n{}", motion_map);

                cam_left.clear_motion_detection().await?;
                cam_right.clear_motion_detection().await?;
                // debug!("...reset");
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

struct BinocularMotionMap<LM: MotionMap, RM: MotionMap> {
    left: LM,
    right: RM,
}

impl<LM: MotionMap, RM: MotionMap> BinocularMotionMap<LM, RM> {
    pub fn new(left: LM, right: RM) -> Self {
        assert!(left.height() == right.height());

        Self { left, right }
    }
}

impl<LM: MotionMap, RM: MotionMap> MotionMap for BinocularMotionMap<LM, RM> {
    fn width(&self) -> usize {
        self.left.width() + self.right.width()
    }

    fn height(&self) -> usize {
        self.left.height()
    }
}

impl<LM: MotionMap, RM: MotionMap> Index<(usize, usize)> for BinocularMotionMap<LM, RM> {
    type Output = bool;

    fn index(&self, (x, y): (usize, usize)) -> &Self::Output {
        if x >= self.right.width() {
            &self.left[(x - self.right.width(), y)]
        } else {
            &self.right[(x, y)]
        }
    }
}

impl<LM: MotionMap, RM: MotionMap> Format for BinocularMotionMap<LM, RM> {
    fn format(&self, fmt: Formatter) {
        MotionMap::format(self, fmt);
    }
}
