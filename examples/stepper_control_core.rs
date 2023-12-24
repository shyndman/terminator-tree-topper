#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#[allow(dead_code)]
extern crate alloc;

use defmt::{info, unwrap};
use defmt_rtt as _;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    bind_interrupts,
    clocks::{ClockConfig, RoscRng},
    config,
    gpio::{self, AnyPin, Level, Output},
    multicore::{spawn_core1, Stack},
    peripherals::UART1,
    uart::{self, BufferedInterruptHandler, BufferedUart},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    channel::Channel,
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use fixed::prelude::ToFixed;
use panic_probe as _;
use rand::prelude::*;
use static_cell::{make_static, StaticCell};
use t800::{
    stepper::{
        motor_constants::NEMA11_11HS18_0674S_CONSTANTS,
        tmc2209::{
            motion::{
                ConstantMotionProfile,
                // ConstantMotionProfile,
                Tmc2209MotionDriver,
            },
            tune::tune_driver,
            Tmc2209UartConnection, UART_BAUD_RATE,
        },
    },
    uart::bus::UartDevice,
};

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();
static STEPPER_TARGET_MICROSTEP_CHANNEL: Channel<CriticalSectionRawMutex, u16, 1> =
    Channel::new();
static STEPPER_ON_TARGET_SIGNAL: Signal<CriticalSectionRawMutex, u16> = Signal::new();

const MICROSTEPS_PER_STEP: u16 = 256;
const MAX_MICROSTEP: u16 = MICROSTEPS_PER_STEP * 200;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    // Wait a few seconds on startup so we always have a chance to flash
    Timer::after(Duration::from_secs(3)).await;

    let rng = embassy_rp::clocks::RoscRng;

    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            unwrap!(spawner.spawn(stepper_control_core_task(
                p.PIN_3.into(),
                p.PIN_6.into(),
                p.PIN_7.into(),
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
            )))
        });
    });

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| unwrap!(spawner.spawn(target_generation_core_task(rng))));
}

#[embassy_executor::task]
async fn target_generation_core_task(mut rng: RoscRng) {
    info!("Hello from target generation core");

    loop {
        let next_target = rng.gen_range(0..MAX_MICROSTEP);
        info!("Setting target to {}", next_target);
        STEPPER_TARGET_MICROSTEP_CHANNEL.send(next_target).await;
        info!("...waiting");
        STEPPER_ON_TARGET_SIGNAL.wait().await;
    }
}

#[embassy_executor::task]
async fn stepper_control_core_task(
    enable_pin: AnyPin,
    direction_pin: AnyPin,
    step_pin: AnyPin,
    uart1: BufferedUart<'static, UART1>,
) {
    info!("START Stepper control core");

    configure_stepper(uart1, enable_pin).await;

    info!("Stepper configured");

    run_motion_loop(direction_pin, step_pin).await;

    loop {
        Timer::after(Duration::from_secs(60 * 60)).await;
    }
}

async fn configure_stepper(
    uart1: BufferedUart<'static, embassy_rp::peripherals::UART1>,
    enable_pin: gpio::AnyPin,
) {
    info!("Starting stepper management");
    info!("Creating UART1 bus");
    let uart1_bus: &'static Mutex<
        NoopRawMutex,
        BufferedUart<'static, embassy_rp::peripherals::UART1>,
    > = make_static!({ Mutex::<NoopRawMutex, _>::new(uart1) });

    info!("Disabling power stage");
    let mut enable_power_stage_pin = Output::new(enable_pin, Level::High);

    info!("Creating connection to pan stepper driver");
    let mut pan_driver = Tmc2209UartConnection::connect(UartDevice::new(uart1_bus), 0x00)
        .await
        .unwrap();

    info!("...tuning");
    tune_driver(&mut pan_driver, NEMA11_11HS18_0674S_CONSTANTS)
        .await
        .unwrap();

    info!("Re-enabling power stage");
    enable_power_stage_pin.set_low();

    let mut vactual = tmc2209::reg::VACTUAL::default();
    vactual.set(0);
    pan_driver.write_register(vactual).await.unwrap();
}

async fn run_motion_loop(direction_pin: AnyPin, step_pin: AnyPin) {
    let mut driver =
        Tmc2209MotionDriver::new(direction_pin, step_pin, ConstantMotionProfile::default());
    let velocity: fixed::types::U16F16 = (MAX_MICROSTEP / 200).to_fixed();
    let receiver = STEPPER_TARGET_MICROSTEP_CHANNEL.receiver();

    loop {
        let target_step = receiver.receive().await;
        driver
            .update_target_step(target_step, velocity)
            .expect("Failed to update target step");
    }
}
