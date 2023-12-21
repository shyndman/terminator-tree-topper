//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#[allow(dead_code)]
extern crate alloc;

use anyhow::Result;
use defmt::{assert, *};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    adc, bind_interrupts,
    clocks::ClockConfig,
    config,
    gpio::{self, Input, Level, Output, Pull},
    peripherals::{PIO0, UART1},
    pio::{self},
    uart::{self, BufferedInterruptHandler, BufferedUart},
};
use embassy_sync::{
    blocking_mutex::raw::NoopRawMutex, mutex::Mutex, pubsub::PubSubChannel, signal::Signal,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use futures::{pin_mut, prelude::*};
use panic_probe as _;
use static_cell::make_static;
use t800::{
    stepper::{
        motor_constants::NEMA11_11HS18_0674S_CONSTANTS,
        ramp_generator::RampGenerator,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
    stream::channel_to_stream,
    uart::bus::UartDevice,
};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO =>adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type LifecycleChangedSignal = Signal<NoopRawMutex, LifecycleStage>;
#[derive(Clone, Copy, Format, PartialEq)]
#[allow(dead_code)]
enum LifecycleStage {
    Startup,
    SteppersConfigured,
    Homed,
}

type MotorLoadSignal = Signal<NoopRawMutex, u16>;

type MotorCommandChannel = PubSubChannel<NoopRawMutex, MotorCommand, 3, 10, 3>;
#[derive(Clone, Format, PartialEq)]
#[allow(dead_code)]
enum MotorCommand {
    RequestVelocity(i32),
    SetStallThreshold(u16),
    StallDetected,
    LogStatus,
    SignalLoad,
    ChannelTime(Instant),
}

struct LifeStage {
    current: Mutex<NoopRawMutex, LifecycleStage>,
    signal: LifecycleChangedSignal,
}
impl LifeStage {
    fn new() -> Self {
        Self {
            current: Mutex::new(LifecycleStage::Startup),
            signal: LifecycleChangedSignal::new(),
        }
    }

    async fn get(&self) -> LifecycleStage {
        *self.current.lock().await
    }

    async fn set(&self, stage: LifecycleStage) {
        assert!(self.get().await != stage, "Stage is already {}", stage);

        info!("Setting stage to {}", stage);

        {
            let mut current = self.current.lock().await;
            *current = stage;
        }
        self.signal.signal(stage);
    }

    async fn wait_for(&self, stage: LifecycleStage) {
        if self.get().await == stage {
            return;
        }

        loop {
            if self.signal.wait().await == stage {
                return;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    // Wait a few seconds on startup so we always have a chance to flash
    Timer::after(Duration::from_secs(3)).await;

    let life_stage: &'static mut LifeStage = make_static!(LifeStage::new());

    let mut motor_load_signal: &'static mut MotorLoadSignal =
        make_static!(MotorLoadSignal::new());

    let motor_commands: &'static mut MotorCommandChannel =
        make_static!(MotorCommandChannel::new());

    spawner
        .spawn(watch_for_stepper_interrupts(
            motor_commands,
            p.PIN_17.into(),
            p.PIN_18.into(),
        ))
        .unwrap();

    spawner
        .spawn(manage_steppers(
            life_stage,
            motor_commands,
            motor_load_signal,
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
            p.PIN_16.into(),
        ))
        .unwrap();

    // let command_pub = motor_commands
    //     .publisher()
    //     .expect("Could not create motor command publisher");

    // Wait until steppers are configured
    life_stage
        .wait_for(LifecycleStage::SteppersConfigured)
        .await;

    // Home the steppers
    home_steppers(motor_commands, motor_load_signal)
        .await
        .expect("Failed to home stpepers");

    // Test channel time
    // command_pub
    //     .publish(MotorCommand::ChannelTime(Instant::now()))
    //     .await;

    // Count the number of steps from home to the extent
    // command_pub.publish(MotorCommand::CountSteps).await;

    loop {
        Timer::after(Duration::from_secs(60 * 60 * 24)).await;
    }
}

#[embassy_executor::task]
async fn manage_steppers(
    life_stage: &'static LifeStage,
    motor_commands: &'static MotorCommandChannel,
    motor_load_signal: &'static MotorLoadSignal,
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
    // vactual.set(HOMING_VELOCITY);
    pan_driver.write_register(vactual).await.unwrap();

    info!("Listening to motor command stream");
    let command_channel_stream = channel_to_stream(motor_commands);
    let periodic_status_logs =
        Ticker::every(Duration::from_secs(5)).map(|_| MotorCommand::LogStatus);
    let events = stream::select(command_channel_stream, periodic_status_logs);
    pin_mut!(events);

    life_stage.set(LifecycleStage::SteppersConfigured).await;

    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        async fn set_velocity(
            d: &mut Tmc2209UartConnection<NoopRawMutex, UART1>,
            val: i32,
        ) -> &mut Tmc2209UartConnection<NoopRawMutex, UART1> {
            d.write_register({
                let mut r = tmc2209::reg::VACTUAL::default();
                r.set(val);
                r
            })
            .await
            .expect("Failed to write VACTUAL");
            d
        }

        match event {
            MotorCommand::ChannelTime(sent_ts) => {
                info!(
                    "Command took {}us to begin executing",
                    (Instant::now() - sent_ts).as_micros()
                );
            }

            MotorCommand::RequestVelocity(vactual) => {
                set_velocity(&mut pan_driver, vactual).await;
            }

            MotorCommand::SetStallThreshold(threshold) => {
                pan_driver
                    .write_register({
                        let mut reg = tmc2209::reg::SGTHRS::default();
                        reg.0 = threshold as u32;
                        reg
                    })
                    .await
                    .expect("Failed to write stall threshold");
            }

            MotorCommand::LogStatus => {
                fn log_status(
                    label: &'static str,
                    status_reg: tmc2209::reg::DRV_STATUS,
                    step_reg: tmc2209::reg::MSCNT,
                ) {
                    println!("{} status:\n  microstep: {}\n  stealth?: {}\n  standstill?: {}\n  current: {}\n  overtemperature? {}\n  pre-overtemperature? {}\n  open load A: {}\n  open load B: {}",
                        label,
                        step_reg.get(),
                        status_reg.stealth(),
                        status_reg.stst(),
                        status_reg.cs_actual(),
                        status_reg.ot(),
                        status_reg.otpw(),
                        status_reg.ola(),
                        status_reg.olb());
                }

                log_status(
                    "PAN",
                    pan_driver
                        .read_register::<tmc2209::reg::DRV_STATUS>()
                        .await
                        .unwrap(),
                    pan_driver
                        .read_register::<tmc2209::reg::MSCNT>()
                        .await
                        .unwrap(),
                );
            }
            // Used to detect threshold
            MotorCommand::SignalLoad => {
                let load = pan_driver
                    .read_register::<tmc2209::reg::SG_RESULT>()
                    .await
                    .unwrap();
                debug!("Motor load: {}", load.get());
                motor_load_signal.signal(load.get());
            }
            _ => {}
        }
    }
}

const HOMING_VELOCITY: i32 = -8_000;
const HOMING_ACCELERATION: u32 = 300;
const HOME_MANEUVERING_VELOCITY: i32 = -12000;
const HOMING_STALL_THRESHOLD: u16 = 10;
const RUNTIME_STALL_THRESHOLD: u16 = 0xff;

async fn home_steppers(
    motor_commands: &'static MotorCommandChannel,
    motor_load_signal: &'static MotorLoadSignal,
) -> Result<()> {
    info!("Homing rotation");

    let cmd = motor_commands
        .publisher()
        .expect("Failed to build publisher");

    // Rotate counterclockwise for an amount of time that guarantees we hit the endstop
    cmd.publish(MotorCommand::RequestVelocity(-HOME_MANEUVERING_VELOCITY))
        .await;
    Timer::after(Duration::from_millis(1800)).await;

    cmd.publish(MotorCommand::RequestVelocity(0)).await;
    Timer::after(Duration::from_millis(300)).await;

    // Set stall threshold and begin rotating clockwise
    let mut homing_ramp = RampGenerator::new(1.8, 256, HOMING_ACCELERATION);
    homing_ramp.set_target_speed(HOMING_VELOCITY);

    loop {
        let (v, _) = homing_ramp.next().await;
        cmd.publish(MotorCommand::RequestVelocity(v)).await;

        if v == HOMING_VELOCITY {
            break;
        }
    }
    info!("Up to speed, watching for stall");

    cmd.publish(MotorCommand::SetStallThreshold(HOMING_STALL_THRESHOLD))
        .await;

    // Wait until the next stall
    motor_load_signal.reset();

    while motor_load_signal.wait().await > HOMING_STALL_THRESHOLD * 2 {}

    info!("STALL!");

    cmd.publish(MotorCommand::RequestVelocity(0)).await;
    cmd.publish(MotorCommand::SetStallThreshold(RUNTIME_STALL_THRESHOLD))
        .await;

    info!("HOMED!");

    return Ok(());
}

// MotorCommand::CountSteps => {
//     info!("Counting steps");

//     let mut index_count = 0;
//     set_velocity(&mut pan_driver, 1000).await;

//     let mut step_events = events
//         .by_ref()
//         .filter(|e| future::ready(*e == MotorCommand::SignalLoad));

//     while let Some(_) = step_events.next().await {
//         index_count += 1;
//         info!("steps: {}", index_count);
//     }
// }

#[embassy_executor::task]
async fn watch_for_stepper_interrupts(
    motor_commands: &'static MotorCommandChannel,
    step_pin: gpio::AnyPin,
    diag_pin: gpio::AnyPin,
) {
    enum WatchPinEvent {
        Diag,
        Step,
    }

    let step_pulse_stream =
        stream::unfold(Input::new(step_pin, Pull::Down), |mut input| async {
            input.wait_for_rising_edge().await;
            Some((WatchPinEvent::Step, input))
        });
    let diag_interrupt_stream =
        stream::unfold(Input::new(diag_pin, Pull::Down), |mut input| async {
            input.wait_for_rising_edge().await;
            Some((WatchPinEvent::Diag, input))
        });

    let events = stream::select(step_pulse_stream, diag_interrupt_stream);
    pin_mut!(events);
    let command_pub = motor_commands
        .publisher()
        .expect("Failed to create motor command publisher");

    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        match event {
            WatchPinEvent::Diag => {
                debug!("DIAG interrupt");
                // TODO(shyndman): This pin going high can indicate quite a bit more than
                // detection of a stall. We should handle these other states.
                command_pub.publish(MotorCommand::StallDetected).await;
            }
            WatchPinEvent::Step => {
                debug!("STEP interrupt");
                command_pub.publish(MotorCommand::SignalLoad).await;
            }
        }
    }
}
