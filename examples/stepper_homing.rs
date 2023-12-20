//! This example shows powerful PIO module in the RP2040 chip to communicate with WS2812 LED modules.
//! See (https://www.sparkfun.com/categories/tags/ws2812)

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#[allow(dead_code)]
extern crate alloc;

use defmt::*;
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
    blocking_mutex::raw::{NoopRawMutex, RawMutex},
    mutex::Mutex,
    pubsub::PubSubChannel,
};
use embassy_time::{Duration, Ticker, Timer};
use futures::{pin_mut, prelude::*};
use panic_probe as _;
use static_cell::make_static;
use t800::{
    stepper::{
        motor_constants::NEMA11_11HS18_0674S_CONSTANTS,
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
    uart::bus::UartDevice,
};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO =>adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type MotorCommandChannel = PubSubChannel<NoopRawMutex, MotorCommand, 3, 10, 3>;
#[derive(Clone, Format, PartialEq)]
#[allow(dead_code)]
enum MotorCommand {
    Home,
    RequestVelocity(i32),
    StallDetected,
    LogStatus,
    LogStallLoad,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    Timer::after(Duration::from_secs(2)).await;

    let motor_command_channel: &'static mut MotorCommandChannel =
        make_static!(MotorCommandChannel::new());

    spawner
        .spawn(watch_for_pan_stepper_interrupts(
            motor_command_channel,
            p.PIN_17.into(),
            p.PIN_18.into(),
        ))
        .unwrap();

    spawner
        .spawn(manage_steppers(
            motor_command_channel,
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

    Timer::after(Duration::from_secs(5)).await;
    let command_pub = motor_command_channel
        .publisher()
        .expect("Could not create motor command publisher");
    command_pub.publish(MotorCommand::Home).await;

    loop {
        Timer::after(Duration::from_secs(60 * 60 * 24)).await;
    }
}

#[embassy_executor::task]
async fn watch_for_pan_stepper_interrupts(
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
                command_pub.publish(MotorCommand::LogStallLoad).await;
            }
        }
    }
}

const HOMING_VELOCITY: i32 = -8000;
const HOMING_STALL_THRESHOLD: u32 = 0x03;
const RUNTIME_STALL_THRESHOLD: u32 = 0xff;

#[embassy_executor::task]
async fn manage_steppers(
    motor_commands: &'static MotorCommandChannel,
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

    info!("Listening to motor command stream");
    let command_channel_stream = channel_to_stream(motor_commands);
    let periodic_status_logs =
        Ticker::every(Duration::from_secs(5)).map(|_| MotorCommand::LogStatus);
    let events = stream::select(command_channel_stream, periodic_status_logs);
    pin_mut!(events);

    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        match event {
            MotorCommand::Home => {
                info!("Homing rotation");

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

                async fn set_sgthrs(
                    d: &mut Tmc2209UartConnection<NoopRawMutex, UART1>,
                    val: u32,
                ) -> &mut Tmc2209UartConnection<NoopRawMutex, UART1> {
                    d.write_register(tmc2209::reg::SGTHRS {
                        0: val,
                        ..tmc2209::reg::SGTHRS::default()
                    })
                    .await
                    .expect("Failed to write SGTHRS");
                    d
                }

                // Step counterclockwise in case we're already right next to the stall
                // point
                set_velocity(&mut pan_driver, -HOMING_VELOCITY).await;
                Timer::after(Duration::from_secs(1)).await;
                set_velocity(&mut pan_driver, 0).await;

                // Set stall threshold and begin rotating clockwise
                set_sgthrs(&mut pan_driver, HOMING_STALL_THRESHOLD).await;
                set_velocity(&mut pan_driver, HOMING_VELOCITY).await;

                // Wait until the first detected stall event
                events
                    .by_ref()
                    .skip_while(|e| future::ready(*e != MotorCommand::StallDetected))
                    .next()
                    .await;

                set_velocity(&mut pan_driver, 0).await;
                set_sgthrs(&mut pan_driver, RUNTIME_STALL_THRESHOLD).await;

                info!("Homed!");
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
            // MotorCommand::LogStallLoad => {
            //     let stall_load = pan_driver
            //         .read_register::<tmc2209::reg::SG_RESULT>()
            //         .await
            //         .unwrap();
            //     println!("stall load: {}", stall_load.get());
            // }
            _ => {}
        }
    }
}

pub fn channel_to_stream<
    M: RawMutex,
    T: Clone,
    const CAP: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static PubSubChannel<M, T, CAP, SUBS, PUBS>,
) -> impl Stream<Item = T> {
    stream::unfold(channel.subscriber().unwrap(), |mut sub| async {
        // TODO(shyndman): Maybe add optional logging on lagged streams?
        Some((sub.next_message_pure().await, sub))
    })
}
