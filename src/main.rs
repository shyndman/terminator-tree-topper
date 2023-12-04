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
    adc, bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    peripherals::{DMA_CH0, PIN_16, PIN_17, PIO0, UART1},
    pio::{self, Pio},
    uart::{self, BufferedInterruptHandler, BufferedUart},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, RawMutex},
    mutex::Mutex,
    pubsub::PubSubChannel,
};
use embassy_time::{Duration, Ticker, Timer};
use futures::{pin_mut, prelude::*};
use panic_probe as _;
use rgb::RGB8;
use static_cell::make_static;
use t800::{
    led::{Ws2812Chain, Ws2812FrameProvider},
    stepper::{
        motor_constants::{
            MINI_GM15BY_VSM1527_100_10D_CONSTANTS, NEMA11_11HS18_0674S_CONSTANTS,
        },
        tune::tune_driver,
        uart::{Tmc2209UartConnection, UART_BAUD_RATE},
    },
    uart::bus::UartDevice,
};
use tmc2209::reg::SG_RESULT;
extern crate alloc;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO =>adc::InterruptHandler;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

type SystemEventChannel = PubSubChannel<CriticalSectionRawMutex, SystemEvent, 3, 10, 3>;
#[derive(Clone, Format)]
enum SystemEvent {
    PanStepperReady,
    EyeLedsReady,
}

type MotorCommandChannel = PubSubChannel<CriticalSectionRawMutex, MotorCommand, 3, 10, 3>;
#[derive(Clone, Format)]
enum MotorCommand {
    Home,
    TogglePower,
    LogStatus,
    RequestVelocity(i8),
    CheckStallLoad,
    StallDetected,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Start");
    let p = embassy_rp::init(Default::default());
    let pio0 = Pio::new(p.PIO0, Irqs);

    let mut led = Output::new(p.PIN_25, Level::High);
    led.set_high();

    Timer::after(Duration::from_secs(1)).await;

    let system_event_channel: &mut SystemEventChannel =
        make_static!(SystemEventChannel::new());
    let motor_command_channel: &mut MotorCommandChannel =
        make_static!(MotorCommandChannel::new());

    // spawner
    //     .spawn(manage_eye_leds(pio0, p.DMA_CH0, p.PIN_16))
    //     .unwrap();

    let mut adc = adc::Adc::new(p.ADC, Irqs, adc::Config::default());
    let mut speed_adc_channel = adc::Channel::new_pin(p.PIN_26, Pull::None);

    spawner
        .spawn(process_velocity_input(
            motor_command_channel,
            adc,
            speed_adc_channel,
        ))
        .unwrap();

    spawner
        .spawn(watch_for_pan_stepper_interrupts(
            motor_command_channel,
            p.PIN_5,
            p.PIN_6,
            p.PIN_17,
        ))
        .unwrap();

    spawner
        .spawn(manage_steppers(
            motor_command_channel,
            system_event_channel,
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

const SPEED_INPUT_SAMPLING_RATE: Duration = Duration::from_hz(4);

#[embassy_executor::task]
async fn process_velocity_input(
    motor_command_channel: &'static MotorCommandChannel,
    mut adc: adc::Adc<'static, adc::Async>,
    mut adc_channel: adc::Channel<'static>,
) {
    const MAX_VALUE: u16 = 1 << 12;
    let mut ticker = Ticker::every(SPEED_INPUT_SAMPLING_RATE);

    let mut last_requested_vel = 0;
    let command_pub = motor_command_channel.publisher().unwrap();
    loop {
        // A value between -15 and 16 representing the user's requested velocity
        let input_vel: i8 =
            ((MAX_VALUE - adc.read(&mut adc_channel).await.unwrap() >> 7) - 15) as i8;

        if input_vel != last_requested_vel {
            println!("new velocity request: {}", input_vel);

            command_pub
                .publish(MotorCommand::RequestVelocity(input_vel))
                .await;
            last_requested_vel = input_vel;
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn watch_for_pan_stepper_interrupts(
    motor_command_channel: &'static MotorCommandChannel,
    step_pin: embassy_rp::peripherals::PIN_5,
    diag_pin: embassy_rp::peripherals::PIN_6,
    power_button: PIN_17,
) {
    enum WatchPinEvent {
        Diag,
        Step,
        TogglePower,
    }

    let power_button_stream =
        stream::unfold(Input::new(power_button, Pull::Down), |mut input| async {
            input.wait_for_rising_edge().await;
            Some((WatchPinEvent::TogglePower, input))
        });
    let diag_interrupt_stream =
        stream::unfold(Input::new(diag_pin, Pull::Down), |mut input| async {
            input.wait_for_rising_edge().await;
            Some((WatchPinEvent::Diag, input))
        });
    let step_pulse_stream =
        stream::unfold(Input::new(step_pin, Pull::Down), |mut input| async {
            input.wait_for_rising_edge().await;
            Some((WatchPinEvent::Step, input))
        });

    let events = stream::select(
        step_pulse_stream,
        stream::select(power_button_stream, diag_interrupt_stream),
    );
    pin_mut!(events);

    let command_pub = motor_command_channel.publisher().unwrap();
    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        match event {
            WatchPinEvent::Diag => {
                // TODO(shyndman): This pin going high can indicate quite a bit more than
                // detection of a stall. We should handle these other states.
                command_pub.publish(MotorCommand::StallDetected).await
            }
            WatchPinEvent::Step => command_pub.publish(MotorCommand::CheckStallLoad).await,
            WatchPinEvent::TogglePower => {
                command_pub.publish(MotorCommand::TogglePower).await
            }
        }
    }
}

#[embassy_executor::task]
async fn manage_steppers(
    motor_command_channel: &'static MotorCommandChannel,
    system_event_channel: &'static SystemEventChannel,
    uart1: BufferedUart<'static, embassy_rp::peripherals::UART1>,
    enable_pin: embassy_rp::peripherals::PIN_7,
) {
    info!("Starting stepper management");
    info!("Creating UART1 bus");
    let uart1_bus: &'static Mutex<
        CriticalSectionRawMutex,
        BufferedUart<'static, embassy_rp::peripherals::UART1>,
    > = make_static!({ Mutex::<CriticalSectionRawMutex, _>::new(uart1) });

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

    info!("Creating connection to eye stepper driver");
    let mut eye_driver = Tmc2209UartConnection::connect(UartDevice::new(uart1_bus), 0x01)
        .await
        .unwrap();
    info!("...tuning");
    tune_driver(&mut eye_driver, MINI_GM15BY_VSM1527_100_10D_CONSTANTS)
        .await
        .unwrap();

    info!("Re-enabling power stage");
    enable_power_stage_pin.set_low();

    info!("Listening to motor command stream");
    let command_channel_stream = channel_to_stream(motor_command_channel);
    let periodic_status_logs =
        Ticker::every(Duration::from_secs(5)).map(|_| MotorCommand::LogStatus);
    let events = stream::select(command_channel_stream, periodic_status_logs);
    pin_mut!(events);

    // We don't write this value to the register until a ChangeDirection command is received
    let mut pan_vactual = tmc2209::reg::VACTUAL::default();

    let eye_vactual = tmc2209::reg::VACTUAL { 0: 120000 };
    eye_driver.write_register(eye_vactual).await.unwrap();

    // Indicate to the system that we are now processing motor commands
    system_event_channel
        .publisher()
        .unwrap()
        .publish(SystemEvent::PanStepperReady)
        .await;

    loop {
        let event = match events.next().await {
            Some(e) => e,
            None => break,
        };

        match event {
            MotorCommand::Home => {
                println!("Home");
            }
            MotorCommand::TogglePower => {
                println!("Toggle power");
                pan_vactual.set(pan_vactual.get() * -1);
                pan_driver.write_register(pan_vactual).await.unwrap();
            }
            MotorCommand::CheckStallLoad => {
                let stall_load = pan_driver.read_register::<SG_RESULT>().await.unwrap();
                println!("stall: {}", stall_load.get());
            }
            MotorCommand::StallDetected => {
                println!("STALLLLLLL!");
            }
            MotorCommand::RequestVelocity(velocity) => {
                pan_vactual.set(velocity as i32 * 2800);
                pan_driver.write_register(pan_vactual).await.unwrap();
            }
            MotorCommand::LogStatus => {
                let log_status =
                    |label: &'static str,
                     status_reg: tmc2209::reg::DRV_STATUS,
                     step_reg: tmc2209::reg::MSCNT| {
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
                    };

                // log_status(
                //     "PAN",
                //     pan_driver
                //         .read_register::<tmc2209::reg::DRV_STATUS>()
                //         .await
                //         .unwrap(),
                //     pan_driver
                //         .read_register::<tmc2209::reg::MSCNT>()
                //         .await
                //         .unwrap(),
                // );
                log_status(
                    "EYE",
                    eye_driver
                        .read_register::<tmc2209::reg::DRV_STATUS>()
                        .await
                        .unwrap(),
                    eye_driver
                        .read_register::<tmc2209::reg::MSCNT>()
                        .await
                        .unwrap(),
                );
            }
        }
    }
}

const RED_EYE_COLOR: RGB8 = RGB8::new(0xD6, 0x00, 0x1C);
const GREEN_EYE_COLOR: RGB8 = RGB8::new(0x00, 0x87, 0x3E);

#[embassy_executor::task]
async fn manage_eye_leds(pio0: Pio<'static, PIO0>, dma0: DMA_CH0, led_data_pin: PIN_16) -> ! {
    info!("Starting eye LEDs");

    // Number of LEDs in the string
    const LED_N: usize = 1;
    let mut eye_frames = EyeLightFrameSource::<LED_N>::new(RED_EYE_COLOR);

    let Pio {
        mut common, sm0, ..
    } = pio0;
    let mut eye_led_chain =
        Ws2812Chain::<PIO0, 0, LED_N>::new(&mut common, sm0, dma0, led_data_pin, &eye_frames)
            .await;
    eye_led_chain.brightness = 2;

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

// fn construct_display_event_stream(
//     velocity_command_channel: &'static VelocityCommandChannel,
//     velocity_state_channel: &'static VelocityStateChannel,
// ) -> impl Stream<Item = DisplayEvent> {
//     let command_sub_stream =
//         channel_to_stream(velocity_command_channel).map(|v| DisplayEvent::TargetVelocity {
//             degrees_per_second: v.degrees_per_second,
//         });
//     let state_sub_stream =
//         channel_to_stream(velocity_state_channel).map(|v| DisplayEvent::CurrentVelocity {
//             degrees_per_second: v.degrees_per_second,
//         });

//     futures::stream::select(command_sub_stream, state_sub_stream)
// }

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
