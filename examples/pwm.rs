//! This example shows how to use PWM (Pulse Width Modulation) in the RP2040 chip.
//!
//! The LED on the RP Pico W board is connected differently. Add a LED and resistor to another pin.

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::pwm::{Config, Pwm};
use embassy_time::Timer;
use panic_probe as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let config = Default::default();
    let peripherals = embassy_rp::init(config);

    let mut pwm_config: Config = {
        let mut c = embassy_rp::pwm::Config::default();
        c.top = 0x8000;
        c.compare_b = 8;
        c
    };
    let mut pwm =
        Pwm::new_output_b(peripherals.PWM_CH7, peripherals.PIN_15, pwm_config.clone());

    loop {
        info!("current LED duty cycle: {}/32768", pwm_config.compare_b);
        Timer::after_secs(1).await;
        pwm_config.compare_b = pwm_config.compare_b.rotate_left(4);
        pwm.set_config(&pwm_config);
    }
}
