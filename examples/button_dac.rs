//! Attempts to generate software automated mouth PCM data on the Pi Pico
//!
#![no_std]
#![no_main]
#![feature(allocator_api, type_alias_impl_trait)]

extern crate alloc;

use defmt::{println, *};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::{
    clocks::ClockConfig,
    config,
    gpio::{self, Input, Level, Output},
};
use embassy_time::{Duration, Ticker, Timer};
use panic_probe as _;
use rand::{prelude::*, rngs::SmallRng};
use t800 as _;

const DAC_DATA_SETUP_HOLD_DURATION: Duration = Duration::from_millis(2);
const DAC_CLOCK_LEVEL_DURATION: Duration = Duration::from_millis(1);
const DAC_SUBMIT_CHANGE_DURATION: Duration = Duration::from_millis(1);
const DAC_LOAD_PULSE_WIDTH: Duration = Duration::from_millis(1);

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Start");

    let peripherals = embassy_rp::init(config::Config::new(ClockConfig::crystal(14_400_000)));

    let mut next_button = Input::new(peripherals.PIN_15, gpio::Pull::Down);

    let mut dac_clock = Output::new(peripherals.PIN_18, Level::High);
    let mut dac_data = Output::new(peripherals.PIN_19, Level::High);
    let mut dac_submit = Output::new(peripherals.PIN_16, Level::High);
    let mut dac_clear = Output::new(peripherals.PIN_20, Level::High);

    // let dac_chip_select_pin = peripherals.PIN_17;

    // let mut dac = Ad5626Dac::new(
    //     dac_clock_pin,
    //     dac_data_pin,
    //     dac_submit_pin,
    //     dac_chip_select_pin,
    // );

    // dac.encode(0b1100_0101_0010).await;
    // dac.submit().await;

    dac_submit.set_high();
    Timer::after(DAC_SUBMIT_CHANGE_DURATION).await;

    dac_clear.set_low();
    Timer::after(Duration::from_millis(1)).await;
    dac_clear.set_high();

    let mut rng = SmallRng::seed_from_u64(0xbadbeef);

    println!("WAIT");
    next_button.wait_for_rising_edge().await;

    loop {
        let data_u12 = (rng.next_u32() >> 20) as u16;

        println!("START write");
        println!("DATA {}", data_u12);

        for i in 4..16 {
            let shift = 16 - (i + 1);
            let level: Level = ((data_u12 >> shift) & 1 == 0b1).into();

            dac_data.set_level(level);
            dac_clock.set_level(Level::Low);
            Timer::after(DAC_DATA_SETUP_HOLD_DURATION).await;
            dac_clock.set_level(Level::High);
            Timer::after(DAC_DATA_SETUP_HOLD_DURATION).await;
        }

        println!("LOAD");

        Timer::after(DAC_SUBMIT_CHANGE_DURATION).await;
        dac_submit.set_low();
        Timer::after(DAC_SUBMIT_CHANGE_DURATION).await;
        dac_submit.set_high();
        Timer::after(DAC_SUBMIT_CHANGE_DURATION).await;

        println!("END write");
    }
}

struct Ad5626Dac<CLOCK: gpio::Pin, DATA: gpio::Pin, SUBMIT: gpio::Pin, CHIP_SELECT: gpio::Pin>
{
    /// SCLK
    serial_clock_output: Output<'static, CLOCK>,
    /// SDIN
    serial_data_output: Output<'static, DATA>,
    /// LDAC
    submit_output: Output<'static, SUBMIT>,
    /// CS
    chip_select: Output<'static, CHIP_SELECT>,
}

impl<CLOCK: gpio::Pin, DATA: gpio::Pin, SUBMIT: gpio::Pin, CHIP_SELECT: gpio::Pin>
    Ad5626Dac<CLOCK, DATA, SUBMIT, CHIP_SELECT>
{
    pub fn new(
        clock_pin: CLOCK,
        data_pin: DATA,
        submit_pin: SUBMIT,
        chip_select_pin: CHIP_SELECT,
    ) -> Self {
        Self {
            serial_clock_output: Output::new(clock_pin, Level::Low),
            serial_data_output: Output::new(data_pin, Level::Low),
            submit_output: Output::new(submit_pin, Level::High),
            chip_select: Output::new(chip_select_pin, Level::High),
        }
    }

    /// Encodes the 12 least significant bits of `num_12bit` in the DAC's input register.
    /// In order to perform the conversion to analog, `submit()` must be called following
    /// the encode.
    pub async fn encode(&mut self, num_12bit: u16) {
        println!("{:b}", num_12bit);

        // Clock starts low
        self.serial_clock_output.set_low();

        // Hold the submit pin high for the duration of the encoding
        self.submit_output.set_high();

        let mut ticker = Ticker::every(DAC_CLOCK_LEVEL_DURATION);
        for i in 4..16 {
            let level = if (num_12bit >> (16 - (i + 1))) & 0b1 == 0b1 {
                Level::High
            } else {
                Level::Low
            };
            self.serial_data_output.set_level(level);

            self.serial_clock_output.set_low();
            ticker.next().await;
            self.serial_clock_output.set_high();
            ticker.next().await;
        }

        // for i in 4..16 {
        //     let level = if (num_12bit >> (16 - (i + 1))) & 0b1 == 0b1 {
        //         Level::High
        //     } else {
        //         Level::Low
        //     };
        //     println!("{}: {}", i - 4, Debug2Format(&level));

        //     self.serial_data_output.set_level(level);
        //     Timer::after(DAC_DATA_SETUP_HOLD_DURATION.max(DAC_CLOCK_LEVEL_DURATION)).await;
        //     self.serial_clock_output.set_high();
        //     Timer::after(DAC_DATA_SETUP_HOLD_DURATION).await;

        //     self.serial_clock_output.set_low();
        // }
    }

    /// Submits the value buffered in the input register to the DAC
    pub async fn submit(&mut self) {
        //
        Timer::after(DAC_SUBMIT_CHANGE_DURATION).await;
        self.submit_output.set_low();
        Timer::after(DAC_LOAD_PULSE_WIDTH).await;
        self.submit_output.set_high();
    }
}
