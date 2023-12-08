use defmt::{info, Display2Format, Format};
use embassy_rp::{
    clocks, dma,
    dma::{AnyChannel, Channel, Word},
    gpio::{self, Output},
    into_ref,
    pio::{
        Common, Config, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection, StateMachine,
    },
    Peripheral, PeripheralRef,
};
use embassy_time::{Duration, Timer};
use fixed::{
    traits::FromFixed,
    types::{U24F8, U32F0},
};
use fixed_macro::fixed;
use rgb::RGB8;

use crate::gpio::TimedToggle;

pub struct Ad5626<'d, PIO: Instance, const SM: usize, CLEAR: gpio::Pin> {
    dma: PeripheralRef<'d, AnyChannel>,
    sm: StateMachine<'d, PIO, SM>,
    clear_out: Output<'static, CLEAR>,
}
impl<'d, PIO: Instance, const SM: usize, CLEAR: gpio::Pin> Ad5626<'d, PIO, SM, CLEAR> {
    pub async fn new(
        pio: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        dma: impl Peripheral<P = impl Channel> + 'd,
        data_pin: impl PioPin,
        clock_pin: impl PioPin,
        load_pin: impl PioPin,
        clear_pin: CLEAR,
    ) -> Ad5626<'d, PIO, SM, CLEAR> {
        into_ref!(dma);

        let program_w_defines = pio_proc::pio_file!("src/dac/ad5626.pio");
        let program = program_w_defines.program;

        // Pin config
        let pio_cfg = {
            let mut p = Config::default();

            let data_pin = pio.make_pio_pin(data_pin);
            let clock_pin = pio.make_pio_pin(clock_pin);
            let load_pin = pio.make_pio_pin(load_pin);

            p.set_out_pins(&[&data_pin]);
            p.set_set_pins(&[&load_pin]);
            p.use_program(&pio.load_program(&program), &[&clock_pin]);
            sm.set_pin_dirs(
                embassy_rp::pio::Direction::Out,
                &[&data_pin, &load_pin, &clock_pin],
            );

            // Clock config, measured in kHz to avoid overflows
            let clock_freq = U24F8::from_num(clocks::clk_sys_freq());
            p.clock_divider = clock_freq / 10_000_000;

            info!("AD5626 clock divider: {}", Display2Format(&p.clock_divider));
            let pio_hz: f32 = (clock_freq / p.clock_divider).to_num();
            info!("Time per PIO cycle: {}ns", 1E9 / pio_hz);

            // FIFO config
            p.fifo_join = FifoJoin::TxOnly;
            p.shift_out = ShiftConfig {
                auto_fill: true,
                threshold: 16,
                direction: ShiftDirection::Left,
            };
            p
        };

        // Clear the DAC's input register
        let mut clear_out = gpio::Output::new(clear_pin, gpio::Level::High);
        clear_out.toggle_and_back(Duration::from_micros(1)).await;

        // Assign config to state machine and go!
        sm.set_config(&pio_cfg);
        sm.set_enable(true);

        Ad5626 {
            dma: dma.map_into(),
            clear_out,
            sm,
        }
    }

    pub async fn write(&mut self, sample: u12) {
        let words: [u16; 1] = [sample.into()];
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;
    }
}

/// A 12-bit sample to be converted by the DAC
#[allow(non_camel_case_types)]
pub struct u12 {
    pub inner: u16,
}

impl u12 {
    pub const BITS: u8 = 12;

    pub fn new(inner: u16) -> Self {
        assert!(inner <= 0b1111_1111_1111);

        Self { inner }
    }
}

impl Format for u12 {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "{:04b} {:04b} {:04b}",
            self.inner >> 8,
            self.inner >> 4 & 0b1111,
            self.inner & 0b1111
        );
    }
}

impl From<u12> for u16 {
    fn from(value: u12) -> Self {
        value.inner
    }
}
