use core::default;

use dasp::sample::U12;
use defmt::{info, trace, Display2Format, Format};
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
        chip_select_pin: impl gpio::Pin,
    ) -> Ad5626<'d, PIO, SM, CLEAR> {
        into_ref!(dma);

        sm.clear_fifos();
        sm.clkdiv_restart();
        sm.restart();

        let program = pio_proc::pio_file!("src/dac/ad5626.pio").program;

        // Pin config
        let pio_cfg = {
            let mut p = Config::default();

            let data_pin = pio.make_pio_pin(data_pin);
            let clock_pin = pio.make_pio_pin(clock_pin);
            let load_pin = pio.make_pio_pin(load_pin);

            p.set_out_pins(&[&data_pin]);
            p.use_program(&pio.load_program(&program), &[&clock_pin, &load_pin]);

            sm.set_pin_dirs(
                embassy_rp::pio::Direction::Out,
                &[&data_pin, &clock_pin, &load_pin],
            );

            // Clock config, measured in kHz to avoid overflows
            let clock_freq = U24F8::from_num(clocks::clk_sys_freq());
            p.clock_divider = clock_freq / 1_000_000;

            info!("AD5626 clock divider: {}", Display2Format(&p.clock_divider));
            let pio_hz: f32 = (clock_freq / p.clock_divider).to_num();
            info!("Time per PIO cycle: {}ns", 1E9 / pio_hz);

            // FIFO config
            p.fifo_join = FifoJoin::TxOnly;
            p.shift_out = ShiftConfig {
                direction: ShiftDirection::Left,
                ..Default::default()
            };
            p
        };

        // Clear the DAC's input register
        let mut clear_out = gpio::Output::new(clear_pin, gpio::Level::High);
        Timer::after(Duration::from_micros(1)).await;

        // Assign config to state machine and go!
        sm.set_config(&pio_cfg);
        sm.set_enable(true);

        let mut dac = Ad5626 {
            dma: dma.map_into(),
            clear_out,
            sm,
        };

        dac.clear();
        dac
    }

    pub async fn clear(&mut self) {
        self.clear_out
            .toggle_and_back(Duration::from_micros(1))
            .await;
    }

    pub async fn write(&mut self, sample: U12) {
        let sample_u16 = sample.inner() as u16;
        trace!("Writing ____{:012b} ({})", sample_u16, sample_u16);
        trace!(
            "TX level: {} stalled? {} overflowed? {} empty? {} full? {} ",
            self.sm.tx().level(),
            self.sm.tx().stalled(),
            self.sm.tx().overflowed(),
            self.sm.tx().empty(),
            self.sm.tx().full(),
        );

        let words: [u16; 1] = [sample_u16];
        self.sm.tx().dma_push(self.dma.reborrow(), &words).await;
    }
}
