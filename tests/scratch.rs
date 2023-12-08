#![no_std]
#![no_main]

use t800 as _; // memory layout + panic handler

// See https://crates.io/crates/defmt-test/0.3.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    use defmt::{assert, println};
    use micromath::F32Ext;

    #[test]
    fn it_works() {
        assert!(true)
    }

    #[test]
    fn generate_pdm_test() {
        const TICK_HZ: u64 = 200_000;
        const A440_HZ: u64 = 440;
        const US_PER_TICK: u64 = 1_000_000 / TICK_HZ;
        const US_PER_WAVE: f32 = 1_000_000.0 / A440_HZ as f32;

        fn us_to_radians(us: f32) -> f32 {
            us * 2.0 * core::f32::consts::PI / US_PER_WAVE
        }

        for i in 0..(TICK_HZ / A440_HZ) {
            let elapsed_us = (i * US_PER_TICK) as f32 % US_PER_WAVE;
            let elapsed_radians = us_to_radians(elapsed_us);
            let amplitude = elapsed_radians.sin();
            let amp_pcm = (((amplitude+1.0)/2.0)*255.0) as u8;


            println!("{} {} ({})", elapsed_radians, amplitude, amp_pcm);
        }
    }
}
