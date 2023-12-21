use embassy_rp::uart;
use embassy_sync::blocking_mutex::raw::RawMutex;
use micromath::F32Ext;

use super::{error::ErrorKind, uart::Tmc2209UartConnection};
use crate::stepper::motor_constants::{MotorConstants, TMC2209_VSENSE_OHMS};

const IHOLD_DELAY: u8 = 12;
const SEMIN: u16 = 8;
const SEMAX: u16 = 4;
const SEUP: u16 = 3;
const SEDN: u16 = 0;
const SEIMIN: bool = false; // If we drop to 1/4 current, high accels don't work right

pub async fn tune_driver<M: RawMutex + 'static, P: uart::Instance>(
    driver: &mut Tmc2209UartConnection<M, P>,
    motor_constants: MotorConstants,
) -> Result<(), ErrorKind> {
    let result: Result<(), ErrorKind> = try {
        // Disable automatic power down, because its pin is shared
        let mut gconf = tmc2209::reg::GCONF::default();
        gconf.set_pdn_disable(true);
        gconf.set_shaft(true);
        driver.write_register(gconf).await?;

        let mut ihold_irun = tmc2209::reg::IHOLD_IRUN::default();
        let (vsense, irun) = tmc2209::rms_current_to_vsense_cs(
            TMC2209_VSENSE_OHMS,
            motor_constants.run_current_milliamps(),
        );
        let ihold = 0; // Use passive breaking during standby
        defmt::info!("Setting IRUN={}, IHOLD={}", irun, ihold);
        ihold_irun.set_irun(irun);
        ihold_irun.set_ihold(ihold);
        ihold_irun.set_ihold_delay(IHOLD_DELAY);
        driver.write_register(ihold_irun).await?;

        let mut tpowerdown = tmc2209::reg::TPOWERDOWN::default();
        tpowerdown.0 = 20;
        driver.write_register(tpowerdown).await?;

        let mut chopconf = tmc2209::reg::CHOPCONF::default();
        let (hstrt, hend) = motor_constants.hysteresis(None, None, None, None);
        chopconf.set_hstrt(hstrt);
        chopconf.set_hstrt(hend);
        chopconf.set_mres(0b00); // 256 steps
        chopconf.set_intpol(true); // Interpolation of microsteps to 256
        chopconf.set_vsense(vsense);
        chopconf.set_tbl(1);
        chopconf
            .set_toff(((0.85e-5 * tmc2209::INTERNAL_CLOCK_HZ - 12.0) / 32.0).ceil() as u32);
        chopconf.set_dedge(true);
        driver.write_register(chopconf).await?;

        // Automatic stealthchop <-> spreadcycle switching
        let mut tpwnthrs = tmc2209::reg::TPWMTHRS::default();
        // tpwnthrs.set(0xfffff); // This keeps the stepper in SpreadCycle mode
        tpwnthrs.set(0); // This keeps the stepper in StealthChop mode
        driver.write_register(tpwnthrs).await?;

        // Coolstep
        let mut tcoolthrs = tmc2209::reg::TCOOLTHRS::default();
        tcoolthrs.set(2_u32.pow(20) - 1);
        driver.write_register(tcoolthrs).await?;

        // Stallguard
        let mut sgthrs = tmc2209::reg::SGTHRS::default();
        sgthrs.0 = 0xff;
        driver.write_register(sgthrs).await?;

        // StealthChop configuration
        let mut pwmconf = tmc2209::reg::PWMCONF::default();
        pwmconf.set_pwm_autoscale(true);
        pwmconf.set_pwm_autograd(true);
        pwmconf.set_pwm_freq(0b10);
        pwmconf.set_pwm_grad(motor_constants.pwm_gradient(None, None) as u8);
        pwmconf.set_pwm_ofs(motor_constants.pwm_output_frequency(None) as u8);
        pwmconf.set_pwm_reg(15);
        pwmconf.set_pwm_lim(4);
        pwmconf.set_freewheel(2); // Passive breaking
        driver.write_register(pwmconf).await.unwrap();

        let mut coolconf = tmc2209::reg::COOLCONF::default();
        coolconf.set_semin(SEMIN);
        coolconf.set_semax(SEMAX);
        coolconf.set_seup(SEUP);
        coolconf.set_sedn(SEDN);
        coolconf.set_seimin(SEIMIN);
        driver.write_register(coolconf).await?;

        // gconf.set_en_spread_cycle(true); // SpreadCycle
        gconf.set_en_spread_cycle(false); // StealthChop
        gconf.set_i_scale_analog(false);
        gconf.set_multistep_filt(true);
        gconf.set_mstep_reg_select(true); // Set microsteps via registers
        driver.write_register(gconf).await?;
    };

    result
}
