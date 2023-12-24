use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_time::{Duration, Instant, Timer};
use fixed::{
    traits::FixedUnsigned,
    types::{
        extra::{LeEqU32, Unsigned},
        U16F16, U43F21,
    },
    FixedU32,
};
use snafu::prelude::ensure;

use super::error::*;
use crate::{gpio::TimedToggle, stepper::MotionDirection};

type Result<T, E = MotionErrorKind> = core::result::Result<T, E>;

// NOTE: These are minimum durations for holding pins at specific values -- if we hold for
// longer (which we will, because we can't manage nano-accuracy), it will not be a
// problem.

const STEP_PULSE_TIME: Duration = Duration::from_nanos(100);
const DIRECTION_SETUP_TIME: Duration = Duration::from_nanos(20);

/// Pinned to 256 microsteps
pub struct Tmc2209MotionDriver<Profile: MotionProfile> {
    current_step: u16,
    current_direction: MotionDirection,
    direction_out: Output<'static, AnyPin>,
    step_out: Output<'static, AnyPin>,
    last_update_time: Instant,
    profile: Profile,
}

impl<Profile: MotionProfile> Tmc2209MotionDriver<Profile> {
    pub fn new(direction_pin: AnyPin, step_pin: AnyPin, profile: Profile) -> Self {
        Self {
            current_step: 0,
            current_direction: MotionDirection::Forward,
            step_out: Output::new(step_pin, Level::Low),
            direction_out: Output::new(direction_pin, Level::Low),
            last_update_time: Instant::MIN,
            profile,
        }
    }

    pub fn get_current_step(&self) -> u16 {
        self.current_step
    }

    pub fn get_current_direction(&self) -> MotionDirection {
        self.current_direction
    }

    pub fn update_target_step(
        &mut self,
        target_step: u16,
        max_velocity: U16F16,
    ) -> Result<()> {
        self.profile.update_target_step(target_step, max_velocity);

        Ok(())
    }

    pub async fn next(&mut self) {
        let next_command = self.profile.next(
            self.current_direction,
            self.current_step,
            Some(self.last_update_time),
        );
    }

    // TODO(shyndman): These pin mutation methods should be

    async fn step(&mut self) -> Result<()> {
        ensure!(self.step_out.is_set_low(), StepPinStateSnafu);
        self.step_out.toggle_for(STEP_PULSE_TIME).await;
        Ok(())
    }

    async fn change_direction(&mut self) -> Result<()> {
        self.direction_out.toggle();
        Timer::after(DIRECTION_SETUP_TIME).await;
        Ok(())
    }
}

pub enum MotionCommand {
    Step(Duration),
    ChangeDirection(MotionDirection),
}

pub trait MotionProfile {
    fn update_target_step(&mut self, target_step: u16, max_velocity: U16F16) -> Result<()>;

    fn next(
        &mut self,
        current_direction: MotionDirection,
        current_step: u16,
        last_update_time: Option<Instant>,
    ) -> Option<MotionCommand>;
}

pub struct ConstantMotionProfile {
    target_step: u16,
    velocity: U16F16,
}

impl Default for ConstantMotionProfile {
    fn default() -> Self {
        Self {
            target_step: Default::default(),
            velocity: Default::default(),
        }
    }
}

impl MotionProfile for ConstantMotionProfile {
    fn update_target_step(&mut self, target_step: u16, velocity: U16F16) -> Result<()> {
        self.target_step = target_step;
        self.velocity = U16F16::from_num(velocity);

        Ok(())
    }

    fn next(
        &mut self,
        current_direction: MotionDirection,
        current_step: u16,
        last_step_time: Option<Instant>,
    ) -> Option<MotionCommand> {
        let step_count: i16 = self.target_step as i16 - current_step as i16;
        if step_count == 0 {
            return None;
        }

        let direction: MotionDirection = step_count.into();
        if current_direction != direction {
            return Some(MotionCommand::ChangeDirection(direction));
        }

        self.velocity.recip();

        Some(MotionCommand::Step(Duration::from_hz(10)))
    }
}
