pub mod motor_constants;

pub mod tmc2209;

use defmt::Format;

#[derive(Clone, Copy, Format, PartialEq)]
pub enum MotionDirection {
    Forward,
    Backward,
}

impl Into<MotionDirection> for i16 {
    fn into(self) -> MotionDirection {
        if self >= 0 {
            MotionDirection::Forward
        } else {
            MotionDirection::Backward
        }
    }
}
