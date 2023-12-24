use core::{error::Error, fmt::Display};

use snafu::prelude::Snafu;

#[derive(Debug, Snafu)]
pub enum UartErrorKind {
    ConnectionFailed,
    UartGeneral {
        inner: embassy_rp::uart::Error,
    },
    UartReadExact {
        inner: embedded_io::ReadExactError<embassy_rp::uart::Error>,
    },
    MissingResponse,
    WriteFailed,
}

#[derive(Debug, Snafu, defmt::Format)]
#[snafu(visibility(pub))]
pub enum MotionErrorKind {
    StepPinState,
}
