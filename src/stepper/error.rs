use core::{error::Error, fmt::Display};

use snafu::prelude::Snafu;

#[derive(Debug, Snafu)]
pub enum ErrorKind {
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
