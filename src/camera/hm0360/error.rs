use core::{error::Error, fmt::Display};

use snafu::prelude::Snafu;

#[derive(Debug, Snafu)]
pub enum ErrorKind {
    ConnectionFailed,
    I2c,
    StateError,
}
