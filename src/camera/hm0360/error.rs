use core::{error::Error, fmt::Display};

use anyhow::anyhow;

#[derive(Debug)]
pub enum ErrorKind {
    ConnectionFailed,
    I2c,
    StateError,
}
impl Display for ErrorKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}
impl Error for ErrorKind {}
