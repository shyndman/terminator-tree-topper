use core::{error::Error, fmt::Display};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::{gpio, i2c};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_hal_1::i2c::{AddressMode, SevenBitAddress};
use embedded_hal_async::i2c::I2c;
use snafu::prelude::Snafu;

pub const DEFAULT_ADDRESS: SevenBitAddress = 0x62;

/// Writes data to the DAC in fast mode (400kbps)
const CMD_WRITEDAC: u8 = 0x00;

pub struct Mcp4725<
    'd,
    M: RawMutex + 'static,
    B: i2c::Instance + 'static,
    I2cAddress: AddressMode,
> {
    i2c_device: I2cDevice<'d, M, i2c::I2c<'d, B, i2c::Async>>,
    i2c_address: I2cAddress,
}

impl<M: RawMutex + 'static, B: i2c::Instance + 'static>
    Mcp4725<'static, M, B, SevenBitAddress>
{
    pub fn new(
        mut i2c_device: I2cDevice<'static, M, i2c::I2c<'static, B, i2c::Async>>,
        i2c_address: SevenBitAddress,
    ) -> Self {
        Self {
            i2c_device,
            i2c_address,
        }
    }

    pub async fn set_voltage(&mut self, output: u16) -> Result<(), ErrorKind> {
        let buf: [u8; 3] = [
            CMD_WRITEDAC,
            (output / 16) as u8,        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
            ((output % 16) << 4) as u8, // Lower data bits (D3.D2.D1.D0.x.x.x.x)
        ];

        if let Err(e) = self.i2c_device.write(self.i2c_address, &buf).await {
            return Err(ErrorKind::I2c);
        }

        Ok(())
    }
}

#[derive(Debug, Snafu)]
pub enum ErrorKind {
    I2c,
    StateError,
}
