use core::fmt::Debug;

use anyhow::{bail, Result};
use defmt::{unwrap, Debug2Format};
use embassy_rp::uart::{self, BufferedUart};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embedded_io_async::{Read, Write};

use super::error::ErrorKind;
use crate::uart::bus::UartDevice;

/// NOTE: Setting this value too low can create its own set of problems. For example,
/// 28.8K consistently encounters framing errors.
pub const UART_BAUD_RATE: u32 = 345600;

/// Represents the connection to a TMC2209's UART interface
pub struct Tmc2209UartConnection<M: RawMutex + 'static, P: 'static + uart::Instance> {
    uart_device: UartDevice<'static, M, P>,
    uart_address: u8,
    change_count: u32,
}

impl<M: RawMutex + 'static, P: 'static + uart::Instance> Tmc2209UartConnection<M, P> {
    pub async fn connect(
        uart_device: UartDevice<'static, M, P>,
        uart_address: u8,
    ) -> Result<Self, ErrorKind> {
        defmt::debug!("Connecting to TMC2209 UART bus address {}", uart_address);

        let mut connection = Tmc2209UartConnection {
            uart_device,
            uart_address,
            change_count: 0,
        };
        connection
            .initialize()
            .await
            .map_err(|_| ErrorKind::ConnectionFailed)?;
        Ok(connection)
    }

    /// Initializes the state of the connection
    async fn initialize(&mut self) -> Result<()> {
        self.change_count = self.read_register::<tmc2209::reg::IFCNT>().await?.0;
        Ok(())
    }

    pub async fn read_register<R: tmc2209::reg::Register + From<u32>>(
        &mut self,
    ) -> Result<R> {
        let register_address = R::ADDRESS;
        defmt::trace!(
            "Writing read request to {:?}@{}",
            defmt::Debug2Format(&register_address),
            self.uart_address
        );

        let req = tmc2209::ReadRequest::from_addr(self.uart_address, register_address);
        defmt::trace!("read request bytes: {}", req.bytes());
        if let Err(e) = self.uart_device.write_all(req.bytes()).await {
            defmt::error!("{}", e);
            bail!(ErrorKind::UartGeneral(e))
        }

        let mut buffer: [u8; tmc2209::ReadRequest::LEN_BYTES +
            tmc2209::ReadResponse::LEN_BYTES] =
            [0; tmc2209::ReadRequest::LEN_BYTES + tmc2209::ReadResponse::LEN_BYTES];
        match self.uart_device.read_exact(&mut buffer).await {
            Ok(_) => {
                defmt::trace!("bytes read\n {}", buffer);
            }
            Err(e) => {
                defmt::error!("{}", e);
                bail!(ErrorKind::UartReadExact(e));
            }
        }

        let mut reader = tmc2209::Reader::default();
        defmt::trace!("...reading response");
        Ok(R::from(
            match reader.read_response(&buffer[tmc2209::ReadRequest::LEN_BYTES..12]) {
                (_, Some(res)) => {
                    let state = res.reg_state().unwrap();
                    defmt::trace!("response {:?}", Debug2Format(&state));
                    state
                }
                (bytes, _) => {
                    defmt::error!("no response: {} bytes read", bytes);
                    defmt::error!("{}", &buffer);
                    bail!(ErrorKind::MissingResponse)
                }
            }
            .into(),
        ))
    }

    pub async fn write_register<R>(&mut self, register: R) -> Result<u32>
    where
        R: tmc2209::reg::WritableRegister + Debug,
    {
        defmt::debug!("Write register {:?}", defmt::Debug2Format(&register));

        let req = tmc2209::WriteRequest::new(self.uart_address, register);
        defmt::trace!("write request: {}", req.bytes());
        if let Err(e) = self.uart_device.write_all(req.bytes()).await {
            defmt::error!("{}", e);
            bail!(ErrorKind::UartGeneral(e));
        }

        // Clear the echo
        defmt::trace!("...reading echo");
        let mut buffer: [u8; tmc2209::WriteRequest::LEN_BYTES] =
            [0; tmc2209::WriteRequest::LEN_BYTES];
        match self.uart_device.read_exact(&mut buffer).await {
            Ok(_) => defmt::trace!("echo received\n{}", buffer),
            Err(e) => {
                defmt::error!("{}", e);
                panic!();
            }
        }

        // Check the change count register to make sure the write was successful\
        let new_change_count = self.read_register::<tmc2209::reg::IFCNT>().await?.0;
        let prev_change_count = self.change_count;
        self.change_count = new_change_count;

        if (prev_change_count + 1) % 256 == new_change_count {
            Ok(new_change_count)
        } else {
            defmt::error!(
                "change count mismatch: {} + 1 != {}",
                prev_change_count,
                new_change_count
            );
            bail!(ErrorKind::WriteFailed)
        }
    }
}
