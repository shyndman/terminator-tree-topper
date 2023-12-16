use core::fmt::Debug;

use embassy_rp::uart;
use embassy_sync::{blocking_mutex::raw::RawMutex, mutex::Mutex};
use embedded_io::ReadExactError;
use embedded_io_async::{Read, Write};

/// Error returned by I2C device implementations in this crate.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UartDeviceError<BUS> {
    /// An operation on the inner I2C bus failed.
    Uart(BUS),
    /// Configuration of the inner I2C bus failed.
    Config,
}

/// UART device on a shared bus.
pub struct UartDevice<'a, M: 'static + RawMutex, P: 'static + uart::Instance> {
    bus: &'a Mutex<M, uart::BufferedUart<'a, P>>,
}

// TODO(shyndman): This is the WRONG interface for TMC2209 communication. Locks need to
// be held across multiple operations, as we are often following a request-response model
// when communicating with the stepper driver.
impl<'a, M: RawMutex, P: 'static + uart::Instance> UartDevice<'a, M, P> {
    pub fn new(bus: &'a Mutex<M, uart::BufferedUart<'a, P>>) -> Self {
        Self { bus }
    }

    pub async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), ReadExactError<uart::Error>> {
        let mut bus = self.bus.lock().await;
        bus.read_exact(buf).await
    }

    pub async fn write_all(&mut self, buf: &[u8]) -> Result<(), uart::Error> {
        let mut bus = self.bus.lock().await;
        bus.write_all(buf).await
    }

    // TODO(shyndman): The TMC2209 UART layer NEEDs this to safely access the serial line.
    // Implement!
    //
    // pub async fn transaction<F: FnMut(&mut uart::BufferedUart)>(
    //     &mut self,
    //     mut critical_section: F,
    // ) {
    //     let mut bus = self.bus.lock().await;
    //     critical_section(bus.borrow_mut());
    // }
}
