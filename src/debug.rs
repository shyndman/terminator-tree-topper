use base64::prelude::BASE64_STANDARD_NO_PAD;
use defmt::println;
use embassy_rp::uart::{self, BufferedUart};
use embedded_graphics::{
    image::ImageRaw,
    pixelcolor::{
        raw::{BigEndian, ByteOrder},
        Rgb888,
    },
    prelude::*,
};
use embedded_io_async::Write;
use snafu::IntoError;

pub struct DebugUart<P: uart::Instance + 'static> {
    uart: BufferedUart<'static, P>,
}

const ANSI_ESCAPE: char = '\x1b';
const IMAGE_BYTE_LENGTH: usize = 768; // 3 * 16 * 16;

impl<P: uart::Instance> DebugUart<P> {
    pub fn new(uart: BufferedUart<'static, P>) -> Self {
        Self { uart }
    }

    pub async fn render(&mut self) -> Result<(), uart::Error> {
        println!("render()");

        let mut image_data = [0u8; IMAGE_BYTE_LENGTH];
        ImageRaw::<Rgb888, BigEndian>::new(&image_data, 16);

        let buf = "does this work properly???\r\n".as_bytes();
        self.uart
            .blocking_write(&buf)
            .expect("Failed to write to debug UART");

        Ok(())
    }
}
