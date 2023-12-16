pub mod error;
pub mod init;
pub mod reg;

use alloc::vec::Vec;

use anyhow::{bail, Result};
use defmt::{debug, error, println, trace};
use embassy_embedded_hal::shared_bus::{asynch::i2c::I2cDevice, I2cDeviceError};
use embassy_rp::{dma, gpio, i2c, into_ref, pio, Peripheral, PeripheralRef};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Timer};
use embedded_hal_1::i2c::{AddressMode, SevenBitAddress};
use embedded_hal_async::i2c::I2c;
use error::ErrorKind;
use init::HM0360_DEFAULT_REGISTERS;
use reg::RegisterAddress as Addr;

use crate::camera::hm0360::init::HM0360_DEFAULT_REGISTERS2;

const BUS_ADDRESS: u8 = 0x24;
const RESET_WAIT_DURATION: Duration = Duration::from_millis(100);
const IMAGE_BYTE_COUNT: usize = 320 * 240;

// MODE_SELECT[2:0]=001

pub struct Hm0360<
    'd,
    M: RawMutex + 'static,
    B: i2c::Instance + 'static,
    Address: AddressMode,
    PIO: pio::Instance,
    const SM: usize,
    Vsync: gpio::Pin,
    Reset: gpio::Pin,
> {
    i2c_device: I2cDevice<'d, M, i2c::I2c<'d, B, i2c::Async>>,
    i2c_address: Address,
    dma: PeripheralRef<'d, dma::AnyChannel>,
    sm: pio::StateMachine<'d, PIO, SM>,
    vsync_in: gpio::Input<'d, Vsync>,
    reset_out: gpio::Output<'d, Reset>,
}

impl<
        M: RawMutex + 'static,
        B: i2c::Instance + 'static,
        PIO: pio::Instance + 'static,
        const SM: usize,
        Vsync: gpio::Pin,
        Reset: gpio::Pin,
    > Hm0360<'static, M, B, SevenBitAddress, PIO, SM, Vsync, Reset>
{
    pub async fn new(
        mut i2c_device: I2cDevice<'static, M, i2c::I2c<'static, B, i2c::Async>>,
        pio: &mut pio::Common<'static, PIO>,
        mut sm: pio::StateMachine<'static, PIO, SM>,
        dma: impl Peripheral<P = impl dma::Channel> + 'static,
        serial_data_pin: impl pio::PioPin,
        hsync_pin: impl pio::PioPin,
        pixel_clock_pin: impl pio::PioPin,
        vsync_pin: Vsync,
        reset_pin: Reset,
    ) -> Self {
        into_ref!(dma);

        sm.clear_fifos();
        sm.clkdiv_restart();
        sm.restart();

        let program = pio_proc::pio_file!("src/camera/hm0360/read_image.pio").program;
        let pio_cfg = {
            let mut cfg = pio::Config::default();

            // Pin config
            let serial_data_pin = pio.make_pio_pin(serial_data_pin);
            let hsync_pin = pio.make_pio_pin(hsync_pin);
            let pixel_clock_pin = pio.make_pio_pin(pixel_clock_pin);

            let input_pins = [&serial_data_pin, &hsync_pin, &pixel_clock_pin];
            cfg.set_in_pins(&input_pins);
            sm.set_pin_dirs(pio::Direction::In, &input_pins);
            cfg.use_program(&pio.load_program(&program), &[]);

            // FIFO config
            cfg.fifo_join = pio::FifoJoin::RxOnly;
            cfg.shift_in = pio::ShiftConfig {
                direction: pio::ShiftDirection::Left,
                auto_fill: true,
                threshold: 8,
                ..Default::default()
            };
            cfg
        };

        // Configure the state machine, but do not start running until the first frame
        // is requested
        sm.set_config(&pio_cfg);
        sm.set_enable(false);

        // Build a new camera and initialize

        let mut camera = Self {
            i2c_device,
            i2c_address: BUS_ADDRESS,
            dma: dma.map_into(),
            sm,
            vsync_in: gpio::Input::new(vsync_pin, gpio::Pull::Down),
            reset_out: gpio::Output::new(reset_pin, gpio::Level::Low),
        };

        camera.reset().await.unwrap();

        // Read the device's identifier

        debug!("Verifying camera hardware identifier");

        let h = camera
            .read_register(Addr::ModelIdHigh.into())
            .await
            .unwrap();
        let l = camera.read_register(Addr::ModelIdLow.into()).await.unwrap();

        println!("h: {:02x} l: {:02x}", h, l);

        assert!(
            h == 0x03 && l == 0x60,
            "Hm0360 is the only camera supported"
        );

        // Write load registers

        debug!("Writing camera hardware initialization registers");

        for (reg_address, value) in HM0360_DEFAULT_REGISTERS2 {
            trace!("Writing {:#X}", reg_address);
            loop {
                match camera.write_register(reg_address, value).await {
                    Ok(_) => break,
                    Err(e) => {
                        trace!("Retrying");
                    }
                }
            }
        }

        debug!("Done writing init registers");

        camera
    }

    pub async fn reset(&mut self) -> Result<()> {
        debug!("Triggering camera reset");

        self.reset_out.set_low();
        Timer::after(RESET_WAIT_DURATION).await;
        self.reset_out.set_high();
        Timer::after(RESET_WAIT_DURATION).await;

        Ok(())
    }

    pub async fn capture_frame(&mut self) -> [u8; IMAGE_BYTE_COUNT] {
        debug!("Waiting for next frame to begin");

        self.vsync_in.wait_for_high().await;
        self.vsync_in.wait_for_low().await;

        debug!("Enabling state machine");
        self.sm.clear_fifos();

        debug!("Capturing frame");

        let mut image_buf = [0; IMAGE_BYTE_COUNT];
        self.sm.set_enable(true);
        self.sm
            .rx()
            .dma_pull(self.dma.reborrow(), &mut image_buf)
            .await;
        self.sm.set_enable(false);

        debug!("Captured!");

        // debug!("Sanitizing scanlines");

        // let mut x = 0_usize;

        // let mut y = 24_usize;
        // let mut x = 0_usize;
        // let mut index = 0_usize;
        // loop {
        //     if y >= 216 {
        //         break;
        //     }

        //     x = 64 + (1 + x) % 2;
        //     loop {
        //         if x >= 256 {
        //             break;
        //         }

        //         image_buf[index] = image_buf[y * 320 + x];
        //         index += 1;

        //         x += 2;
        //     }

        //     y += 2;
        // }

        image_buf
    }

    pub async fn read_register(&mut self, reg_address: u16) -> Result<u8> {
        debug!("read_register {:#X}", reg_address);
        let a = reg_address.to_be_bytes();
        let mut buf = [0u8];
        match self
            .i2c_device
            .write_read(self.i2c_address, &a, &mut buf)
            .await
        {
            Ok(()) => {
                debug!("value: {:#b}", buf[0]);
                Ok(buf[0])
            }
            Err(e) => {
                error!("error encountered: {}", e);
                bail!(ErrorKind::I2c)
            }
        }
    }

    // pub async fn read_typed_register<R: Register>(&mut self) -> anyhow::Result<R> {
    //     let reg_byte = self.read_register(R::address().into()).await?;
    //     Ok(R::from_bytes([reg_byte]))
    // }

    pub async fn write_register(&mut self, reg_address: u16, value: u8) -> Result<()> {
        let a = reg_address.to_be_bytes();
        let mut write = [a[0], a[1], value];
        match self.i2c_device.write(self.i2c_address, &write).await {
            Ok(()) => Ok(()),
            Err(e) => {
                error!("error encountered: {}", e);
                bail!(ErrorKind::I2c);
            }
        }
    }
}
