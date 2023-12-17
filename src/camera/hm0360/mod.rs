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

use self::reg::{
    Register, WritableRegister, HIMAX_MD_ROI_QQVGA_H, HIMAX_MD_ROI_QQVGA_W,
    HIMAX_MD_ROI_QVGA_H, HIMAX_MD_ROI_QVGA_W, HIMAX_MD_ROI_VGA_H, HIMAX_MD_ROI_VGA_W,
};
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

        let h = camera.read_raw(Addr::ModelIdHigh).await.unwrap();
        let l = camera.read_raw(Addr::ModelIdLow).await.unwrap();

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
                match camera.write_raw(reg_address, value).await {
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
        self.sm.set_enable(true);

        debug!("Capturing frame");

        let mut image_buf = [0; IMAGE_BYTE_COUNT];
        self.sm
            .rx()
            .dma_pull(self.dma.reborrow(), &mut image_buf)
            .await;
        self.sm.set_enable(false);

        debug!("Captured!");

        image_buf
    }

    pub async fn set_motion_detection_threshold(&mut self, threshold: u8) -> Result<()> {
        // Set motion detection threshold/sensitivity.
        self.write_raw(Addr::MdThStrL, threshold).await?;
        self.write_raw(Addr::MdThStrH, threshold).await?;
        self.write_raw(Addr::MdLightCoef, threshold).await?;

        Ok(())
    }

    pub async fn set_motion_detection_window(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
    ) -> Result<()> {
        let x1: u16 = x;
        let y1: u16 = y;
        let x2: u16 = x + w;
        let y2: u16 = y + h;

        let hsub: u8 = self.read_raw(Addr::HSubsample).await? & 0x03;
        let vsub: u8 = self.read_raw(Addr::VSubsample).await? & 0x03;

        let (roi_w, roi_h, roi_max_h) = match ((hsub, vsub)) {
            (0, 0) => (HIMAX_MD_ROI_VGA_W as u16, HIMAX_MD_ROI_VGA_H as u16, 14),
            (1, 1) => (HIMAX_MD_ROI_QVGA_W as u16, HIMAX_MD_ROI_QVGA_H as u16, 14),
            (2, 2) => (HIMAX_MD_ROI_QQVGA_W as u16, HIMAX_MD_ROI_QQVGA_H as u16, 13),
            _ => bail!(ErrorKind::StateError),
        };

        let x1 = ((x1 / roi_w).saturating_sub(1)).max(0) as u8;
        let y1 = (y1 / roi_h).saturating_sub(1).max(0) as u8;
        let x2 = ((x2 / roi_w) + ((x2 % roi_w) > 0) as u16).min(0xF) as u8;
        let y2 = ((y2 / roi_h) + ((y2 % roi_h) > 0) as u16).min(roi_max_h) as u8;

        self.write_raw(Addr::RoiStartEndH, ((x2 & 0xF) << 4) | (x1 & 0x0F))
            .await?;
        self.write_raw(Addr::RoiStartEndV, ((y2 & 0xF) << 4) | (y1 & 0x0F))
            .await?;

        Ok(())
    }

    pub async fn enable_motion_detection(&mut self) -> Result<()> {
        self.clear_motion_detection().await?;
        let reg = self.read_raw(Addr::MdCtrl).await?;
        self.write_raw(Addr::MdCtrl, reg | 1).await;
        Ok(())
    }

    pub async fn clear_motion_detection(&mut self) -> Result<()> {
        self.write_raw(Addr::ClearInterrupts, 1 << 3).await
    }

    pub async fn get_motion_map(&mut self) -> Result<impl MotionMap> {
        let mut buf = self
            .read_raw_n::<u16, ROI_BYTE_COUNT>(0x20A1 as u16)
            .await?;

        println!("{:b}", buf);
        buf.reverse();

        Ok(SliceMotionMap::new(ROI_COLUMN_COUNT, ROI_ROW_COUNT, buf))
    }

    pub async fn read<R: Register + defmt::Format>(&mut self) -> Result<R> {
        match self.read_raw(R::address()).await {
            Ok(val) => {
                debug!("value: {:#b}", val);
                Ok(R::from_bytes([val]))
            }
            Err(e) => {
                bail!(e)
            }
        }
    }

    pub async fn read_raw<A: Into<u16> + defmt::Format>(
        &mut self,
        reg_address: A,
    ) -> Result<u8> {
        debug!("read_raw {:#X}", reg_address);
        let buf = self.read_raw_n::<A, 1>(reg_address).await?;
        Ok(buf[0])
    }

    pub async fn read_raw_n<A: Into<u16> + defmt::Format, const LEN: usize>(
        &mut self,
        reg_address: A,
    ) -> Result<[u8; LEN]> {
        debug!("read_raw_n {:#X} len={}", reg_address, LEN);
        let a = (reg_address.into() as u16).to_be_bytes();
        let mut buf = [0u8; LEN];
        match self
            .i2c_device
            .write_read(self.i2c_address, &a, &mut buf)
            .await
        {
            Ok(()) => {
                debug!("value: {:#b}", buf);
                Ok(buf)
            }
            Err(e) => {
                error!("error encountered: {}", e);
                bail!(ErrorKind::I2c)
            }
        }
    }

    pub async fn write<R: WritableRegister + defmt::Format>(&mut self, reg: R) -> Result<()> {
        debug!("write_register {}", reg);
        self.write_raw(R::address(), reg.into_bytes()[0]).await?;
        Ok(())
    }

    pub async fn write_raw<A: Into<u16> + defmt::Format>(
        &mut self,
        reg_address: A,
        value: u8,
    ) -> Result<()> {
        debug!("write_register {:#X} {:b}", reg_address, value);

        let a = (reg_address.into() as u16).to_be_bytes();
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

/// Origin is bottom-left
pub trait MotionMap: core::ops::Index<(u8, u8), Output = bool> + defmt::Format {}

const ROI_BYTE_COUNT: usize = (0x20C0 - 0x20A1) + 1;
const ROI_COLUMN_COUNT: u8 = 16;
const ROI_ROW_COUNT: u8 = 16;

pub struct SliceMotionMap {
    pub width: u8,
    pub height: u8,
    buf: [u8; ROI_BYTE_COUNT],
}

impl SliceMotionMap {
    fn new(width: u8, height: u8, buf: [u8; ROI_BYTE_COUNT]) -> Self {
        Self { width, height, buf }
    }

    pub fn width(&self) -> u8 {
        self.width
    }

    pub fn height(&self) -> u8 {
        self.height
    }
}

impl MotionMap for SliceMotionMap {}
impl core::ops::Index<(u8, u8)> for SliceMotionMap {
    type Output = bool;

    fn index(&self, index: (u8, u8)) -> &Self::Output {
        let (x, y) = index;
        defmt::assert!(x < self.width, "x is out of range, {}", x);
        defmt::assert!(y < self.height, "y is out of range, {}", x);

        let bit_index = y * self.width + x;
        let byte_index = bit_index / 8;
        let lsb_bit_in_bucket_index = bit_index % 8;

        let lsb_bucket = self.buf[byte_index as usize].reverse_bits(); // reverse the bits to lsb-bit

        let mask = 1 << lsb_bit_in_bucket_index;
        let has_motion = (lsb_bucket & mask) >> lsb_bit_in_bucket_index == 1;
        if has_motion {
            &true
        } else {
            &false
        }
    }
}

impl defmt::Format for SliceMotionMap {
    fn format(&self, fmt: defmt::Formatter) {
        for y in (0..self.height).rev() {
            let i = (y * 2) as usize;
            // println!("y: {} i: {}", y, i);
            let (row_l, row_h) = (self.buf[i], self.buf[i + 1]);
            defmt::write!(fmt, "{:08b}{:08b}", row_l, row_h);
            if y + 1 <= self.height {
                defmt::write!(fmt, "\n");
            }
        }
    }
}
