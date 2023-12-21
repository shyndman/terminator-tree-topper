pub mod error;
pub mod init;
pub mod reg;

use alloc::vec::Vec;

use defmt::{debug, error, println, trace, warn, Debug2Format};
use embassy_embedded_hal::shared_bus::{asynch::i2c::I2cDevice, I2cDeviceError};
use embassy_rp::{
    dma,
    gpio::{self, Level, Output},
    i2c, into_ref, pio, Peripheral, PeripheralRef,
};
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_time::{Duration, Timer};
use embedded_hal_1::i2c::{AddressMode, SevenBitAddress};
use embedded_hal_async::i2c::I2c;
use error::ErrorKind;
use init::HM0360_DEFAULT_REGISTERS;
use reg::RegisterAddress as Addr;
use snafu::prelude::ensure;

use self::reg::{
    Register, WritableRegister, HIMAX_MD_ROI_QQVGA_H, HIMAX_MD_ROI_QQVGA_W,
    HIMAX_MD_ROI_QVGA_H, HIMAX_MD_ROI_QVGA_W, HIMAX_MD_ROI_VGA_H, HIMAX_MD_ROI_VGA_W,
};
use crate::camera::hm0360::{init::HM0360_DEFAULT_REGISTERS2, reg::ModeSelectRegister};

pub const BUS_ADDRESS_DEFAULT: SevenBitAddress = 0x24;
pub const BUS_ADDRESS_ALT1: SevenBitAddress = 0x25;

const RESET_WAIT_DURATION: Duration = Duration::from_millis(100);
const IMAGE_BYTE_COUNT: usize = 320 * 240;

// MODE_SELECT[2:0]=001

pub struct Hm0360<
    'd,
    M: RawMutex + 'static,
    B: i2c::Instance + 'static,
    I2cAddress: AddressMode,
    Reset: gpio::Pin,
> {
    i2c_device: I2cDevice<'d, M, i2c::I2c<'d, B, i2c::Async>>,
    i2c_address: I2cAddress,
    reset_out: Output<'static, Reset>,
}

impl<M: RawMutex + 'static, B: i2c::Instance + 'static, Reset: gpio::Pin>
    Hm0360<'static, M, B, SevenBitAddress, Reset>
{
    pub async fn new(
        mut i2c_device: I2cDevice<'static, M, i2c::I2c<'static, B, i2c::Async>>,
        i2c_address: SevenBitAddress,
        reset_pin: Reset,
    ) -> Self {
        debug!("[{:#x}] Creating new camera", i2c_address);

        // Build a new camera and initialize
        let mut camera = Self {
            i2c_device,
            i2c_address,
            reset_out: Output::new(reset_pin, Level::High),
        };

        camera.reset().await.unwrap();

        // Read the device's identifier

        trace!("[{:#x}] Verifying camera hardware identifier", i2c_address);

        let h = camera.read_raw(Addr::ModelIdHigh).await.unwrap();
        let l = camera.read_raw(Addr::ModelIdLow).await.unwrap();

        assert!(
            h == 0x03 && l == 0x60,
            "Hm0360 is the only camera supported"
        );

        // Write load registers

        trace!(
            "[{:#x}] Writing camera hardware initialization registers",
            i2c_address
        );

        for (reg_address, value) in HM0360_DEFAULT_REGISTERS2 {
            trace!("Writing {:#X}", reg_address);
            loop {
                match camera.write_raw(reg_address, value).await {
                    Ok(_) => break,
                    Err(e) => {
                        trace!("[{:#x}] Retrying", i2c_address);
                    }
                }
            }
        }

        trace!("[{:#x}] Done writing init registers", i2c_address);

        camera
    }

    pub async fn reset(&mut self) -> Result<(), ErrorKind> {
        trace!("[{:#x}] Triggering camera reset", self.i2c_address);

        self.reset_out.set_high();
        Timer::after(RESET_WAIT_DURATION).await;
        self.reset_out.set_low();
        Timer::after(RESET_WAIT_DURATION).await;
        self.reset_out.set_high();
        Timer::after(RESET_WAIT_DURATION).await;

        Ok(())
    }

    pub async fn set_motion_detection_threshold(
        &mut self,
        threshold: u8,
    ) -> Result<(), ErrorKind> {
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
    ) -> Result<(), ErrorKind> {
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
            _ => return Err(ErrorKind::StateError),
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

    pub async fn enable_motion_detection(&mut self) -> Result<(), ErrorKind> {
        self.clear_motion_detection().await?;
        let reg = self.read_raw(Addr::MdCtrl).await?;
        self.write_raw(Addr::MdCtrl, reg | 1).await;
        Ok(())
    }

    pub async fn is_motion_detected(&mut self) -> Result<bool, ErrorKind> {
        Ok(self
            .read::<reg::InterruptIndicator>()
            .await?
            .motion_detected)
    }

    pub async fn clear_motion_detection(&mut self) -> Result<(), ErrorKind> {
        self.write_raw(Addr::ClearInterrupts, 1 << 3).await
    }

    pub async fn get_motion_map(&mut self) -> Result<impl MotionMap, ErrorKind> {
        let mut buf = self
            .read_raw_n::<u16, ROI_BYTE_COUNT>(0x20A1 as u16)
            .await?;
        buf.reverse();

        Ok(SliceMotionMap::new(ROI_COLUMN_COUNT, ROI_ROW_COUNT, buf))
    }

    pub async fn read<R: Register + defmt::Format>(&mut self) -> Result<R, ErrorKind> {
        match self.read_raw(R::address()).await {
            Ok(val) => Ok(R::from_bytes([val])),
            Err(e) => {
                return Err(e);
            }
        }
    }

    pub async fn read_raw<A: Into<u16> + defmt::Format>(
        &mut self,
        reg_address: A,
    ) -> Result<u8, ErrorKind> {
        let buf = self.read_raw_n::<A, 1>(reg_address).await?;
        Ok(buf[0])
    }

    pub async fn read_raw_n<A: Into<u16> + defmt::Format, const LEN: usize>(
        &mut self,
        reg_address: A,
    ) -> Result<[u8; LEN], ErrorKind> {
        trace!(
            "[{:#x}] read_raw_n {:#X} len={}",
            self.i2c_address,
            reg_address,
            LEN
        );
        let a = (reg_address.into() as u16).to_be_bytes();
        let mut buf = [0u8; LEN];
        match self
            .i2c_device
            .write_read(self.i2c_address, &a, &mut buf)
            .await
        {
            Ok(()) => {
                trace!("[{:#x}] value: {:#b}", self.i2c_address, buf);
                Ok(buf)
            }
            Err(e) => {
                trace!("[{:#x}] error encountered: {}", self.i2c_address, e);
                Err(ErrorKind::I2c)
            }
        }
    }

    pub async fn write<R: WritableRegister + defmt::Format>(
        &mut self,
        reg: R,
    ) -> Result<(), ErrorKind> {
        self.write_raw(R::address(), reg.into_bytes()[0]).await?;
        Ok(())
    }

    pub async fn write_raw<A: Into<u16> + defmt::Format>(
        &mut self,
        reg_address: A,
        value: u8,
    ) -> Result<(), ErrorKind> {
        trace!(
            "[{:#x}] write_register {:#X} {:b}",
            self.i2c_address,
            reg_address,
            value
        );

        let a = (reg_address.into() as u16).to_be_bytes();
        let mut write = [a[0], a[1], value];
        match self.i2c_device.write(self.i2c_address, &write).await {
            Ok(()) => {
                trace!("[{:#x}] write success", self.i2c_address);
                Ok(())
            }
            Err(e) => {
                trace!("[{:#x}] error encountered: {}", self.i2c_address, e);
                return Err(ErrorKind::I2c);
            }
        }
    }
}

/// Origin is bottom-left
pub trait MotionMap: core::ops::Index<(usize, usize), Output = bool> + defmt::Format {
    fn width(&self) -> usize;
    fn height(&self) -> usize;

    fn format(&self, fmt: defmt::Formatter) {
        for y in (0..self.height()).rev() {
            let i = (y * 2) as usize;
            for x in 0..self.width() {
                let val = self[(x, y)];
                defmt::write!(fmt, "{}", val as u8);
            }

            if y + 1 <= self.height() {
                defmt::write!(fmt, "\n");
            }
        }
    }
}

const ROI_BYTE_COUNT: usize = (0x20C0 - 0x20A1) + 1;
const ROI_COLUMN_COUNT: usize = 16;
const ROI_ROW_COUNT: usize = 16;

pub struct SliceMotionMap {
    pub width: usize,
    pub height: usize,
    buf: [u8; ROI_BYTE_COUNT],
}

impl SliceMotionMap {
    fn new(width: usize, height: usize, buf: [u8; ROI_BYTE_COUNT]) -> Self {
        Self { width, height, buf }
    }
}

impl MotionMap for SliceMotionMap {
    fn width(&self) -> usize {
        self.width
    }

    fn height(&self) -> usize {
        self.height
    }
}

impl core::ops::Index<(usize, usize)> for SliceMotionMap {
    type Output = bool;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        let (x, y) = index;
        defmt::assert!(x < self.width(), "x is out of range, {}", x);
        defmt::assert!(y < self.height(), "y is out of range, {}", x);

        let bit_index = y * self.width + x;
        let byte_index = bit_index / 8;
        let lsb_bit_in_bucket_index = bit_index % 8;

        let lsb_bucket = self.buf[byte_index].reverse_bits(); // reverse the bits to lsb-bit

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
        MotionMap::format(self, fmt);
    }
}
