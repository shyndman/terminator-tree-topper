use bondrewd::{BitfieldEnum, Bitfields};

pub(super) const REG_SIZE: usize = 1;

pub trait Register: bondrewd::Bitfields<REG_SIZE> {
    fn address() -> RegisterAddress;
}

pub trait WritableRegister: Register + Default {}

#[repr(u16)]
pub enum RegisterAddress {
    ModelIdHigh = 0x0000,
    ModelIdLow = 0x0001,
    SiliconRev = 0x0002,
    FrameCountH = 0x0005,
    FrameCountL = 0x0006,

    ModeSelect = 0x0100,
    ImageOrientation = 0x0101,

    ClockControl1 = 0x0300,
}
impl Into<u16> for RegisterAddress {
    fn into(self) -> u16 {
        self as u16
    }
}
impl Into<[u8; 2]> for RegisterAddress {
    fn into(self) -> [u8; 2] {
        (self as u16).to_be_bytes()
    }
}

#[derive(Bitfields, Clone, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1)]
#[derive(defmt::Format)]
pub struct ModeSelectRegister {
    #[bondrewd(bit_length = 5, reserve)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 3)]
    pub mode: Mode,
}
impl Register for ModeSelectRegister {
    fn address() -> RegisterAddress {
        RegisterAddress::ModeSelect
    }
}

#[derive(Bitfields, Clone, PartialEq)]
#[bondrewd(default_endianness = "be", enforce_bytes = 1)]
#[derive(defmt::Format)]
pub struct ClockControl1Register {
    #[bondrewd(bit_length = 4, reserve)]
    reserved: u8,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    pixel_clock_divider: PixelClockDivider,
    #[bondrewd(enum_primitive = "u8", bit_length = 2)]
    sensor_core_divider: SensorCoreDivider,
}
impl Register for ClockControl1Register {
    fn address() -> RegisterAddress {
        RegisterAddress::ClockControl1
    }
}

#[derive(PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 3)]
#[derive(defmt::Format)]
pub enum Mode {
    Standby = 0b000,
    I2cContinuousStreaming = 0b001,
    I2cAutomaticWakeUp = 0b010,
    I2cSnapshotNFrames = 0b011,
    PinContinuousStreaming = 0b100,
    PinSnapshotNFrames = 0b110,
    PinAutomaticWakeUp = 0b111,
    Invalid,
}

#[derive(PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 2)]
#[derive(defmt::Format)]
pub enum PixelClockDivider {
    One = 0b00,
    OneAlt = 0b01,
    Two = 0b10,
    Four = 0b11,
}

#[derive(PartialEq, Clone, BitfieldEnum)]
#[bondrewd_enum(u8, bit_length = 2)]
#[derive(defmt::Format)]
pub enum SensorCoreDivider {
    One = 0b00,
    Two = 0b01,
    Four = 0b10,
    Eight = 0b11,
}
