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
    EmbeddedLineEn = 0x0102,
    SoftwareReset = 0x0103,
    CommandUpdate = 0x0104,

    // Sensor exposure gain control
    IntegrationH = 0x0202,
    IntegrationL = 0x0203,
    AnalogGain = 0x0205,
    DigitalGainH = 0x020E,
    DigitalGainL = 0x020F,

    // Clock control
    ClockControl1 = 0x0300,
    ClockControl2 = 0x0301,
    ClockControl3 = 0x0302,

    // Frame timing control
    FrameLenLinesH = 0x0340,
    FrameLenLinesL = 0x0341,
    LineLenPckH = 0x0342,
    LineLenPckL = 0x0343,

    // Monochrome programming
    MonoMode = 0x0370,
    MonoModeIsp = 0x0371,
    MonoModeSel = 0x0372,

    // Binning mode control
    HSubsample = 0x0380,
    VSubsample = 0x0381,
    BinningMode = 0x0382,

    // Test pattern control
    TestPatternMode = 0x0601,

    // Black level control
    BlcTgt = 0x1004,
    Blc2Tgt = 0x1009,
    MonoCtrl = 0x100A,

    // VSYNC / HSYNC / pixel shift registers
    OpfmCtrl = 0b0001_0000_0000_0000,

    // Tone mapping registers
    CmprsCtrl = 0x102F,
    Cmprs01 = 0x1030,
    Cmprs02 = 0x1031,
    Cmprs03 = 0x1032,
    Cmprs04 = 0x1033,
    Cmprs05 = 0x1034,
    Cmprs06 = 0x1035,
    Cmprs07 = 0x1036,
    Cmprs08 = 0x1037,
    Cmprs09 = 0x1038,
    Cmprs10 = 0x1039,
    Cmprs11 = 0x103A,
    Cmprs12 = 0x103B,
    Cmprs13 = 0x103C,
    Cmprs14 = 0x103D,
    Cmprs15 = 0x103E,
    Cmprs16 = 0x103F,

    // Automatic exposure control
    AeCtrl = 0x2000,
    AeCtrl1 = 0x2001,
    CntOrghH = 0x2002,
    CntOrghL = 0x2003,
    CntOrgvH = 0x2004,
    CntOrgvL = 0x2005,
    CntSthH = 0x2006,
    CntSthL = 0x2007,
    CntStvH = 0x2008,
    CntStvL = 0x2009,
    CtrlPgSkipcnt = 0x200A,
    BvWinWeightEn = 0x200D,
    MaxIntgH = 0x2029,
    MaxIntgL = 0x202A,
    MaxAgain = 0x202B,
    MaxDgainH = 0x202C,
    MaxDgainL = 0x202D,
    MinIntg = 0x202E,
    MinAgain = 0x202F,
    MinDgain = 0x2030,
    TDamping = 0x2031,
    NDamping = 0x2032,
    AlcTh = 0x2033,
    AeTargetMean = 0x2034,
    AeMinMean = 0x2035,
    AeTargetZone = 0x2036,
    ConvergeInTh = 0x2037,
    ConvergeOutTh = 0x2038,
    FsCtrl = 0x203B,
    Fs60HzH = 0x203C,
    Fs60HzL = 0x203D,
    Fs50HzH = 0x203E,
    Fs50HzL = 0x203F,
    FrameCntTh = 0x205B,
    AeMean = 0x205D,
    AeConverge = 0x2060,
    AeBliTgt = 0x2070,

    // Interrupt control
    PulseMode = 0x2061,
    PulseThH = 0x2062,
    PulseThL = 0x2063,
    IntIndic = 0x2064,
    IntClear = 0x2065,
    // Motion detection control
    MdCtrl = 0x2080,
    RoiStartEndV = 0x2081,
    RoiStartEndH = 0x2082,
    MdThMin = 0x2083,
    MdThStrL = 0x2084,
    MdThStrH = 0x2085,
    MdLightCoef = 0x2099,
    MdBlockNumTh = 0x209B,
    MdLatency = 0x209C,
    MdLatencyTh = 0x209D,
    MdCtrl1 = 0x209E,
    // Context switch control registers
    PmuCfg3 = 0x3024,
    PmuCfg4 = 0x3025,
    PmuCfg7 = 0x3028,

    // Operation mode control
    WinMode = 0x3030,

    // IO and clock control
    AnaRegister04 = 0x310F,
    AnaRegister07 = 0x3112,

    // Unnamed(u16) = 0xffff,
}
impl Into<u16> for RegisterAddress {
    fn into(self) -> u16 {
        self as u16

        // if let RegisterAddress::Numeric(address) = self {
        //     address
        // } else {
        //     self as u16
        // }
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
