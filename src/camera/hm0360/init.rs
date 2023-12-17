use super::reg::{
    Mode, RegisterAddress as Addr, HIMAX_FRAME_LENGTH_QVGA, HIMAX_LINE_LEN_PCK_QVGA,
};

pub const HM0360_DEFAULT_REGISTERS: [(u16, u8); 242] = [
    (Addr::SoftwareReset as u16, 0x00),
    (Addr::MonoMode as u16, 0x01),
    (Addr::MonoModeIsp as u16, 0x01),
    (Addr::MonoModeSel as u16, 0x00),
    // BLC control
    (0x1000, 0x43),
    (0x1003, 0x20),
    (Addr::BlcTgt as u16, 0x20),
    (0x1007, 0x01),
    (0x1008, 0x20),
    (Addr::Blc2Tgt as u16, 0x20),
    (Addr::MonoCtrl as u16, 0x01),
    // Output format control
    (Addr::OpfmCtrl as u16, 0b0000_0100),
    // Reserved regs
    (0x101D, 0x00),
    (0x101E, 0x01),
    (0x101F, 0x00),
    (0x1020, 0x01),
    (0x1021, 0x00),
    (Addr::CmprsCtrl as u16, 0x00),
    (Addr::Cmprs01 as u16, 0x09),
    (Addr::Cmprs02 as u16, 0x12),
    (Addr::Cmprs03 as u16, 0x23),
    (Addr::Cmprs04 as u16, 0x31),
    (Addr::Cmprs05 as u16, 0x3E),
    (Addr::Cmprs06 as u16, 0x4B),
    (Addr::Cmprs07 as u16, 0x56),
    (Addr::Cmprs08 as u16, 0x5E),
    (Addr::Cmprs09 as u16, 0x65),
    (Addr::Cmprs10 as u16, 0x72),
    (Addr::Cmprs11 as u16, 0x7F),
    (Addr::Cmprs12 as u16, 0x8C),
    (Addr::Cmprs13 as u16, 0x98),
    (Addr::Cmprs14 as u16, 0xB2),
    (Addr::Cmprs15 as u16, 0xCC),
    (Addr::Cmprs16 as u16, 0xE6),
    (Addr::ClockControl1 as u16, 0xff), // Core = 12MHz PCLKO = 24MHz I2C = 12MHz
    (Addr::ClockControl2 as u16, 0x0A), // MIPI pre-dev (default)
    (Addr::ClockControl3 as u16, 0x77), // PMU/MIPI pre-dev (default)
    (Addr::PmuCfg3 as u16, 0x08),       // Disable context switching
    (Addr::AnaRegister04 as u16, 0b1000_0000),
    (Addr::AnaRegister07 as u16, 0b1100), // PCLKO_polarity falling
    // (Addr::AeCtrl as u16, 0x7F),        // Automatic Exposure (NOTE: Auto framerate enabled)
    // (Addr::AeCtrl1 as u16, 0x00),
    // (Addr::TDamping as u16, 0x20),      // AE T damping factor
    // (Addr::NDamping as u16, 0x00),      // AE N damping factor
    // (Addr::AeTargetMean as u16, 0x64),  // AE target
    // (Addr::AeMinMean as u16, 0x0A),     // AE min target mean
    // (Addr::AeTargetZone as u16, 0x23),  // AE target zone
    // (Addr::ConvergeInTh as u16, 0x03),  // AE converge in threshold
    // (Addr::ConvergeOutTh as u16, 0x05), // AE converge out threshold
    (
        Addr::MaxIntgH as u16,
        ((HIMAX_FRAME_LENGTH_QVGA - 4) >> 8) as u8,
    ),
    (
        Addr::MaxIntgL as u16,
        ((HIMAX_FRAME_LENGTH_QVGA - 4) & 0xFF) as u8,
    ),
    // (Addr::MaxAgain as u16, 0x04), // Maximum analog gain
    // (Addr::MaxDgainH as u16, 0x03),
    // (Addr::MaxDgainL as u16, 0x3F),
    // (Addr::IntegrationH as u16, 0x01),
    // (Addr::IntegrationL as u16, 0x08),
    // (Addr::MdCtrl as u16, 0x6A),
    // (Addr::MdThMin as u16, 0x01),
    // (Addr::MdBlockNumTh as u16, 0x01),
    // (Addr::MdCtrl1 as u16, 0x06),
    (Addr::PulseMode as u16, 0x00), // Interrupt in level mode.
    (Addr::RoiStartEndV as u16, 0xE0),
    (Addr::RoiStartEndH as u16, 0xF0),
    (
        Addr::FrameLenLinesH as u16,
        (HIMAX_FRAME_LENGTH_QVGA >> 8) as u8,
    ),
    (
        Addr::FrameLenLinesL as u16,
        (HIMAX_FRAME_LENGTH_QVGA & 0xFF) as u8,
    ),
    (
        Addr::LineLenPckH as u16,
        (HIMAX_LINE_LEN_PCK_QVGA >> 8) as u8,
    ),
    (
        Addr::LineLenPckL as u16,
        (HIMAX_LINE_LEN_PCK_QVGA & 0xFF) as u8,
    ),
    (Addr::HSubsample as u16, 0x01),
    (Addr::VSubsample as u16, 0x01),
    (Addr::BinningMode as u16, 0x00),
    (Addr::WinMode as u16, 0x01),
    (Addr::ImageOrientation as u16, 0x00),
    (Addr::CommandUpdate as u16, 0x01),
    /// SYNC function config.
    (0x3010, 0x00),
    (0x3013, 0x01),
    (0x3019, 0x00),
    (0x301A, 0x00),
    (0x301B, 0x20),
    (0x301C, 0xFF),
    // PREMETER config.
    (0x3026, 0x03),
    (0x3027, 0x81),
    (0x3028, 0x01),
    (0x3029, 0x00),
    (0x302A, 0x30),
    (0x302E, 0x00),
    (0x302F, 0x00),
    // Magic regs 🪄.
    (0x302B, 0x2A),
    (0x302C, 0x00),
    (0x302D, 0x03),
    (0x3031, 0x01),
    (0x3051, 0x00),
    (0x305C, 0x03),
    (0x3060, 0x00),
    (0x3061, 0xFA),
    (0x3062, 0xFF),
    (0x3063, 0xFF),
    (0x3064, 0xFF),
    (0x3065, 0xFF),
    (0x3066, 0xFF),
    (0x3067, 0xFF),
    (0x3068, 0xFF),
    (0x3069, 0xFF),
    (0x306A, 0xFF),
    (0x306B, 0xFF),
    (0x306C, 0xFF),
    (0x306D, 0xFF),
    (0x306E, 0xFF),
    (0x306F, 0xFF),
    (0x3070, 0xFF),
    (0x3071, 0xFF),
    (0x3072, 0xFF),
    (0x3073, 0xFF),
    (0x3074, 0xFF),
    (0x3075, 0xFF),
    (0x3076, 0xFF),
    (0x3077, 0xFF),
    (0x3078, 0xFF),
    (0x3079, 0xFF),
    (0x307A, 0xFF),
    (0x307B, 0xFF),
    (0x307C, 0xFF),
    (0x307D, 0xFF),
    (0x307E, 0xFF),
    (0x307F, 0xFF),
    (0x3080, 0x01),
    (0x3081, 0x01),
    (0x3082, 0x03),
    (0x3083, 0x20),
    (0x3084, 0x00),
    (0x3085, 0x20),
    (0x3086, 0x00),
    (0x3087, 0x20),
    (0x3088, 0x00),
    (0x3089, 0x04),
    (0x3094, 0x02),
    (0x3095, 0x02),
    (0x3096, 0x00),
    (0x3097, 0x02),
    (0x3098, 0x00),
    (0x3099, 0x02),
    (0x309E, 0x05),
    (0x309F, 0x02),
    (0x30A0, 0x02),
    (0x30A1, 0x00),
    (0x30A2, 0x08),
    (0x30A3, 0x00),
    (0x30A4, 0x20),
    (0x30A5, 0x04),
    (0x30A6, 0x02),
    (0x30A7, 0x02),
    (0x30A8, 0x01),
    (0x30A9, 0x00),
    (0x30AA, 0x02),
    (0x30AB, 0x34),
    (0x30B0, 0x03),
    (0x30C4, 0x10),
    (0x30C5, 0x01),
    (0x30C6, 0xBF),
    (0x30C7, 0x00),
    (0x30C8, 0x00),
    (0x30CB, 0xFF),
    (0x30CC, 0xFF),
    (0x30CD, 0x7F),
    (0x30CE, 0x7F),
    (0x30D3, 0x01),
    (0x30D4, 0xFF),
    (0x30D5, 0x00),
    (0x30D6, 0x40),
    (0x30D7, 0x00),
    (0x30D8, 0xA7),
    (0x30D9, 0x05),
    (0x30DA, 0x01),
    (0x30DB, 0x40),
    (0x30DC, 0x00),
    (0x30DD, 0x27),
    (0x30DE, 0x05),
    (0x30DF, 0x07),
    (0x30E0, 0x40),
    (0x30E1, 0x00),
    (0x30E2, 0x27),
    (0x30E3, 0x05),
    (0x30E4, 0x47),
    (0x30E5, 0x30),
    (0x30E6, 0x00),
    (0x30E7, 0x27),
    (0x30E8, 0x05),
    (0x30E9, 0x87),
    (0x30EA, 0x30),
    (0x30EB, 0x00),
    (0x30EC, 0x27),
    (0x30ED, 0x05),
    (0x30EE, 0x00),
    (0x30EF, 0x40),
    (0x30F0, 0x00),
    (0x30F1, 0xA7),
    (0x30F2, 0x05),
    (0x30F3, 0x01),
    (0x30F4, 0x40),
    (0x30F5, 0x00),
    (0x30F6, 0x27),
    (0x30F7, 0x05),
    (0x30F8, 0x07),
    (0x30F9, 0x40),
    (0x30FA, 0x00),
    (0x30FB, 0x27),
    (0x30FC, 0x05),
    (0x30FD, 0x47),
    (0x30FE, 0x30),
    (0x30FF, 0x00),
    (0x3100, 0x27),
    (0x3101, 0x05),
    (0x3102, 0x87),
    (0x3103, 0x30),
    (0x3104, 0x00),
    (0x3105, 0x27),
    (0x3106, 0x05),
    (0x310B, 0x10),
    (0x3113, 0xA0),
    (0x3114, 0x67),
    (0x3115, 0x42),
    (0x3116, 0x10),
    (0x3117, 0x0A),
    (0x3118, 0x3F),
    (0x311C, 0x10),
    (0x311D, 0x06),
    (0x311E, 0x0F),
    (0x311F, 0x0E),
    (0x3120, 0x0D),
    (0x3121, 0x0F),
    (0x3122, 0x00),
    (0x3123, 0x1D),
    (0x3126, 0x03),
    (0x3128, 0x57),
    (0x312A, 0x11),
    (0x312B, 0x40),
    (0x312E, 0x00),
    (0x312F, 0x00),
    (0x3130, 0x0C),
    (0x3141, 0x2A),
    (0x3142, 0x9F),
    (0x3147, 0x18),
    (0x3149, 0x18),
    (0x314B, 0x01),
    (0x3150, 0x50),
    (0x3152, 0x00),
    (0x3156, 0x2C),
    (0x315A, 0x0A),
    (0x315B, 0x2F),
    (0x315C, 0xE0),
    (0x315F, 0x02),
    (0x3160, 0x1F),
    (0x3163, 0x1F),
    (0x3164, 0x7F),
    (0x3165, 0x7F),
    (0x317B, 0x94),
    (0x317C, 0x00),
    (0x317D, 0x02),
    (0x318C, 0x00),
    (Addr::ModeSelect as u16, Mode::I2cContinuousStreaming as u8),
    (Addr::CommandUpdate as u16, 0x01),
];

pub const HM0360_DEFAULT_REGISTERS2: [(u16, u8); 519] = [
    (Addr::ClockControl1 as u16, 0b0111_0111), // Core = 12MHz PCLKO = 24MHz I2C = 12MHz
    (Addr::ClockControl2 as u16, 0x0A),        // MIPI pre-dev (default)
    (Addr::ClockControl3 as u16, 0x77),        // PMU/MIPI pre-dev (default)
    (0x0350, 0xE0),                            //?not in datasheet
    (0x0370, 0x01),
    (0x0371, 0x01),
    (0x0372, 0x00),
    (
        Addr::MaxIntgH as u16,
        ((HIMAX_FRAME_LENGTH_QVGA - 4) >> 8) as u8,
    ),
    (
        Addr::MaxIntgL as u16,
        ((HIMAX_FRAME_LENGTH_QVGA - 4) & 0xFF) as u8,
    ),
    (
        Addr::FrameLenLinesH as u16,
        (HIMAX_FRAME_LENGTH_QVGA >> 8) as u8,
    ),
    (
        Addr::FrameLenLinesL as u16,
        (HIMAX_FRAME_LENGTH_QVGA & 0xFF) as u8,
    ),
    (
        Addr::LineLenPckH as u16,
        (HIMAX_LINE_LEN_PCK_QVGA >> 8) as u8,
    ),
    (
        Addr::LineLenPckL as u16,
        (HIMAX_LINE_LEN_PCK_QVGA & 0xFF) as u8,
    ),
    (Addr::HSubsample as u16, 0x01),
    (Addr::VSubsample as u16, 0x01),
    //Black Level Control Registers
    (0x1000, 0x43),
    (0x1001, 0x80),
    (0x1003, 0x20),
    (0x1004, 0x20),
    (0x1007, 0x01),
    (0x1008, 0x20),
    (0x1009, 0x20),
    (0x100A, 0x05),
    (0x100B, 0x20),
    (0x100C, 0x20),
    (0x1013, 0x00),
    (0x1014, 0b0000_0100),
    (0x1018, 0x00),
    (0x101D, 0xCF),
    (0x101E, 0x01),
    (0x101F, 0x00),
    (0x1020, 0x01),
    (0x1021, 0x5D),
    (0x102F, 0x08),
    //Tone Mapping Registers
    (0x1030, 0x04),
    (0x1031, 0x08),
    (0x1032, 0x10),
    (0x1033, 0x18),
    (0x1034, 0x20),
    (0x1035, 0x28),
    (0x1036, 0x30),
    (0x1037, 0x38),
    (0x1038, 0x40),
    (0x1039, 0x50),
    (0x103A, 0x60),
    (0x103B, 0x70),
    (0x103C, 0x80),
    (0x103D, 0xA0),
    (0x103E, 0xC0),
    (0x103F, 0xE0),
    (0x1041, 0x00), //?not in datasheet
    //Automatic Exposure
    (0x2000, 0x7F),
    (0x202B, 0x03),
    (0x202C, 0x03),
    (0x202D, 0x00),
    (0x2031, 0x60),
    (0x2032, 0x08),
    (0x2034, 0xA0), //AE Target this is adviced by Himax to improve brightness
    (0x2036, 0x19),
    (0x2037, 0x08),
    (0x2038, 0x10),
    (0x203C, 0x01),
    (0x203D, 0x04),
    (0x203E, 0x01),
    (0x203F, 0x38),
    (0x2048, 0x00),
    (0x2049, 0x10),
    (0x204A, 0x40),
    (0x204B, 0x00),
    (0x204C, 0x08),
    (0x204D, 0x20),
    (0x204E, 0x00),
    (0x204F, 0x38),
    (0x2050, 0xE0),
    (0x2051, 0x00),
    (0x2052, 0x1C),
    (0x2053, 0x70),
    (0x2054, 0x00),
    (0x2055, 0x1A),
    (0x2056, 0xC0),
    (0x2057, 0x00),
    (0x2058, 0x06),
    (0x2059, 0xB0),
    (0x2061, 0x00),
    (0x2062, 0x00),
    (0x2063, 0xC8),
    (0x2080, 0x41),
    (0x2081, 0xE0),
    (0x2082, 0xF0),
    (0x2083, 0x01),
    (0x2084, 0x10),
    (0x2085, 0x10),
    (0x2086, 0x01),
    (0x2087, 0x06),
    (0x2088, 0x0C),
    (0x2089, 0x12),
    (0x208A, 0x1C),
    (0x208B, 0x30),
    (0x208C, 0x10),
    (0x208D, 0x02),
    (0x208E, 0x08),
    (0x208F, 0x0D),
    (0x2090, 0x14),
    (0x2091, 0x1D),
    (0x2092, 0x30),
    (0x2093, 0x08),
    (0x2094, 0x0A),
    (0x2095, 0x0F),
    (0x2096, 0x14),
    (0x2097, 0x18),
    (0x2098, 0x20),
    (0x2099, 0x10),
    (0x209A, 0x00),
    (0x209B, 0x01),
    (0x209C, 0x01),
    (0x209D, 0x11),
    (0x209E, 0x06),
    (0x209F, 0x20),
    (0x20A0, 0x10),
    (0x2590, 0x01),
    (0x2800, 0x00),
    (0x2804, 0x02),
    (0x2805, 0x03),
    (0x2806, 0x03),
    (0x2807, 0x08),
    (0x2808, 0x04),
    (0x2809, 0x0C),
    (0x280A, 0x03),
    (0x280F, 0x03),
    (0x2810, 0x03),
    (0x2811, 0x00),
    (0x2812, 0x09),
    (0x2821, 0xDE),
    (0x3010, 0x00),
    (0x3013, 0x01),
    (0x3019, 0x00),
    (0x301A, 0x00),
    (0x301B, 0x20),
    (0x301C, 0xFF),
    (0x3020, 0x00),
    (0x3021, 0x00),
    (0x3024, 0x08),
    (0x3025, 0x12),
    (0x3026, 0x03),
    (0x3027, 0x81),
    (0x3028, 0x01),
    (0x3029, 0x00),
    (0x302A, 0x30),
    (0x302B, 0x2A),
    (0x302C, 0x00),
    (0x302D, 0x03),
    (0x302E, 0x00),
    (0x302F, 0x00),
    (0x3030, 0x01), //output 640x480
    (0x3031, 0x01),
    (0x3034, 0x00),
    (0x3035, 0x01),
    (0x3051, 0x00),
    (0x305C, 0x03),
    (0x3060, 0x00),
    (0x3061, 0xFA),
    (0x3062, 0xFF),
    (0x3063, 0xFF),
    (0x3064, 0xFF),
    (0x3065, 0xFF),
    (0x3066, 0xFF),
    (0x3067, 0xFF),
    (0x3068, 0xFF),
    (0x3069, 0xFF),
    (0x306A, 0xFF),
    (0x306B, 0xFF),
    (0x306C, 0xFF),
    (0x306D, 0xFF),
    (0x306E, 0xFF),
    (0x306F, 0xFF),
    (0x3070, 0xFF),
    (0x3071, 0xFF),
    (0x3072, 0xFF),
    (0x3073, 0xFF),
    (0x3074, 0xFF),
    (0x3075, 0xFF),
    (0x3076, 0xFF),
    (0x3077, 0xFF),
    (0x3078, 0xFF),
    (0x3079, 0xFF),
    (0x307A, 0xFF),
    (0x307B, 0xFF),
    (0x307C, 0xFF),
    (0x307D, 0xFF),
    (0x307E, 0xFF),
    (0x307F, 0xFF),
    (0x3080, 0x00),
    (0x3081, 0x00),
    (0x3082, 0x00),
    (0x3083, 0x20),
    (0x3084, 0x00),
    (0x3085, 0x20),
    (0x3086, 0x00),
    (0x3087, 0x20),
    (0x3088, 0x00),
    (0x3089, 0x04),
    (0x3094, 0x02),
    (0x3095, 0x02),
    (0x3096, 0x00),
    (0x3097, 0x02),
    (0x3098, 0x00),
    (0x3099, 0x02),
    (0x309E, 0x05),
    (0x309F, 0x02),
    (0x30A0, 0x02),
    (0x30A1, 0x00),
    (0x30A2, 0x08),
    (0x30A3, 0x00),
    (0x30A4, 0x20),
    (0x30A5, 0x04),
    (0x30A6, 0x02),
    (0x30A7, 0x02),
    (0x30A8, 0x01),
    (0x30A9, 0x00),
    (0x30AA, 0x02),
    (0x30AB, 0x34),
    (0x30B0, 0x03),
    (0x30C4, 0x10),
    (0x30C5, 0x01),
    (0x30C6, 0xBF),
    (0x30C7, 0x00),
    (0x30C8, 0x00),
    (0x30CB, 0xFF),
    (0x30CC, 0xFF),
    (0x30CD, 0x7F),
    (0x30CE, 0x7F),
    (0x30D3, 0x01),
    (0x30D4, 0xFF),
    (0x30D5, 0x00),
    (0x30D6, 0xFF),
    (0x30D7, 0x00),
    (0x30D8, 0xA7),
    (0x30D9, 0x05),
    (0x30DA, 0x40),
    (0x30DB, 0xFF),
    (0x30DC, 0x00),
    (0x30DD, 0x27),
    (0x30DE, 0x05),
    (0x30DF, 0x80),
    (0x30E0, 0xFF),
    (0x30E1, 0x00),
    (0x30E2, 0x27),
    (0x30E3, 0x05),
    (0x30E4, 0xC0),
    (0x30E5, 0xFF),
    (0x30E6, 0x00),
    (0x30E7, 0x27),
    (0x30E8, 0x05),
    (0x30E9, 0xC0),
    (0x30EA, 0xFF),
    (0x30EB, 0x00),
    (0x30EC, 0x27),
    (0x30ED, 0x05),
    (0x30EE, 0x00),
    (0x30EF, 0xFF),
    (0x30F0, 0x00),
    (0x30F1, 0xA7),
    (0x30F2, 0x05),
    (0x30F3, 0x40),
    (0x30F4, 0xFF),
    (0x30F5, 0x00),
    (0x30F6, 0x27),
    (0x30F7, 0x05),
    (0x30F8, 0x80),
    (0x30F9, 0xFF),
    (0x30FA, 0x00),
    (0x30FB, 0x27),
    (0x30FC, 0x05),
    (0x30FD, 0xC0),
    (0x30FE, 0xFF),
    (0x30FF, 0x00),
    (0x3100, 0x27),
    (0x3101, 0x05),
    (0x3102, 0xC0),
    (0x3103, 0xFF),
    (0x3104, 0x00),
    (0x3105, 0x27),
    (0x3106, 0x05),
    (0x310B, 0x10),
    (0x310F, 0b1000_0000),
    (0x3112, 0b1100),
    (0x3113, 0xA0),
    (0x3114, 0x67),
    (0x3115, 0x42),
    (0x3116, 0x10),
    (0x3117, 0x0A),
    (0x3118, 0x3F),
    (0x311C, 0x17),
    (0x311D, 0x02),
    (0x311E, 0x0F),
    (0x311F, 0x0E),
    (0x3120, 0x0D),
    (0x3121, 0x0F),
    (0x3122, 0x00),
    (0x3123, 0x1D),
    (0x3126, 0x03),
    (0x3128, 0x57),
    (0x312B, 0x41),
    (0x312E, 0x00),
    (0x312F, 0x00),
    (0x3130, 0x0C),
    (0x3142, 0x9F),
    (0x3147, 0x18),
    (0x3149, 0x18),
    (0x314B, 0x01),
    (0x3150, 0x50),
    (0x3152, 0x00),
    (0x3156, 0x2C),
    (0x315A, 0x05),
    (0x315B, 0x2F),
    (0x315C, 0xE3),
    (0x315F, 0x02),
    (0x3160, 0x1F),
    (0x3163, 0x1F),
    (0x3164, 0x77),
    (0x3165, 0x7F),
    (0x317B, 0x94),
    (0x317C, 0x00),
    (0x317D, 0x02),
    (0x318C, 0x00),
    (0x3500, 0x74),
    (0x3501, 0x0A),
    (0x3502, 0x77),
    (0x3503, 0x02),
    (0x3504, 0x14),
    (0x3505, 0x03),
    (0x3506, 0x00),
    (0x3507, 0x00),
    (0x3508, 0x00),
    (0x3509, 0x00),
    (0x350A, 0xFF),
    (0x350B, 0x00),
    (0x350C, 0x00),
    (0x350D, 0x01),
    (0x350F, 0x00),
    (0x3510, 0x03),
    (0x3512, 0x7F),
    (0x3513, 0x00),
    (0x3514, 0x00),
    (0x3515, 0x01),
    (0x3516, 0x00),
    (0x3517, 0x02),
    (0x3518, 0x00),
    (0x3519, 0x7F),
    (0x351A, 0x00),
    (0x351B, 0x5F),
    (0x351C, 0x00),
    (0x351D, 0x02),
    (0x351E, 0x08),
    (0x351F, 0x03),
    (0x3520, 0x03),
    (0x3521, 0x00),
    (0x3523, 0x60),
    (0x3524, 0x08),
    (0x3525, 0x19),
    (0x3526, 0x08),
    (0x3527, 0x10),
    (0x352A, 0x01),
    (0x352B, 0x04),
    (0x352C, 0x01),
    (0x352D, 0x38),
    (0x3535, 0x02),
    (0x3536, 0x03),
    (0x3537, 0x03),
    (0x3538, 0x08),
    (0x3539, 0x04),
    (0x353A, 0x0C),
    (0x353B, 0x03),
    (0x3540, 0x03),
    (0x3541, 0x03),
    (0x3542, 0x00),
    (0x3543, 0x09),
    (0x3549, 0x04),
    (0x354A, 0x35),
    (0x354B, 0x21),
    (0x354C, 0x01),
    (0x354D, 0xE0),
    (0x354E, 0xF0),
    (0x354F, 0x10),
    (0x3550, 0x10),
    (0x3551, 0x10),
    (0x3552, 0x20),
    (0x3553, 0x10),
    (0x3554, 0x01),
    (0x3555, 0x06),
    (0x3556, 0x0C),
    (0x3557, 0x12),
    (0x3558, 0x1C),
    (0x3559, 0x30),
    (0x355A, 0x74),
    (0x355B, 0x0A),
    (0x355C, 0x77),
    (0x355D, 0x01),
    (0x355E, 0x1C),
    (0x355F, 0x03),
    (0x3560, 0x00),
    (0x3561, 0x01),
    (0x3562, 0x01),
    (0x3563, 0x00),
    (0x3564, 0xFF),
    (0x3565, 0x00),
    (0x3566, 0x00),
    (0x3567, 0x01),
    (0x3569, 0x00),
    (0x356A, 0x03),
    (0x356C, 0x7F),
    (0x356D, 0x00),
    (0x356E, 0x00),
    (0x356F, 0x01),
    (0x3570, 0x00),
    (0x3571, 0x02),
    (0x3572, 0x00),
    (0x3573, 0x3F),
    (0x3574, 0x00),
    (0x3575, 0x2F),
    (0x3576, 0x00),
    (0x3577, 0x01),
    (0x3578, 0x04),
    (0x3579, 0x03),
    (0x357A, 0x03),
    (0x357B, 0x00),
    (0x357D, 0x60),
    (0x357E, 0x08),
    (0x357F, 0x19),
    (0x3580, 0x08),
    (0x3581, 0x10),
    (0x3584, 0x01),
    (0x3585, 0x04),
    (0x3586, 0x01),
    (0x3587, 0x38),
    (0x3588, 0x02),
    (0x3589, 0x12),
    (0x358A, 0x04),
    (0x358B, 0x24),
    (0x358C, 0x06),
    (0x358D, 0x36),
    (0x358F, 0x02),
    (0x3590, 0x03),
    (0x3591, 0x03),
    (0x3592, 0x08),
    (0x3593, 0x04),
    (0x3594, 0x0C),
    (0x3595, 0x03),
    (0x359A, 0x03),
    (0x359B, 0x03),
    (0x359C, 0x00),
    (0x359D, 0x09),
    (0x35A3, 0x02),
    (0x35A4, 0x03),
    (0x35A5, 0x21),
    (0x35A6, 0x01),
    (0x35A7, 0xE0),
    (0x35A8, 0xF0),
    (0x35A9, 0x10),
    (0x35AA, 0x10),
    (0x35AB, 0x10),
    (0x35AC, 0x20),
    (0x35AD, 0x10),
    (0x35AE, 0x01),
    (0x35AF, 0x06),
    (0x35B0, 0x0C),
    (0x35B1, 0x12),
    (0x35B2, 0x1C),
    (0x35B3, 0x30),
    (0x35B4, 0x74),
    (0x35B5, 0x0A),
    (0x35B6, 0x77),
    (0x35B7, 0x00),
    (0x35B8, 0x94),
    (0x35B9, 0x03),
    (0x35BA, 0x00),
    (0x35BB, 0x03),
    (0x35BD, 0x00),
    (0x35BE, 0xFF),
    (0x35BF, 0x00),
    (0x35C0, 0x01),
    (0x35C1, 0x01),
    (0x35C3, 0x00),
    (0x35C4, 0x00),
    (0x35C6, 0x7F),
    (0x35C7, 0x00),
    (0x35C8, 0x00),
    (0x35C9, 0x01),
    (0x35CA, 0x00),
    (0x35CB, 0x02),
    (0x35CC, 0x00),
    (0x35CD, 0x0F),
    (0x35CE, 0x00),
    (0x35CF, 0x0B),
    (0x35D0, 0x00),
    (0x35D3, 0x03),
    (0x35D7, 0x60),
    (0x35D8, 0x04),
    (0x35D9, 0x20),
    (0x35DA, 0x08),
    (0x35DB, 0x14),
    (0x35DC, 0x60),
    (0x35DE, 0x00),
    (0x35DF, 0x82),
    (0x35E9, 0x02),
    (0x35EA, 0x03),
    (0x35EB, 0x03),
    (0x35EC, 0x08),
    (0x35ED, 0x04),
    (0x35EE, 0x0C),
    (0x35EF, 0x03),
    (0x35F4, 0x03),
    (0x35F5, 0x03),
    (0x35F6, 0x00),
    (0x35F7, 0x09),
    (0x35FD, 0x00),
    (0x35FE, 0x5E),
    (0x0104, 0x01),
    (0x0100, 0x01),
];
