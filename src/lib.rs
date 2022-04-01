#![deny(unsafe_code)]
#![no_std]

extern crate embedded_hal as hal;
pub use hal::spi::{MODE_0, MODE_3};

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI communication error
    Spi(SpiError),
    /// Chip select pin set error
    Pin(PinError),
}

/// SPI interface
#[derive(Debug, Default)]
pub struct SpiInterface<SPI, CS> {
    pub(crate) spi: SPI,
    pub(crate) cs: CS,
}

pub mod regs {
    rumio::define_mmio_register! {
        ProtectionRegister: u16 {
            rw StatusRegisterProtect0: 7,
            rw BlockProtect3:          6,
            rw BlockProtect2:          5,
            rw BlockProtect1:          4,
            rw BlockProtect0:          3,
            rw TopBottomProtect:       2,
            rw WPEnable:               1,
            rw StatusRegisterProtect1: 0,
        }
    }

    rumio::define_mmio_register! {
        ConfigurationRegister: u16 {
            rw OtpDataPagesLock:    7,
            rw EnterOtpMode:        6,
            rw StatusRegister1Lock: 5,
            rw EnableECC:           4,
            rw BufferMode:          3,
            rw OutputDriverStrength: 1..2 = enum DriverStrength [
                Strength100 = 0b00,
                Strength75 =  0b01,
                Strength50 =  0b10,
                Strength25 =  0b11,
            ],
            rw HoldDisable:         0,
        }
    }

    rumio::define_mmio_register! {
        StatusRegister: u16 {
            rw BbmLutFull:          6,
            rw EccStatusBit: 4..5 = enum EccStatus [
                /// Entire data output is successful, without any ECC correction.
                Success =                   0b00,
                /// Entire data output is successful, with 1~4 bit/page ECC corrections in either
                /// a single page or multiple pages.
                SuccessWithCorrections =    0b01,
                /// Entire data output contains more than 4 bits errors only in a single page which
                /// cannot be repaired by ECC.
                /// In the Continuous Read Mode, an additional command can be used to read out the
                /// Page Address (PA) which had the errors.
                SinglePageFailure =         0b10,
                /// Entire data output contains more than 4 bits errors/page in multiple pages.
                /// In the Continuous Read Mode, the additional command can only provide the last
                /// Page Address (PA) that had failures, the user cannot obtain the PAs for other
                /// failure pages.
                ///
                /// Data is not suitable to use.
                MultiPageFailure =          0b11,
            ],
            rw ProgramFailure:      3,
            rw EraseFailure:        2,
            rw WriteEnableLatch:    1,
            rw OperationInProgress: 0,
        }
    }
}

pub(crate) struct Commands;
#[allow(non_camel_case_types, dead_code)]
impl Commands {
    const UNIQUE_ID: u8 = 0x4B;
    const DEVICE_RESET: u8 = 0xFF;

    /// Dummy EFh AAh 20h
    const JEDEC_ID: u8 = 0x9F;

    // / 05h SR Addr S7-0 S7-0 S7-0 S7-0 S7-0 S7-0 S7-0
    const READ_STATUS_REGISTER: u8 = 0x0F;

    /// / 01h SR Addr S7-0
    const WRITE_STATUS_REGISTER: u8 = 0x1F;

    const WRITE_ENABLE: u8 = 0x06;

    const WRITE_DISABLE: u8 = 0x04;

    /// LBA LBA PBA PBA
    const BB_MANAGEMENT_SWAP_BLOCKS: u8 = 0xA1;

    /// Dummy LBA0 LBA0 PBA0 PBA0 LBA1 LBA1 PBA1
    const READ_BBM_LUT: u8 = 0xA5;

    /// Dummy PA15-8 PA7-0
    const LAST_ECC_FAILURE_PAGE_ADDRESS: u8 = 0xA9;

    /// Dummy PA15-8 PA7-0
    const BLOCK_ERASE: u8 = 0xD8;

    /// CA15-8 CA7-0 Data-0 Data-1 Data-2 Data-3 Data-4 Data-5
    const PROGRAM_DATA_LOAD_RESET_BUFFER: u8 = 0x02;

    /// CA15-8 CA7-0 Data-0 Data-1 Data-2 Data-3 Data-4 Data-5
    const RANDOM_PROGRAM_DATA_LOAD: u8 = 0x84;

    /// CA15-8 CA7-0 Data-0 / 4 Data-1 / 4 Data-2 / 4 Data-3 / 4 Data-4 / 4 Data-5 / 4
    const QUAD_PROGRAM_DATA_LOAD_RESET_BUFFER: u8 = 0x32;

    /// CA15-8 CA7-0 Data-0 / 4 Data-1 / 4 Data-2 / 4 Data-3 / 4 Data-4 / 4 Data-5 / 4
    const RANDOM_QUAD_PROGRAM_DATA_LOAD: u8 = 0x34;

    /// Dummy PA15-8 PA7-0
    const PROGRAM_EXECUTE: u8 = 0x10;

    /// Dummy PA15-8 PA7-0
    const PAGE_DATA_READ: u8 = 0x13;

    /// CA15-8 CA7-0 Dummy D7-0 D7-0 D7-0 D7-0 D7-0
    const READ: u8 = 0x03;

    /// CA15-8 CA7-0 Dummy D7-0 D7-0 D7-0 D7-0 D7-0
    const FAST_READ: u8 = 0x0B;

    /// CA15-8 CA7-0 Dummy Dummy Dummy D7-0 D7-0 D7-0
    const FAST_READ_4_BYTE_ADDRESS: u8 = 0x0C;

    /// CA15-8 CA7-0 Dummy D7-0 / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2
    const FAST_READ_DUAL_OUTPUT: u8 = 0x3B;

    /// CA15-8 CA7-0 Dummy Dummy Dummy D7-0 / 2 D7-0 / 2 D7-0 / 2
    const FAST_READ_DUAL_OUTPUT_WITH_4_BYTE_ADDRESS: u8 = 0x3C;

    /// CA15-8 CA7-0 Dummy D7-0 / 4 D7-0 / 4 D7-0 / 4 D7-0 / 4 D7-0 / 4
    const FAST_READ_QUAD_OUTPUT: u8 = 0x6B;

    /// 6Ch CA15-8 CA7-0 Dummy Dummy Dummy D7-0 / 4 D7-0 / 4 D7-0 / 4
    const FAST_READ_QUAD_OUTPUT_WITH_4_BYTE_ADDRESS: u8 = 0x6C;

    /// CA15-8 / 2 CA7-0 / 2 Dummy / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2
    const FAST_READ_DUAL_I_O: u8 = 0xBB;

    /// CA15-8 / 2 CA7-0 / 2 Dummy / 2 Dummy / 2 Dummy / 2 D7-0 / 2 D7-0 / 2 D7-0 / 2
    const FAST_READ_DUAL_I_O_WITH_4_BYTE_ADDRESS: u8 = 0xBC;

    /// CA15-8 / 4 CA7-0 / 4 Dummy / 4 Dummy / 4 D7-0 / 4 D7-0 / 4 D7-0 / 4 D7-0 / 4
    const FAST_READ_QUAD_I_O: u8 = 0xEB;

    /// CA15-8 / 4 CA7-0 / 4 Dummy / 4 Dummy / 4 Dummy / 4 Dummy / 4 Dummy / 4 D7-0 / 4
    const FAST_READ_QUAD_I_O_WITH_4_BYTE_ADDRESS: u8 = 0xEC;

    const DEEP_POWER_DOWN: u8 = 0xB9;

    const RELEASE_POWER_DOWN: u8 = 0xAB;

    /// Chip Erase C7h/60h
    const CHIP_ERASE: u8 = 0xC7;

    const ENABLE_RESET: u8 = 0x66;

    const RESET_DEVICE: u8 = 0x99;
}

impl<SPI, CS> SpiInterface<SPI, CS> {}
