#![deny(unsafe_code)]
#![no_std]

extern crate embedded_hal as hal;
pub use hal::spi::{MODE_0, MODE_3};

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI communication error
    Spi(SpiError),
    /// Chip select pin set error
    Pin(PinError),

    /// Wrong Jedec id detected
    WrongJedecId(JedecId),
}

impl<SpiError, PinError> From<SpiError> for Error<SpiError, PinError> {
    fn from(e: SpiError) -> Self {
        Error::Spi(e)
    }
}

/// SPI interface
#[derive(Debug, Default)]
pub struct SpiInterface<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
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

/// Flash chip api
pub struct W25n512gv<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    bus: SpiInterface<SPI, CS>,
}

impl<SPI, CS, SE, PE> W25n512gv<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, Error<SE, PE>> {
        let mut dev = Self {
            bus: SpiInterface { spi, cs },
        };
        /*let jedec = dev.read_jedec_id()?;
        if jedec.manufacturer_id != 0xEF {
            Err(Error::WrongJedecId(jedec))
        } else if jedec.device_id != [0xAA, 0x20] {
            Err(Error::WrongJedecId(jedec))
        } else {
            Ok(dev)
        }*/
        Ok(dev)
    }

    pub fn read_jedec_id(&mut self) -> Result<JedecId, Error<SE, PE>> {
        let jedec: [u8; 3] = self.bus.execute::<0, 1, 3>(Commands::JEDEC_ID, [])?;
        Ok(JedecId {
            manufacturer_id: jedec[0],
            device_id: [jedec[1], jedec[2]],
        })
    }

    pub fn read_regester(&mut self, register: u8) -> Result<u8, Error<SE, PE>> {
        let value: [u8; 1] = self
            .bus
            .execute::<1, 0, 1>(Commands::READ_STATUS_REGISTER, [register])?;
        Ok(value[0])
    }

    pub fn write_register(&mut self, register: u8, value: u8) -> Result<(), Error<SE, PE>> {
        let _: [u8; 0] = self
            .bus
            .execute::<2, 0, 0>(Commands::WRITE_STATUS_REGISTER, [register, value])?;
        Ok(())
    }

    pub fn disable_write(&mut self) -> Result<(), Error<SE, PE>> {
        let _: [u8; 0] = self.bus.execute::<0, 0, 0>(Commands::WRITE_DISABLE, [])?;
        Ok(())
    }

    pub fn enable_write(&mut self) -> Result<(), Error<SE, PE>> {
        let _: [u8; 0] = self.bus.execute::<0, 0, 0>(Commands::WRITE_ENABLE, [])?;
        Ok(())
    }

    // Skipped bad block managment through block erase not yet implemented

    /// Writes `buf` into the flash chip's internal memory buffer.
    /// This is the first step in writing data to the persistent memory in the flash chip
    /// The next step is issuing a program execute intsrution which will transfer the data in the
    /// memory buffer to a specific page on the flash memory
    pub fn load_program_data(&mut self, buf: &[u8]) -> Result<(), Error<SE, PE>> {
        Ok(())
    }
}

impl<SPI, CS, SE, PE> SpiInterface<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    /// Performs a raw SPI write to the flash chip with `buf`
    pub fn write<const N: usize>(&mut self, buf: [u8; N]) -> Result<(), Error<SE, PE>> {
        {
            self.cs_enable()?;
            self.spi.write(&buf)?;
            self.cs_disable()?;
        }
        Ok(())
    }

    /// Performs a raw SPI transfer with the flash chip, sending it the bits inside `buf` and
    /// immediately writing recieved bits from the flash chip into `buf`
    pub(crate) fn transfer(&mut self, buf: &mut [u8]) -> Result<(), Error<SE, PE>> {
        {
            self.cs_enable()?;
            self.spi.transfer(&mut buf[..])?;
            self.cs_disable()?;
        }
        Ok(())
    }

    /// Executs `instruction` on the flash chip with `params`, returning the requesed number of
    /// result bytes.
    pub fn execute<const PARAMS: usize, const DUMMY_BYTES: usize, const RESULTS: usize>(
        &mut self,
        instruction: u8,
        params: [u8; PARAMS],
    ) -> Result<[u8; RESULTS], Error<SE, PE>> {
        let mut header = Self::prep_header::<PARAMS, DUMMY_BYTES>(instruction, params, RESULTS);

        // `Self::transfer` manages CS for us
        self.transfer(&mut header)?;

        let len = header.len();
        let mut dst = [0u8; RESULTS];
        dst.copy_from_slice(&header[len - RESULTS..]);
        Ok(dst)
    }

    /// Reads a dynamic amount of bytes by executing `instruction` on the flash chip with `params`
    pub fn read_unbounded<const PARAMS: usize, const DUMMY_BYTES: usize>(
        &mut self,
        instruction: u8,
        params: [u8; PARAMS],
        buf: &mut [u8],
    ) -> Result<(), Error<SE, PE>> {
        let header = Self::prep_header::<PARAMS, DUMMY_BYTES>(instruction, params, 0);
        {
            self.cs_enable()?;
            self.spi.write(&header)?;
            // FIXME:
            // Make sure we dont miss bits here! If the function calls too slowly, that may happen!
            self.spi.transfer(&mut buf[..])?;
            self.cs_disable()?;
        }

        Ok(())
    }

    /// Writes a dynamic amount of bytes by executing `instruction` on the flash chip with `params`
    pub fn write_unbounded<const PARAMS: usize, const DUMMY_BYTES: usize>(
        &mut self,
        instruction: u8,
        params: [u8; PARAMS],
        buf: &[u8],
    ) -> Result<(), Error<SE, PE>> {
        let header = Self::prep_header::<PARAMS, DUMMY_BYTES>(instruction, params, 0);

        {
            self.cs_enable()?;
            self.spi.write(&header)?;
            // FIXME:
            // Make sure we dont miss bits here! If the function calls too slowly, that may happen!
            self.spi.write(buf)?;
            self.cs_disable()?;
        }

        Ok(())
    }

    pub(crate) fn cs_disable(&mut self) -> Result<(), Error<SE, PE>> {
        self.cs.set_high().map_err(|pe| Error::Pin(pe))
    }

    pub(crate) fn cs_enable(&mut self) -> Result<(), Error<SE, PE>> {
        self.cs.set_low().map_err(|pe| Error::Pin(pe))
    }

    pub(crate) fn prep_header<const PARAMS: usize, const DUMMY_BYTES: usize>(
        instruction: u8,
        params: [u8; PARAMS],
        additional_bytes: usize,
    ) -> heapless::Vec<u8, 16> {
        let mut header = heapless::Vec::<u8, 16>::new();
        header.push(instruction).unwrap();

        for &param in params.iter() {
            header.push(param).unwrap();
        }
        for _ in 0..DUMMY_BYTES {
            header.push(0).unwrap();
        }
        for _ in 0..additional_bytes {
            header.push(0).unwrap();
        }
        header
    }
}

pub(crate) struct Addresses;
#[allow(dead_code)]
impl Addresses {
    const PROTECTION_REGISTER: u8 = 0xA0;
    const CONFIGURATION_REGISTER: u8 = 0xB0;
    const STATUS_REGISTER: u8 = 0xC0;
}

pub(crate) struct Commands;
#[allow(dead_code)]
impl Commands {
    const UNIQUE_ID: u8 = 0x4B;
    const DEVICE_RESET: u8 = 0xFF;

    /// Dummy, EFh, AAh, 20h
    const JEDEC_ID: u8 = 0x9F;

    /// / 05h. SR_Addr S7-0 S7-0 S7-0 S7-0 S7-0 S7-0 S7-0
    const READ_STATUS_REGISTER: u8 = 0x0F;

    /// / 01h. SR_Addr S7-0
    const WRITE_STATUS_REGISTER: u8 = 0x1F;

    const WRITE_ENABLE: u8 = 0x06;

    const WRITE_DISABLE: u8 = 0x04;

    /// LBA, LBA, PBA, PBA
    const BB_MANAGEMENT_SWAP_BLOCKS: u8 = 0xA1;

    /// Dummy, LBA0, LBA0, PBA0, PBA0, LBA1, LBA1, PBA1
    const READ_BBM_LUT: u8 = 0xA5;

    /// Dummy, PA15,-8 PA7-0
    const LAST_ECC_FAILURE_PAGE_ADDRESS: u8 = 0xA9;

    /// Dummy, PA15-8 PA7-0
    const BLOCK_ERASE: u8 = 0xD8;

    /// CA15-8, CA7-0, Data-0, Data-1, Data-2, Data-3, Data-4, Data-5
    const PROGRAM_DATA_LOAD_RESET_BUFFER: u8 = 0x02;

    /// CA15-8, CA7-0, Data-0, Data-1, Data-2, Data-3, Data-4, Data-5
    const RANDOM_PROGRAM_DATA_LOAD: u8 = 0x84;

    /// CA15-8, CA7-0, Data-0 / 4, Data-1 / 4, Data-2 / 4, Data-3 / 4, Data-4 / 4, Data-5 / 4
    const QUAD_PROGRAM_DATA_LOAD_RESET_BUFFER: u8 = 0x32;

    /// CA15-8, CA7-0, Data-0 / 4, Data-1 / 4, Data-2 / 4, Data-3 / 4, Data-4 / 4, Data-5 / 4
    const RANDOM_QUAD_PROGRAM_DATA_LOAD: u8 = 0x34;

    /// Dummy, PA15-8, PA7-0
    const PROGRAM_EXECUTE: u8 = 0x10;

    /// Dummy, PA15-8, PA7-0
    const PAGE_DATA_READ: u8 = 0x13;

    /// (CA15-8, CA7-0), Dummy, D7-0, D7-0, D7-0, D7-0, D7-0
    const READ: u8 = 0x03;

    /// (CA15-8, CA7-0), Dummy, D7-0, D7-0, D7-0, D7-0, D7-0
    const FAST_READ: u8 = 0x0B;

    /// (CA15-8, CA7-0), Dummy, Dummy, Dummy, D7-0, D7-0, D7-0
    const FAST_READ_4_BYTE_ADDRESS: u8 = 0x0C;

    /// (CA15-8, CA7-0), Dummy, D7-0 / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2
    const FAST_READ_DUAL_OUTPUT: u8 = 0x3B;

    /// (CA15-8, CA7-0), Dummy, Dummy, Dummy, D7-0 / 2, D7-0 / 2, D7-0 / 2
    const FAST_READ_DUAL_OUTPUT_WITH_4_BYTE_ADDRESS: u8 = 0x3C;

    /// (CA15-8, CA7-0), Dummy, D7-0 / 4, D7-0 / 4, D7-0 / 4, D7-0 / 4, D7-0 / 4
    const FAST_READ_QUAD_OUTPUT: u8 = 0x6B;

    /// / 6Ch (CA15-8, CA7-0), Dummy, Dummy, Dummy, D7-0 / 4 D7-0 / 4 D7-0 / 4
    const FAST_READ_QUAD_OUTPUT_WITH_4_BYTE_ADDRESS: u8 = 0x6C;

    /// (CA15-8 / 2, CA7-0 / 2), Dummy / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2
    const FAST_READ_DUAL_I_O: u8 = 0xBB;

    /// (CA15-8 / 2, CA7-0 / 2), Dummy / 2, Dummy / 2, Dummy / 2, D7-0 / 2, D7-0 / 2, D7-0 / 2
    const FAST_READ_DUAL_I_O_WITH_4_BYTE_ADDRESS: u8 = 0xBC;

    /// (CA15-8 / 4, CA7-0 / 4), Dummy / 4, Dummy / 4, D7-0 / 4, D7-0 / 4, D7-0 / 4, D7-0 / 4
    const FAST_READ_QUAD_I_O: u8 = 0xEB;

    /// (CA15-8 / 4, CA7-0 / 4), Dummy / 4, Dummy / 4, Dummy / 4, Dummy / 4, Dummy / 4, D7-0 / 4
    const FAST_READ_QUAD_I_O_WITH_4_BYTE_ADDRESS: u8 = 0xEC;

    const DEEP_POWER_DOWN: u8 = 0xB9;

    const RELEASE_POWER_DOWN: u8 = 0xAB;

    /// Chip Erase C7h/60h
    const CHIP_ERASE: u8 = 0xC7;

    const ENABLE_RESET: u8 = 0x66;

    const RESET_DEVICE: u8 = 0x99;
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct JedecId {
    pub manufacturer_id: u8,
    pub device_id: [u8; 2],
}
