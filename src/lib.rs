#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]
// Used for parameterizing writable and mode status into wrapper types

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
pub use embedded_hal::spi::{MODE_0, MODE_3};
//FIXME: Once stm32f4xx hal compiles on nightly, we can re-add tock-registers!
//TOCK_TODO use tock_registers::LocalRegisterCopy;

pub const RAW_PAGE_SIZE: usize = 2112;
pub const PAGE_SIZE_WITHOUT_ECC: usize = RAW_PAGE_SIZE;
pub const PAGE_SIZE_WITH_ECC: usize = 2048;

/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI communication error
    Spi(SpiError),
    /// Chip select pin set error
    Pin(PinError),

    /// Wrong Jedec id detected
    WrongJedecId(JedecId),

    LengthTooLarge(usize),
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
    /*tock_registers::register_bitfields! [
        u8,
        pub Protection [
            /// Status register protect lock-0
            SRP0  OFFSET(7) NUMBITS(1) [],
            /// Block protect bit 3
            BP3  OFFSET(6) NUMBITS(1) [],
            /// Block protect bit 2
            BP2  OFFSET(5) NUMBITS(1) [],
            /// Block protect bit 1
            BP1  OFFSET(4) NUMBITS(1) [],
            /// Block protect bit 0
            BP0  OFFSET(3) NUMBITS(1) [],
            /// Top/Bottom block protect bit
            TB  OFFSET(2) NUMBITS(1) [],
            /// WP-enable bit
            WP_E  OFFSET(1) NUMBITS(1) [],
            /// Status register protect lock-1
            SRP1  OFFSET(0) NUMBITS(1) [],
        ],
        pub Configuration [
            /// One time program lock
            OTP_L  OFFSET(7) NUMBITS(1) [],
            /// Enter OPT mode
            OTP_E  OFFSET(6) NUMBITS(1) [],
            /// Status register 1 lock
            SR1_L  OFFSET(5) NUMBITS(1) [],
            /// Enable ECC
            ECC_E  OFFSET(4) NUMBITS(1) [],
            /// Buffer mode
            BUF  OFFSET(3) NUMBITS(1) [
                BufferRead = 0,
                ContinuousBufferRead = 1,
            ],

            /// Output strength
            ODS OFFSET(1) NUMBITS(2) [
                P100 = 0b00,
                P75 = 0b01,
                P50 = 0b10,
                P25 = 0b11,
            ],
            /// Hold disable
            H_DIS  OFFSET(0) NUMBITS(1) [],
        ],

        pub Status [
            /// Enter OPT mode
            LUT_F  OFFSET(6) NUMBITS(1) [],
            ECC  OFFSET(4) NUMBITS(2) [
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

            P_FAIL  OFFSET(3) NUMBITS(1) [],
            E_FAIL  OFFSET(2) NUMBITS(1) [],
            WEL  OFFSET(1) NUMBITS(1) [],
            BUSY  OFFSET(0) NUMBITS(1) [],
        ],
    ];
    */
}

/// Low level flash chip api with no type protections for device/register misuse
pub struct W25n512gvImpl<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    bus: SpiInterface<SPI, CS>,
}

#[derive(PartialEq, Eq)]
pub enum Writability {
    Enabled,
    Disabled,
}

pub struct W25n512gvWE<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    inner: W25n512gvImpl<SPI, CS>,
}

pub struct W25n512gvWD<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    inner: W25n512gvImpl<SPI, CS>,
}

pub fn new<SPI, CS, SE, PE>(spi: SPI, cs: CS) -> Result<W25n512gvWD<SPI, CS>, Error<SE, PE>>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    Ok(W25n512gvWD {
        inner: W25n512gvImpl::new(spi, cs)?,
    })
}

pub trait W25n512gv<SPI, CS, SE, PE>: Sized
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type BufferType: BufferRef<SPI, CS, SE, PE>;

    fn new_impl(inner: W25n512gvImpl<SPI, CS>) -> Self;

    fn get_impl(&mut self) -> &mut W25n512gvImpl<SPI, CS>;
    fn take_impl(self) -> W25n512gvImpl<SPI, CS>;

    fn read_protection_register(&mut self) -> Result<u8, Error<SE, PE>> {
        Ok(self
            .get_impl()
            .read_regester_impl(Addresses::STATUS_REGISTER)?)
    }

    fn read_configuration_register(&mut self) -> Result<u8, Error<SE, PE>> {
        Ok(self
            .get_impl()
            .read_regester_impl(Addresses::CONFIGURATION_REGISTER)?)
    }

    fn read_status_register(&mut self) -> Result<u8, Error<SE, PE>> {
        Ok(self
            .get_impl()
            .read_regester_impl(Addresses::STATUS_REGISTER)?)
    }

    fn write_protection_register(&mut self, value: u8) -> Result<(), Error<SE, PE>> {
        self.get_impl()
            .write_register_impl(Addresses::PROTECTION_REGISTER, value)
    }

    fn write_configuration_register(&mut self, value: u8) -> Result<(), Error<SE, PE>> {
        self.get_impl()
            .write_register_impl(Addresses::CONFIGURATION_REGISTER, value)
    }

    fn write_status_register(&mut self, value: u8) -> Result<(), Error<SE, PE>> {
        self.get_impl()
            .write_register_impl(Addresses::STATUS_REGISTER, value)
    }

    fn modify_status_register<F>(&mut self, f: F) -> Result<(), Error<SE, PE>>
    where
        F: FnOnce(&mut u8),
    {
        let mut val = self.read_status_register()?;
        f(&mut val);
        self.write_status_register(val)
    }

    fn modify_protection_register<F>(&mut self, f: F) -> Result<(), Error<SE, PE>>
    where
        F: FnOnce(&mut u8),
    {
        let mut val = self.read_protection_register()?;
        f(&mut val);
        self.write_protection_register(val)
    }

    fn modify_configuration_register<F>(&mut self, f: F) -> Result<(), Error<SE, PE>>
    where
        F: FnOnce(&mut u8),
    {
        let mut val = self.read_configuration_register()?;
        f(&mut val);
        self.write_configuration_register(val)
    }

    /// Reads the specified memory page on the flag chip to the flash chip's internal buffer.
    ///
    /// This is the first step in reading data from the flash chip
    // TODO: Return future for more efficent waiting while we upload to the flash chip
    fn read_sync(mut self, page_addr: u16) -> Result<Self::BufferType, Error<SE, PE>> {
        self.get_impl().page_data_read_impl(page_addr)?;
        self.get_impl().block_until_not_busy_impl()?;
        Ok(Self::BufferType::new(self.take_impl()))
    }

    /// Resets the chip to its default state.
    ///
    /// Consumes the device, returning the bus.
    fn reset(mut self, delay: &mut impl DelayUs<u16>) -> (SPI, CS) {
        // Make this infailable so that we can still reset in the event of an error
        let _ = self.get_impl().reset_impl();
        let bus = self.into_inner();
        delay.delay_us(500);
        bus
    }

    /// Consumes the device, returning the bus.
    fn into_inner(self) -> (SPI, CS) {
        self.take_impl().bus.into_inner()
    }
}

impl<SPI, CS, SE, PE> W25n512gvWE<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    pub fn disable_write(mut self) -> Result<W25n512gvWD<SPI, CS>, Error<SE, PE>> {
        self.get_impl().disable_write_impl()?;
        Ok(W25n512gvWD {
            inner: self.take_impl(),
        })
    }

    /// Uploads `data` to the flash chip's internal buffer, blocking until completed.
    ///
    /// This is the first step in programming the flash chip
    // TODO: Return future for more efficent waiting while we upload to the flash chip
    pub fn upload_to_buffer_sync(
        mut self,
        data: &[u8],
    ) -> Result<BufferRefWE<SPI, CS>, Error<SE, PE>> {
        self.get_impl().load_program_data_impl(0, data)?;
        Ok(BufferRefWE {
            inner: self.take_impl(),
        })
    }

    /// Erases 64 pages (one block) on the flash chip
    pub fn erase_block(mut self, block_addr: u16) -> Result<W25n512gvWD<SPI, CS>, Error<SE, PE>> {
        self.get_impl().block_erase_impl(block_addr)?;
        self.get_impl().block_until_not_busy_impl()?;
        Ok(W25n512gvWD {
            inner: self.take_impl(),
        })
    }

    /// Erases the entire contents of the flash chip
    pub fn erase_all(mut self) -> Result<W25n512gvWD<SPI, CS>, Error<SE, PE>> {
        self.get_impl().chip_erase_impl()?;
        self.get_impl().block_until_not_busy_impl()?;
        Ok(W25n512gvWD {
            inner: self.take_impl(),
        })
    }
}

impl<SPI, CS, SE, PE> W25n512gv<SPI, CS, SE, PE> for W25n512gvWE<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type BufferType = BufferRefWE<SPI, CS>;

    fn new_impl(inner: W25n512gvImpl<SPI, CS>) -> Self {
        Self { inner }
    }

    fn get_impl(&mut self) -> &mut W25n512gvImpl<SPI, CS> {
        &mut self.inner
    }

    fn take_impl(self) -> W25n512gvImpl<SPI, CS> {
        self.inner
    }
}

impl<SPI, CS, SE, PE> W25n512gv<SPI, CS, SE, PE> for W25n512gvWD<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type BufferType = BufferRefWD<SPI, CS>;

    fn new_impl(inner: W25n512gvImpl<SPI, CS>) -> Self {
        Self { inner }
    }

    fn get_impl(&mut self) -> &mut W25n512gvImpl<SPI, CS> {
        &mut self.inner
    }

    fn take_impl(self) -> W25n512gvImpl<SPI, CS> {
        self.inner
    }
}

pub trait BufferRef<SPI, CS, SE, PE>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type Device: W25n512gv<SPI, CS, SE, PE>;

    fn new(inner: W25n512gvImpl<SPI, CS>) -> Self;

    fn finish(self) -> Self::Device;

    fn get_inner(&mut self) -> &mut W25n512gvImpl<SPI, CS>;

    /// Reads the content of the flash chip's buffer into memory using SPI, blocking until complete
    fn download_from_buffer_sync(&mut self, data: &mut [u8]) -> Result<(), Error<SE, PE>> {
        self.get_inner().page_read_continous_impl(data)
    }
}

pub struct BufferRefWE<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    inner: W25n512gvImpl<SPI, CS>,
}

impl<SPI, CS, SE, PE> BufferRef<SPI, CS, SE, PE> for BufferRefWE<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type Device = W25n512gvWE<SPI, CS>;

    fn new(inner: W25n512gvImpl<SPI, CS>) -> Self {
        Self { inner }
    }

    fn finish(self) -> Self::Device {
        W25n512gvWE::new_impl(self.inner)
    }

    fn get_inner(&mut self) -> &mut W25n512gvImpl<SPI, CS> {
        &mut self.inner
    }
}

pub struct BufferRefWD<SPI, CS>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    inner: W25n512gvImpl<SPI, CS>,
}

impl<SPI, CS, SE, PE> BufferRef<SPI, CS, SE, PE> for BufferRefWD<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    type Device = W25n512gvWD<SPI, CS>;

    fn new(inner: W25n512gvImpl<SPI, CS>) -> Self {
        Self { inner }
    }

    fn finish(self) -> W25n512gvWD<SPI, CS> {
        W25n512gv::new_impl(self.inner)
    }

    fn get_inner(&mut self) -> &mut W25n512gvImpl<SPI, CS> {
        &mut self.inner
    }
}

impl<SPI, CS, SE, PE> BufferRefWE<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    /// Writes the content of the flash chip's buffer to a page within the flash storage of the
    /// flash chip, blocking until completed
    pub fn commit_sync(mut self, page_addr: u16) -> Result<BufferRefWD<SPI, CS>, Error<SE, PE>> {
        self.inner.program_execute_impl(page_addr)?;
        self.inner.block_until_not_busy_impl()?;
        Ok(BufferRefWD { inner: self.inner })
    }
}

impl<SPI, CS, SE, PE> BufferRefWD<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
}

impl<SPI, CS, SE, PE> W25n512gvWD<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    pub fn enable_write(mut self) -> Result<W25n512gvWE<SPI, CS>, Error<SE, PE>> {
        self.get_impl().enable_write_impl()?;
        Ok(W25n512gvWE { inner: self.inner })
    }
}

impl<SPI, CS, SE, PE> W25n512gvImpl<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    pub fn new(spi: SPI, cs: CS) -> Result<Self, Error<SE, PE>> {
        let mut dev = Self {
            bus: SpiInterface { spi, cs },
        };
        let jedec = dev.read_jedec_id_impl()?;

        if jedec == JedecId::new(0xEF, 0xAA, 0x20) || jedec == JedecId::new(0xEF, 0xAA, 0x21) {
            Ok(dev)
        } else {
            Err(Error::WrongJedecId(jedec))
        }
    }
}

/// Low level implementation functions
///
/// Each _impl function wraps an instruction that can be executed by the flash chip. This API maps
/// very closely to what is provided on the chip, so virtually no invariants are enforced at this
/// level.
///
/// See the high level implementations which use rust types to make things safer
impl<SPI, CS, SE, PE> W25n512gvImpl<SPI, CS>
where
    SPI: Transfer<u8, Error = SE> + Write<u8, Error = SE>,
    CS: OutputPin<Error = PE>,
{
    pub(crate) fn read_jedec_id_impl(&mut self) -> Result<JedecId, Error<SE, PE>> {
        // 4 elements because the first 8 bits are garbage
        let jedec: [u8; 4] = self.bus.execute([Commands::JEDEC_ID])?;
        Ok(JedecId {
            manufacturer_id: jedec[1],
            device_id: [jedec[2], jedec[3]],
        })
    }

    pub(crate) fn read_regester_impl(&mut self, register: u8) -> Result<u8, Error<SE, PE>> {
        let value: [u8; 1] = self
            .bus
            .execute([Commands::READ_STATUS_REGISTER, register])?;
        Ok(value[0])
    }

    pub(crate) fn write_register_impl(
        &mut self,
        register: u8,
        value: u8,
    ) -> Result<(), Error<SE, PE>> {
        self.bus
            .write([Commands::WRITE_STATUS_REGISTER, register, value])
    }

    pub(crate) fn disable_write_impl(&mut self) -> Result<(), Error<SE, PE>> {
        self.bus.write([Commands::WRITE_DISABLE])
    }

    pub(crate) fn enable_write_impl(&mut self) -> Result<(), Error<SE, PE>> {
        self.bus.write([Commands::WRITE_ENABLE])
    }

    // Skipped bad block managment

    /// NEEDS BLOCK
    pub(crate) fn block_erase_impl(&mut self, block_address: u16) -> Result<(), Error<SE, PE>> {
        let block_address = block_address.to_be_bytes();
        self.bus
            .write([Commands::BLOCK_ERASE, 0, block_address[0], block_address[1]])
    }

    /// Writes `buf` into the flash chip's internal memory buffer.
    /// This is the first step in writing data to the persistent memory in the flash chip
    /// The next step is issuing a program execute intsrution which will transfer the data in the
    /// memory buffer to a specific page on the flash memory
    ///
    /// NOTE: Requires write enable to be active before the device will accept the data
    pub(crate) fn load_program_data_impl(
        &mut self,
        col_addr: u16,
        buf: &[u8],
    ) -> Result<(), Error<SE, PE>> {
        if buf.len() > RAW_PAGE_SIZE {
            return Err(Error::LengthTooLarge(buf.len()));
        }
        let col_addr = col_addr.to_be_bytes();
        self.bus.write_unbounded(
            [
                Commands::PROGRAM_DATA_LOAD_RESET_BUFFER,
                col_addr[0],
                col_addr[1],
            ],
            buf,
        )
    }

    /// NEEDS BLOCK
    /// The Program Execute instruction is the second step of the Program operation. After the program data are
    /// loaded into the 2,112-Byte Data Buffer (or 2,048 bytes when ECC is enabled), the Program Execute
    /// instruction will program the Data Buffer content into the physical memory page that is specified in the
    /// instruction. The instruction is initiated by driving the /CS pin low then shifting the instruction code “10h”
    /// followed by 8-bit dummy clocks and the 16-bit Page Address into the DI pin as shown in Figure 17.
    ///
    /// After /CS is driven high to complete the instruction cycle, the self-timed Program Execute instruction will
    /// commence for a time duration of tpp (See AC Characteristics). While the Program Execute cycle is in
    /// progress, the Read Status Register instruction may still be used for checking the status of the BUSY bit.
    /// The BUSY bit is a 1 during the Program Execute cycle and becomes a 0 when the cycle is finished and the
    /// device is ready to accept other instructions again. After the Program Execute cycle has finished, the Write
    /// Enable Latch (WEL) bit in the Status Register is cleared to 0. The Program Execute instruction will not be
    /// executed if the addressed page is protected by the Block Protect (TB, BP2, BP1, and BP0) bits. Only 4
    /// partial page program times are allowed on every single page.
    ///
    /// The pages within the block have to be programmed sequentially from the lower order page address to the
    /// higher order page address within the block. Programming pages out of sequence is prohibited.
    pub(crate) fn program_execute_impl(&mut self, page_address: u16) -> Result<(), Error<SE, PE>> {
        let page_address = page_address.to_be_bytes();
        self.bus.write([
            Commands::PROGRAM_EXECUTE,
            0, // 8 dummy bits
            page_address[0],
            page_address[1],
        ])
    }

    /// NEEDS BLOCK
    /// The Page Data Read instruction will transfer the data of the specified memory page into the 2,112-Byte
    /// Data Buffer. The instruction is initiated by driving the /CS pin low then shifting the instruction code “13h”
    /// followed by 8-bit dummy clocks and the 16-bit Page Address into the DI pin as shown in Figure 18.
    ///
    /// After /CS is driven high to complete the instruction cycle, the self-timed Read Page Data instruction will
    /// commence for a time duration of tRD (See AC Characteristics). While the Read Page Data cycle is in
    /// progress, the Read Status Register instruction may still be used for checking the status of the BUSY bit.
    /// The BUSY bit is a 1 during the Read Page Data cycle and becomes a 0 when the cycle is finished and the
    /// device is ready to accept other instructions again.
    ///
    /// After the 2,112 bytes of page data are loaded into the Data Buffer, several Read instructions can be issued
    /// to access the Data Buffer and read out the data. Depending on the BUF bit setting in the Status Register,
    /// either “Buffer Read Mode” or “Continuous Read Mode” may be used to accomplish the read operations.
    pub(crate) fn page_data_read_impl(&mut self, page_address: u16) -> Result<(), Error<SE, PE>> {
        let page_address = page_address.to_be_bytes();
        self.bus.write([
            Commands::PAGE_DATA_READ,
            0, // 8 dummy bits
            page_address[0],
            page_address[1],
        ])
    }

    /// Blocks the current thread until the flash chip's BUSY bit is no longer set
    pub(crate) fn block_until_not_busy_impl(&mut self) -> Result<(), Error<SE, PE>> {
        loop {
            let reg = self.read_regester_impl(Addresses::STATUS_REGISTER)?;

            /* TOCK_TODO
            let reg = LocalRegisterCopy::<u8, regs::Status::Register>::new(reg);
            if !reg.is_set(regs::Status::BUSY) {
                break Ok(());
            }
            */

            // BUSY bit is at position 0, and is unset when the device is ready for commands
            if reg & 0b0000_0001 == 0 {
                break Ok(());
            }
        }
    }

    /// The Read Data instruction allows one or more data bytes to be sequentially read from the Data Buffer after
    /// executing the Read Page Data instruction. The Read Data instruction is initiated by driving the /CS pin low
    /// and then shifting the instruction code “03h” followed by the 16-bit Column Address and 8-bit dummy clocks
    /// or a 24-bit dummy clocks into the DI pin. After the address is received, the data byte of the addressed Data
    /// Buffer location will be shifted out on the DO pin at the falling edge of CLK with most significant bit (MSB)
    /// first. The address is automatically incremented to the next higher address after each byte of data is shifted
    /// out allowing for a continuous stream of data. The instruction is completed by driving /CS high.
    ///
    /// The Read Data instruction sequence is shown in Figure 19a & 19b. When BUF=1, the device is in the Buffer
    /// Read Mode. The data output sequence will start from the Data Buffer location specified by the 16-bit Column
    /// Address and continue to the end of the Data Buffer. Once the last byte of data is output, the output pin will
    /// become Hi-Z state. When BUF=0, the device is in the Continuous Read Mode, the data output sequence
    /// will start from the first byte of the Data Buffer and increment to the next higher address. When the end of
    /// the Data Buffer is reached, the data of the first byte of next memory page will be following and continues
    /// through the entire memory array. This allows using a single Read instruction to read out the entire memory
    /// array and is also compatible to Winbond’s SpiFlash NOR flash memory command sequence.
    // We only implement continous reads because that is the default for this chip and we dont read
    // much anyway. TODO: Implement a nice wrapper that allows the user to
    pub(crate) fn page_read_continous_impl(&mut self, buf: &mut [u8]) -> Result<(), Error<SE, PE>> {
        let page_address = 0u16.to_be_bytes();
        // 8 dummy bits
        self.bus
            .read_unbounded([Commands::READ, page_address[0], page_address[1], 0], buf)
    }

    /// NEEDS BLOCK
    pub(crate) fn chip_erase_impl(&mut self) -> Result<(), Error<SE, PE>> {
        self.bus.write([Commands::CHIP_ERASE])
    }

    /// NEEDS BLOCK for 500 us
    pub(crate) fn reset_impl(&mut self) -> Result<(), Error<SE, PE>> {
        self.bus.write([Commands::DEVICE_RESET])
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

    /// Executs a request on the flash chip by sending `request`
    ///
    /// Returns the requesed number of result bytes.
    pub fn execute<const REQUEST_LEN: usize, const RESULTS_LEN: usize>(
        &mut self,
        request: [u8; REQUEST_LEN],
    ) -> Result<[u8; RESULTS_LEN], Error<SE, PE>> {
        let mut tmp = [0u8; 16];
        for (i, &byte) in request.iter().enumerate() {
            tmp[i] = byte;
        }
        let buf = &mut tmp[..REQUEST_LEN + RESULTS_LEN];
        // `Self::transfer` manages CS for us
        self.transfer(buf)?;

        let mut dst = [0u8; RESULTS_LEN];
        dst.copy_from_slice(&buf[(buf.len() - RESULTS_LEN)..]);
        Ok(dst)
    }

    /// Reads a dynamic amount of bytes by executing `instruction` on the flash chip with `params`
    pub fn read_unbounded<const REQUEST_LEN: usize>(
        &mut self,
        request: [u8; REQUEST_LEN],
        buf: &mut [u8],
    ) -> Result<(), Error<SE, PE>> {
        {
            self.cs_enable()?;
            self.spi.write(&request)?;
            // FIXME:
            // Make sure we dont miss bits here! If the function calls too slowly, that may happen!
            self.spi.transfer(&mut buf[..])?;
            self.cs_disable()?;
        }

        Ok(())
    }

    /// Writes a dynamic amount of bytes by executing `instruction` on the flash chip with `params`
    pub fn write_unbounded<const PARAMS: usize>(
        &mut self,
        header: [u8; PARAMS],
        buf: &[u8],
    ) -> Result<(), Error<SE, PE>> {
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

    pub(crate) fn into_inner(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }
}

pub struct Addresses;
#[allow(dead_code)]
impl Addresses {
    pub const PROTECTION_REGISTER: u8 = 0xA0;
    pub const CONFIGURATION_REGISTER: u8 = 0xB0;
    pub const STATUS_REGISTER: u8 = 0xC0;
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

impl JedecId {
    pub fn new(manufacturer_id: u8, dev1: u8, dev0: u8) -> Self {
        Self {
            manufacturer_id,
            device_id: [dev1, dev0],
        }
    }
}
