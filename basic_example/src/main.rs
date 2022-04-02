#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::fmt::Write;
use core::mem::MaybeUninit;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity};
use hal::pac::USART2;

use crate::hal::{pac, prelude::*, spi};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use w25n512gv::{regs, Addresses, W25n512gv};

static WRITER: Writer = Writer(UnsafeCell::new(MaybeUninit::uninit()));

struct Writer(UnsafeCell<MaybeUninit<hal::serial::Tx<USART2>>>);

unsafe impl Sync for Writer {}
unsafe impl Send for Writer {}

/// # Safety
/// This function must only be called after `WRITER` is initialized
unsafe fn get_writer() -> &'static mut hal::serial::Tx<USART2> {
    unsafe { (*WRITER.0.get()).assume_init_mut() }
}

macro_rules! println {
    () => {{
        let writer = unsafe { get_writer() };
        writeln!(writer).unwrap();
    }};
    ($($arg:tt)*) => {{
        let writer = unsafe { get_writer() };
        writeln!(writer, $($arg)*).unwrap();
    }};
}

macro_rules! print {
    () => {{
        let writer = unsafe { get_writer() };
        write!(writer).unwrap();
    }};
    ($($arg:tt)*) => {{
        let writer = unsafe { get_writer() };
        write!(writer, $($arg)*).unwrap();
    }};
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(40.MHz()).freeze();

    let mut delay = dp.TIM1.delay_us(&clocks);

    let tx_pin = gpioa.pa2.into_alternate();

    let serial = dp.USART2.tx(tx_pin, 9600.bps(), &clocks).unwrap();
    let writer = WRITER.0.get();
    unsafe { writer.write(MaybeUninit::new(serial)) };

    // "High-Speed"/H_ SPI for flash chip:
    // SCK PC10
    // MISO PC11
    // MOSI PC12
    //
    // Regular SPI:
    // SCK PA5
    // MISO PA6
    // MOSI PA7
    //
    // CS:
    // FLASH PB13
    // ALTIMETER PC5
    // HIGH_G/ACCEL PB2
    //

    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();
    let flash_cs = gpiob.pb13.into_push_pull_output();

    let pins = (sck, miso, mosi);

    let spi = spi::Spi::new(
        dp.SPI3,
        pins,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1000.kHz(),
        &clocks,
    );

    println!();
    println!();
    println!("========================================");
    println!();

    println!("Starting initialization.");

    delay.delay_ms(100u32);

    let mut flash = W25n512gv::new(spi, flash_cs /*, &mut delay*/)
        .map_err(|e| {
            println!("Flash chip failed to intialize. {e:?}");
        })
        .unwrap();

    flash.reset(&mut delay).unwrap();

    let mut config = flash
        .read_regester(Addresses::CONFIGURATION_REGISTER)
        .unwrap();
    config |= 1 << 4; // Enable ECC
    config |= 1; // disable HOLD
    flash
        .write_register(Addresses::CONFIGURATION_REGISTER, config)
        .unwrap();

    fn status<SPI, CS, SE, PE>(chip: &mut W25n512gv<SPI, CS>)
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SE>
            + embedded_hal::blocking::spi::Write<u8, Error = SE>,
        CS: OutputPin<Error = PE>,
    {
        let val = chip
            .read_regester(Addresses::STATUS_REGISTER)
            .unwrap_or_else(|_| panic!());
        let strs = [
            "BUSY",
            "Write enable",
            "Erase Failure",
            "Program failure",
            "ECC 0",
            "ECC 1",
            "Look up table full",
        ];
        println!("Status:");
        for (i, s) in strs.iter().enumerate() {
            let bit = (val >> i) & 0b1;
            if bit != 0 {
                println!("{}: {}", s, bit);
            }
        }
        println!();
    }

    fn protection<SPI, CS, SE, PE>(chip: &mut W25n512gv<SPI, CS>)
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SE>
            + embedded_hal::blocking::spi::Write<u8, Error = SE>,
        CS: OutputPin<Error = PE>,
    {
        let val = chip
            .read_regester(Addresses::STATUS_REGISTER)
            .unwrap_or_else(|_| panic!());
        let strs = ["SRP1", "WP-E", "TB", "BP0", "BP1", "BP2", "BP3", "SRP0"];
        println!("protection:");
        for (i, s) in strs.iter().enumerate() {
            let bit = (val >> i) & 0b1;
            if bit != 0 {
                println!("{}: {}", s, bit);
            }
        }
        println!();
    }
    println!("Initialized.");
    protection(&mut flash);
    let id = flash.read_jedec_id().unwrap();
    println!("Id {id:?}");

    status(&mut flash);

    println!("Erasing chip...");
    flash.enable_write().unwrap();
    flash.chip_erase().unwrap();
    println!("Done!");

    fn dump_flash<SPI, CS, SE, PE>(chip: &mut W25n512gv<SPI, CS>)
    where
        SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SE>
            + embedded_hal::blocking::spi::Write<u8, Error = SE>,
        CS: OutputPin<Error = PE>,
    {
        let mut buf = [0u8; w25n512gv::RAW_PAGE_SIZE];
        chip.page_read_continous(&mut buf)
            .unwrap_or_else(|_| panic!());

        for i in 0..16 {
            print!("{}, ", buf[i]);
        }
        println!();
    }

    println!();
    println!("before");
    let test_page = 2u16.pow(6);
    flash.page_data_read(test_page).unwrap();
    dump_flash(&mut flash);
    status(&mut flash);

    let mut index: u8 = 0;
    let test_data = [0u8; w25n512gv::PAGE_SIZE_WITH_ECC].map(|_| {
        index = index.wrapping_add(1);
        index
    });
    delay.delay_us(10u8);

    flash.enable_write().unwrap();
    flash.load_program_data(0, &test_data).unwrap();
    for i in 0..128 {
        println!("in buffer");
        status(&mut flash);
        dump_flash(&mut flash);
        delay.delay_us(10u8);
        flash.enable_write().unwrap();
        flash.program_execute(i).unwrap();
    }

    println!("after");
    status(&mut flash);
    flash.page_data_read(test_page).unwrap();
    dump_flash(&mut flash);

    status(&mut flash);
    println!(
        "config {:0b}",
        flash
            .read_regester(Addresses::CONFIGURATION_REGISTER)
            .unwrap()
    );
    println!(
        "protec {:0b}",
        flash.read_regester(Addresses::PROTECTION_REGISTER).unwrap()
    );

    println!("OK");
    loop {
        /*let sample = ms6511
            .get_second_order_sample(Oversampling::OS_256, &mut delay)
            .unwrap();

        h3lis331dl.readAxes(&mut x, &mut y, &mut z).unwrap();

        writeln!(
            serial,
            "Temp: {}, Pressure: {}\nX: {}, Y: {}, Z: {}",
            sample.temperature, sample.pressure, x, y, z
        )
        .unwrap();
        */
    }
}

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
