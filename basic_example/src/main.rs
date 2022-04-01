#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::fmt::Write;
use core::mem::MaybeUninit;

use embedded_hal::spi::{Mode, Phase, Polarity};
use hal::pac::USART2;

use crate::hal::{pac, prelude::*, spi};
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal as hal;

use w25n512gv::W25n512gv;

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
        writeln!(writer);
    }};
    ($($arg:tt)*) => {{
        let writer = unsafe { get_writer() };
        writeln!(writer, $($arg)*);
    }};
}

macro_rules! print {
    () => {{
        let writer = unsafe { get_writer() };
        write!(writer);
    }};
    ($($arg:tt)*) => {{
        let writer = unsafe { get_writer() };
        write!(writer, $($arg)*);
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

    println!("\n\n========================================\n");
    println!("Starting initialization.");

    delay.delay_ms(100u32);

    let mut flash = W25n512gv::new(spi, flash_cs /*, &mut delay*/)
        .map_err(|e| {
            println!("Flash chip failed to intialize. {e:?}");
        })
        .unwrap();

    println!("Initialized.");
    let id = flash.read_jedec_id().unwrap();
    println!("Id {id:?}");

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
