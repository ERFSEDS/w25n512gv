#![no_std]
#![no_main]

use core::cell::UnsafeCell;
use core::fmt::Write;
use core::mem::MaybeUninit;
use core::time::Duration;

use embedded_hal::spi::{Mode, Phase, Polarity};
use hal::pac::USART2;
use hal::timer::{Event, Timer};
use ms5611_spi::{Ms5611, Oversampling};

use crate::hal::{pac, prelude::*, spi};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

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

const BYTES: usize = 31_000;
struct Data(UnsafeCell<heapless::Vec<u8, BYTES>>);

static DATA: Data = Data(UnsafeCell::new(heapless::Vec::new()));

unsafe impl Sync for Data {}
unsafe impl Send for Data {}

enum SampleKind {
    Gyro,
    Pressure,
    Accel,
}

static LAST_TICKS: AtomicU32 = AtomicU32::new(0);
static TICKS: AtomicU32 = AtomicU32::new(0);

fn push_impl(val: u8) -> Result<(), ()> {
    let data: &mut heapless::Vec<u8, BYTES> = unsafe { &mut *DATA.0.get() };
    data.push(val).map_err(|_| ())
}

fn data_usage() -> usize {
    let data: &mut heapless::Vec<u8, BYTES> = unsafe { &mut *DATA.0.get() };
    data.len()
}

use hal::pac::interrupt;

#[interrupt]
fn TIM2() {
    stm32f4xx_hal::pac::NVIC::unpend(hal::pac::Interrupt::TIM2);
    TICKS.fetch_add(1, Ordering::AcqRel);
}

/// Returns true if full
fn add_sample(kind: SampleKind, data: &[u8]) -> Result<(), ()> {
    let kind_bits = match kind {
        SampleKind::Gyro => 0xFF,
        SampleKind::Pressure => 0xEE,
        SampleKind::Accel => 0xDD,
    };
    push_impl(kind_bits)?;
    for &byte in data {
        push_impl(byte)?;
    }

    Ok(())
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();

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

    println!();
    println!();
    println!("========================================");
    println!();

    println!("Starting initialization.");

    delay.delay_ms(100u32);

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let baro_cs = gpioc.pc5.into_push_pull_output();
    let gyro_accel_cs = gpiob.pb0.into_push_pull_output();
    let gyro_cs = gpiob.pb1.into_push_pull_output();

    let pins = (sck, miso, mosi);

    let spi = spi::Spi::new(
        dp.SPI1,
        pins,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        1000.kHz(),
        &clocks,
    );

    println!("Starting initialization.");

    let spi_bus = shared_bus::BusManagerSimple::new(spi);

    let mut ms6511 = Ms5611::new(spi_bus.acquire_spi(), baro_cs, &mut delay)
        .map_err(|_| {
            println!("Barometer failed to initialize.");
        })
        .unwrap();

    println!("Barometer initialized.");

    let mut bmi088_accel = bmi088::Builder::new_accel_spi(spi_bus.acquire_spi(), gyro_accel_cs);

    if let Err(_) = bmi088_accel.setup(&mut delay) {
        println!("Low-G accelerometer failed to initialize.");
        panic!();
    }

    println!("Low-G accelerometer initialized.");

    let mut bmi088_gyro = bmi088::Builder::new_gyro_spi(spi_bus.acquire_spi(), gyro_cs);

    if let Err(_) = bmi088_gyro.setup(&mut delay) {
        println!("Gyro failed to initialize.");
        panic!();
    }

    println!("Gyro initialized.");

    /*
    let mut h3lis331dl = h3lis331dl::H3LIS331DL::new(spi_bus.acquire_spi(), high_g_accel_cs)
        .map_err(|e| {
            println!("Accelerometer failed to initialize: {:?}.", e).unwrap();
        })
        .unwrap();
    println!("Accelerometer initialized.").unwrap();
    */

    println!("Initialized.");

    println!("Entering wait loop");
    let mut largest = 0;
    loop {
        if let Ok(sample) = bmi088_accel.get_accel() {
            let total =
                (sample[0] as i32).abs() + (sample[1] as i32).abs() + (sample[2] as i32).abs();
            if total > largest {
                largest = total;
            }
            println!("{total} - {largest}");
            if total > 150_000 {
                //if total > 40_000 {
                break;
            }
        }
    }

    let mut run_loop = || -> Result<(), ()> {
        loop {
            {
                let sample = ms6511
                    .get_second_order_sample(Oversampling::OS_256, &mut delay)
                    .unwrap();

                let data: [u8; 8] = unsafe { core::mem::transmute(sample) };
                add_sample(SampleKind::Pressure, &data)?;
            }

            if let Ok(sample) = bmi088_accel.get_accel() {
                let data: [u8; 6] = unsafe { core::mem::transmute(sample) };
                add_sample(SampleKind::Accel, &data)?;
            }

            if let Ok(sample) = bmi088_gyro.get_gyro() {
                let data: [u8; 6] = unsafe { core::mem::transmute(sample) };
                add_sample(SampleKind::Gyro, &data)?;
            }
            //println!("{}", data_usage());
            delay.delay_ms(25u32);
        }
    };

    let _ = run_loop();

    println!("Done");

    loop {
        println!("DATA DUMP:");
        // wait for interrupt and then print data
        let data = unsafe { &*DATA.0.get() };
        println!("bytes {}", data.len());
        for val in data.iter() {
            print!("{val:0x}");
        }
        println!("END DATA DUMP");
    }
}

use core::panic::PanicInfo;
use core::sync::atomic::{self, AtomicU32, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    println!("{}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
