#![no_std]
#![no_main]
#![deny(unsafe_op_in_unsafe_fn)]

use core::fmt::Write;

use embedded_hal::blocking;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::spi::{Mode, Phase, Polarity};
use hal::pac::USART2;

use crate::hal::{pac, prelude::*, spi};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use w25n512gv::{regs, Addresses, BufferRef, W25n512gv};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    let mut delay = dp.TIM1.delay_us(&clocks);

    let tx_pin = gpioa.pa2.into_alternate();

    let mut serial = dp.USART2.tx(tx_pin, 9600.bps(), &clocks).unwrap();

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

    let sck = gpioc.pc10;
    let miso = gpioc.pc11;
    let mosi = gpioc.pc12;
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

    fn dump_buf<SPI, CS, SE, PE>(
        r: &mut impl BufferRef<SPI, CS, SE, PE>,
        page: &mut [u8; w25n512gv::PAGE_SIZE_WITH_ECC],
        len: usize,
        serial: &mut impl Write,
    ) where
        SPI: blocking::spi::Transfer<u8, Error = SE> + blocking::spi::Write<u8, Error = SE>,
        CS: OutputPin<Error = PE>,
    {
        if let Err(err) = r.download_from_buffer_sync(page) {
            writeln!(serial, "Failed to dump flash buffer!");
            panic!();
        }
        writeln!(serial, "Dumping {} bytes of flash from buffer", len);
        for &byte in page.iter().take(len) {
            write!(serial, "{}, ", byte);
        }
        writeln!(serial);
    }

    writeln!(serial);
    writeln!(serial);
    writeln!(serial, "========================================");
    writeln!(serial);

    writeln!(serial, "Starting initialization.");

    delay.delay_ms(100u32);

    writeln!(serial, "Initializing flash chip");
    let flash = w25n512gv::W25n512gv::new(spi, flash_cs)
        .map_err(|e| {
            writeln!(serial, "Flash chip failed to intialize. {e:?}");
        })
        .unwrap();

    let (spi, flash_cs) = flash.reset(&mut delay);

    let mut flash = w25n512gv::W25n512gv::new(spi, flash_cs /*, &mut delay*/)
        .map_err(|e| {
            writeln!(serial, "Flash chip failed to intialize. {e:?}");
        })
        .unwrap();

    flash.modify_configuration_register(|r| {
        *r |= 1 << 4; // Enable ECC
        *r |= 1 << 0; // disable HOLD
    });

    // Disable all protections
    flash.modify_protection_register(|r| *r = 0);

    writeln!(serial, "Initialized.");

    let test_page = 7;

    writeln!(serial, "Persistent data from last time");

    let mut page = [0u8; w25n512gv::PAGE_SIZE_WITH_ECC];
    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 64, &mut serial);
    let flash = r.finish();

    writeln!(serial, "Erasing chip...");
    let flash = flash.enable_write().unwrap();
    let flash = flash.erase_all().unwrap().enable_write().unwrap();

    writeln!(serial, "page 0 after erase");

    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 64, &mut serial);
    let flash = r.finish();

    writeln!(serial, "writing first time");
    let mut index: u8 = 0;
    let test_data = [0u8; w25n512gv::PAGE_SIZE_WITH_ECC].map(|_| {
        let before = index;
        index = index.wrapping_add(2);
        before
    });

    let r = flash.upload_to_buffer_sync(&test_data).unwrap();
    let flash = r.commit_sync(test_page).unwrap().finish();

    writeln!(serial, "after 2 increment write");
    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 16, &mut serial);
    let flash = r.finish();

    let mut index: u8 = 0;
    let test_data = [0u8; w25n512gv::PAGE_SIZE_WITH_ECC].map(|_| {
        let before = index;
        index = index.wrapping_add(1);
        before
    });
    delay.delay_us(10u8);

    let flash = flash.enable_write().unwrap();
    let mut flash = flash.erase(test_page).unwrap().enable_write().unwrap();

    writeln!(serial, "writing second time");
    let mut r = flash.upload_to_buffer_sync(&test_data).unwrap();
    let flash = r.commit_sync(test_page).unwrap().finish();

    writeln!(serial, "after normal write");

    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 16, &mut serial);
    let flash = r.finish();

    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 16, &mut serial);
    let flash = r.finish();

    let mut r = flash.read_sync(test_page).unwrap();
    dump_buf(&mut r, &mut page, 16, &mut serial);
    let flash = r.finish();

    writeln!(serial, "OK");
    loop {}
}

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}
