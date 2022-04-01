#![no_std]
#![no_main]

use core::fmt::Write;

use embedded_hal::spi::{Mode, Phase, Polarity};

use crate::hal::{pac, prelude::*, spi};
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal as hal;

use w25n512gv::W25n512gv;

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

    writeln!(serial, "\n\n========================================\n").unwrap();
    writeln!(serial, "Starting initialization.").unwrap();

    let (mut flash, id) = W25n512gv::new(spi, flash_cs /*, &mut delay*/)
        .map_err(|_| {
            writeln!(serial, "Barometer failed to intialize.").unwrap();
        })
        .unwrap();

    delay.delay_ms(100u32);

    writeln!(serial, "Flash initialized. {id:?}").unwrap();

    writeln!(serial, "Initialized.").unwrap();

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
