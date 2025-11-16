#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;

use defmt::info;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, StyledDrawable};
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::time::{Duration, Instant, Rate};
use esp_hal::{gpio, i2c, main, spi};
use esp32_segment::{Accel, OutputDataRate, Scale};
use ssd1306::Ssd1306;
use ssd1306::prelude::*;

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(unused)]
const ACCEL_ADDR: u8 = 0x1D;
#[allow(unused)]
const SEG_ADDR: u8 = 0x70;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("Init");

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .expect("failed to initialize I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_cell = RefCell::new(i2c);

    info!("i2c created");

    let mut accel = Accel::new(RefCellDevice::new(&i2c_cell), ACCEL_ADDR);
    accel
        .init(Scale::Scale2G, OutputDataRate::Odr800)
        .expect("failed to initialize accelerometer");

    info!("accel init");

    let sck = peripherals.GPIO18;
    let mosi = peripherals.GPIO23;
    let cs = Output::new(peripherals.GPIO16, Level::Low, OutputConfig::default());

    let spi = spi::master::Spi::new(
        peripherals.SPI2,
        spi::master::Config::default().with_frequency(Rate::from_mhz(1)),
    )
    .expect("failed to initialize SPI")
    .with_sck(sck)
    .with_mosi(mosi);

    let spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    info!("spi created");

    let mut rst = Output::new(
        peripherals.GPIO4,
        gpio::Level::Low,
        gpio::OutputConfig::default(),
    );
    let dc = Output::new(
        peripherals.GPIO17,
        gpio::Level::Low,
        gpio::OutputConfig::default(),
    );
    let interface = ssd1306::prelude::SPIInterface::new(spi_dev, dc);

    let mut oled = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    oled.reset(&mut rst, &mut Delay::default())
        .expect("failed to reset OLED");
    oled.init().unwrap();
    oled.set_display_on(true).unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut oled)
        .unwrap();

    oled.flush().unwrap();

    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(500) {}

    let point_style = PrimitiveStyleBuilder::new()
        .fill_color(BinaryColor::On)
        .build();

    let (width, height) = oled.dimensions();

    info!("width: {}, height: {}", width, height);

    let mut i = 0;

    loop {
        oled.clear(BinaryColor::Off).unwrap();

        // for some reason the z value and x value are the same when reading all registers at the same time
        // let (x, y, z) = accel.get_xyz().unwrap();
        let x = accel.get_x().unwrap();
        let y = accel.get_y().unwrap();
        let z = accel.get_z().unwrap();

        // swapping because of the relative orientation of the accelerometer and OLED
        let oled_x = (-y.0) / 16 + (width / 2) as i16;
        let oled_y = (-x.0) / 16 + (height / 2) as i16;
        let oled_size = z.0 / 32;

        if i % 100 == 0 {
            info!(
                "x: {}, y: {}, z: {} | oled_x: {}, oled_y: {}, oled_size: {}",
                x.0, y.0, z.0, oled_x, oled_y, oled_size
            );
        }
        i += 1;

        Circle::with_center(
            Point::new(oled_x as i32, oled_y as i32),
            oled_size.clamp(1, 15) as u32,
        )
        .draw_styled(&point_style, &mut oled)
        .unwrap();

        oled.flush().unwrap();
    }
}
