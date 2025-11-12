#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;

use defmt::info;
use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::prelude::*;
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
        .init(Scale::Scale4G, OutputDataRate::Odr100)
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
    // .with_cs(cs);

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
    // oled.clear().unwrap();
    oled.set_display_on(true).unwrap();

    // oled.write_str("hello").unwrap();

    // let mut seg = segment_rs::SevenSeg::init(RefCellDevice::new(&i2c_cell), 0x70, 15);

    // let raw: ImageRaw<BinaryColor> = ImageRaw::new(include_bytes!("../rust.raw"), 64);
    // let im = Image::new(&raw, Point::new(0, 0));
    // im.draw(&mut oled).unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut oled)
        .unwrap();

    oled.flush().unwrap();

    loop {
        let (x, y, z) = accel.get_xyz().unwrap();
        info!("x: {}, y: {}, z: {}", x.0, y.0, z.0);
        // write!(oled, "{}", x.0).unwrap();

        // oled.flush().unwrap();

        // seg.write_int((x.0 * x.0.signum()) as u16);

        if let Some(tap) = accel.read_tap().unwrap() {
            info!("tap detected: {:?}", tap);
        }

        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(50) {}
    }
}
