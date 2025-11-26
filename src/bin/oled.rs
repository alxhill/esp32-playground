#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;

use core::fmt::Write;
use defmt::error;
use defmt::info;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::*;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::Output;
use esp_hal::i2c::master::I2c;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::{Duration, Instant};
use esp_hal::{gpio, i2c, main};
use heapless::String;
use ssd1306::Ssd1306;
use ssd1306::prelude::*;

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const OLED_ADDR: u8 = 0x3C;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("Init");

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .expect("failed to initialize I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_cell = RefCell::new(i2c);

    info!("i2c created");

    let mut rst = Output::new(
        peripherals.GPIO4,
        gpio::Level::Low,
        gpio::OutputConfig::default(),
    );

    let interface = I2CInterface::new(RefCellDevice::new(&i2c_cell), OLED_ADDR, 0x40);

    info!("iface");

    let mut oled = Ssd1306::new(interface, DisplaySize64x48, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    info!("oled");
    oled.reset(&mut rst, &mut Delay::default())
        .expect("failed to reset OLED");
    if let Err(e) = oled.init() {
        error!("Failed to initialize OLED: {:?}", e);
        loop {}
    }

    info!("oled initialized");
    oled.set_display_on(true).unwrap();

    info!("oled on");

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut oled)
        .unwrap();

    oled.flush().unwrap();

    let delay_start = Instant::now();
    while delay_start.elapsed() < Duration::from_millis(500) {}

    let (width, height) = oled.dimensions();

    info!("width: {}, height: {}", width, height);

    let rtc = Rtc::new(peripherals.LPWR);

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut oled)
        .unwrap();
    oled.flush().unwrap();

    delay.delay_millis(5_000);

    let mut time_str: String<16> = String::new();
    loop {
        oled.clear(BinaryColor::Off).unwrap();

        let time = (rtc.current_time_us() / 10_000) as u16;

        time_str.clear();
        write!(&mut time_str, "{:08}", time).unwrap();

        info!("time str: {}", time_str);

        Text::with_baseline(time_str.as_str(), Point::zero(), text_style, Baseline::Top)
            .draw(&mut oled)
            .unwrap();

        oled.flush().unwrap();
    }
}
