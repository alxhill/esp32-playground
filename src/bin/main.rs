#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::time::{Duration, Instant};
use esp_hal::{i2c, main};
use segment_rs::SevenSeg;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());

    let seg_i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);

    let mut seg = SevenSeg::init(seg_i2c, 0x70, 15);

    let mut val: u16 = 0;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    loop {
        let delay_start = Instant::now();
        info!("Printing: {}", val);
        if val % 200 == 0 {
            led.toggle();
        }
        seg.write_int(val);
        val = (val + 1) % 10000;
        while delay_start.elapsed() < Duration::from_millis(10) {}
    }
}
