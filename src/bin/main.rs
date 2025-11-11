#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use core::cell::RefCell;

use defmt::info;
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::time::{Duration, Instant};
use esp_hal::{i2c, main};
use esp32_segment::{Accel, OutputDataRate, Scale};
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

    let mut led = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());

    info!("Init");

    let mut i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .expect("failed to initialize I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let mut i2c_cell = RefCell::new(i2c);

    info!("i2c created");

    let mut accel = Accel::new(RefCellDevice::new(&i2c_cell), ACCEL_ADDR);
    accel
        .init(Scale::Scale8G, OutputDataRate::Odr100)
        .expect("failed to initialize accelerometer");

    let mut seg = segment_rs::SevenSeg::init(RefCellDevice::new(&i2c_cell), 0x70, 15);

    loop {
        let (x, y, z) = accel.get_xyz_scaled().unwrap();
        info!("x: {}, y: {}, z: {}", x.0, y.0, z.0);

        seg.write_int((x.0 * 100.0) as u16);

        if let Some(tap) = accel.read_tap().unwrap() {
            info!("tap detected: {:?}", tap);
        }

        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(100) {}
    }
}
