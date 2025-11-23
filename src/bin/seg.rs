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
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::{i2c, main};
use esp32_segment::{Accel, Scale};

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const SEG_ADDR: u8 = 0x70;
const ACCEL_ADDR: u8 = 0x1D;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let rtc = Rtc::new(peripherals.LPWR);
    let delay = Delay::new();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("Init");

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .expect("failed to initialize I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_cell = RefCell::new(i2c);

    info!("i2c created");

    let mut seg = segment_rs::SevenSeg::init(RefCellDevice::new(&i2c_cell), SEG_ADDR);

    info!("seg display created");

    let mut accel = Accel::new(RefCellDevice::new(&i2c_cell), ACCEL_ADDR);

    accel
        .init(Scale::Scale2G, esp32_segment::OutputDataRate::Odr800)
        .unwrap();

    loop {
        let x = accel.get_x().unwrap();
        seg.write_int(x.0 as u16);

        delay.delay_millis(50);
    }
}
