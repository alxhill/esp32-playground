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
use esp_hal::{i2c, main};
use max170xx::Max17048;
use segment_rs::{Digit, Seg, segs};

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const SEG_ADDR: u8 = 0x70;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let delay = Delay::new();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("init");

    let i2c = I2c::new(peripherals.I2C0, i2c::master::Config::default())
        .expect("failed to initialize I2C")
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO22);
    let i2c_cell = RefCell::new(i2c);

    info!("i2c created");

    let mut seg = segment_rs::SevenSeg::init(RefCellDevice::new(&i2c_cell), SEG_ADDR);
    seg.set_brightness(5);

    info!("seg display created");

    let mut bat = Max17048::new(RefCellDevice::new(&i2c_cell));
    bat.quickstart().unwrap();

    let v_segs = segs!(Seg::TopL, Seg::BotL, Seg::Bot, Seg::BotR, Seg::TopR);

    info!("battery initialized");

    loop {
        info!("charge rate: {}", bat.charge_rate().unwrap());
        seg.write_percent(bat.charge_rate().unwrap().abs());
        delay.delay_millis(2000);
        let soc = bat.soc().unwrap();
        info!("soc: {}", soc);
        if soc > 100.0 {
            seg.write_uint(soc as u16);
        } else {
            seg.write_percent(soc);
        }
        delay.delay_millis(2000);

        let voltage = bat.voltage().unwrap();
        info!("voltage: {}", voltage);
        seg.write(
            0,
            (Digit::from_u16(voltage as u16), Seg::Dot),
            Digit::from_u16((voltage * 10.0) as u16),
            v_segs,
            false,
        );
        delay.delay_millis(2000);
    }
}
