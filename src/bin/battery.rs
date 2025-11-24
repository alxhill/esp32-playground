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
use esp_hal::rom::rtc_get_reset_reason;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::rtc_cntl::sleep::{RtcSleepConfig, TimerWakeupSource};
use esp_hal::system::reset_reason;
use esp_hal::{i2c, main, ram};
use max170xx::Max17048;
use segment_rs::{Digit, Seg, segs};

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const SEG_ADDR: u8 = 0x70;

#[ram(unstable(rtc_slow, persistent))]
static mut LOOP_COUNT: [u8; 2] = [0; 2];

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut rtc = Rtc::new(peripherals.LPWR);
    let delay = Delay::new();

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    info!("init");

    if let Some(reason) = reset_reason() {
        info!("reset reason: {}", reason as u8);
    } else {
        info!("No reset reason");
    }

    info!("RTC reset reason: {:?}", rtc_get_reset_reason(0));

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

    let loops = unsafe { LOOP_COUNT[0] };
    info!("loop: {}", loops);
    seg.write_uint(loops as u16);

    unsafe {
        LOOP_COUNT[0] = LOOP_COUNT[0].wrapping_add(1);
    }

    delay.delay_millis(3000);

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
        Digit::from_u16(((voltage * 10.0) as u16) % 10),
        v_segs,
        false,
    );
    delay.delay_millis(2000);

    seg.clear();

    delay.delay_millis(100);

    let mut sleep_cfg = RtcSleepConfig::deep();
    // sleep_cfg.set_rtc_fastmem_pd_en(false);
    sleep_cfg.set_rtc_slowmem_pd_en(false);

    rtc.sleep(
        &sleep_cfg,
        &[&TimerWakeupSource::new(core::time::Duration::from_secs(60))],
    );
    unreachable!();
}
