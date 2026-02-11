#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::{info, warn};
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::main;
use esp_hal::time::Duration;
use esp_hal::twai::{self as can, EspTwaiFrame};

use {esp_backtrace as _, esp_println as _};

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);

    let timing = can::BaudRate::B500K;

    let rx_pin = peripherals.GPIO2;
    let tx_pin = peripherals.GPIO4;

    let can_conf = can::TwaiConfiguration::new(
        peripherals.TWAI0,
        rx_pin,
        tx_pin,
        timing,
        can::TwaiMode::Normal,
    );

    let mut can = can_conf.start();
    let mut led = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    let mut count = 0;
    loop {
        // led.toggle();
        let frame = EspTwaiFrame::new(
            can::StandardId::new(1).unwrap(),
            &[count, count, 2, 3, 4, 5, 6, 7],
        )
        .unwrap();

        loop {
            match can.transmit(&frame) {
                Ok(_) => {
                    info!("transmitted frame {}", count);
                    break;
                }
                Err(nb::Error::WouldBlock) => {
                    warn!("blocking");
                    led.set_high();
                    Delay::new().delay(Duration::from_millis(5));
                }
                Err(error) => {
                    warn!("failed to transmit frame {}: {}", count, error);
                    break;
                }
            }
        }
        led.set_low();

        count = count.wrapping_add(1);

        Delay::new().delay(Duration::from_millis(10));
    }
}
