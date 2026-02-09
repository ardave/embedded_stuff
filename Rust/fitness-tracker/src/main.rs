mod queue;
mod tasks;

use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::units::FromValueType;
use log::info;
use std::thread;
use std::time::Duration;

use queue::FreeRtosQueue;
use tasks::gps::GpsReading;

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    // Enable I2C / STEMMA QT power (GPIO 7 must be HIGH on Adafruit ESP32-S2 Feather)
    let mut power_pin = PinDriver::output(peripherals.pins.gpio7).expect("Failed to init GPIO 7");
    power_pin.set_high().expect("Failed to set GPIO 7 HIGH");

    // Give the bus a moment to stabilize
    thread::sleep(Duration::from_millis(10));

    let i2c = I2cDriver::new(
        peripherals.i2c0,
        peripherals.pins.gpio3, // SDA
        peripherals.pins.gpio4, // SCL
        &I2cConfig::new().baudrate(100.kHz().into()),
    )
    .expect("Failed to init I2C");

    // Create GPS data queues (leaked to 'static — they live for the program's lifetime)
    let queue1: &'static FreeRtosQueue<GpsReading> =
        Box::leak(Box::new(FreeRtosQueue::new(4).expect("Failed to create queue1")));
    let queue2: &'static FreeRtosQueue<GpsReading> =
        Box::leak(Box::new(FreeRtosQueue::new(4).expect("Failed to create queue2")));

    let _gps_thread = tasks::gps::start(i2c, &[queue1, queue2]);
    let _task1_thread = tasks::task1::start(queue1);
    let _task2_thread = tasks::task2::start(queue2);

    // Keep power_pin alive so it stays HIGH
    loop {
        info!("Hello, world!");
        thread::sleep(Duration::from_secs(2));
    }
}
