mod tasks;

use std::sync::Mutex;
use std::thread;
use std::time::Duration;

use embedded_hal_bus::i2c::MutexDevice;
use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::queue::Queue;
use esp_idf_svc::hal::units::FromValueType;
use log::info;

use tasks::gps_acquisition::{GpsReading, GpsSentence};
use tasks::user_display::DisplayMessage;

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

    // Share I2C bus among dependent tasks via Mutex
    let i2c_bus: &'static Mutex<I2cDriver<'static>> = Box::leak(Box::new(Mutex::new(i2c)));

    // Setup display bits:
    let display_i2c = MutexDevice::new(i2c_bus);
    let display_queue: &'static Queue<DisplayMessage> = Box::leak(Box::new(Queue::new(4)));
    let _display_thread = tasks::user_display::start(display_i2c, display_queue);

    // Setup GPS bits:
    let gps_i2c = MutexDevice::new(i2c_bus);
    let gps_sentence_queue: &'static Queue<GpsSentence> = Box::leak(Box::new(Queue::new(4)));
    let gps_reading_queue: &'static Queue<GpsReading> = Box::leak(Box::new(Queue::new(4)));
    let _gps_thread = tasks::gps_acquisition::start(gps_i2c, gps_sentence_queue, gps_reading_queue);
    let _gps_aggregator_thread = tasks::gps_aggregator::start(gps_sentence_queue);

    // Setup SD Card bits:
    let _sd_card_thread = tasks::sd_card::start(gps_reading_queue);

    // Keep power_pin alive so it stays HIGH
    loop {
        info!("Hello, world!");
        thread::sleep(Duration::from_secs(2));
    }
}
