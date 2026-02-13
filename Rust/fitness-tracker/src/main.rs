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

use tasks::user_display::{DisplayLine, DisplayMessage};

use crate::tasks::gps_acquisition::FitnessTrackerSentence;

fn main() {
    thread::sleep(Duration::from_secs(5));
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    // Enable I2C / STEMMA QT power (GPIO 7 must be HIGH on Adafruit ESP32-S2 Feather)
    let mut power_pin = PinDriver::output(peripherals.pins.gpio7).expect("Failed to init GPIO 7");
    power_pin.set_high().expect("Failed to set GPIO 7 HIGH");

    // Wait for STEMMA QT devices to power up (display needs ≥250ms after VCC)
    thread::sleep(Duration::from_millis(250));

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
    let display_queue_receiver = QueueReceiver(display_queue);
    let _display_thread = tasks::user_display::start(display_i2c, display_queue_receiver);

    // Send a test message to verify the display is working
    display_queue
        .send_back(DisplayMessage::Line1(DisplayLine::new("Hello!")), 0)
        .expect("Failed to enqueue display test message");

    // Setup GPS bits:
    let gps_i2c = MutexDevice::new(i2c_bus);
    let gps_sentence_queue: &'static Queue<FitnessTrackerSentence> =
        Box::leak(Box::new(Queue::new(4)));
    let _gps_acquisition_thread =
        tasks::gps_acquisition::start(gps_i2c, QueueSender(gps_sentence_queue));
    let _gps_aggregator_thread = tasks::gps_aggregator::start(QueueReceiver(gps_sentence_queue));

    // Setup SD Card bits:
    //let _sd_card_thread = tasks::sd_card::start(QueueReceiver(gps_reading_queue));

    // Keep power_pin alive so it stays HIGH
    loop {
        log_stack_high_water_marks();
        thread::sleep(Duration::from_secs(2));
    }
}

pub struct QueueSender<T: 'static>(&'static Queue<T>);

impl<T: Copy + 'static> QueueReceiver<T> {
    pub fn recv_front(&self, timeout: u32) -> Option<(T, bool)> {
        self.0.recv_front(timeout)
    }
}

pub struct QueueReceiver<T: 'static>(&'static Queue<T>);

impl<T: Copy> QueueSender<T> {
    pub fn send_back(&self, item: T, timeout: u32) -> Result<bool, esp_idf_svc::sys::EspError> {
        self.0.send_back(item, timeout)
    }
}

/// Log the minimum free stack (bytes) each FreeRTOS task has ever had.
/// A value approaching zero means that task is close to overflowing.
fn log_stack_high_water_marks() {
    const TASK_NAMES: &[&[u8]] = &[b"GPS\0", b"Display\0", b"Task1\0", b"Task2\0"];
    for name in TASK_NAMES {
        unsafe {
            let handle = esp_idf_svc::sys::xTaskGetHandle(name.as_ptr() as *const _);
            if !handle.is_null() {
                let hwm = esp_idf_svc::sys::uxTaskGetStackHighWaterMark(handle);
                let task_name = core::str::from_utf8(&name[..name.len() - 1]).unwrap_or("?");
                info!("[Stack] {}: {} bytes free", task_name, hwm);
            }
        }
    }
}
