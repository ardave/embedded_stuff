mod tasks;

use std::thread;
use std::time::Duration;

#[cfg(not(feature = "fake-gps"))]
use esp_idf_svc::hal::gpio::AnyIOPin;
use esp_idf_svc::hal::gpio::PinDriver;
use esp_idf_svc::hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::spi::config::{DriverConfig, MODE_0};
use esp_idf_svc::hal::spi::{Dma, SpiDeviceDriver, SpiDriver};
use esp_idf_svc::hal::task::queue::Queue;
#[cfg(not(feature = "fake-gps"))]
use esp_idf_svc::hal::uart::{self, UartDriver};
use esp_idf_svc::hal::units::FromValueType;
use log::info;

use tasks::user_display::DisplayContent;
use testable_logic::gps_sentence_joining::FitnessTrackerSentence;

use crate::tasks::gps_acquisition::GPSAcquisitionError;

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
        &I2cConfig::new().baudrate(200.kHz().into()),
    )
    .expect("Failed to init I2C");

    // Setup display (sole I2C consumer):
    let display_queue: &'static Queue<DisplayContent> = Box::leak(Box::new(Queue::new(1)));
    let display_queue_receiver = QueueReceiver(display_queue);
    let _display_thread = tasks::user_display::start(i2c, display_queue_receiver);

    // Send initial dashes while GPS starts up
    display_queue
        .send_back(DisplayContent::Reading(None, None), 0)
        .expect("Failed to enqueue display message");

    // Setup GPS Acquisition:
    let gps_sentence_queue: &'static Queue<Result<FitnessTrackerSentence, GPSAcquisitionError>> =
        Box::leak(Box::new(Queue::new(4)));

    #[cfg(not(feature = "fake-gps"))]
    let _gps_acquisition_thread = {
        let uart_config =
            uart::config::Config::new().baudrate(esp_idf_svc::hal::units::Hertz(9600));
        let gps_uart = UartDriver::new(
            peripherals.uart1,
            peripherals.pins.gpio39, // TX → GPS RXI
            peripherals.pins.gpio38, // RX ← GPS TXO
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &uart_config,
        )
        .expect("Failed to init UART1 for GPS");
        tasks::gps_acquisition::start(gps_uart, QueueSender(gps_sentence_queue))
    };

    #[cfg(feature = "fake-gps")]
    let _gps_acquisition_thread = tasks::fake_gps::start(QueueSender(gps_sentence_queue));

    // Setup SD Card SPI bus (GPIO 36 SCK, 35 MOSI, 37 MISO):
    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        peripherals.pins.gpio36,       // SCK
        peripherals.pins.gpio35,       // MOSI
        Some(peripherals.pins.gpio37), // MISO
        &DriverConfig::new().dma(Dma::Disabled),
    )
    .expect("Failed to init SPI bus");

    let spi_device = SpiDeviceDriver::new(
        spi_driver,
        Some(peripherals.pins.gpio10), // CS
        &esp_idf_svc::hal::spi::config::Config::new()
            .baudrate(100.kHz().into())
            .data_mode(MODE_0),
    )
    .expect("Failed to init SPI device");

    // SD card queue
    let sd_card_queue: &'static Queue<FitnessTrackerSentence> = Box::leak(Box::new(Queue::new(8)));

    // Setup GPS Aggregation (or, routing):
    let _gps_aggregator_thread = tasks::gps_aggregator::start(
        QueueReceiver(gps_sentence_queue),
        QueueSender(display_queue),
        QueueSender(sd_card_queue),
    );

    // Setup SD Card logging:
    let _sd_card_thread = tasks::sd_card::start(spi_device, QueueReceiver(sd_card_queue));

    // Keep power_pin alive so it stays HIGH
    loop {
        //log_stack_high_water_marks();
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

    pub fn overwrite(&self, item: T) {
        unsafe {
            esp_idf_svc::sys::xQueueGenericSend(
                self.0.as_raw(),
                &item as *const T as *const _,
                0, // timeout irrelevant for overwrite
                2, // queueOVERWRITE
            );
        }
    }
}

/// Log the minimum free stack (bytes) each FreeRTOS task has ever had.
/// A value approaching zero means that task is close to overflowing.
fn log_stack_high_water_marks() {
    const TASK_NAMES: &[&[u8]] = &[b"GPS\0", b"Display\0", b"Task1\0", b"SDCard\0"];
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
