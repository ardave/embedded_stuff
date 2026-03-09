#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use domain::display_content::DisplayContent;
use domain::gps_stuff::FitnessTrackerSentence;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::Config as I2cConfig;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use static_cell::StaticCell;

use crate::tasks::usb_otg_logger::{LogMessage, usb_log};

pub mod sh1107;
pub mod tasks;

static DISPLAY_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DisplayContent>> =
    StaticCell::new();

static LOG_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, LogMessage, 8>> = StaticCell::new();

static GPS_POSITION_CHANNEL: StaticCell<Channel<CriticalSectionRawMutex, FitnessTrackerSentence, 8>> = StaticCell::new();

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 139264);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    println!("Fitness tracker embassy started!");

    let log_channel = LOG_CHANNEL.init(Channel::new());
    let gps_channel = GPS_POSITION_CHANNEL.init(Channel::new());

    // PA1010D GPS on UART1: board TX (GPIO39) → GPS RXI, board RX (GPIO38) ← GPS TXO
    let uart_config = esp_hal::uart::Config::default()
        .with_baudrate(9600);
    let (uart_rx, _uart_tx) = esp_hal::uart::Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO39)
        .with_rx(peripherals.GPIO38)
        .into_async()
        .split();

    // Enable I2C / STEMMA QT power (GPIO7 must be HIGH on Adafruit ESP32-S2 Feather Rev C)
    let _i2c_power = esp_hal::gpio::Output::new(
        peripherals.GPIO7,
        esp_hal::gpio::Level::High,
        esp_hal::gpio::OutputConfig::default(),
    );
    Timer::after(Duration::from_millis(250)).await;

    let i2c0_config = I2cConfig::default().with_frequency(esp_hal::time::Rate::from_khz(200));
    let i2c0 = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c0_config)
        .unwrap()
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO4)
        .into_async();

    let display_signal = DISPLAY_SIGNAL.init(Signal::new());

    let heartbeat_fut = async {
        loop {
            let sender = log_channel.sender();
            usb_log(&sender, "hello world");
            Timer::after(Duration::from_secs(1)).await;
        }
    };

    spawner
        .spawn(tasks::display::display_task(i2c0, display_signal))
        .unwrap();

    spawner
        .spawn(tasks::usb_otg_logger::logging_task(
            peripherals.USB0,
            peripherals.GPIO19,
            peripherals.GPIO20,
            log_channel.receiver(),))
        .unwrap();

    spawner
        .spawn(tasks::gps_acquisition::gps_acquisition_task(
            uart_rx,
            gps_channel.sender()))
        .unwrap();

    display_signal.signal(DisplayContent::Initialized);

    heartbeat_fut.await;

    // join never returns — USB device runs forever
    #[allow(clippy::empty_loop)]
    loop {}

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
