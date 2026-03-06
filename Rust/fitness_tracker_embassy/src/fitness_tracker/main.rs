#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use domain::display_content::DisplayContent;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::otg_fs::asynch::{Config as OtgConfig, Driver};
use esp_hal::otg_fs::Usb;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use static_cell::StaticCell;

pub mod display;

static DISPLAY_SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, DisplayContent>> = StaticCell::new();

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

    // Initialize USB OTG peripheral (GPIO20 = D+, GPIO19 = D-)
    let usb = Usb::new(peripherals.USB0, peripherals.GPIO20, peripherals.GPIO19);

    // Create async USB driver
    let mut ep_out_buffer = [0u8; 1024];
    let driver = Driver::new(usb, &mut ep_out_buffer, OtgConfig::default());

    // Configure USB device descriptor
    let mut usb_config = embassy_usb::Config::new(0x303A, 0x3001);
    usb_config.manufacturer = Some("Dave Falkner");
    usb_config.product = Some("Fitness Tracker");
    usb_config.serial_number = Some("00000001");
    // Required for Windows composite device compatibility
    usb_config.device_class = 0xEF;
    usb_config.device_sub_class = 0x02;
    usb_config.device_protocol = 0x01;
    usb_config.composite_with_iads = true;

    // Descriptor buffers
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let i2c0_config = I2cConfig::default().with_frequency(esp_hal::time::Rate::from_khz(200));
    let i2c0 = esp_hal::i2c::master::I2c::new(peripherals.I2C0, i2c0_config).unwrap()
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO4)
        .into_async();

    DISPLAY_SIGNAL.init(Signal::new());

    let mut usb_builder = Builder::new(
        driver,
        usb_config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create CDC ACM serial class (max packet size 64)
    let mut class = CdcAcmClass::new(&mut usb_builder, &mut state, 64);

    let mut usb_dev = usb_builder.build();

    // Run USB device stack and heartbeat writer concurrently
    let usb_fut = usb_dev.run();
    let heartbeat_fut = async {
        loop {
            class.wait_connection().await;
            println!("USB CDC connected");
            loop {
                if class.write_packet(b"heartbeat\r\n").await.is_err() {
                    break;
                }
                Timer::after(Duration::from_secs(1)).await;
            }
            println!("USB CDC disconnected");
        }
    };



    spawner.spawn(display::display_task(i2c0, DISPLAY_SIGNAL)).unwrap();

    join(usb_fut, heartbeat_fut).await;

    // join never returns — USB device runs forever
    loop {}

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v~1.0/examples
}
