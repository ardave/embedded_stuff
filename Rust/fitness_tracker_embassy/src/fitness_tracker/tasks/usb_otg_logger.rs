use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::{Channel, Receiver}};
use embassy_usb::{Builder, class::cdc_acm::CdcAcmClass};
use esp_hal::{otg_fs::{Usb, asynch::{Config as OtgConfig, Driver}}, peripherals::{GPIO19, GPIO20, USB0}};
use esp_println::println;

pub(crate) type LogMessage = heapless::String<128>;
pub(crate) type LogReceiver = Receiver<'static, CriticalSectionRawMutex, LogMessage, 8>;


#[embassy_executor::task]
pub(crate) async fn logging_task(
    usb0: USB0<'static>,
    gpio19: GPIO19<'static>,
    gpio20: GPIO20<'static>,
    log_channel: LogReceiver
) {
    // Initialize USB OTG peripheral (GPIO20 = D+, GPIO19 = D-)
    let usb = Usb::new(usb0, gpio20, gpio19);

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

    // Create async USB driver
    let mut ep_out_buffer = [0u8; 1024];
    let driver = Driver::new(usb, &mut ep_out_buffer, OtgConfig::default());

    let mut state = embassy_usb::class::cdc_acm::State::new();

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


    loop {
        let msg = log_channel.receive().await;

        if let Err(e) = class.write_packet(msg.as_bytes()).await {
            println!("shrug?");
        }

        // if class.write_packet(b"heartbeat\r\n").await.is_err() {
        //     break;
        // }
    }
}