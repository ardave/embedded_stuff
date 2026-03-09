use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Receiver, Sender};
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::CdcAcmClass;
use esp_hal::otg_fs::Usb;
use esp_hal::otg_fs::asynch::{Config as OtgConfig, Driver};
use esp_hal::peripherals::{GPIO19, GPIO20, USB0};
use esp_println::println;

pub(crate) type LogMessage = heapless::String<128>;
pub(crate) type LogReceiver = Receiver<'static, CriticalSectionRawMutex, LogMessage, 8>;
pub(crate) type LogSender = Sender<'static, CriticalSectionRawMutex, LogMessage, 8>;

/// Send a `&str` as a log message. Non-blocking; silently drops if the
/// channel is full or the message exceeds 128 bytes.
pub(crate) fn usb_log(sender: &LogSender, msg: &str) {
    if let Ok(s) = LogMessage::try_from(msg) {
        let _ = sender.try_send(s);
    }
}

/// Formatted USB CDC log macro. Non-blocking; silently drops on overflow
/// or full channel.
///
/// Usage: `usb_log_fmt!(sender, "Speed: {:.1} mph", speed);`
#[macro_export]
macro_rules! usb_log_fmt {
    ($sender:expr, $($arg:tt)*) => {{
        use core::fmt::Write as _;
        let mut s = $crate::tasks::usb_otg_logger::LogMessage::new();
        let _ = core::write!(s, $($arg)*);
        let _ = $sender.try_send(s);
    }};
}


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

    let mut usb_dev = usb_builder.build();

    let usb_fut = usb_dev.run();
    let writer_fut = async {
        loop {
            class.wait_connection().await;
            println!("USB CDC connected");
            loop {
                let msg = log_channel.receive().await;
                if class.write_packet(msg.as_bytes()).await.is_err() {
                    break;
                }
                if class.write_packet(b"\r\n").await.is_err() {
                    break;
                }
            }
            println!("USB CDC disconnected");
        }
    };

    join(usb_fut, writer_fut).await;
}