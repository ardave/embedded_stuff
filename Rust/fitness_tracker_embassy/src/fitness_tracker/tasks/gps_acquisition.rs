use domain::gps_stuff::FitnessTrackerSentence;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Publisher};
use esp_hal::uart::UartRx;
use nmea0183::Parser;

pub(crate) type GpsPublisher =
    Publisher<'static, CriticalSectionRawMutex, FitnessTrackerSentence, 8, 4, 1>;

// todo Select which NMEA sentences to output.
#[embassy_executor::task]
pub async fn gps_acquisition_task(
    mut uart: UartRx<'static, esp_hal::Async>,
    gps_publisher: GpsPublisher,
) {
    let mut buf = [0u8; 128];
    let mut parser = Parser::new();

    loop {
        let len = match uart.read_async(&mut buf).await {
            Ok(len) => len,
            Err(_rx_error) => {
                continue
            },
        };

        for byte in &buf[..len] {
            if let Some(result) = parser.parse_from_byte(*byte) {
                if let Ok(sentence) = result {
                    match sentence {
                        nmea0183::ParseResult::GGA(Some(gga)) => {
                            gps_publisher.publish(FitnessTrackerSentence::FixData(gga.into())).await;
                        }
                        nmea0183::ParseResult::RMC(Some(rmc)) => {
                            gps_publisher.publish(FitnessTrackerSentence::MinNav(rmc.into())).await;
                        }
                        _ => {} // ignore other sentence types
                    }
                }
            }
        }
    }
}