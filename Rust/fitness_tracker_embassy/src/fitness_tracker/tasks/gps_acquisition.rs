use domain::display_content::DisplayContent;
use domain::gps_stuff::{FitnessTrackerSentence, GPSAcquisitionError};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::Publisher;
use embassy_sync::signal::Signal;
use esp_hal::uart::UartRx;
use nmea0183::Parser;

pub(crate) type GpsPublisher = Publisher<
    'static,
    CriticalSectionRawMutex,
    Result<FitnessTrackerSentence, GPSAcquisitionError>,
    8,
    4,
    1,
>;

// todo Select which NMEA sentences to output.
#[embassy_executor::task]
pub async fn gps_acquisition_task(
    mut uart: UartRx<'static, esp_hal::Async>,
    gps_publisher: GpsPublisher,
    display_signal: &'static Signal<CriticalSectionRawMutex, DisplayContent>,
) {
    let mut buf = [0u8; 128];
    let mut parser = Parser::new();

    loop {
        let len = match uart.read_async(&mut buf).await {
            Ok(len) => len,
            Err(_rx_error) => {
                display_signal.signal(DisplayContent::DbgUartErr);
                continue;
            }
        };

        for byte in &buf[..len] {
            if let Some(result) = parser.parse_from_byte(*byte) {
                match result {
                    Ok(nmea0183::ParseResult::GGA(Some(gga))) => {
                        gps_publisher
                            .publish(Ok(FitnessTrackerSentence::FixData(gga.into())))
                            .await;
                    }
                    Ok(nmea0183::ParseResult::RMC(Some(rmc))) => {
                        gps_publisher
                            .publish(Ok(FitnessTrackerSentence::MinNav(rmc.into())))
                            .await;
                    }
                    Ok(nmea0183::ParseResult::GGA(None) | nmea0183::ParseResult::RMC(None)) => {
                        gps_publisher.publish(Err(GPSAcquisitionError::NoFix)).await;
                    }
                    Ok(_) => {}  // ignore other sentence types
                    Err(_) => {} // ignore parse errors
                }
            }
        }
    }
}
