use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::hal::uart::UartDriver;
use log::{error, warn};
use nmea0183::{ParseResult, Parser};
use std::thread;
use testable_logic::gps_sentence_joining::FitnessTrackerSentence;

use crate::QueueSender;

const GPS_BUF_SIZE: usize = 255;
const UART_READ_TIMEOUT_TICKS: u32 = 20;

pub fn start(
    uart: UartDriver<'static>,
    sentence_queue: QueueSender<Result<FitnessTrackerSentence, GPSAcquisitionError>>,
) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"GPS\0"),
        stack_size: 6144,
        priority: 10,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set GPS thread config");

    thread::Builder::new()
        .name("GPS".to_string())
        .stack_size(6144)
        .spawn(move || {
            let mut parser = Parser::new();
            let mut buf = [0u8; GPS_BUF_SIZE];

            loop {
                match uart.read(&mut buf, UART_READ_TIMEOUT_TICKS) {
                    Ok(0) => {}
                    Ok(n) => {
                        for result in parser.parse_from_bytes(&buf[..n]) {
                            match result {
                                Ok(ParseResult::GGA(Some(gga))) => {
                                    let _ = sentence_queue
                                        .send_back(
                                            Ok(FitnessTrackerSentence::FixData(gga.into())),
                                            0,
                                        )
                                        .map_err(|_| {
                                            error!(
                                                "Error enqueueing FitnessTrackerSentence::FixData"
                                            )
                                        });
                                }
                                Ok(ParseResult::RMC(Some(rmc))) => {
                                    let _ = sentence_queue
                                        .send_back(
                                            Ok(FitnessTrackerSentence::MinNav(rmc.into())),
                                            0,
                                        )
                                        .map_err(|_| {
                                            error!(
                                                "Error enqueueing FitnessTrackerSentence::MinNav"
                                            )
                                        });
                                }
                                Ok(ParseResult::GGA(None) | ParseResult::RMC(None)) => {
                                    warn!("GPS: no fix");
                                }
                                Ok(_) => {}
                                Err(_) => {
                                    error!("NMEA parse error.",);
                                    let _ = sentence_queue
                                        .send_back(Err(GPSAcquisitionError::NMEAParseError), 0)
                                        .map_err(|_| error!("Error enqueueing NMEA parse error."));
                                }
                            }
                        }
                    }
                    Err(e) if e.code() == esp_idf_svc::sys::ESP_ERR_TIMEOUT => {}
                    Err(_) => {
                        error!("GPS UART read error");
                        let _ = sentence_queue
                            .send_back(Err(GPSAcquisitionError::UartReadError), 0)
                            .map_err(|_| error!("Error enqueueing GPS UART read error."));
                    }
                }
            }
        })
        .expect("Failed to spawn GPS task")
}

#[derive(Clone, Copy, Debug)]
pub enum GPSAcquisitionError {
    UartReadError,
    NMEAParseError,
}
