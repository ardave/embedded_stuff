use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::{error, info, warn};
use nmea0183::{ParseResult, Parser};
use std::thread;
use std::time::Duration;
use testable_logic::gps_sentence_joining::FitnessTrackerSentence;

use crate::QueueSender;

const PA1010D_ADDR: u8 = 0x10;
const GPS_BUF_SIZE: usize = 255;
const PADDING_BYTE: u8 = 0x0A;

pub fn start<I: I2c + Send + 'static>(
    mut i2c: I,
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
                match i2c.read(PA1010D_ADDR, &mut buf) {
                    Ok(()) => {
                        let end = buf
                            .iter()
                            .rposition(|&b| b != PADDING_BYTE)
                            .map(|i| (i + 2).min(buf.len()))
                            .unwrap_or(0);

                        for result in parser.parse_from_bytes(&buf[..end]) {
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
                    Err(_) => {
                        error!("GPS I2C read error");
                        let _ = sentence_queue
                            .send_back(Err(GPSAcquisitionError::I2cReadError), 0)
                            .map_err(|_| error!("Error enqueueing GPS I2C read error."));
                    }
                }

                thread::sleep(Duration::from_millis(100));
            }
        })
        .expect("Failed to spawn GPS task")
}

#[derive(Clone, Copy, Debug)]
pub enum GPSAcquisitionError {
    I2cReadError,
    NMEAParseError,
}
