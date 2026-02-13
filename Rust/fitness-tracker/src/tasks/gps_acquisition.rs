use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use nmea0183::{ParseResult, Parser};
use std::thread;
use std::time::{Duration, Instant};

use crate::QueueSender;

const PA1010D_ADDR: u8 = 0x10;
const GPS_BUF_SIZE: usize = 255;
const PADDING_BYTE: u8 = 0x0A;
const MAX_SENTENCE_AGE: Duration = Duration::from_secs(2);

pub fn start<I: I2c + Send + 'static>(
    mut i2c: I,
    sentence_queue: QueueSender<FitnessTrackerSentence>,
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
                            .map(|i| i + 1)
                            .unwrap_or(0);

                        for result in parser.parse_from_bytes(&buf[..end]) {
                            match result {
                                Ok(ParseResult::GGA(Some(gga))) => {
                                    let _ = sentence_queue
                                        .send_back(FitnessTrackerSentence::FixData(gga.into()), 0);
                                }
                                Ok(ParseResult::RMC(Some(rmc))) => {
                                    let _ = sentence_queue
                                        .send_back(FitnessTrackerSentence::MinNav(rmc.into()), 0);
                                }
                                Ok(ParseResult::GGA(None) | ParseResult::RMC(None)) => {
                                    info!("GPS: no fix");
                                }
                                Ok(_) => {}
                                Err(e) => {
                                    info!("NMEA parse error: {}", e);
                                }
                            }
                        }
                    }
                    Err(_) => {
                        info!("GPS I2C read error");
                    }
                }

                thread::sleep(Duration::from_millis(100));
            }
        })
        .expect("Failed to spawn GPS task")
}

#[derive(Clone, Copy)]
pub enum FitnessTrackerSentence {
    FixData(FixData),
    MinNav(MinNavData),
}

#[derive(Clone, Copy)]
pub struct FixData {}

impl From<nmea0183::GGA> for FixData {
    fn from(value: nmea0183::GGA) -> Self {
        Self {}
    }
}

#[derive(Clone, Copy)]
pub struct MinNavData {}

impl From<nmea0183::RMC> for MinNavData {
    fn from(value: nmea0183::RMC) -> Self {
        Self {}
    }
}
