use crate::queue::FreeRtosQueue;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use nmea0183::{ParseResult, Parser};
use std::thread;
use std::time::Duration;

const PA1010D_ADDR: u8 = 0x10;
const GPS_BUF_SIZE: usize = 255;
const PADDING_BYTE: u8 = 0x0A;

#[derive(Clone, Copy)]
pub struct GpsReading {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub speed_knots: f32,
    pub course_degrees: f32,
    pub satellite_count: u8,
}

impl Default for GpsReading {
    fn default() -> Self {
        Self {
            latitude: 0.0,
            longitude: 0.0,
            altitude_m: 0.0,
            speed_knots: 0.0,
            course_degrees: 0.0,
            satellite_count: 0,
        }
    }
}

pub fn start<I: I2c + Send + 'static>(
    mut i2c: I,
    queues: &[&'static FreeRtosQueue<GpsReading>],
) -> thread::JoinHandle<()> {
    let queues: Vec<&'static FreeRtosQueue<GpsReading>> = queues.to_vec();

    ThreadSpawnConfiguration {
        name: Some(b"GPS\0"),
        stack_size: 4096,
        priority: 10,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set GPS thread config");

    thread::Builder::new()
        .name("GPS".to_string())
        .stack_size(4096)
        .spawn(move || {
            let mut parser = Parser::new();
            let mut buf = [0u8; GPS_BUF_SIZE];
            let mut reading = GpsReading::default();

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
                                    reading.latitude = gga.latitude.as_f64();
                                    reading.longitude = gga.longitude.as_f64();
                                    reading.altitude_m = gga
                                        .altitude
                                        .map(|a| a.meters)
                                        .unwrap_or(0.0);
                                    reading.satellite_count = gga.sat_in_use;

                                    info!(
                                        "GGA fix={:?} sats={} lat={:.6} lon={:.6} alt={}m",
                                        gga.gps_quality,
                                        gga.sat_in_use,
                                        reading.latitude,
                                        reading.longitude,
                                        reading.altitude_m,
                                    );

                                    for q in &queues {
                                        let _ = q.try_send(&reading);
                                    }
                                }
                                Ok(ParseResult::RMC(Some(rmc))) => {
                                    reading.speed_knots = rmc.speed.as_knots() as f32;
                                    reading.course_degrees = rmc
                                        .course
                                        .map(|c| c.degrees)
                                        .unwrap_or(0.0);

                                    info!(
                                        "RMC speed={:.1}kn course={}",
                                        reading.speed_knots,
                                        reading.course_degrees,
                                    );
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
