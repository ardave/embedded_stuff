use crate::queue::FreeRtosQueue;
use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use nmea0183::{ParseResult, Parser};
use std::thread;
use std::time::{Duration, Instant};

const PA1010D_ADDR: u8 = 0x10;
const GPS_BUF_SIZE: usize = 255;
const PADDING_BYTE: u8 = 0x0A;
const MAX_SENTENCE_AGE: Duration = Duration::from_secs(2);

#[derive(Clone, Copy)]
pub struct GpsReading {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub speed_knots: f32,
    pub course_degrees: f32,
    pub satellite_count: u8,
}

#[derive(Clone, Copy)]
pub struct GgaData {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f32,
    pub satellite_count: u8,
}

#[derive(Clone, Copy)]
pub struct RmcData {
    pub speed_knots: f32,
    pub course_degrees: f32,
}

#[derive(Clone, Copy)]
pub enum GpsSentence {
    Gga(GgaData),
    Rmc(RmcData),
}

fn assemble_reading(gga: &GgaData, rmc: &RmcData) -> GpsReading {
    GpsReading {
        latitude: gga.latitude,
        longitude: gga.longitude,
        altitude_m: gga.altitude_m,
        speed_knots: rmc.speed_knots,
        course_degrees: rmc.course_degrees,
        satellite_count: gga.satellite_count,
    }
}

pub fn start<I: I2c + Send + 'static>(
    mut i2c: I,
    sentence_queues: &[&'static FreeRtosQueue<GpsSentence>],
    reading_queues: &[&'static FreeRtosQueue<GpsReading>],
) -> thread::JoinHandle<()> {
    let sentence_queues: Vec<&'static FreeRtosQueue<GpsSentence>> = sentence_queues.to_vec();
    let reading_queues: Vec<&'static FreeRtosQueue<GpsReading>> = reading_queues.to_vec();

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
            let mut _last_gga: Option<(GgaData, Instant)> = None;
            let mut last_rmc: Option<(RmcData, Instant)> = None;

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
                                    let data = GgaData {
                                        latitude: gga.latitude.as_f64(),
                                        longitude: gga.longitude.as_f64(),
                                        altitude_m: gga.altitude.map(|a| a.meters).unwrap_or(0.0),
                                        satellite_count: gga.sat_in_use,
                                    };

                                    let sentence = GpsSentence::Gga(data);
                                    for q in &sentence_queues {
                                        let _ = q.try_send(&sentence);
                                    }

                                    _last_gga = Some((data, Instant::now()));

                                    if let Some((rmc, rmc_time)) = &last_rmc {
                                        if rmc_time.elapsed() < MAX_SENTENCE_AGE {
                                            let reading = assemble_reading(&data, rmc);
                                            for q in &reading_queues {
                                                let _ = q.try_send(&reading);
                                            }
                                        }
                                    }

                                    info!(
                                        "GGA fix={:?} sats={} lat={:.6} lon={:.6} alt={}m",
                                        gga.gps_quality,
                                        gga.sat_in_use,
                                        data.latitude,
                                        data.longitude,
                                        data.altitude_m,
                                    );
                                }
                                Ok(ParseResult::RMC(Some(rmc))) => {
                                    let data = RmcData {
                                        speed_knots: rmc.speed.as_knots(),
                                        course_degrees: rmc.course.map(|c| c.degrees).unwrap_or(0.0),
                                    };

                                    let sentence = GpsSentence::Rmc(data);
                                    for q in &sentence_queues {
                                        let _ = q.try_send(&sentence);
                                    }

                                    last_rmc = Some((data, Instant::now()));

                                    info!(
                                        "RMC speed={:.1}kn course={}",
                                        data.speed_knots, data.course_degrees,
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
