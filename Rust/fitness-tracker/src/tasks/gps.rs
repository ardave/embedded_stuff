use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use nmea0183::{ParseResult, Parser};
use std::thread;
use std::time::Duration;

const PA1010D_ADDR: u8 = 0x10;
const GPS_BUF_SIZE: usize = 255;
const PADDING_BYTE: u8 = 0x0A;

pub fn start<I: I2c + Send + 'static>(mut i2c: I) -> thread::JoinHandle<()> {
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
                                    info!(
                                        "GGA fix={:?} sats={} lat={:.6} lon={:.6} alt={}m",
                                        gga.gps_quality,
                                        gga.sat_in_use,
                                        gga.latitude.as_f64(),
                                        gga.longitude.as_f64(),
                                        gga.altitude
                                            .map(|a| a.meters)
                                            .unwrap_or(0.0),
                                    );
                                }
                                Ok(ParseResult::RMC(Some(rmc))) => {
                                    info!(
                                        "RMC speed={:.1}kn course={}",
                                        rmc.speed.as_knots(),
                                        rmc.course
                                            .map(|c| c.degrees)
                                            .unwrap_or(0.0),
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
