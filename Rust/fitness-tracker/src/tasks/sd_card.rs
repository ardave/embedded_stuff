use core::fmt::Write as _;
use std::thread;
use std::time::Duration;

use embedded_hal::digital::InputPin;
use embedded_hal::spi::SpiDevice;
use embedded_sdmmc::{SdCard, VolumeIdx, VolumeManager};
use esp_idf_svc::hal::delay::Ets;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::{error, info, warn};
use testable_logic::gps_sentence_joining::{
    FixMethod, FitnessTrackerSentence, NavMode, NavSystem,
};

use crate::QueueReceiver;

struct DummyTimesource;

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

pub fn start<SPI, CD>(
    spi: SPI,
    card_detect: CD,
    queue: QueueReceiver<FitnessTrackerSentence>,
) -> thread::JoinHandle<()>
where
    SPI: SpiDevice<u8> + Send + 'static,
    CD: InputPin + Send + 'static,
{
    ThreadSpawnConfiguration {
        name: Some(b"SDCard\0"),
        stack_size: 8192,
        priority: 5,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set SDCard thread config");

    thread::Builder::new()
        .name("SDCard".to_string())
        .stack_size(8192)
        .spawn(move || {
            let mut card_detect = card_detect;
            // Wait for card insertion before initializing
            wait_for_card(&mut card_detect, &queue);

            info!("SD card detected, initializing...");
            let sdcard = SdCard::new(spi, Ets);

            match sdcard.num_bytes() {
                Ok(size) => info!("SD card size: {} bytes", size),
                Err(e) => {
                    error!("SD card init failed: {:?}", e);
                    loop {
                        thread::sleep(Duration::from_secs(10));
                    }
                }
            }

            let mut volume_mgr = VolumeManager::new(sdcard, DummyTimesource);
            loop {
                match log_to_file(&mut volume_mgr, &mut card_detect, &queue) {
                    Ok(()) => info!("SD: file session closed normally"),
                    Err(e) => {
                        error!("SD write error: {}", e);
                        thread::sleep(Duration::from_secs(2));
                    }
                }
            }
        })
        .expect("Failed to spawn SDCard")
}

fn wait_for_card<CD>(card_detect: &mut CD, queue: &QueueReceiver<FitnessTrackerSentence>)
where
    CD: InputPin,
{
    loop {
        // DET pin shorts to GND when card inserted; internal pull-up makes it HIGH when absent
        if card_detect.is_low().unwrap_or(false) {
            return;
        }
        info!("SD: waiting for card...");
        thread::sleep(Duration::from_secs(2));
        // Drain queued items while waiting so we don't block producers
        while queue.recv_front(0).is_some() {}
    }
}

fn log_to_file<D, CD>(
    volume_mgr: &mut VolumeManager<D, DummyTimesource>,
    card_detect: &mut CD,
    queue: &QueueReceiver<FitnessTrackerSentence>,
) -> Result<(), &'static str>
where
    D: embedded_sdmmc::BlockDevice,
    <D as embedded_sdmmc::BlockDevice>::Error: core::fmt::Debug,
    CD: InputPin,
{
    let raw_vol = volume_mgr
        .open_raw_volume(VolumeIdx(0))
        .map_err(|_| "Failed to open volume")?;
    let raw_dir = volume_mgr
        .open_root_dir(raw_vol)
        .map_err(|_| "Failed to open root dir")?;
    let raw_file = volume_mgr
        .open_file_in_dir(
            raw_dir,
            "GPS_LOG.TXT",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .map_err(|_| "Failed to open GPS_LOG.TXT")?;

    info!("SD: GPS_LOG.TXT opened for append");

    let mut write_count: u32 = 0;
    let one_second_in_ticks = esp_idf_svc::sys::CONFIG_FREERTOS_HZ;

    loop {
        // Check card is still present
        if card_detect.is_high().unwrap_or(false) {
            warn!("SD card removed!");
            let _ = volume_mgr.close_file(raw_file);
            let _ = volume_mgr.close_dir(raw_dir);
            let _ = volume_mgr.close_volume(raw_vol);
            return Err("Card removed");
        }

        if let Some((sentence, _)) = queue.recv_front(one_second_in_ticks) {
            let mut buf = [0u8; 256];
            let len = format_json_line(&sentence, &mut buf);
            if len > 0 {
                if volume_mgr.write(raw_file, &buf[..len]).is_err() {
                    let _ = volume_mgr.close_file(raw_file);
                    let _ = volume_mgr.close_dir(raw_dir);
                    let _ = volume_mgr.close_volume(raw_vol);
                    return Err("Write failed");
                }
                write_count += 1;

                if write_count % 10 == 0
                    && volume_mgr.flush_file(raw_file).is_err()
                {
                    let _ = volume_mgr.close_file(raw_file);
                    let _ = volume_mgr.close_dir(raw_dir);
                    let _ = volume_mgr.close_volume(raw_vol);
                    return Err("Flush failed");
                }
            }
        }
    }
}

fn format_json_line(sentence: &FitnessTrackerSentence, buf: &mut [u8]) -> usize {
    let mut w = BufWriter::new(buf);
    match sentence {
        FitnessTrackerSentence::FixData(fix) => {
            let fix_str = match fix.fix_method {
                FixMethod::Invalid => "Invalid",
                FixMethod::GPS => "GPS",
                FixMethod::DGPS => "DGPS",
                FixMethod::PPS => "PPS",
                FixMethod::RTKFixed => "RTKFixed",
                FixMethod::RTKFloat => "RTKFloat",
                FixMethod::Estimated => "Estimated",
                FixMethod::Manual => "Manual",
                FixMethod::Simulation => "Simulation",
            };
            let _ = write!(
                w,
                "{{\"type\":\"fix\",\"utc_time\":\"{}\",\"lat\":{:.6},\"lon\":{:.6},",
                fix.utc_time, fix.lat, fix.lon
            );
            match fix.altitude_meters {
                Some(alt) => {
                    let _ = write!(w, "\"alt_m\":{:.1},", alt);
                }
                None => {
                    let _ = write!(w, "\"alt_m\":null,");
                }
            }
            let _ = writeln!(
                w,
                "\"sats\":{},\"hdop\":{:.1},\"fix\":\"{}\"}}",
                fix.num_satellites, fix.hdop, fix_str
            );
        }
        FitnessTrackerSentence::MinNav(nav) => {
            let mode_str = match nav.mode {
                NavMode::Autonomous => "Autonomous",
                NavMode::Differential => "Differential",
                NavMode::Estimated => "Estimated",
                NavMode::Manual => "Manual",
                NavMode::Simulator => "Simulator",
                NavMode::NotValid => "NotValid",
            };
            let system_str = match nav.source {
                NavSystem::GPS => "GPS",
                NavSystem::GLONASS => "GLONASS",
                NavSystem::Galileo => "Galileo",
                NavSystem::Beidou => "Beidou",
                NavSystem::GNSS => "GNSS",
            };
            let _ = write!(
                w,
                "{{\"type\":\"nav\",\"timestamp\":\"{}\",\"lat\":{:.6},\"lon\":{:.6},\"speed_mph\":{:.1},",
                nav.timestamp.format("%Y-%m-%dT%H:%M:%SZ"),
                nav.lat,
                nav.lon,
                nav.speed_mph
            );
            match nav.course {
                Some(c) => {
                    let _ = write!(w, "\"course\":{:.1},", c);
                }
                None => {
                    let _ = write!(w, "\"course\":null,");
                }
            }
            let _ = writeln!(
                w,
                "\"mode\":\"{}\",\"system\":\"{}\"}}",
                mode_str, system_str
            );
        }
    }
    w.pos
}

struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl core::fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = self.buf.len() - self.pos;
        let to_copy = bytes.len().min(remaining);
        self.buf[self.pos..self.pos + to_copy].copy_from_slice(&bytes[..to_copy]);
        self.pos += to_copy;
        if to_copy < bytes.len() {
            Err(core::fmt::Error)
        } else {
            Ok(())
        }
    }
}
