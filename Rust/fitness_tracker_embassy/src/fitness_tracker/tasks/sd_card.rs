use core::fmt::Write as _;

use domain::gps_stuff::{FitnessTrackerSentence, FixMethod, NavMode, NavSystem};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{RawDirectory, RawFile, RawVolume, SdCard, VolumeIdx, VolumeManager};
use esp_hal::delay::Delay;
use esp_hal::gpio::Output;
use esp_hal::spi::master::Spi;
use esp_println::println;

use crate::tasks::gps_aggregator::GpsSubscriber;

type SdSpiDevice = ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>;
type SdVolumeManager = VolumeManager<SdCard<SdSpiDevice, Delay>, DummyTimesource>;

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

struct SdHandles {
    raw_vol: RawVolume,
    raw_dir: RawDirectory,
    activities_dir: RawDirectory,
    raw_file: RawFile,
}

impl SdHandles {
    fn close_all(self, volume_mgr: &mut SdVolumeManager) {
        let _ = volume_mgr.close_file(self.raw_file);
        let _ = volume_mgr.close_dir(self.activities_dir);
        let _ = volume_mgr.close_dir(self.raw_dir);
        let _ = volume_mgr.close_volume(self.raw_vol);
    }
}

#[allow(clippy::large_stack_frames)]
#[embassy_executor::task]
pub(crate) async fn sd_task(spi_device: SdSpiDevice, mut gps_subscriber: GpsSubscriber) {
    let sdcard = SdCard::new(spi_device, Delay::new());

    // Retry SD card init until successful
    loop {
        match sdcard.num_bytes() {
            Ok(size) => {
                println!("[SD] card size: {} bytes", size);
                break;
            }
            Err(e) => {
                println!("[SD] init attempt failed ({:?}), retrying in 1s...", e);
                sdcard.mark_card_uninit();
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    }

    let mut volume_mgr = VolumeManager::new(sdcard, DummyTimesource);

    loop {
        let handles = match initialize(&mut volume_mgr) {
            Ok(h) => h,
            Err(e) => {
                println!("[SD] init error: {}", e);
                Timer::after(Duration::from_secs(2)).await;
                continue;
            }
        };

        println!("[SD] ACTIVITY/GPS_LOG.TXT opened for append");

        let err = loop {
            let gps_result = gps_subscriber.next_message_pure().await;

            if let Ok(sentence) = gps_result
                && let Err(e) = log_to_file(&mut volume_mgr, &handles, &sentence)
            {
                break e;
            }
        };

        println!("[SD] write error: {}", err);
        handles.close_all(&mut volume_mgr);
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[allow(clippy::large_stack_frames)]
fn initialize(volume_mgr: &mut SdVolumeManager) -> Result<SdHandles, &'static str> {
    let raw_vol = volume_mgr.open_raw_volume(VolumeIdx(0)).map_err(|e| {
        println!("[SD] failed to open volume: {:?}", e);
        "Failed to open volume"
    })?;
    let raw_dir = volume_mgr.open_root_dir(raw_vol).map_err(|e| {
        println!("[SD] failed to open root dir: {:?}", e);
        let _ = volume_mgr.close_volume(raw_vol);
        "Failed to open root dir"
    })?;

    // Create ACTIVITY subdirectory (8.3 name limit; ignore "already exists" error)
    match volume_mgr.make_dir_in_dir(raw_dir, "ACTIVITY") {
        Ok(()) => println!("[SD] created ACTIVITY directory"),
        Err(embedded_sdmmc::Error::DirAlreadyExists) => {
            println!("[SD] ACTIVITY directory already exists");
        }
        Err(e) => {
            println!("[SD] failed to create ACTIVITY dir: {:?}", e);
            let _ = volume_mgr.close_dir(raw_dir);
            let _ = volume_mgr.close_volume(raw_vol);
            return Err("Failed to create ACTIVITY dir");
        }
    }

    let activities_dir = volume_mgr.open_dir(raw_dir, "ACTIVITY").map_err(|e| {
        println!("[SD] failed to open ACTIVITY dir: {:?}", e);
        let _ = volume_mgr.close_dir(raw_dir);
        let _ = volume_mgr.close_volume(raw_vol);
        "Failed to open ACTIVITY dir"
    })?;

    let raw_file = volume_mgr
        .open_file_in_dir(
            activities_dir,
            "GPS_LOG.TXT",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .map_err(|e| {
            println!("[SD] failed to open GPS_LOG.TXT: {:?}", e);
            let _ = volume_mgr.close_dir(activities_dir);
            let _ = volume_mgr.close_dir(raw_dir);
            let _ = volume_mgr.close_volume(raw_vol);
            "Failed to open GPS_LOG.TXT"
        })?;

    Ok(SdHandles {
        raw_vol,
        raw_dir,
        activities_dir,
        raw_file,
    })
}

#[allow(clippy::large_stack_frames)]
fn log_to_file(
    volume_mgr: &mut SdVolumeManager,
    handles: &SdHandles,
    sentence: &FitnessTrackerSentence,
) -> Result<(), &'static str> {
    let mut buf = [0u8; 256];
    let len = format_json_line(sentence, &mut buf);
    if len > 0 {
        volume_mgr
            .write(handles.raw_file, &buf[..len])
            .map_err(|_| "Write failed")?;

        println!("[SD] wrote {} bytes", len);

        volume_mgr
            .flush_file(handles.raw_file)
            .map_err(|_| "Flush failed")?;
    }
    Ok(())
}

#[allow(clippy::large_stack_frames)]
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
                nav.timestamp, nav.lat, nav.lon, nav.speed_mph
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
