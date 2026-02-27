use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;
use std::time::{Duration, Instant};
use testable_logic::gps_sentence_joining::{
    FixMethod, FixQualityData, FitnessTrackerSentence, NavMode, NavSystem, RequiredNavData,
};

use crate::QueueSender;
use crate::tasks::gps_acquisition::GPSAcquisitionError;

/// Simulated GPS route point.
struct RoutePoint {
    lat: f64,
    lon: f64,
    alt_m: f32,
    speed_mph: f32,
    course: Option<f32>,
}

/// A short loop of points (roughly a lap around a park).
/// Customize these to whatever location/path you like.
const ROUTE: &[RoutePoint] = &[
    RoutePoint { lat: 40.748817, lon: -73.985428, alt_m: 10.0, speed_mph: 3.2, course: Some(90.0) },
    RoutePoint { lat: 40.748900, lon: -73.985000, alt_m: 10.5, speed_mph: 3.4, course: Some(45.0) },
    RoutePoint { lat: 40.749100, lon: -73.984600, alt_m: 11.0, speed_mph: 3.1, course: Some(0.0) },
    RoutePoint { lat: 40.749300, lon: -73.985000, alt_m: 10.8, speed_mph: 3.5, course: Some(315.0) },
    RoutePoint { lat: 40.749100, lon: -73.985428, alt_m: 10.2, speed_mph: 3.3, course: Some(270.0) },
    RoutePoint { lat: 40.748817, lon: -73.985428, alt_m: 10.0, speed_mph: 3.2, course: Some(180.0) },
];

pub fn start(
    sentence_queue: QueueSender<Result<FitnessTrackerSentence, GPSAcquisitionError>>,
) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"GPS\0"),
        stack_size: 4096,
        priority: 10,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set fake GPS thread config");

    thread::Builder::new()
        .name("GPS".to_string())
        .stack_size(4096)
        .spawn(move || {
            info!("[FakeGPS] started — cycling through {} route points", ROUTE.len());

            // Start a few seconds in so the display shows "--" briefly, like a real cold start
            thread::sleep(Duration::from_secs(3));

            let base_time = chrono::Utc::now();
            let mut tick: u32 = 0;

            loop {
                let pt = &ROUTE[tick as usize % ROUTE.len()];
                let timestamp = base_time + chrono::Duration::seconds(tick as i64);
                let utc_time = timestamp.time();

                // Send GGA-equivalent (FixData)
                let fix = FixQualityData {
                    utc_time,
                    lat: pt.lat,
                    lon: pt.lon,
                    fix_method: FixMethod::GPS,
                    hdop: 1.2,
                    num_satellites: 8,
                    altitude_meters: Some(pt.alt_m),
                    received_at: Instant::now(),
                };
                let _ = sentence_queue.send_back(Ok(FitnessTrackerSentence::FixData(fix)), 0);

                // Small delay between the two sentences, like real GPS
                thread::sleep(Duration::from_millis(100));

                // Send RMC-equivalent (MinNav)
                let nav = RequiredNavData {
                    timestamp,
                    source: NavSystem::GPS,
                    lat: pt.lat,
                    lon: pt.lon,
                    speed_mph: pt.speed_mph,
                    course: pt.course,
                    mode: NavMode::Autonomous,
                    received_at: Instant::now(),
                };
                let _ = sentence_queue.send_back(Ok(FitnessTrackerSentence::MinNav(nav)), 0);

                tick += 1;
                thread::sleep(Duration::from_millis(900));
            }
        })
        .expect("Failed to spawn fake GPS task")
}
