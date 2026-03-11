use domain::gps_stuff::{
    FixMethod, FixQualityData, FitnessTrackerSentence, GPSAcquisitionError, NavMode, NavSystem,
    RequiredNavData,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::Publisher;
use embassy_time::{Duration, Instant, Timer};

pub(crate) type GpsPublisher = Publisher<
    'static,
    CriticalSectionRawMutex,
    Result<FitnessTrackerSentence, GPSAcquisitionError>,
    8,
    4,
    1,
>;

struct Waypoint {
    lat: f64,
    lon: f64,
    altitude: f32,
    speed_mph: f32,
    course: f32,
}

const ROUTE: &[Waypoint] = &[
    Waypoint { lat: 40.7484, lon: -73.9856, altitude: 10.0, speed_mph: 3.1, course: 0.0 },
    Waypoint { lat: 40.7486, lon: -73.9853, altitude: 10.5, speed_mph: 3.3, course: 45.0 },
    Waypoint { lat: 40.7488, lon: -73.9850, altitude: 11.0, speed_mph: 3.5, course: 90.0 },
    Waypoint { lat: 40.7490, lon: -73.9848, altitude: 10.8, speed_mph: 3.4, course: 180.0 },
    Waypoint { lat: 40.7488, lon: -73.9850, altitude: 10.3, speed_mph: 3.2, course: 270.0 },
    Waypoint { lat: 40.7486, lon: -73.9853, altitude: 10.0, speed_mph: 3.1, course: 315.0 },
];

#[allow(clippy::large_stack_frames, reason = "async task state machine; runs on its own stack")]
#[embassy_executor::task]
pub async fn fake_gps_task(gps_publisher: GpsPublisher) {
    // Simulate cold start delay
    Timer::after(Duration::from_secs(3)).await;

    let mut idx: usize = 0;
    let mut seconds: u32 = 0;

    loop {
        let wp = &ROUTE[idx % ROUTE.len()];
        let hours = (seconds / 3600) % 24;
        let minutes = (seconds % 3600) / 60;
        let secs = seconds % 60;

        let utc_time =
            domain::chrono::NaiveTime::from_hms_opt(hours, minutes, secs).expect("invalid fake GPS time");

        let fix_data = FixQualityData {
            utc_time,
            lat: wp.lat,
            lon: wp.lon,
            fix_method: FixMethod::GPS,
            hdop: 1.2,
            num_satellites: 8,
            altitude_meters: Some(wp.altitude),
            received_at: Instant::now(),
        };
        gps_publisher
            .publish(Ok(FitnessTrackerSentence::FixData(fix_data)))
            .await;

        Timer::after(Duration::from_millis(100)).await;

        let date = domain::chrono::NaiveDate::from_ymd_opt(2026, 3, 11).expect("invalid fake GPS date");
        let timestamp = date
            .and_hms_opt(hours, minutes, secs)
            .expect("invalid fake GPS datetime");

        let nav_data = RequiredNavData {
            timestamp,
            source: NavSystem::GPS,
            lat: wp.lat,
            lon: wp.lon,
            speed_mph: wp.speed_mph,
            course: Some(wp.course),
            mode: NavMode::Simulator,
            received_at: Instant::now(),
        };
        gps_publisher
            .publish(Ok(FitnessTrackerSentence::MinNav(nav_data)))
            .await;

        idx += 1;
        seconds += 1;

        Timer::after(Duration::from_millis(900)).await;
    }
}
