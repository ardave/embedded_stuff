use crate::queue::FreeRtosQueue;
use crate::tasks::gps::GpsReading;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;

pub fn start(queue: &'static FreeRtosQueue<GpsReading>) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Task2\0"),
        stack_size: 4096,
        priority: 5,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set Task2 thread config");

    thread::Builder::new()
        .name("Task2".to_string())
        .stack_size(4096)
        .spawn(move || loop {
            let reading = queue.recv_blocking();
            info!(
                "[Task2] lat={:.6} lon={:.6} alt={:.1}m sats={} spd={:.1}kn crs={:.1}",
                reading.latitude,
                reading.longitude,
                reading.altitude_m,
                reading.satellite_count,
                reading.speed_knots,
                reading.course_degrees,
            );
        })
        .expect("Failed to spawn Task2")
}
