use crate::tasks::gps_acquisition::FitnessTrackerSentence;
use crate::QueueReceiver;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;

// Combines GGA and RMEA sentences, provided that they are timely enough, before sending
// the joined Reading to downstream Queue(s)
pub fn start(queue: QueueReceiver<FitnessTrackerSentence>) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Task1\0"),
        stack_size: 6144,
        priority: 5,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set Task1 thread config");

    thread::Builder::new()
        .name("Task1".to_string())
        .stack_size(6144)
        .spawn(move || loop {
            let (sentence, _) = queue.recv_front(u32::MAX).unwrap();
            match sentence {
                FitnessTrackerSentence::FixData(fix_data) => {}
                FitnessTrackerSentence::MinNav(min_nav_data) => {} // GpsSentence::Gga(g) => {
                                                                   //     info!(
                                                                   //         "[Task1] GGA lat={:.6} lon={:.6} alt={:.1}m sats={}",
                                                                   //         g.latitude, g.longitude, g.altitude_m, g.satellite_count,
                                                                   //     );
                                                                   // }
                                                                   // GpsSentence::Rmc(r) => {
                                                                   //     info!(
                                                                   //         "[Task1] RMC speed={:.1}kn course={:.1}",
                                                                   //         r.speed_knots, r.course_degrees,
                                                                   //     );
                                                                   // }
            }
        })
        .expect("Failed to spawn Task1")
}
