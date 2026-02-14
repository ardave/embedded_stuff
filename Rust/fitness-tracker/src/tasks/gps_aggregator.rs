use crate::QueueReceiver;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;
use testable_logic::gps_sentence_joining::{FitnessTrackerSentence, GPSSentenceState};

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

    let mut state = GPSSentenceState::default();

    thread::Builder::new()
        .name("Task1".to_string())
        .stack_size(6144)
        .spawn(move || loop {
            let one_second_in_ticks = esp_idf_svc::sys::CONFIG_FREERTOS_HZ;
            match queue.recv_front(one_second_in_ticks) {
                Some((sentence, _)) => {
                    info!("Received sentence like: {:?}", sentence);
                    match sentence {
                        FitnessTrackerSentence::FixData(fix_data) => {
                            state.maybe_fix_data = Some(fix_data);
                        }
                        FitnessTrackerSentence::MinNav(min_nav_data) => {
                            state.maybe_min_nav_data = Some(min_nav_data);
                        }
                    }
                }
                None => {
                    info!("GPS aggregator status: {:?}", state);
                }
            }
        })
        .expect("Failed to spawn Task1")
}
