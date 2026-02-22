use crate::QueueReceiver;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use std::thread;
use testable_logic::gps_sentence_joining::FitnessTrackerSentence;

pub fn start(queue: QueueReceiver<FitnessTrackerSentence>) -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Task2\0"),
        stack_size: 6144,
        priority: 5,
        inherit: false,
        pin_to_core: None,
        ..Default::default()
    }
    .set()
    .expect("Failed to set Task2 thread config");

    thread::Builder::new()
        .name("Task2".to_string())
        .stack_size(6144)
        .spawn(move || loop {
            match queue.recv_front(u32::MAX) {
                Some((FitnessTrackerSentence::FixData(fix), _)) => { /* use fix */ }
                Some((FitnessTrackerSentence::MinNav(nav), _)) => { /* use nav */ }
                None => { /* handle None */ }
            }
        })
        .expect("Failed to spawn Task2")
}
