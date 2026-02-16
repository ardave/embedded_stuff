use crate::{
    tasks::user_display::{DisplayContent, NumSatellites, MPH},
    QueueReceiver, QueueSender,
};
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::{error, info};
use std::{
    thread,
    time::{Duration, Instant},
};
use testable_logic::gps_sentence_joining::{
    FitnessTrackerSentence, FixQualityData, RequiredNavData,
};

// Combines GGA and RMEA sentences, provided that they are timely enough, before sending
// the joined Reading to downstream Queue(s)
pub fn start(
    sentence_queue: QueueReceiver<FitnessTrackerSentence>,
    display_queue: QueueSender<DisplayContent>,
) -> thread::JoinHandle<()> {
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

    let mut state: (Option<RequiredNavData>, Option<FixQualityData>) = (None, None);

    thread::Builder::new()
        .name("Task1".to_string())
        .stack_size(6144)
        .spawn(move || loop {
            let one_second_in_ticks = esp_idf_svc::sys::CONFIG_FREERTOS_HZ;
            match sentence_queue.recv_front(one_second_in_ticks) {
                Some((sentence, _)) => {
                    info!("Received sentence like: {:?}", sentence);
                    match sentence {
                        FitnessTrackerSentence::FixData(fix_quality_data) => {
                            state.1 = Some(fix_quality_data);
                        }
                        FitnessTrackerSentence::MinNav(required_nav_data) => {
                            state.0 = Some(required_nav_data);
                        }
                    }

                    let display_content = state_to_display_content(&state);
                    let _ = display_queue
                        .send_back(display_content, one_second_in_ticks)
                        .map_err(|_| error!("Error enqueueing DisplayContent."));
                }
                None => {
                    info!("GPS aggregator status: {:?}", state);
                }
            }
        })
        .expect("Failed to spawn Task1")
}

fn state_to_display_content(
    state: &(Option<RequiredNavData>, Option<FixQualityData>),
) -> DisplayContent {
    let now = Instant::now();
    let maybe_mph = state
        .0
        .filter(|req| now - req.received_at < Duration::from_millis(1500))
        .map(|req| MPH(req.speed_mph));

    let maybe_num_sats = state
        .1
        .filter(|qual| now - qual.received_at < Duration::from_millis(1500))
        .map(|qual| NumSatellites(qual.num_satellites));

    DisplayContent::PositionFix(maybe_mph, maybe_num_sats)
}
