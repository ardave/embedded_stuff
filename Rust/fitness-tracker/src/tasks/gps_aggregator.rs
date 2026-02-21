use crate::{
    tasks::{
        gps_acquisition::GPSAcquisitionError,
        user_display::{DisplayContent, NumSatellites, MPH},
    },
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
    sentence_queue: QueueReceiver<Result<FitnessTrackerSentence, GPSAcquisitionError>>,
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

    let mut state: Result<(Option<RequiredNavData>, Option<FixQualityData>), GPSAcquisitionError> =
        Ok((None, None));

    thread::Builder::new()
        .name("tasks::gps_aggregator".to_string())
        .stack_size(6144)
        .spawn(move || loop {
            let one_second_in_ticks = esp_idf_svc::sys::CONFIG_FREERTOS_HZ;
            match sentence_queue.recv_front(one_second_in_ticks) {
                Some((sentence_result, _)) => {
                    info!("Received sentence result like: {:?}", sentence_result);

                    match sentence_result {
                        Ok(sentence) => {
                            let (existing_nav, existing_fix) = state.unwrap_or((None, None));
                            match sentence {
                                FitnessTrackerSentence::FixData(fix_quality_data) => {
                                    state = Ok((existing_nav, Some(fix_quality_data)));
                                }
                                FitnessTrackerSentence::MinNav(required_nav_data) => {
                                    state = Ok((Some(required_nav_data), existing_fix));
                                }
                            }
                        }
                        Err(e) => state = Err(e),
                    }
                }
                None => {
                    info!("GPS aggregator status: {:?}", state);
                }
            }
            let display_content = to_display_content(&state);
            display_queue.overwrite(display_content);
        })
        .expect("Failed to spawn Task1")
}

fn to_display_content(
    state: &Result<(Option<RequiredNavData>, Option<FixQualityData>), GPSAcquisitionError>,
) -> DisplayContent {
    match state {
        Ok(state) => {
            let now = Instant::now();
            let maybe_mph = state
                .0
                .filter(|req| now - req.received_at < Duration::from_millis(5000))
                .map(|req| MPH(req.speed_mph));

            let maybe_num_sats = state
                .1
                .filter(|qual| now - qual.received_at < Duration::from_millis(5000))
                .map(|qual| NumSatellites(qual.num_satellites));

            DisplayContent::Reading(maybe_mph, maybe_num_sats)
        }
        Err(_gps_acquisition_error) => DisplayContent::GPSError,
    }
}
