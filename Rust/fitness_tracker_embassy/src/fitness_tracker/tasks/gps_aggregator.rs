use domain::{
    display_content::{DisplayContent, Mph, NumSatellites},
    gps_stuff::{FitnessTrackerSentence, FixQualityData, GPSAcquisitionError, RequiredNavData},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber, signal::Signal,
};
use embassy_time::{Duration, Instant};

pub(crate) type GpsSubscriber = Subscriber<
    'static,
    CriticalSectionRawMutex,
    Result<FitnessTrackerSentence, GPSAcquisitionError>,
    8,
    4,
    1,
>;

#[embassy_executor::task]
pub async fn gps_aggregator_task(
    mut gps_subscriber: GpsSubscriber,
    display_signal: &'static Signal<CriticalSectionRawMutex, DisplayContent>,
) {
    let mut state: Result<(Option<RequiredNavData>, Option<FixQualityData>), GPSAcquisitionError> =
        Ok((None, None));

    loop {
        // There's an alternate fn which requires you to acknowlege possible dropped messages:
        let gps_result = gps_subscriber.next_message_pure().await;

        match gps_result {
            Ok(sentence) => {
                let (existing_rnd, existing_fqd) = state.unwrap_or((None, None));
                match sentence {
                    FitnessTrackerSentence::FixData(fqd) => {
                        state = Ok((existing_rnd, Some(fqd)));
                    }
                    FitnessTrackerSentence::MinNav(rnd) => {
                        state = Ok((Some(rnd), existing_fqd));
                    }
                }
            }
            Err(gps_acquisition_error) => state = Err(gps_acquisition_error),
        }

        display_signal.signal(to_display_content(&state));
    }
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
                .map(|req| Mph(req.speed_mph));

            let maybe_num_sats = state
                .1
                .filter(|qual| now - qual.received_at < Duration::from_millis(5000))
                .map(|qual| NumSatellites(qual.num_satellites));

            DisplayContent::Reading(maybe_mph, maybe_num_sats)
        }
        Err(GPSAcquisitionError::NoFix) => DisplayContent::GPSNoFix,
        Err(_) => DisplayContent::GPSError,
    }
}
