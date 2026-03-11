use domain::gps_stuff::FitnessTrackerSentence;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pubsub::Subscriber};

pub(crate) type GpsSubscriber =
    Subscriber<'static, CriticalSectionRawMutex, FitnessTrackerSentence, 8, 4, 1>;

#[embassy_executor::task]
pub async fn gps_aggregator_task(mut gps_subscriber: GpsSubscriber) {
    loop {
        let _sentence = gps_subscriber.next_message_pure().await;
    }
}