use domain::gps_stuff::FitnessTrackerSentence;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Sender};
use esp_hal::uart::UartRx;

pub(crate) type GpsSender = Sender<'static, CriticalSectionRawMutex, FitnessTrackerSentence, 8>;

// todo Select which NMEA sentences to output.
#[embassy_executor::task]
pub async fn gps_acquisition_task(
    uart: UartRx<'static, esp_hal::Async>,
    gps_sender: GpsSender,
) {
    loop {

    }
}