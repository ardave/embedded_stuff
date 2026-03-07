use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

pub(crate) type LogMessage = heapless::String<128>;
pub(crate) type LogChannel = Channel<CriticalSectionRawMutex, LogMessage, 8>;

#[embassy_executor::task]
pub(crate) async fn logging_task(log_channel: &'static LogChannel) {
    loop {
        let msg = log_channel.receive().await;
    }
}