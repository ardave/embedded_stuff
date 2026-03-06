use domain::display_content::DisplayContent;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::i2c::master::I2c;



#[embassy_executor::task]
pub async fn display_task(
    i2c: I2c<'static, I2C0, esp_hal::Async>,
    signal: &'static Signal<CriticalSectionRawMutex, DisplayContent>
) {
    loop {
        let next = signal.wait().await;
    }
}