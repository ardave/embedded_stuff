mod tasks;

use log::info;
use std::thread;
use std::time::Duration;

fn main() {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let _gps_thread = tasks::fake_gps::start();

    loop {
        info!("Hello, world!");
        thread::sleep(Duration::from_secs(2));
    }
}
