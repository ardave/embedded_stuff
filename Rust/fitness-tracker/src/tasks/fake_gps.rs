use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use log::info;
use std::thread;
use std::time::Duration;

pub fn start() -> thread::JoinHandle<()> {
    ThreadSpawnConfiguration {
        name: Some(b"Poll GPS\0"),
        stack_size: 4096,
        priority: 10, // 1-24, higher = more priority
        inherit: false,
        pin_to_core: None, // ESP32-S2 is single-core, doesn't matter
        ..Default::default()
    }
    .set()
    .expect("Failed to set thread config");

    thread::Builder::new()
        .name("Poll GPS".to_string())
        .stack_size(4096)
        .spawn(|| loop {
            info!("Polling GPS...");
            thread::sleep(Duration::from_secs(1));
        })
        .expect("Failed to span Poll GPS Task")
}
