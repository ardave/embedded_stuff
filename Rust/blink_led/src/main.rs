use esp_idf_svc::hal::gpio::{InterruptType, PinDriver, Pull};
use esp_idf_svc::hal::peripherals::Peripherals;
use std::sync::mpsc;
use std::time::Duration;

// GPIO 8 = MI pin on QT Py C3 (LED)
// GPIO 4 = A0 pin on QT Py C3 (Switch)

fn main() {
    // Required for esp-idf-sys patches to link properly
    esp_idf_svc::sys::link_patches();

    // Initialize ESP logging
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("GPIO button-controlled LED starting");

    // Take ownership of peripherals
    let peripherals = Peripherals::take().expect("Failed to take peripherals");

    // Configure LED output (GPIO 8)
    let mut led = PinDriver::output(peripherals.pins.gpio8).expect("Failed to configure LED pin");
    led.set_low().expect("Failed to set LED low initially");

    // Configure switch input with pull-up (GPIO 4)
    let mut switch = PinDriver::input(peripherals.pins.gpio4).expect("Failed to configure switch pin");
    switch.set_pull(Pull::Up).expect("Failed to set pull-up");
    switch
        .set_interrupt_type(InterruptType::AnyEdge)
        .expect("Failed to set interrupt type");

    // Create a channel for ISR to task communication (similar to FreeRTOS queue)
    let (tx, rx) = mpsc::sync_channel::<()>(10);

    // Subscribe to GPIO interrupts with a callback
    // The callback runs in ISR context and sends a notification through the channel
    unsafe {
        switch
            .subscribe(move || {
                // Send notification (ignore errors if channel is full)
                let _ = tx.try_send(());
            })
            .expect("Failed to subscribe to GPIO interrupt");
    }

    // Enable the interrupt
    switch.enable_interrupt().expect("Failed to enable interrupt");

    log::info!("GPIO interrupt configured, waiting for button events...");

    // Main loop: process button events from the channel
    loop {
        match rx.recv_timeout(Duration::from_millis(100)) {
            Ok(()) => {
                // Re-enable interrupt for next edge
                switch.enable_interrupt().expect("Failed to re-enable interrupt");

                // Read the current switch state (low = pressed due to pull-up)
                let switch_pressed = switch.is_low();

                // Set LED based on switch state
                if switch_pressed {
                    led.set_high().expect("Failed to set LED high");
                    log::info!("Switch pressed - LED ON");
                } else {
                    led.set_low().expect("Failed to set LED low");
                    log::info!("Switch released - LED OFF");
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                // No event, continue waiting
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                log::error!("Channel disconnected unexpectedly");
                break;
            }
        }
    }
}
