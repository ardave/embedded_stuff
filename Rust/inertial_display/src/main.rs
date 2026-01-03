mod accelerometer;
mod display;

use embedded_graphics::{
    mono_font::MonoTextStyle,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
    Drawable,
};
use profont::PROFONT_24_POINT;
use esp_idf_hal::{
    delay::FreeRtos,
    i2c::{I2cConfig, I2cDriver},
    prelude::*,
};

use accelerometer::{determine_motion_state, AccelData, Icm20948, MotionState};
use display::Sh1107Display;

fn main() {
    // Initialize ESP-IDF
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Inertial Display starting...");

    // Get peripherals
    let peripherals = Peripherals::take().unwrap();

    // Configure I2C
    // QT Py ESP32 Pico STEMMA QT pins: SDA = GPIO22, SCL = GPIO19
    let sda = peripherals.pins.gpio22;
    let scl = peripherals.pins.gpio19;

    let i2c_config = I2cConfig::new().baudrate(400_000.Hz());
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config).unwrap();

    log::info!("I2C initialized");

    // Create display driver (which owns the I2C bus)
    let mut display = Sh1107Display::new(i2c);

    // Initialize ICM-20948 via the display's I2C
    let imu_initialized = match Icm20948::init(display.i2c_mut()) {
        Ok(_) => {
            log::info!("ICM-20948 initialized successfully at 0x{:02X}", accelerometer::ICM20948_ADDR);
            true
        }
        Err(e) => {
            log::warn!("Failed to initialize ICM-20948 at 0x{:02X}: {:?}", accelerometer::ICM20948_ADDR, e);
            log::info!("Note: Check I2C scan results above for actual device addresses");
            log::info!("Continuing without IMU for display testing...");
            false
        }
    };

    // Initialize SH1107 display
    match display.init() {
        Ok(_) => log::info!("SH1107 display initialized successfully"),
        Err(e) => {
            log::error!("Failed to initialize display: {:?}", e);
            panic!("Display initialization failed");
        }
    }

    display.clear();
    let _ = display.flush();

    log::info!("Starting main loop...");

    let text_style = MonoTextStyle::new(&PROFONT_24_POINT, BinaryColor::On);
    let mut last_state = MotionState::Idle;
    let mut error_count = 0u32;

    loop {
        // Read accelerometer data
        let accel = if imu_initialized {
            match Icm20948::read_accel(display.i2c_mut()) {
                Ok(data) => data,
                Err(e) => {
                    error_count += 1;
                    if error_count <= 3 {
                        log::warn!("Failed to read accelerometer: {:?}", e);
                    }
                    AccelData::default()
                }
            }
        } else {
            AccelData::default()
        };

        // Determine motion state
        let state = determine_motion_state(&accel);

        // Only update display if state changed (reduces flicker)
        if state != last_state {
            log::info!("Motion: {} (X={}, Y={}, Z={})", state.as_str(), accel.x.0, accel.y.0, accel.z.0);

            display.clear();

            // Draw centered text
            Text::with_alignment(
                state.as_str(),
                Point::new(64, 70),  // Center of 128x128 display
                text_style,
                Alignment::Center,
            )
            .draw(&mut display)
            .unwrap();

            let _ = display.flush();
            last_state = state;
        }

        // Small delay to prevent overwhelming the I2C bus
        FreeRtos::delay_ms(50);
    }
}
