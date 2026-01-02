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
use esp_idf_svc::sys::EspError;

// I2C addresses
const ICM20948_ADDR: u8 = 0x69;  // IMU (AD0 pin high on Adafruit board)
const SH1107_ADDR: u8 = 0x3D;   // Display

// ICM-20948 register addresses
const ICM20948_WHO_AM_I: u8 = 0x00;
const ICM20948_PWR_MGMT_1: u8 = 0x06;
const ICM20948_ACCEL_XOUT_H: u8 = 0x2D;

// Expected WHO_AM_I value for ICM-20948
const ICM20948_WHO_AM_I_VAL: u8 = 0xEA;

// Acceleration thresholds (in raw units, ~16384 = 1g for ±2g range)
// Using ~0.15g as threshold for detecting significant motion
const ACCEL_THRESHOLD: i16 = 2500;

// SH1107 display dimensions
const DISPLAY_WIDTH: u32 = 128;
const DISPLAY_HEIGHT: u32 = 128;

// Motion state enum
#[derive(Debug, Clone, Copy, PartialEq)]
enum MotionState {
    Idle,
    Go,
    Stop,
    Left,
    Right,
}

impl MotionState {
    fn as_str(&self) -> &'static str {
        match self {
            MotionState::Idle => "idle",
            MotionState::Go => "GO",
            MotionState::Stop => "STOP",
            MotionState::Left => "LEFT",
            MotionState::Right => "RIGHT",
        }
    }
}

// Simple SH1107 display driver that owns the I2C driver
struct Sh1107Display<'a> {
    i2c: I2cDriver<'a>,
    buffer: [u8; (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) as usize],
}

impl<'a> Sh1107Display<'a> {
    fn new(i2c: I2cDriver<'a>) -> Self {
        Self {
            i2c,
            buffer: [0; (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8) as usize],
        }
    }

    fn send_command(&mut self, cmd: u8) -> Result<(), EspError> {
        self.i2c.write(SH1107_ADDR, &[0x00, cmd], 100)
    }

    fn send_commands(&mut self, cmds: &[u8]) -> Result<(), EspError> {
        for &cmd in cmds {
            self.send_command(cmd)?;
        }
        Ok(())
    }

    fn init(&mut self) -> Result<(), EspError> {
        // SH1107 initialization sequence for 128x128
        self.send_commands(&[
            0xAE,       // Display off
            0xDC, 0x00, // Set display start line to 0
            0x81, 0x4F, // Set contrast (0x4F is mid-range)
            0x20,       // Set memory addressing mode (page addressing)
            0xA0,       // Set segment re-map (normal)
            0xC0,       // Set COM output scan direction (normal)
            0xA8, 0x7F, // Set multiplex ratio (128)
            0xD3, 0x60, // Set display offset (96 for 128x128)
            0xD5, 0x51, // Set display clock divide ratio
            0xD9, 0x22, // Set pre-charge period
            0xDB, 0x35, // Set VCOMH deselect level
            0xB0,       // Set page address to 0
            0xA4,       // Entire display ON (resume from RAM)
            0xA6,       // Set normal display (not inverted)
            0xAF,       // Display on
        ])?;

        FreeRtos::delay_ms(100);
        Ok(())
    }

    fn clear(&mut self) {
        self.buffer.fill(0);
    }

    fn flush(&mut self) -> Result<(), EspError> {
        // SH1107 uses page addressing mode
        // Each page is 8 pixels high, so 128/8 = 16 pages
        for page in 0..16u8 {
            // Set page address
            self.send_command(0xB0 | page)?;
            // Set column address to 0 (lower nibble)
            self.send_command(0x00)?;
            // Set column address to 0 (upper nibble)
            self.send_command(0x10)?;

            // Send page data (128 bytes for 128 columns)
            let start = (page as usize) * 128;
            let end = start + 128;

            // Send data with 0x40 prefix indicating data mode
            let mut data_cmd = [0u8; 129];
            data_cmd[0] = 0x40; // Data mode
            data_cmd[1..].copy_from_slice(&self.buffer[start..end]);
            self.i2c.write(SH1107_ADDR, &data_cmd, 100)?;
        }
        Ok(())
    }

    fn set_pixel(&mut self, x: u32, y: u32, on: bool) {
        if x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT {
            return;
        }

        // SH1107 memory layout: each byte is 8 vertical pixels
        // Page number = y / 8
        // Bit position = y % 8
        let page = y / 8;
        let bit = y % 8;
        let index = (page * DISPLAY_WIDTH + x) as usize;

        if on {
            self.buffer[index] |= 1 << bit;
        } else {
            self.buffer[index] &= !(1 << bit);
        }
    }

    // Read from the IMU via the shared I2C bus
    fn read_accel(&mut self) -> Result<(i16, i16), EspError> {
        let mut buf = [0u8; 6];
        self.i2c.write_read(ICM20948_ADDR, &[ICM20948_ACCEL_XOUT_H], &mut buf, 100)?;

        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);

        Ok((x, y))
    }

    // Initialize the IMU via the shared I2C bus
    fn init_imu(&mut self) -> Result<(), EspError> {
        // Check WHO_AM_I
        let mut who_am_i = [0u8; 1];
        self.i2c.write_read(ICM20948_ADDR, &[ICM20948_WHO_AM_I], &mut who_am_i, 100)?;
        log::info!("ICM-20948 WHO_AM_I: 0x{:02X} (expected 0x{:02X})", who_am_i[0], ICM20948_WHO_AM_I_VAL);

        // Reset the device
        self.i2c.write(ICM20948_ADDR, &[ICM20948_PWR_MGMT_1, 0x80], 100)?;
        FreeRtos::delay_ms(100);

        // Wake up the device (clear sleep bit, use best available clock)
        self.i2c.write(ICM20948_ADDR, &[ICM20948_PWR_MGMT_1, 0x01], 100)?;
        FreeRtos::delay_ms(50);

        Ok(())
    }
}

// Implement DrawTarget for embedded-graphics integration
impl<'a> DrawTarget for Sh1107Display<'a> {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels {
            if coord.x >= 0 && coord.y >= 0 {
                self.set_pixel(coord.x as u32, coord.y as u32, color.is_on());
            }
        }
        Ok(())
    }
}

impl<'a> OriginDimensions for Sh1107Display<'a> {
    fn size(&self) -> Size {
        Size::new(DISPLAY_WIDTH, DISPLAY_HEIGHT)
    }
}

fn determine_motion_state(accel_x: i16, accel_y: i16) -> MotionState {
    // X-axis: forward/backward (go/stop)
    // Y-axis: left/right
    // Board orientation: laying flat, X pointing forward direction of travel

    let abs_x = accel_x.abs();
    let abs_y = accel_y.abs();

    // Check if any significant motion is detected
    if abs_x < ACCEL_THRESHOLD && abs_y < ACCEL_THRESHOLD {
        return MotionState::Idle;
    }

    // Determine dominant axis
    if abs_x > abs_y {
        // Forward/backward motion dominant
        if accel_x > ACCEL_THRESHOLD {
            // When accelerating forward, sensor experiences backward force
            MotionState::Stop  // Adjust based on actual board orientation
        } else if accel_x < -ACCEL_THRESHOLD {
            MotionState::Go
        } else {
            MotionState::Idle
        }
    } else {
        // Left/right motion dominant
        if accel_y > ACCEL_THRESHOLD {
            MotionState::Right  // Adjust based on actual board orientation
        } else if accel_y < -ACCEL_THRESHOLD {
            MotionState::Left
        } else {
            MotionState::Idle
        }
    }
}

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
    let mut i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &i2c_config).unwrap();

    log::info!("I2C initialized");

    // Scan I2C bus to find connected devices
    log::info!("Scanning I2C bus...");
    for addr in 1..127 {
        let mut buf = [0u8; 1];
        if i2c.read(addr, &mut buf, 10).is_ok() {
            log::info!("Found device at address 0x{:02X}", addr);
        }
    }
    log::info!("I2C scan complete");

    // Create display driver (which owns the I2C bus)
    let mut display = Sh1107Display::new(i2c);

    // Initialize ICM-20948 via the display's I2C
    // Try primary address first, then alternate
    let imu_initialized = match display.init_imu() {
        Ok(_) => {
            log::info!("ICM-20948 initialized successfully at 0x{:02X}", ICM20948_ADDR);
            true
        }
        Err(e) => {
            log::warn!("Failed to initialize ICM-20948 at 0x{:02X}: {:?}", ICM20948_ADDR, e);
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
        let (accel_x, accel_y) = if imu_initialized {
            match display.read_accel() {
                Ok((x, y)) => (x, y),
                Err(e) => {
                    error_count += 1;
                    if error_count <= 3 {
                        log::warn!("Failed to read accelerometer: {:?}", e);
                    }
                    (0, 0)
                }
            }
        } else {
            (0, 0)  // No IMU, stay in idle
        };

        // Determine motion state
        let state = determine_motion_state(accel_x, accel_y);

        // Only update display if state changed (reduces flicker)
        if state != last_state {
            log::info!("Motion: {} (X={}, Y={})", state.as_str(), accel_x, accel_y);

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
