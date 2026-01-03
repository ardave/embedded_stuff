use esp_idf_hal::{
    delay::FreeRtos,
    i2c::I2cDriver,
};
use esp_idf_svc::sys::EspError;

// I2C address for ICM-20948 IMU (AD0 pin high on Adafruit board)
pub const ICM20948_ADDR: u8 = 0x69;

// ICM-20948 register addresses
const ICM20948_WHO_AM_I: u8 = 0x00;
const ICM20948_PWR_MGMT_1: u8 = 0x06;
const ICM20948_ACCEL_XOUT_H: u8 = 0x2D;

// Expected WHO_AM_I value for ICM-20948
const ICM20948_WHO_AM_I_VAL: u8 = 0xEA;

// Acceleration thresholds (in raw units, ~16384 = 1g for ±2g range)
// Using ~0.15g as threshold for detecting significant motion
pub const ACCEL_THRESHOLD: i16 = 2500;

/// Raw X-axis acceleration (forward/backward in board frame)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccelX(pub i16);

/// Raw Y-axis acceleration (left/right in board frame)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccelY(pub i16);

/// Raw Z-axis acceleration (up/down, perpendicular to board)
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccelZ(pub i16);

/// All three axes of acceleration data from the IMU
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AccelData {
    pub x: AccelX,
    pub y: AccelY,
    pub z: AccelZ,
}

/// Motion state detected from accelerometer data
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotionState {
    Idle,
    Go,
    Stop,
    Left,
    Right,
}

impl MotionState {
    pub fn as_str(&self) -> &'static str {
        match self {
            MotionState::Idle => "idle",
            MotionState::Go => "GO",
            MotionState::Stop => "STOP",
            MotionState::Left => "LEFT",
            MotionState::Right => "RIGHT",
        }
    }
}

/// ICM-20948 IMU driver
pub struct Icm20948;

impl Icm20948 {
    /// Initialize the ICM-20948 IMU
    pub fn init(i2c: &mut I2cDriver) -> Result<(), EspError> {
        // Check WHO_AM_I
        let mut who_am_i = [0u8; 1];
        i2c.write_read(ICM20948_ADDR, &[ICM20948_WHO_AM_I], &mut who_am_i, 100)?;
        log::info!("ICM-20948 WHO_AM_I: 0x{:02X} (expected 0x{:02X})", who_am_i[0], ICM20948_WHO_AM_I_VAL);

        // Reset the device
        i2c.write(ICM20948_ADDR, &[ICM20948_PWR_MGMT_1, 0x80], 100)?;
        FreeRtos::delay_ms(100);

        // Wake up the device (clear sleep bit, use best available clock)
        i2c.write(ICM20948_ADDR, &[ICM20948_PWR_MGMT_1, 0x01], 100)?;
        FreeRtos::delay_ms(50);

        Ok(())
    }

    /// Read accelerometer X, Y, and Z values
    pub fn read_accel(i2c: &mut I2cDriver) -> Result<AccelData, EspError> {
        let mut buf = [0u8; 6];
        i2c.write_read(ICM20948_ADDR, &[ICM20948_ACCEL_XOUT_H], &mut buf, 100)?;

        Ok(AccelData {
            x: AccelX(i16::from_be_bytes([buf[0], buf[1]])),
            y: AccelY(i16::from_be_bytes([buf[2], buf[3]])),
            z: AccelZ(i16::from_be_bytes([buf[4], buf[5]])),
        })
    }
}

/// Determine motion state from accelerometer readings
pub fn determine_motion_state(accel: &AccelData) -> MotionState {
    // X-axis: forward/backward (go/stop)
    // Y-axis: left/right
    // Board orientation: laying flat, X pointing forward direction of travel

    let abs_x = accel.x.0.abs();
    let abs_y = accel.y.0.abs();

    // Check if any significant motion is detected
    if abs_x < ACCEL_THRESHOLD && abs_y < ACCEL_THRESHOLD {
        return MotionState::Idle;
    }

    // Determine dominant axis
    if abs_x > abs_y {
        // Forward/backward motion dominant
        if accel.x.0 > ACCEL_THRESHOLD {
            // When accelerating forward, sensor experiences backward force
            MotionState::Stop  // Adjust based on actual board orientation
        } else if accel.x.0 < -ACCEL_THRESHOLD {
            MotionState::Go
        } else {
            MotionState::Idle
        }
    } else {
        // Left/right motion dominant
        if accel.y.0 > ACCEL_THRESHOLD {
            MotionState::Right  // Adjust based on actual board orientation
        } else if accel.y.0 < -ACCEL_THRESHOLD {
            MotionState::Left
        } else {
            MotionState::Idle
        }
    }
}
