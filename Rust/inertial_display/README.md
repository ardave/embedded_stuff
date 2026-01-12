# Rust ESP32 Inertial Motion Display

An embedded Rust project that reads accelerometer data from an ICM-20948 9-axis IMU and displays the detected motion state on an SH1107 OLED display.

## Features

- **ICM-20948 IMU**: Reads 3-axis accelerometer data via I2C
- **Motion Detection**: Classifies device orientation/motion state
- **OLED Display**: Shows current motion state on 128x128 SH1107 display
- **Optimized Updates**: Only redraws display when state changes to reduce flicker
- **Embedded Graphics**: Uses `embedded-graphics` crate for text rendering

## Hardware

- Adafruit QT Py ESP32 Pico
- ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer)
- SH1107 128x128 OLED display
- STEMMA QT / Qwiic connectors for I2C

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| I2C SDA  | 22   | STEMMA QT |
| I2C SCL  | 19   | STEMMA QT |

## Motion States

The system detects and displays states such as:
- Idle
- Tilted orientations
- Movement detection

## Architecture

```
ICM-20948 Accelerometer
        │
        ▼
   Read X, Y, Z acceleration
        │
        ▼
   Determine motion state
        │
        ▼
   Update display (if changed)
```

## Building

```bash
cargo build --release
cargo espflash flash --monitor
```

## Key Dependencies

- `esp-idf-hal` - Hardware abstraction
- `embedded-graphics` - 2D graphics primitives
- `profont` - Font rendering
