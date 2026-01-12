# Rust ESP32-C3 GPIO Interrupt Demo

An embedded Rust project for ESP32-C3 demonstrating GPIO interrupts with FreeRTOS task notifications.

## Features

- **Safe Interrupt Handling**: Uses `esp-idf-svc` abstractions for type-safe GPIO configuration
- **Task Notifications**: FreeRTOS task notification pattern for ISR-to-task communication
- **Edge Detection**: Responds to both rising and falling edges of button input

## Hardware

- Adafruit QT Py ESP32-C3
- LED connected to GPIO 8 (MI pin)
- Momentary push button connected to GPIO 4 (A0 pin)

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| LED      | 8    | Output |
| Switch   | 4    | Input with internal pull-up |

## Architecture

```
Button Press/Release
        │
        ▼
   GPIO Interrupt (ISR)
        │
        ▼
   Task Notification (ISR-safe)
        │
        ▼
   Main Loop (blocks on notification)
        │
        ▼
   Read switch state, update LED
```

## Building

```bash
# Ensure ESP-IDF and Rust ESP toolchain are installed
cargo build --release
cargo espflash flash --monitor
```

## Key Dependencies

- `esp-idf-svc` - ESP-IDF Rust bindings
- `esp-idf-hal` - Hardware abstraction layer
