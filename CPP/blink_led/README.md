# C++ ESP32 GPIO with FreeRTOS

An ESP-IDF C++ project demonstrating GPIO interrupts with FreeRTOS queues and modern C++ patterns.

## Features

- **RAII Wrappers**: `GpioPin` and `GpioEventQueue` classes manage hardware resources safely
- **FreeRTOS Integration**: Queue-based ISR-to-task communication
- **Modern C++**: Uses C++17/20 features like `[[nodiscard]]`, deleted copy constructors
- **Clean Architecture**: Separation between ISR context and task context

## Hardware

- Adafruit QT Py ESP32-C3
- LED connected to GPIO 8 (MI pin)
- Momentary push button connected to GPIO 4 (A0 pin)

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| LED      | 8    | Output |
| Switch   | 4    | Input with pull-up, any-edge interrupt |

## Architecture

```
Button Event
     │
     ▼
GPIO ISR (IRAM_ATTR)
     │
     ▼
xQueueSendFromISR()
     │
     ▼
FreeRTOS Task (gpioTaskHandler)
     │
     ▼
Read switch, update LED
```

## Key Classes

### GpioPin
RAII wrapper for ESP-IDF GPIO:
- Automatic pin reset on construction
- Methods for input/output configuration
- Interrupt attachment

### GpioEventQueue
RAII wrapper for FreeRTOS queue:
- ISR-safe send method
- Blocking receive with timeout
- Automatic cleanup on destruction

## Building

```bash
# Source ESP-IDF environment
get_idf  # or: . $HOME/esp/esp-idf/export.sh

# Build and flash
idf.py build
idf.py flash monitor
```

## Why This Design?

Demonstrates that modern C++ can be used effectively in embedded contexts:
- RAII prevents resource leaks
- Type safety catches errors at compile time
- Zero-cost abstractions (no runtime overhead vs C)
