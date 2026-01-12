# MicroPython Interrupt-Driven LED Control

A MicroPython project demonstrating hardware interrupt handling for responsive button-to-LED control.

## Features

- **Hardware Interrupts**: Uses GPIO interrupts on both rising and falling edges
- **ISR Best Practices**: Minimal ISR that defers work via `micropython.schedule()`
- **Debounce-Free Design**: Direct hardware response without software debouncing

## Hardware

- Adafruit QT Py ESP32-C3 (or similar ESP32 board)
- LED connected to GPIO 8
- Momentary push button connected to GPIO 4 (active low with internal pull-up)

## Pin Configuration

| Function | GPIO | Notes |
|----------|------|-------|
| LED      | 8    | Output |
| Switch   | 4    | Input with internal pull-up |

## How It Works

1. Button press triggers a GPIO interrupt (any edge)
2. The ISR uses `micropython.schedule()` to safely defer work to the main context
3. The scheduled callback reads the button state and updates the LED
4. LED turns on when button is pressed, off when released

## Key Concepts Demonstrated

- `micropython.alloc_emergency_exception_buf()` for ISR debugging
- `Pin.irq()` configuration for edge-triggered interrupts
- Safe ISR patterns using `micropython.schedule()` to avoid heap allocation in ISR context
