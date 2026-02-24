# Embedded Systems Portfolio

A collection of embedded systems projects demonstrating proficiency across multiple languages and platforms, created as part of a career transition from web/business application development into embedded systems.

## Skills Demonstrated

- **Languages**: C, C++, Rust, Python (MicroPython)
- **Platforms**: STM32, ESP32, ROS2
- **Concepts**: GPIO interrupts, I2C, SPI and UART communication, sensor integration, FreeRTOS, real-time systems, WiFi networking, power management (deep sleep), Protocol Buffers
- **Hardware**: BME280 (temperature/humidity/pressure), ICM-20948 (9-axis IMU), SH1107 OLED displays, WS2812 NeoPixel LEDs, GPS modules, USB cameras

## Project Structure

```
embedded_stuff/
├── Python/
│   ├── hello_world/     # MicroPython sensor station with OLED display
│   └── blink_led/       # Interrupt-driven button/LED control
├── Rust/
│   ├── blink_led/       # ESP32-C3 GPIO interrupts with task notifications
│   ├── bme280_sender/   # Weather station with WiFi and HTTP/Protobuf
│   ├── inertial_display/# IMU-based motion detection display
│   └── playground/      # Data structure implementations (ring buffer)
└── CPP/
    ├── blink_led/       # ESP-IDF with FreeRTOS queues and C++ RAII
    └── ros2_playground/ # ROS2 nodes for camera and sensor fusion
```

## Highlights

### Embedded Rust on ESP32
Multiple projects using `esp-idf-svc` and `esp-idf-hal` crates, demonstrating:
- Safe abstractions over hardware peripherals
- Interrupt handling with FreeRTOS task notifications
- WiFi connectivity and HTTPS with certificate bundles
- Protocol Buffers for efficient data serialization
- Deep sleep for battery-powered applications

# C on STM32
Interpreting signals from rotary encoder

### C on ESP32
ESP-IDF projects using C:
- Interactions with GPIO and FreeRTOS primitives
- Type-safe interrupt handling with queues
- Clean separation of concerns

### ROS2 Integration
Component-based ROS2 nodes demonstrating:
- Camera capture via OpenCV
- Sensor data fusion (GPS + imagery) (forthcoming)
- Publisher/subscriber patterns

### MicroPython
Quick prototyping projects showing:
- I2C sensor and display interfacing
- Hardware interrupt handling
- WiFi connectivity
