# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

This is an ESP-IDF project targeting the Adafruit QT Py ESP32-C3. Requires ESP-IDF environment to be sourced.

```bash
idf.py build              # Build the project
idf.py flash monitor      # Flash to device and open serial monitor
idf.py fullclean          # Clean build artifacts (use when CMake changes)
```

## Configuration

Build configuration is read from a `.env` file at the repository root (`../../../.env` relative to `main/`). Required variables:

```
WIFI_SSID=your_ssid
WIFI_PASSWORD=your_password
API_ENDPOINT=https://your-endpoint.com/api
DEVICE_ID=esp32-bme280    # optional, has default
```

These are injected as compile-time definitions via `main/CMakeLists.txt`.

## Architecture

The firmware reads BME280 sensor data, POSTs it to an HTTP endpoint as a protobuf, then enters deep sleep for 15 minutes.

**Execution flow** (`weather_server.c`):
1. Init NVS → LED → BME280 sensor → WiFi → data poster
2. POST sensor reading (with retry/backoff)
3. Show success/failure on LED
4. Enter deep sleep

**Modules:**
- `bme280_sensor.c` - I2C bus setup (pins 5/6) and BME280 initialization at address 0x77
- `wifi_manager.c` - Event-driven WiFi connection with retry logic
- `data_poster.c` - Reads sensor, converts units (C→F, Pa→inHg), sends protobuf
- `http_client.c` - HTTP POST with exponential backoff retry
- `led_status.c` - WS2812 NeoPixel control via RMT peripheral (data pin 2, power pin 8)

**Protobuf:**
- `sensor_reading.proto` defines the message format
- `sensor_reading.pb.c/.h` are pre-generated nanopb files (regenerate with nanopb if proto changes)

**Coding Guidelines**

## Hardware

- Board: Adafruit QT Py ESP32-C3
- Sensor: BME280 on STEMMA QT I2C connector (address 0x77)
- LED: Onboard NeoPixel (power on GPIO8, data on GPIO2)

