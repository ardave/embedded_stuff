# Rust ESP32 Weather Station

A battery-optimized weather station that reads environmental data from a BME280 sensor and transmits it via HTTP POST using Protocol Buffers, then enters deep sleep to conserve power.

## Features

- **BME280 Sensor**: Reads temperature, pressure, and humidity via I2C
- **WiFi Connectivity**: WPA2 connection with retry logic
- **Protocol Buffers**: Efficient binary serialization for sensor data
- **HTTPS with TLS**: Secure data transmission using ESP-IDF certificate bundle
- **Deep Sleep**: 15-minute sleep cycles for battery-powered operation
- **WS2812 NeoPixel**: Visual status indication (yellow=connecting, green=success, red=error)
- **Retry Logic**: Exponential backoff for HTTP requests

## Hardware

- Adafruit QT Py ESP32-C3
- BME280 temperature/humidity/pressure sensor
- Built-in WS2812 NeoPixel LED

## Pin Configuration

| Function      | GPIO | Notes |
|---------------|------|-------|
| I2C SDA       | 5    | BME280 |
| I2C SCL       | 6    | BME280 |
| NeoPixel Data | 2    | WS2812 |
| NeoPixel Power| 8    | Enable for LED |

## Configuration

Create a `.env` file with:

```
WIFI_SSID=YourSSID
WIFI_PASSWORD=YourPassword
API_ENDPOINT=https://your-api.example.com/sensor
DEVICE_ID=weather-station-1
```

## Protocol Buffer Schema

```protobuf
message SensorReading {
  float temperature_f = 1;
  float pressure_inhg = 2;
  float humidity_percent = 3;
  string device_id = 4;
}
```

## Building

```bash
cargo build --release
cargo espflash flash --monitor
```

## Power Optimization

The device wakes every 15 minutes to:
1. Read sensor data
2. Connect to WiFi
3. POST data to server
4. Return to deep sleep

This cycle typically takes 5-10 seconds, enabling months of battery life with appropriate power supply.
