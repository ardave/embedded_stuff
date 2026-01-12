# MicroPython Sensor Station

A MicroPython project for the Adafruit QT Py ESP32 Pico that reads environmental data from a BME280 sensor and displays it on an SH1107 OLED display.

## Features

- **BME280 Sensor Integration**: Reads temperature, pressure, and humidity via I2C
- **OLED Display**: Shows real-time sensor data on a 128x128 SH1107 display
- **WiFi Connectivity**: Connects to WiFi network on startup
- **Unit Conversion**: Displays temperature in Fahrenheit and pressure in inHg

## Hardware

- Adafruit QT Py ESP32 Pico
- BME280 temperature/humidity/pressure sensor (I2C address 0x77)
- SH1107 128x128 OLED display (I2C address 0x3D)
- STEMMA QT / Qwiic connectors for I2C

## Pin Configuration

| Function | GPIO |
|----------|------|
| I2C SDA  | 22   |
| I2C SCL  | 19   |

## Dependencies

- `bme280.py` - BME280 driver
- `sh1107.py` - SH1107 OLED driver
- `wifi_secrets.py` - WiFi credentials (not committed)

## Usage

1. Create `wifi_secrets.py` with your credentials:
   ```python
   wifi_ssid = "YourSSID"
   wifi_password = "YourPassword"
   ```

2. Upload all files to the device using Thonny or mpremote

3. The device will connect to WiFi and begin displaying sensor readings every 60 seconds
