# SPDX-FileCopyrightText: 2022 Ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
"""MicroPython WiFi + BME280 sensor demo"""
import time
from machine import Pin, I2C
import network

from wifi_secrets import wifi_ssid, wifi_password
from bme280 import BME280
from sh1107 import SH1107_I2C

# ============================================
# Initialize I2C and devices
# ============================================
print("=" * 40)
print("Initializing I2C...")

# Adafruit QT Py ESP32 Pico STEMMA QT connector: SDA1=GPIO22, SCL1=GPIO19
i2c = I2C(0, scl=Pin(19), sda=Pin(22), freq=100000)

# Scan for I2C devices
devices: list[int] = i2c.scan()  # pyright: ignore[reportUnknownMemberType,reportUnknownVariableType]
print(f"I2C devices found: {[hex(d) for d in devices]}")

# Initialize BME280
print("Initializing BME280 sensor...")
bme = BME280(i2c, addr=0x77)

# Initialize OLED display
print("Initializing OLED display...")
display = SH1107_I2C(128, 128, i2c, addr=0x3D, rotate=90)

print("=" * 40)

# ============================================
# WiFi Connection
# ============================================
station: network.WLAN = network.WLAN(network.STA_IF)
station.active(True)

print("Connecting to WiFi...")
while not station.isconnected():
    print(f"Connecting to {wifi_ssid}...")
    station.connect(wifi_ssid, wifi_password)
    time.sleep(10)

print("Connected!")
print("My IP Address:", station.ifconfig()[0])

print("=" * 40)


# ============================================
# Main loop - update sensor and display
# ============================================
def update_display():
    """Read BME280 and update OLED display"""
    try:
        temp_c, pressure_hpa, humidity = bme.read()
        # Convert to USA units
        temp_f: float = temp_c * 9 / 5 + 32
        pressure_inhg: float = pressure_hpa * 0.02953

        print(f"Temperature: {temp_f:.1f} F | Pressure: {pressure_inhg:.2f} inHg | Humidity: {humidity:.1f} %")

        # Update display
        display.fill(0)
        display.text("Hello World!", 0, 0, 1)
        display.text("BME280 Sensor:", 0, 16, 1)
        display.text(f"{temp_f:.1f}F", 0, 32, 1)
        display.text(f"{pressure_inhg:.2f} inHg", 0, 48, 1)
        display.text(f"{humidity:.1f}% RH", 0, 64, 1)
        display.show()

    except Exception as e:
        print(f"Error: {e}")


print("Starting main loop (updating every 60 seconds)...")
while True:
    update_display()
    time.sleep(60)
