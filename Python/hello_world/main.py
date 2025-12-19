# SPDX-FileCopyrightText: 2022 Ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
"""MicroPython WiFi + BME280 sensor demo"""
import time
import machine
from machine import Pin, I2C
import network
import urequests

from wifi_secrets import wifi_ssid, wifi_password
from bme280 import BME280
from sh1107 import SH1107_I2C

# ============================================
# BME280 Sensor Reading (before WiFi)
# ============================================
print("=" * 40)
print("Initializing BME280 sensor via I2C...")

# Adafruit QT Py ESP32 Pico STEMMA QT connector: SDA1=GPIO22, SCL1=GPIO19
i2c = I2C(0, scl=Pin(19), sda=Pin(22), freq=100000)

# Scan for I2C devices
devices = i2c.scan()
print(f"I2C devices found: {[hex(d) for d in devices]}")

temp_f = None
pressure_inhg = None
humidity = None

try:
    bme = BME280(i2c, addr=0x77)
    temp_c, pressure_hpa, humidity = bme.read()
    # Convert to USA units
    temp_f = temp_c * 9 / 5 + 32
    pressure_inhg = pressure_hpa * 0.02953
    print(f"Temperature: {temp_f:.1f} F")
    print(f"Pressure: {pressure_inhg:.2f} inHg")
    print(f"Humidity: {humidity:.1f} %")
except Exception as e:
    print(f"BME280 error: {e}")

print("=" * 40)

# ============================================
# OLED Display
# ============================================
print("Initializing OLED display...")
try:
    display = SH1107_I2C(128, 128, i2c, addr=0x3D, rotate=90)
    display.fill(0)
    display.text("Hello World!", 0, 0, 1)
    display.text("BME280 Sensor:", 0, 16, 1)
    if temp_f is not None:
        display.text(f"{temp_f:.1f}F", 0, 32, 1)
        display.text(f"{pressure_inhg:.2f} inHg", 0, 48, 1)
        display.text(f"{humidity:.1f}% RH", 0, 64, 1)
    display.show()
    print("OLED display updated!")
except Exception as e:
    print(f"OLED error: {e}")

print("=" * 40)

# ============================================
# WiFi Connection
# ============================================
station: network.WLAN = network.WLAN(network.STA_IF)
station.active(True)

url: str = "http://wifitest.adafruit.com/testwifi/index.html"

print("Scanning for WiFi networks, please wait...")
authmodes: list[str] = [
    'Open',            # 0 - WIFI_AUTH_OPEN
    'WEP',             # 1 - WIFI_AUTH_WEP
    'WPA-PSK',         # 2 - WIFI_AUTH_WPA_PSK
    'WPA2-PSK',        # 3 - WIFI_AUTH_WPA2_PSK
    'WPA/WPA2-PSK',    # 4 - WIFI_AUTH_WPA_WPA2_PSK
    'WPA2-Enterprise', # 5 - WIFI_AUTH_WPA2_ENTERPRISE
    'WPA3-PSK',        # 6 - WIFI_AUTH_WPA3_PSK
    'WPA2/WPA3-PSK',   # 7 - WIFI_AUTH_WPA2_WPA3_PSK
    'WAPI-PSK',        # 8 - WIFI_AUTH_WAPI_PSK
    'OWE',             # 9 - WIFI_AUTH_OWE
    'WPA3-Enterprise', # 10 - WIFI_AUTH_WPA3_ENT_192
]
for (ssid, bssid, channel, RSSI, authmode, hidden) in station.scan():
    ssid: bytes
    bssid: bytes
    channel: int
    RSSI: int
    authmode: int
    hidden: bool
    print("* {:s}".format(ssid))
    print("   - Channel: {}".format(channel))
    print("   - RSSI: {}".format(RSSI))
    print("   - BSSID: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}".format(*bssid))
    auth_name: str = authmodes[authmode] if authmode < len(authmodes) else "Unknown({})".format(authmode)
    print("   - Auth: {}".format(auth_name))
    print()

# Continually try to connect to WiFi access point
while not station.isconnected():
    # Try to connect to WiFi access point
    print("Connecting...")
    station.connect(wifi_ssid, wifi_password)
    time.sleep(10)

# Display connection details
print("Connected!")
print("My IP Address:", station.ifconfig()[0])

# Perform HTTP GET request on a non-SSL web
response: urequests.Response = urequests.get(url)

# Display the contents of the page
print(response.text)

# Deep sleep commented out for development
# print("Entering deep sleep for 60 seconds...")
# machine.deepsleep(60000)
print("Done. Waiting 10 seconds before next run...")
time.sleep(10)
