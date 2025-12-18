# SPDX-FileCopyrightText: 2022 Ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
"""MicroPython simple WiFi connection demo"""
import time
import network
import urequests

from wifi_secrets import wifi_ssid, wifi_password

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
