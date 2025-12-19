#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}" && git rev-parse --show-toplevel)"
ENV_FILE="${REPO_ROOT}/.env"

# Load .env file
if [ -f "$ENV_FILE" ]; then
    export $(grep -v '^#' "$ENV_FILE" | xargs)
else
    echo "Error: .env file not found at $ENV_FILE"
    exit 1
fi

# Validate required variables
if [ -z "$WIFI_SSID" ] || [ -z "$WIFI_PASSWORD" ]; then
    echo "Error: WIFI_SSID and WIFI_PASSWORD must be set in .env"
    exit 1
fi

# Generate wifi_secrets.py
cat > "${SCRIPT_DIR}/wifi_secrets.py" << EOF
# Auto-generated from .env - do not edit directly
wifi_ssid: str = "${WIFI_SSID}"
wifi_password: str = "${WIFI_PASSWORD}"
EOF

echo "Generated wifi_secrets.py"

# Reset device to ensure clean state
echo "Resetting device..."
if mpremote reset; then
    sleep 2
else
    echo "Warning: Could not reset device (may not be connected)"
fi

# Upload to ESP32
mpremote cp "${SCRIPT_DIR}/wifi_secrets.py" :wifi_secrets.py
mpremote cp "${SCRIPT_DIR}/bme280.py" :bme280.py
mpremote cp "${SCRIPT_DIR}/sh1107.py" :sh1107.py
mpremote cp "${SCRIPT_DIR}/main.py" :main.py
echo "Deployed to ESP32"

# Run main.py and show output (Ctrl+C to stop)
echo "Running main.py..."
mpremote run "${SCRIPT_DIR}/main.py"
