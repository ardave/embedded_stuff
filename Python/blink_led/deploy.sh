#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Reset device to ensure clean state
echo "Resetting device..."
if mpremote reset; then
    sleep 2
else
    echo "Warning: Could not reset device (may not be connected)"
fi

# Upload to ESP32
mpremote cp "${SCRIPT_DIR}/main.py" :main.py
echo "Deployed to ESP32"

# Run main.py and show output (Ctrl+C to stop)
echo "Running main.py..."
mpremote run "${SCRIPT_DIR}/main.py"
