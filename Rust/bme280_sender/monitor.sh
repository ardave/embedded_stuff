#!/bin/bash
# Auto-reconnecting serial monitor for ESP32 deep sleep
# Uses ESP-IDF monitor which has built-in reconnection support
# Usage: ./monitor.sh [port] [baud]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_ENV="$SCRIPT_DIR/.embuild/espressif/python_env/idf5.3_py3.12_env/bin/python"

PORT="${1:-}"
BAUD="${2:-115200}"

# Check if Python environment exists
if [ ! -f "$PYTHON_ENV" ]; then
    echo "Error: ESP-IDF Python environment not found at $PYTHON_ENV"
    echo "Make sure you've built the project at least once with 'cargo build'"
    exit 1
fi

# Build port argument if specified
PORT_ARG=""
if [ -n "$PORT" ]; then
    PORT_ARG="--port $PORT"
fi

# Find the ELF file for symbol resolution (optional but helpful)
ELF_FILE=$(find "$SCRIPT_DIR/target" -name "bme280_sender" -type f 2>/dev/null | grep -v deps | head -1)
ELF_ARG=""
if [ -n "$ELF_FILE" ]; then
    ELF_ARG="$ELF_FILE"
fi

echo "Starting ESP-IDF monitor with auto-reconnect..."
echo "Press Ctrl+] to exit"
echo ""

exec "$PYTHON_ENV" -m esp_idf_monitor \
    $PORT_ARG \
    --baud "$BAUD" \
    --target esp32c3 \
    --no-reset \
    $ELF_ARG
