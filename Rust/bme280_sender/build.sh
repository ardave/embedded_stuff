#!/bin/bash
# Build script for bme280_sender
# This script ensures a clean environment for building with ESP-IDF v5.3.3

# Unset any ESP-IDF v6.x environment variables that may interfere
unset IDF_PATH
unset ESP_IDF_VERSION
unset IDF_PYTHON_ENV_PATH
unset IDF_TOOLS_INSTALL_CMD
unset IDF_TOOLS_EXPORT_CMD
unset IDF_DEACTIVATE_FILE_PATH
unset ESP_ROM_ELF_DIR
unset OPENOCD_SCRIPTS

# Set up clean PATH with Python 3.12 (required for ESP-IDF 5.3.x)
export PATH="/opt/homebrew/opt/python@3.12/libexec/bin:/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin:$HOME/.cargo/bin:$HOME/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin"

# Set ESP-IDF version for embuild
export ESP_IDF_VERSION=v5.3.3

# Run cargo build with any additional arguments passed to this script
cargo build "$@"
