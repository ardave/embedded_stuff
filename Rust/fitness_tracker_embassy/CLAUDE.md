# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded Rust fitness tracker targeting **ESP32-S2** using the **Embassy** async runtime. Runs on bare-metal (`no_std`) with the Xtensa ESP Rust toolchain fork.

## Build & Flash Commands

```bash
# Build (debug, size-optimized)
cargo build

# Build release
cargo build --release

# Flash to device and open serial monitor (uses espflash via .cargo/config.toml runner)
cargo run --release

# Format check
cargo fmt --all --check

# Lint (CI runs with -D warnings)
cargo clippy --all-features -- -D warnings
```

## Toolchain

Requires the ESP Xtensa Rust toolchain (`esp` channel), configured in `rust-toolchain.toml`. The target is `xtensa-esp32s2-none-elf` set in `.cargo/config.toml`.

Install via: https://github.com/esp-rs/rust-build

## Architecture

**Async task model:** The application uses Embassy's async executor bridged through `esp-rtos`. The `#[esp_rtos::main]` macro on the main function sets up the RTOS executor and provides a `Spawner` for launching async tasks.

**Key layers:**
- `esp-hal` — Hardware abstraction (GPIO, SPI, UART, I2C, timers)
- `esp-rtos` — Bridges ESP-IDF FreeRTOS with Embassy async executor
- `embassy-executor` / `embassy-time` — Async task scheduling and timers
- `esp-radio` + `smoltcp` + `embassy-net` — Pure-Rust WiFi and TCP/IP networking (DHCP, TCP, UDP)
- `esp-alloc` — Heap allocator (139KB configured in main.rs)

**Entry point:** `src/bin/main.rs` — initializes HAL at max CPU clock, sets up heap, timer group 0, and WiFi radio controller, then enters the async executor loop.

**No linker scripts in-repo** — they are inherited from `esp-hal`. The build script (`build.rs`) adds `linkall.x` and provides diagnostic error messages for common linker failures.

## Wiring & Pinout Reference

### Board: Adafruit ESP32-S2 Feather (PID 5000)

#### Device Connections

| Device                      | PID  | Interface | Bus / Pins                                                 | Address / Baud | Notes                                             |
| --------------------------- | ---- | --------- | ---------------------------------------------------------- | -------------- | ------------------------------------------------- |
| SH1107 128×128 OLED Display | 5297 | I2C       | STEMMA QT (GPIO3 SDA, GPIO4 SCL)                           | 0x3D           | A0 jumper open (default)                          |
| PA1010D Mini GPS            | 4415 | UART      | GPIO39 TX → RXI, GPIO38 RX ← TXO                           | 9600 8N1       | Cross-connect: board TX↔GPS RXI, board RX↔GPS TXO |
| Micro SD Breakout           | 4682 | SPI       | GPIO36 SCK, GPIO35 MOSI, GPIO37 MISO, GPIO10 CS, GPIO6 DET | —              | 3 V only; uses default hardware SPI bus           |

#### I2C Bus (STEMMA QT)

| Signal | GPIO | Notes                                                              |
| ------ | ---- | ------------------------------------------------------------------ |
| SDA    | 3    | 5 kΩ pull-up on board                                              |
| SCL    | 4    | 5 kΩ pull-up on board                                              |
| Power  | 7    | Must drive HIGH to enable (Rev C); wait ≥250 ms before I2C traffic |
| VCC    | —    | 3.3 V via STEMMA QT connector                                      |
| GND    | —    | Via STEMMA QT connector                                            |

#### UART (Feather header)

| Signal | GPIO | Notes                    |
| ------ | ---- | ------------------------ |
| TX     | 39   | Board transmit → GPS RXI |
| RX     | 38   | Board receive ← GPS TXO  |

The ESP32-S2 GPIO matrix allows any GPIO to serve as UART TX/RX; GPIO 39/38 are the designated serial header pins on this Feather and are not used by USB CDC.

#### SPI Bus (Feather header — default hardware SPI)

| Signal | GPIO | Notes                                         |
| ------ | ---- | --------------------------------------------- |
| SCK    | 36   | SPI clock                                     |
| MOSI   | 35   | Master Out / Slave In (labeled MO on Feather) |
| MISO   | 37   | Master In / Slave Out (labeled MI on Feather) |
| CS     | 10   | Active-low chip select for SD card breakout   |

GPIO 36/35/37 are the ESP32-S2's default high-speed SPI peripheral pins exposed on the Feather header. GPIO 10 (D10) is an unused general-purpose digital pin on the right header, chosen for CS because it has no conflicting alternate function.

#### Micro SD Breakout (PID 4682) Pins

| Breakout Pin | Function                 | Feather Connection                                                         |
| ------------ | ------------------------ | -------------------------------------------------------------------------- |
| 3V           | Power in                 | Feather 3V3 pin (3.3 V only — no 5 V!)                                     |
| GND          | Ground                   | Feather GND pin                                                            |
| CLK          | SPI clock input          | GPIO 36 (SCK)                                                              |
| SI           | SPI data in (MOSI)       | GPIO 35 (MOSI/MO)                                                          |
| SO           | SPI data out (MISO)      | GPIO 37 (MISO/MI)                                                          |
| CS           | Chip select (active low) | GPIO 10 (D10)                                                              |
| DET          | Card detect              | GPIO 6 (D6); switch shorts to GND when card inserted; use internal pull-up |

> **3 V ONLY** — this breakout has no level shifter or regulator. Power and all logic pins must be 3.3 V; connecting 5 V will damage the SD card.

#### PA1010D GPS Module (PID 4415) Breakout Pins

| Pin  | Function            | Notes                                                   |
| ---- | ------------------- | ------------------------------------------------------- |
| VIN  | Power in            | 3–5 V (connect to Feather 3.3 V or USB 5 V)             |
| GND  | Ground              | Common ground with Feather                              |
| TXO  | UART transmit (out) | 3.3 V logic; connect to Feather RX (GPIO 38)            |
| RXI  | UART receive (in)   | Level-shifted 3–5 V; connect to Feather TX (GPIO 39)    |
| 3Vo  | 3.3 V regulator out | Output from on-board LDO                                |
| PPS  | Pulse-per-second    | 3.3 V; pulses high ~50–100 ms at 1 Hz when fix acquired |
| RST  | Reset               | Pull low to reset GPS chip                              |
| WAKE | Wake / standby      | For low-power mode                                      |
| SCL  | I2C clock           | 10 kΩ pull-up; not used in UART config                  |
| SDA  | I2C data            | 10 kΩ pull-up; not used in UART config                  |

## Hardware Notes

- **GPIO7** must be driven HIGH to enable I2C/STEMMA QT power on this board; devices need ≥250ms after power-on before I2C communication
- **Display:** SH1107 128×128 OLED (Adafruit PID 5297), I2C address **0x3D** (default, A0 open)
- **Console output** is USB CDC, not UART

## Code Conventions

- Fully `no_std` — no Rust standard library
- All operations must be async (no blocking)
- Critical sections via `critical-section` crate for shared state
- `static_cell` for global mutable statics
- Clippy enforced with stack frame threshold of 1024 bytes (`.clippy.toml`)
- Both debug and release profiles use `opt-level = "s"` (size optimization)
