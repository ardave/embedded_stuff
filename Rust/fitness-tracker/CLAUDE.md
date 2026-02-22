# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test Commands

**Firmware build** (ESP32-S2 target, from repo root):
```bash
cargo build            # dev profile (opt-level "z")
cargo build --release  # release profile (opt-level "s")
```

**Lint & format:**
```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features --workspace -- -D warnings
```

**Host-side unit tests** (runs on the dev machine, not on ESP32):
```bash
cd testable-logic && cargo test
```
There is no `cargo test` at the root — the binary crate sets `harness = false` and there are no integration tests for the ESP target.

**Flash to device** (ESP32-S2 native USB requires `--no-stub` and manual bootloader entry):
1. Put the board into bootloader mode: hold **BOOT**, press **RESET**, release **BOOT**
2. Flash:
```bash
espflash flash --no-stub --port /dev/cu.usbmodem01 target/xtensa-esp32s2-espidf/debug/fitness-tracker
```
3. Press **RESET** to start the firmware (the device does not auto-reset after `--no-stub` flashing)

> **Note:** `cargo run` (which uses `espflash flash --monitor`) and the default flash stub both fail over native USB on this board. Always use `--no-stub`.

**Monitor serial output** (USB CDC, after firmware is running):
```bash
cat /dev/cu.usbmodem01
```
> `espflash monitor` does not work reliably over native USB CDC on this board. Plain `cat` on the serial device works.

## Architecture

This is an embedded Rust firmware for an **Adafruit ESP32-S2 Feather** with a **PA1010D GPS module** (UART, GPIO 39 TX / GPIO 38 RX), an **Adafruit Monochrome 1.12" 128x128 OLED Display (SH1107, Product ID 5297)** connected via I2C (STEMMA QT), and an **Adafruit Micro SD Breakout (PID 4682)** connected via SPI (GPIO 36/35/37/10). It uses the `esp-idf-svc` HAL with FreeRTOS threading — not async/Embassy.

### Task model

Three threads communicate via FreeRTOS queues (see `src/queue.rs` for the type-safe `FreeRtosQueue<T>` wrapper):

```
GPS task (pri 10) ──GpsSentence──▶ Task1 (pri 5)   logs individual GGA/RMC sentences
       │
       └──────────GpsReading───▶ Task2 (pri 5)   logs combined position+speed readings
```

- **GPS task** (`src/tasks/gps.rs`): reads I2C every 100ms, parses NMEA via `nmea0183` crate, uses `GpsJoiner` to correlate GGA+RMC sentences into complete readings.
- **Task1/Task2** (`src/tasks/task1.rs`, `task2.rs`): consumer tasks that block-receive from their respective queues.
- Queues are `Box::leak`'d to `'static` in `main()` so they can be moved into thread closures.

### testable-logic subcrate

`testable-logic/` is a pure-Rust library crate (zero ESP dependencies) that contains:
- Data types: `GgaData`, `RmcData`, `GpsSentence`, `GpsReading`
- `GpsJoiner`: stateful correlator that joins GGA+RMC sentences by checking timestamp freshness (< 2s). Takes `now: Instant` as a parameter for deterministic testing.

The main crate re-exports these types from `src/tasks/gps.rs` via `pub use testable_logic::{...}`, so consumers (`task1.rs`, `task2.rs`, `main.rs`) import from `crate::tasks::gps`.

The subcrate has its own `.cargo/config.toml` that overrides the build target to the host architecture, allowing `cargo test` to work without the ESP toolchain's linker.

## Wiring & Pinout Reference

### Board: Adafruit ESP32-S2 Feather (PID 5000)

#### Device Connections

| Device                      | PID  | Interface | Bus / Pins                       | Address / Baud | Notes                                             |
| --------------------------- | ---- | --------- | -------------------------------- | -------------- | ------------------------------------------------- |
| SH1107 128×128 OLED Display | 5297 | I2C       | STEMMA QT (GPIO3 SDA, GPIO4 SCL) | 0x3D           | A0 jumper open (default)                          |
| PA1010D Mini GPS            | 4415 | UART      | GPIO39 TX → RXI, GPIO38 RX ← TXO | 9600 8N1       | Cross-connect: board TX↔GPS RXI, board RX↔GPS TXO |
| Micro SD Breakout           | 4682 | SPI       | GPIO36 SCK, GPIO35 MOSI, GPIO37 MISO, GPIO10 CS, GPIO6 DET | —  | 3 V only; uses default hardware SPI bus            |

#### I2C Bus (STEMMA QT)

| Signal | GPIO | Notes                                                              |
| ------ | ---- | ------------------------------------------------------------------ |
| SDA    | 3    | 5 kΩ pull-up on board                                              | **** |
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

| Signal | GPIO | Notes                                          |
| ------ | ---- | ---------------------------------------------- |
| SCK    | 36   | SPI clock                                      |
| MOSI   | 35   | Master Out / Slave In (labeled MO on Feather)  |
| MISO   | 37   | Master In / Slave Out (labeled MI on Feather)  |
| CS     | 10   | Active-low chip select for SD card breakout    |

GPIO 36/35/37 are the ESP32-S2's default high-speed SPI peripheral pins exposed on the Feather header. GPIO 10 (D10) is an unused general-purpose digital pin on the right header, chosen for CS because it has no conflicting alternate function.

#### Micro SD Breakout (PID 4682) Pins

| Breakout Pin | Function             | Feather Connection                          |
| ------------ | -------------------- | ------------------------------------------- |
| 3V           | Power in             | Feather 3V3 pin (3.3 V only — no 5 V!)     |
| GND          | Ground               | Feather GND pin                             |
| CLK          | SPI clock input      | GPIO 36 (SCK)                               |
| SI           | SPI data in (MOSI)   | GPIO 35 (MOSI/MO)                           |
| SO           | SPI data out (MISO)  | GPIO 37 (MISO/MI)                           |
| CS           | Chip select (active low) | GPIO 10 (D10)                            |
| DET          | Card detect          | GPIO 6 (D6); switch shorts to GND when card inserted; use internal pull-up |

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

## ESP-IDF / Hardware Details

- **Toolchain:** `esp` channel (Xtensa Rust fork), target `xtensa-esp32s2-espidf`
- **ESP-IDF version:** v5.3.3 (set in `.cargo/config.toml` env)
- **Linker:** `ldproxy` (required for ESP-IDF std builds)
- **`build.rs`** calls `embuild::espidf::sysenv::output()` — ESP-specific, won't compile on host
- **GPIO7** must be driven HIGH to enable I2C/STEMMA QT power on this board; devices need ≥250ms after power-on before I2C communication
- **Display:** SH1107 128×128 OLED (Adafruit PID 5297), I2C address **0x3D** (default, A0 open), driver in `src/tasks/sh1107.rs`
- **Console output** is USB CDC, not UART (`sdkconfig.defaults`)
- **Main task stack** is 8000 bytes (Rust needs more than the 3K default)

## CI

GitHub Actions (`.github/workflows/rust_ci.yml`) runs `build --release`, `fmt`, and `clippy` using the `esp-rs/xtensa-toolchain` action on every push to main and on all PRs.
