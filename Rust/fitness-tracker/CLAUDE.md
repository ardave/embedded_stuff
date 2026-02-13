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

This is an embedded Rust firmware for an **Adafruit ESP32-S2 Feather** with a **PA1010D GPS module** (I2C) and an **Adafruit Monochrome 1.12" 128x128 OLED Display (SH1107, Product ID 5297)** connected via STEMMA QT. It uses the `esp-idf-svc` HAL with FreeRTOS threading — not async/Embassy.

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
