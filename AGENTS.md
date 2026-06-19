# AGENTS.md — Reflex Firmware

## Project Overview

Reflex Firmware is the STM32F411 firmware for a rotary controller board. It provides Digital Read Out (DRO) and single-axis control for CNC-style rotary tables, communicating with the host UI over RS-485/Modbus RTU.

## Sibling Repos

This project is tightly coupled with **reflex-ui**, the Python/Kivy host application.

- **UI repo:** reflex-ui (sibling repository, typically found alongside this repo)
- **Interface:** RS-485 Modbus RTU — the entire `rampsSharedData_t` struct is memory-mapped to Modbus holding registers
- **Version compatibility:** For released versions, matching major.minor implies UI↔FW compatibility. For dev branches, assume the latest commit on each repo's respective branch is compatible. Cross-repo changes affecting the Modbus register interface are called out in commit messages.

## Agent Provisioning

This project has a sibling repository (reflex-ui) that agents may need to reference.
If your runtime supports workspace or permission configuration, grant read access to the sibling repo path.
For opencode, this means configuring `external_directory` permission in your project config to allow access to the reflex-ui repository.

## Building

### Firmware (STM32F411, arm-none-eabi toolchain)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
Outputs: `build/reflex.elf`, `.hex`, `.bin`.

### Emulator (native, x86_64)
```bash
cmake -S emulator -B emulator/build
cmake --build emulator/build
```
Outputs: `emulator/build/lathe-emulator`.

### Always build both after code changes
After any changes to `Core/Src/Ramps.c`, `Core/Inc/Ramps.h`, or other firmware sources, build **both** targets and fix any issues before proceeding. Ensure **all build artifacts stay in `build/` directories** — nothing should leak into the repo root or `emulator/` directory.

## Architecture

### Key files
- `Core/Src/Ramps.c` — core motion control (ISR, indexing, jog, ELS)
- `Core/Inc/Ramps.h` — shared data structs, Modbus-mapped
- `Core/Src/Modbus.c` — Modbus RTU slave (addr 17)
- `Core/Src/Scales.c` — encoder timer init

### Concurrency
- TIM9 ISR (`SynchroRefreshTimerIsr`) handles all motion control at ~100 µs ticks
- FreeRTOS tasks handle Modbus, speed updates, motor enable
- `rampsSharedData_t` is the shared state, memory-mapped to Modbus registers

### Servo modes
- 0: disabled
- 1: indexing (trapezoidal ramp, sync-driven)
- 2: jogging (continuous speed)

### ELS (Electronic Limit Switch) threading
- `elsStop.enable = 1` starts a threading job
- Sync gates on/off via `elsStop.active` and `elsStop.takeupPending`
- Resume sequence (active 1→0): reset state → backlash takeup → phase correction → sync resume
- Phase correction folds to ±pitch/2, then constrained to cutting direction (Fix 3, 2026-06-16)
- Sync un-gates before correction move completes (Fix 4, 2026-06-16)
- `stepsToGo`/`currentSpeed` reset before takeup (Fix 1, 2026-06-16)

### Modbus register layout
The entire `rampsSharedData_t` struct is directly memory-mapped to Modbus holding registers. Adding fields shifts offsets and breaks host compatibility.

### ISR tick order (critical for timing)
1. Reset STEP pin
2. Update execution interval
3. Handle ELS enable/active state transitions
4. Check takeup completion → phase correction
5. Read encoders, compute sync deltas, check ELS trigger
6. Run indexing/jog ramp
7. Generate step pulses
8. Update servoCyclesCounter
