# Firmware Architecture

## Overview

STM32F411 (Cortex-M4 @ 100 MHz) firmware for a CNC rotary table controller. Reads up to 4 encoder inputs and drives a stepper motor with programmable sync ratios, controlled remotely over Modbus RTU (e.g. from a Raspberry Pi).

---

## Layer Stack

```
Modbus Master (Raspberry Pi)
        ↓ USART1 @ 115200 baud
FreeRTOS Tasks (application)
        ↓
SynchroRefreshTimerIsr (TIM9 ISR — bare-metal, high priority)
        ↓
STM32 HAL + CMSIS
        ↓
STM32F411 hardware
```

---

## Key Modules

### `Core/Src/Ramps.c` — Core Motion Control

The heart of the firmware.

- **`SynchroRefreshTimerIsr()`** — high-priority TIM9 ISR running every ~100 µs. Reads all 4 encoder counters, computes deltas with fractional error tracking, applies sync ratios, and generates step/direction pulses on PA0/PB14. Uses the Cortex-M4 DWT cycle counter to measure its own execution time.
- **`updateIndexingPosition()`** — trapezoidal ramp (accel/cruise/decel) for indexed moves.
- **`updateJogPosition()`** — continuous speed control for jogging.
- Three FreeRTOS tasks: `userLedTask`, `updateSpeedTask`, `servoEnableTask`.

### `Core/Src/Modbus.c` — Communications

Full Modbus RTU slave implementation (address 17). The entire `rampsSharedData_t` struct is memory-mapped directly to Modbus holding registers — no translation layer. A master can read/write servo state (mode, target steps, speed, sync ratios) directly via FC3/FC6/FC16.

### `Core/Src/Scales.c` — Encoder Initialisation

Configures TIM1–TIM4 in encoder mode (TI12, both channels). TIM2 is 32-bit for higher resolution; the rest are 16-bit.

### `Core/Src/tim.c`, `usart.c`, `gpio.c` — HAL Peripherals

STM32CubeMX-generated peripheral initialisation code.

---

## Hardware Peripherals

| Peripheral | Role |
|---|---|
| TIM1–TIM4 | Encoder inputs (4 scales) |
| TIM9 | Motion control ISR timebase |
| TIM11 | FreeRTOS systick |
| USART1 | Modbus RTU (PA10 RX, PA15 TX) |
| PA0 | STEP pulse output |
| PB14 | DIR output |
| PB15 | ENA output (active low) |
| PA3, PA4 | Spare debug/scope outputs |
| PB12 | User LED |

**Clock:** HSE → PLL → 100 MHz SYSCLK, hardware FPU enabled.

---

## Concurrency Model

Hybrid bare-metal + FreeRTOS:

- The **TIM9 ISR** handles all timing-critical motion control outside the RTOS scheduler.
- **FreeRTOS tasks** handle everything millisecond-tolerant: Modbus parsing, speed updates, motor enable/disable, and LED status.
- `rampsSharedData_t` is the shared state between the ISR and tasks. The ISR reads servo commands from it; Modbus tasks write to it.

### FreeRTOS Tasks

| Task | Priority | Period | Purpose |
|---|---|---|---|
| `TaskModbusSlave` | Normal | Event-driven | Modbus protocol handler |
| `updateSpeedTask` | Low | 50 ms | Servo speed calculations |
| `servoEnableTask` | Low | 100 ms | Motor enable/disable |
| `userLedTask` | Low | 50 ms | Status LED + Modbus activity indicator |
| `defaultTask` | Normal | 1000 ms | Baseline task |

---

## Servo Modes

Controlled via Modbus register `servoMode`:

| Mode | Behaviour |
|---|---|
| 0 | Disabled — motor idle |
| 1 | Indexing — move `stepsToGo` steps with trapezoidal accel/decel ramp |
| 2 | Jogging — continuous motion at `jogSpeed` |

### Encoder Synchronisation

Encoder input is scaled by a programmable ratio before being added to `desiredSteps`:

```
desiredSteps += encoderDelta × (syncRatioNum / syncRatioDen)
```

Fractional remainders are tracked per-axis to prevent accumulated positioning error.

### ELS Shoulder Stop

Position-based automatic stop for turning or threading to a shoulder, with phase-preserving re-sync on resume.

**On trigger** (selected scale crosses `stopPosition`): firmware latches `active = 1`, snapshots `desiredSteps → elsStopStepsAtStop`, captures `scales[0].position → latchedSpindleEncoder`, resets `accumulatedError = 0`.

**While latched:** integer `scaledDelta` accumulates into `accumulatedError` instead of `desiredSteps`; the fractional residue carried in `scalesSyncDeltaPos[i].error` continues to track sub-step precision. The user is free to jog the carriage (e.g. retract) via `stepsToGo`; `desiredSteps` moves accordingly. `servoEnableTask` suppresses its auto-mode-1 logic to allow this.

**On resume** (SW writes `active = 0`), the firmware computes a re-sync correction. The key insight: `accumulatedError` is the carriage motion sync *would have* commanded during the stop, and `(desiredSteps − elsStopStepsAtStop)` is the carriage motion that *actually* happened (i.e. the retract jog). Their difference, modulo thread pitch, is the shortest move that puts the carriage back on a pitch-aligned position relative to the spindle:

```
jogDisplacement = desiredSteps − elsStopStepsAtStop
totalError      = accumulatedError − jogDisplacement
                  + Σ(scalesSyncDeltaPos[j].error / syncRatioDen[j])  // sub-step residue
correction      = fmod(totalError, threadPitchSteps)  // normalised to ±pitch/2
stepsToGo      += round(correction)
```

The correction is applied as a small jog via `updateIndexingPosition()`, superimposed on the resumed sync motion. Because the modulo brings it within ±pitch/2, it never undoes the retract. If `threadPitchSteps = 0.0` (turning, not threading), no correction is applied.

`hysteresis` (encoder counts) optionally allows the firmware to auto-clear `active` once the carriage retracts past `stopPosition − hysteresis`, instead of requiring an SW write.

Modbus-mapped fields: `enable`, `scaleIndex`, `stopPosition`, `stopDirection`, `active`, `accumulatedError`, `threadPitchSteps`, `hysteresis`, `latchedSpindleEncoder`, plus diagnostic latches (`lastAccumulatedError`, `lastJogDisplacement`, `lastTotalError`, `lastCorrection`) captured at each resume for debugging phase drift.

---

## Build System

- **Toolchain:** `arm-none-eabi-gcc`
- **Build system:** CMake 3.30+
- **Optimisation:** `-Ofast` (Release), `-Og` (Debug)
- **FPU:** `-mfloat-abi=hard -mfpu=fpv4-sp-d16`
- **Linker script:** `STM32F411CEUX_FLASH.ld` (512 KB flash @ 0x8000000, 128 KB RAM @ 0x20000000)
- **Outputs:** `.elf`, `.hex`, `.bin`

### Flashing

```bash
# ST-Link
st-flash --format ihex write rotary-controller-f4.hex

# Raspberry Pi + OpenOCD
openocd -f ./raspberry.cfg
```
