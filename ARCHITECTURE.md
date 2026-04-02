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

Configurable position-based automatic stop for turning or threading to a shoulder. When enabled, the ISR monitors a selected axis scale encoder and latches `elsStop.active = 1` when the position crosses the configured threshold. While latched:

- Sync deltas are accumulated into `elsStop.accumulatedError` instead of driving the stepper — the fractional error accumulator (`scalesSyncDeltaPos[i].error`) continues updating every ISR cycle so the thread pitch relationship is preserved.
- `servoEnableTask` suppresses its auto-mode-1 logic, allowing the user to jog or disable the motor freely.

When SW clears `elsStop.active`, the firmware computes a correction move:

```
totalError = accumulatedError + Σ(scalesSyncDeltaPos[j].error / syncRatioDen[j])
correction = fmod(totalError, threadPitchSteps)  // shortest path within ±pitch/2
stepsToGo += round(correction)
```

If `threadPitchSteps = 0.0` (turning rather than threading), no correction is applied. The existing `updateIndexingPosition()` ramp executes the correction move overlaid on resumed ELS operation.

Configuration is via Modbus registers appended to `rampsSharedData_t` (fields: `enable`, `scaleIndex`, `stopPosition`, `stopDirection`, `active`, `accumulatedError`, `threadPitchSteps`).

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
