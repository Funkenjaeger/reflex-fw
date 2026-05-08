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

Position-based automatic stop for turning or threading to a shoulder, with phase-preserving re-sync on resume. The re-sync uses a **(Z position, spindle position) reference pair** captured at the *first* stop trigger of a threading job, plus a **deterministic backlash takeup** before the phase math runs. This lets the operator retract — or even open the half-nut and reposition the carriage manually — between passes without disturbing thread phase on resume.

**Reference latch.** On `enable` 0→1 (start of a new threading job), `referenceLatched` is cleared. On the *first* stop trigger thereafter, firmware captures `scales[scaleIndex].position → latchedZ` and `scales[0].position → latchedSpindle`, and sets `referenceLatched = 1`. Subsequent stop triggers in the same job reuse the same reference (they only set `active = 1`).

**While stopped** (`active = 1`): sync is gated off — `scaledDelta` is *not* added to `desiredSteps`. The fractional residue carried in `scalesSyncDeltaPos[i].error` continues to track sub-step precision. The user is free to jog or to disengage the half-nut and move the carriage by hand; the Z scale follows the carriage.

**On resume** (SW writes `active = 0`), the firmware runs a small state machine to re-sync:

1. **Take up backlash.** If `backlashSteps != 0`, firmware sets `takeupPending = 1`, adds `backlashSteps` to `stepsToGo`, and latches the target `currentSteps + backlashSteps`. The indexing ramp drives the leadscrew that many steps in the cutting direction. Sync stays paused (the sync gate also checks `takeupPending`). The over-takeup magnitude must be ≥ the lathe's measured backlash so the nut is *guaranteed* to land on the cutting face regardless of where it sat in the play window when resume was clicked. Z scale doesn't change during backlash takeup (the carriage is on a linear scale, not driven by the leadscrew during backlash); only the leadscrew/servo moves.

2. **Compute correction post-takeup.** When `currentSteps` reaches the latched target, firmware reads scales and computes:
   ```
   deltaSpindle  = scales[0].position − latchedSpindle
   deltaZ        = scales[scaleIndex].position − latchedZ
   idealAdvance  = deltaSpindle × syncRatioNum / syncRatioDen     // leadscrew steps that sync would have commanded
   actualAdvance = deltaZ × threadPitchSteps / zCountsPerPitch    // leadscrew steps the carriage actually moved
   phaseError    = idealAdvance − actualAdvance
   correction    = fmod(phaseError, threadPitchSteps)             // normalised to ±pitch/2
   stepsToGo    += round(correction)
   ```
   `idealAdvance` uses the spindle's existing `syncRatio` (which already encodes leadscrew-steps-per-spindle-count). `actualAdvance` uses the new `zCountsPerPitch` field to convert Z scale counts → leadscrew steps. The pitch-modulo bounds the correction to ±pitch/2 — the shortest jog that re-aligns the carriage to the thread.

3. **Resume sync.** `takeupPending` is cleared and `scaledDelta` once again accumulates into `desiredSteps` on every ISR tick.

If `threadPitchSteps = 0.0` or `zCountsPerPitch = 0.0` (turning, not threading), no correction or takeup is applied. If `backlashSteps = 0`, the takeup state is skipped and the correction is computed inline at the resume edge.

`hysteresis` (encoder counts) optionally allows the firmware to auto-clear `active` once the carriage retracts past `stopPosition − hysteresis`, instead of requiring an SW write.

**Modbus-mapped fields.** Config (SW write): `enable`, `scaleIndex`, `stopPosition`, `stopDirection`, `threadPitchSteps`, `hysteresis`, `zCountsPerPitch`, `backlashSteps`. State: `active` (bidirectional), `latchedZ`, `latchedSpindle`, `referenceLatched`, `takeupPending`. Diagnostics latched at every resume: `lastIdealAdvance`, `lastActualAdvance`, `lastPhaseError`, `lastCorrection`.

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
