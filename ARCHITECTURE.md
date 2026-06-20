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

#### Purpose

Stop a synchronized cut at a specific Z position (a shoulder, a workpiece end, an annotated coordinate) and, on resume, preserve thread phase so the cutter re-enters the same helical groove on every subsequent pass — regardless of how the operator got the carriage back to the start (electronic retract, half-nut open + manual reposition + close, or any combination).

#### Operating model

The firmware treats threading as a single *job* with potentially many passes. A job begins when software writes `enable = 1` and ends when software writes `enable = 0`. Within a job, all passes share one reference frame.

A pass has three logical phases:

- **Cut.** Sync is active. Each spindle encoder tick produces a fractional-step contribution to the leadscrew's target position, with integer truncation and remainder tracking so that average tracking error stays near zero. The carriage advances toward the configured stop position.
- **Trigger.** When the Z scale crosses the stop position in the configured direction, the firmware atomically sets `active = 1` (gating sync off) and, on the *first* such trigger of the job, latches a reference pair: the spindle position and the Z position at that instant. Subsequent triggers in the same job set `active = 1` but do not re-latch — the original reference is what defines this job's thread.
- **Resume.** Software clears `active` when the operator is ready to start the next pass. This 1→0 transition is where the re-sync math runs.

#### Re-sync mechanism

Between the trigger and the resume, the operator can do almost anything — jog the carriage electronically, open the half-nut, hand-wheel the carriage to a different location, snap the half-nut closed at a thread-incompatible position. The spindle keeps rotating freely throughout. By the time `active` clears, the carriage may be anywhere, and the leadscrew may be anywhere relative to where pure uninterrupted sync would have placed it.

The re-sync computes two quantities, both expressed in *leadscrew step equivalents*:

- **Ideal advance** — how many steps the leadscrew *would* have moved if pure sync had run continuously since the latch. Derived from accumulated spindle motion times the sync ratio.
- **Actual advance** — how many steps the carriage *did* move since the latch, derived from the Z scale and the configured thread-pitch geometry.

Their difference is a signed phase error in step-equivalents. Modulo the thread pitch and folded to the shortest signed magnitude, it yields a correction the firmware queues as an indexing move *before* sync resumes. That move physically shifts the carriage by a sub-thread-pitch amount in whichever direction lands it on an integer multiple of thread pitch above the stop position. From that adjusted starting position, the subsequent sync return necessarily covers an integer number of thread pitches — meaning the spindle rotates an integer number of revolutions — meaning the spindle phase at the next trigger matches the latched phase, regardless of how the carriage got there.

A backlash takeup move, if configured, executes first (in the direction derived from cut direction and thread geometry); the phase correction runs once the takeup completes, so the takeup itself can't introduce additional phase error.

#### Why it works across workflows

The mechanism does not care whether the carriage was driven electronically or manipulated mechanically. It only sees the carriage's current Z and computes the sub-pitch residue between that Z and the latched Z. Workflow-specific perturbations — half-nut snaps to leadscrew-pitch grid, manual jogs, inconsistent retract distances — all manifest as different residues, all absorbed by the same modular correction. The cut always stops at the configured Z, and the cutter always enters the same thread groove, by construction.

#### Limits

The re-sync assumes the Z scale reading at resume reflects a carriage that is *mechanically coupled* to the leadscrew at that moment (i.e., the half-nut is engaged when the operator presses Cut). Pressing Cut with the half-nut open will produce a physically meaningless leadscrew offset — there is no sensor to warn against this. The thread geometry parameters (sync ratio, thread-pitch-in-steps, Z-counts-per-pitch) must be configured consistently with each other; the correction folds error within `pitch/2`, so a systematic mismatch larger than half a pitch will alias and the cutter will drift to a different groove. Crucially, the firmware's `threadPitchSteps / zCountsPerPitch` ratio (leadscrew steps per Z-encoder count, the firmware's *model* of the carriage drivetrain) must equal the physical drivetrain's actual leadscrew-steps-per-Z-count ratio, or the algorithm's geometry is decoupled from reality and phase will drift in proportion to cut distance.

#### Modbus interface

Configuration (SW write): `enable`, `scaleIndex`, `stopPosition`, `stopDirection`, `threadPitchSteps`, `zCountsPerPitch`, `backlashSteps`, `hysteresis`.

State (firmware-owned, except `active` which is bidirectional): `active`, `latchedZ`, `latchedSpindle`, `referenceLatched`, `takeupPending`.

Per-resume diagnostics: `lastIdealAdvance`, `lastActualAdvance`, `lastPhaseError`, `lastCorrection`.

Field-level semantics (units, sign conventions, who writes what) are documented inline at the `elsStop_t` struct definition in `Core/Inc/Ramps.h`. The algorithm itself is in `applyPhaseCorrection()` and the trigger block in `SynchroRefreshTimerIsr()` in `Core/Src/Ramps.c`.

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
st-flash --format ihex write reflex.hex

# Raspberry Pi + OpenOCD
openocd -f ./raspberry.cfg
```
