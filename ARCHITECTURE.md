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

#### Manual reference latch (re-sync to an existing thread)

The first-trigger auto-latch is not special in itself — it is merely a moment
when the carriage is known to sit on the correct side of the leadscrew lash.
`latchCommand` lets the host request the *same* capture at an operator-chosen
point (a tool physically seated in an existing thread's groove, lash loaded by
a cutting-direction jog), so a re-chucked or foreign thread can be picked up.
The ISR consumes the command in one pass — capturing `latchedSpindle` and
`latchedZ` coherently and setting `referenceLatched`, which is exactly the
guard that suppresses the auto-latch for the rest of the job — and acks by
incrementing `latchSeq`. A latch requested with `enable == 0` is consumed with
*no* ack: the reference would be wiped on the next `enable` 0→1 edge anyway,
and the missing seq edge is how the host reads the refusal. Everything
downstream of the latch is unchanged and unaware of which producer latched.
The operator procedure, the Z-watch tolerance rationale, and the wizard live
in reflex-ui (`reflex/fsms/els_resync.py`, `reflex/help/els_thread_resync.md`).

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

### Closed-loop backlash calibration and take-up confirmation

#### The hole this closes

The "Limits" note above says the firmware cannot tell whether the half-nut is
engaged when the operator presses Cut. That was true because the backlash
take-up was **entirely open loop**: completion was a crossing test against
`servo.currentSteps` — the firmware's own count of pulses *sent*, never motion
*observed*. With the half-nut open, a disabled servo, or a slipped coupling,
`currentSteps` crosses its target exactly on schedule, the firmware reports a
completed take-up into thin air, and `applyPhaseCorrection()` then snapshots a Z
from a drivetrain that was never coupled. The result is a confidently wrong
correction and a pass cut into the wrong groove.

The Z scale is the missing sensor. It is already sampled every ISR tick; only
the comparison was absent.

#### Why measurement and confirmation are one feature

"Z must move `backlashSteps` worth of counts" is wrong, and wrong in the unsafe
direction — it would refuse every correctly configured take-up. The take-up's
whole job is to drive the leadscrew *across* the lash window, and motion spent
inside that window moves the nut, not the carriage. So for a take-up sized *at
or below* the true lash, the carriage may not move at all, and "carriage didn't
move" cannot then distinguish a healthy take-up from an open half-nut.

That is the regime the **old** open-loop take-up lived in, with `backlashSteps`
hand-entered against an unknown lash. It is emphatically *not* the regime this
feature creates, and the distinction is load-bearing: once calibrated, the
commanded take-up exceeds the true lash by construction, so a correct take-up
**must** move the carriage, by

```
carriage motion = commanded − true lash = detection distance + margin
```

Both terms are deliberate. The measurement already reads high by the detection
distance (motion is invisible until it registers on the scale), and the take-up
adds margin on top. On elspi — 1 Z count ≈ 2.52 servo steps, 2-count threshold,
20% / 10-step margin — that is ~6 Z counts of carriage motion at a 0.05 mm lash
rising to ~10 counts at 0.20 mm, i.e. 3–5× over the detection threshold.

This is what lets the gate return a *positive* answer at all. Were "may not
move" true of a calibrated take-up, the gate could only ever withhold.

The discriminator exists only if the firmware deliberately commands *past* the
expected lash and watches for motion on the far side — which is exactly what
calibration does. The step count at which Z first moves after a reversal **is**
the backlash; observed motion **is** the proof of engagement; and "no motion
ever" is simply the failure branch of the measurement's success criterion.
Hence: measure it, always command measured + margin, always require
confirmation, and never trim the take-up toward the minimum.

#### Calibration run

Host-requested via `calCommand`, executed in the ISR (Modbus polling at tens of
Hz cannot observe a transition that happens at 100 kHz). Seat against a flank;
reverse three times, each leg counting servo steps until Z moves; then a final
unmeasured re-seat in the cutting direction so the machine is left with lash
loaded on the side a pass starts from. The host judges consistency and writes
`backlashSteps`; the firmware only measures.

Two details are load-bearing and were both found by test:

- **Sync is gated for the duration.** `scales[].syncEnable` is independent of
  `elsStop.enable`, so a turning spindle drives the leadscrew straight through
  the measurement. A leg is meaningful only if the calibration is the sole thing
  moving the leadscrew.
- **Each leg arms on the first pulse in the new direction**, not at the moment
  the reversal is commanded. The ramp overshoots while decelerating, and that
  return travel *is* lash traversal — baselining at the command instant excludes
  it and under-measures badly.

Motion detection here is polarity-free: during a reversal the carriage is
mechanically stationary while lash is traversed, so any |dZ| past the threshold
means the lash was crossed, whichever way the scale counts. This also lets
calibration run before any thread geometry is configured.

#### Failure behaviour

The take-up gate **fails closed**: no confirmed motion means `takeupPending`
stays set, sync stays gated, and `applyPhaseCorrection()` does not run. Note
where this happens — at the *start* of a pass, tool clear of the work, machine
standing still. Refusing declines to start a pass; it does not abandon a tool
buried in a groove. Between a machine that visibly refuses to start and one that
confidently starts in the wrong groove, the refusal is far the cheaper failure.

`elsStop.enable` 1→0 abandons a withheld take-up. Without that escape hatch,
failing closed would be unrecoverable — a worse defect than the one being fixed.

An unconfigured `calMotionThreshCounts` (0) disables detection and fails
**closed**, never open: a permissive default would silently restore the original
open-loop behaviour on every uncommissioned machine.

#### Modbus interface

Configuration (SW write): `enable`, `scaleIndex`, `stopPosition`, `stopDirection`, `threadPitchSteps`, `zCountsPerPitch`, `backlashSteps`, `hysteresis`, `calCeilingSteps`, `calMotionThreshCounts`.

Command (bidirectional, firmware clears on consume): `calCommand`, `latchCommand`.

State (firmware-owned, except `active` which is bidirectional): `active`, `latchedZ`, `latchedSpindle`, `referenceLatched`, `takeupPending`, `protocolVersion`.

Per-resume diagnostics: `lastIdealAdvance`, `lastActualAdvance`, `lastPhaseError`, `lastCorrection`.

Calibration / take-up outcomes: `calSeq`, `calResult`, `calMeasured[3]`, `takeupSeq`, `takeupResult`, `lastTakeupZDelta`.

Manual latch ack: `latchSeq` (increments only on an *accepted* latch — a latch
with `enable == 0` is consumed with no increment, so the absent edge is the
refusal).

`calSeq`, `takeupSeq`, and `latchSeq` are monotonic outcome counters, not flags —
the command registers are cleared the instant the ISR consumes them, long before
any result is observable, so a host polling a command for completion would read
a stale result. Edge-detect the sequence counters.

`protocolVersion` names this register layout (currently **2**; the manual-latch
pair `latchCommand`/`latchSeq` is the 1→2 append). Bump it whenever
`rampsSharedData_t` changes shape; reflex-ui checks it at connect so a
firmware/UI mismatch reports itself by name instead of surfacing as plausible
garbage in every register past the point of divergence.

Algorithm, units, and the physics constraining all of this are in
`Core/Inc/els_backlash_cal.h`.

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
