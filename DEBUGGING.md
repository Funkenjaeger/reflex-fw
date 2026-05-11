# ELS Phase Correction — Resolved Post-Mortem

**Status: RESOLVED 2026-05-11.** The `applyPhaseCorrection` algorithm in `Core/Src/Ramps.c` works as designed and cuts the same thread groove regardless of operator workflow (electronic retract, half-nut snap, mixed). The bug was in the emulator config, not the firmware.

## Symptom

Within a single Z-start position, spindle phase at trigger was repeatable to ±a few counts. Across different Z-start positions in the same threading job, phase shifted by ~hundreds to ~thousands of counts. The firmware's internal step accounting showed observed leadscrew motion ~1.6× larger than the static model `(actualAdvance + corr) × sync_ratio` predicted, with the excess scaling proportionally to cut distance.

## Root cause

`emulator/config/lathe.toml` had `leadscrew.mm_per_step = 0.0025`, which corresponds to 1270 microsteps per leadscrew turn (3.175 mm/rev ÷ 0.0025 mm/step). The Python host was configured for 800 microsteps/turn, so it wrote `threadPitchSteps = 251.969` (= 800/3.175) and `syncRatioNum/Den = -8/127` to the firmware. The two configs disagreed by a factor of `1270/800 = 1.5875`.

`applyPhaseCorrection` computes `actualAdvance = deltaZ × threadPitchSteps / zCountsPerPitch`, which expresses Z motion in *firmware-model* leadscrew steps. The emulator physics translates leadscrew steps to carriage motion using the *physical* `mm_per_step`. With the two ratios decoupled, the algorithm's correction was computed in a frame that didn't match the world it was correcting — exactly the 1.5875× discrepancy seen in the step-by-step trace.

This is a host↔firmware configuration invariant, not an algorithm fault. The same class of bug could manifest on real hardware if a stepper microstep setting were changed without updating the Python servo ratio.

## Fix

`emulator/config/lathe.toml`: `mm_per_step = 0.00396875` (= 3.175 / 800). With this value, `mm_per_step × encoder_counts_per_mm` (physics) equals `zCountsPerPitch / threadPitchSteps` (firmware), both giving 1.5875 z-counts per leadscrew step.

## Verification

Cross-Z test run 2026-05-11 19:11, single threading job, 1 mm pitch, `backlashSteps=0`. `dPhase` (signed shortest phase delta vs. previous trigger, [-cpr/2, +cpr/2]):

| Pass | Z-start          | corr | dPhase |
|------|------------------|------|--------|
| #2   | 12.700 (snap)    | +111 | +88    |
| #3   | 12.700 (same)    | -100 | -8     |
| #4   | 12.700 (re-snap) | +72  | -16    |
| #5   | 15.875 (snap)    | +70  | +16    |
| #6   | 19.050 (snap)    | -48  | -8     |

All passes within ±88 counts (most within ±16). Pass #2 is the first half-nut snap of the job. The previous order-of-magnitude "hundreds to thousands" cross-Z drift is gone.

The trace also showed that direction reversals (`change = desiredSteps − currentSteps` flipping sign) do occur briefly during correction indexing — pass #2 had 44 flips, pass #4 had 30, pass #5 had 34 — but they're a small fraction of total step pulses (pass #2: 7 reverse pulses out of 525) and the algorithm absorbs them cleanly. Hypothesis #1 from the original investigation (flutter as the dominant error source) is incidental, not causal.

## What survived

- **Step_6 instrumentation in `Core/Src/Ramps.c`** is kept as a permanent emulator-only debug aid (gated by `#ifdef EMULATOR_BUILD`, zero cost on STM32). Future ELS regressions can be diagnosed by reading the per-pass `step6 #N start / t=... / end` log lines.
- **Dashboard geometry consistency check** in `emulator/src/dashboard.cpp` warns at runtime if `mm_per_step × encoder_counts_per_mm` does not match `zCountsPerPitch / threadPitchSteps` once Python has pushed geometry. This catches the exact class of misconfiguration that caused this investigation.
- **ARCHITECTURE.md "Limits" section** now explicitly documents the host↔firmware geometry invariant.
