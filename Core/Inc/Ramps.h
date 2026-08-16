/**
 * Copyright © 2022 <Stefano Bertelli>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef THIRD_PARTY_RAMPS_H_
#define THIRD_PARTY_RAMPS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Modbus.h"
#include "Scales.h"
#include "els_backlash_cal.h"
#include "els_slip.h"

#define MODBUS_ADDRESS 17

#define STEP_PIN GPIO_PIN_0
#define STEP_GPIO_PORT GPIOA

#define DIR_PIN GPIO_PIN_14
#define DIR_GPIO_PORT GPIOB

#define ENA_PIN GPIO_PIN_15
#define ENA_GPIO_PORT GPIOB
#define ENA_DELAY_MS 500

#define USR_LED_Pin GPIO_PIN_12
#define USR_LED_GPIO_Port GPIOB

#define SPARE_1_PIN GPIO_PIN_1
#define SPARE_1_GPIO_PORT GPIOA

#define SPARE_2_PIN GPIO_PIN_3
#define SPARE_2_GPIO_PORT GPIOA

#define SPARE_3_PIN GPIO_PIN_4
#define SPARE_3_GPIO_PORT GPIOA

/* Diagnostic scratchpad geometry. These fix the SIZE of the reserved block in
 * elsStop_t and must NEVER change: the block's entire purpose is that its
 * offset is stable, so adding or retiring a probe never moves a register.
 * Change what goes IN the block (and diagSchema with it), never its size. */
#define ELS_DIAG_TRACE_BUCKETS 50

/* Diagnostic scratchpad schema ids -- which probe is compiled into the block.
 * Part of the register CONTRACT, not an implementation detail: reflex-ui
 * mirrors these and refuses any id it does not recognise, so append only and
 * never renumber. A stale reader that recognises an old id must not silently
 * accept a new probe's data under it. 0 means no probe. */
#define ELS_DIAG_SCHEMA_NONE 0
#define ELS_DIAG_SCHEMA_TAKEUP_SETTLE 1     /* RETIRED -- see v2 */
#define ELS_DIAG_SCHEMA_TAKEUP_SETTLE_V2 2

/* WHICH probe is compiled in, selected by the build as
 * -DELS_DIAG_PROBE=ELS_DIAG_SCHEMA_<NAME>. scripts/build.sh --diag=<name> is the
 * supported way to set it; scripts/lib/diag.sh derives the legal names from the
 * schema defines above, so the script cannot offer a probe the firmware does not
 * have.
 *
 * ONE PROBE AT A TIME, AND THAT IS A PROPERTY OF THE SHAPE, NOT A RULE TO
 * REMEMBER. Every probe writes the same 64 reserved registers; two of them
 * compiled in together would interleave their fields and produce a capture that
 * looks well-formed and means nothing. Because the selection is a single macro
 * holding a single value, "two probes at once" is not a state this build system
 * can represent -- there is no combination of flags that expresses it. A
 * documented prohibition would have needed somebody to read the document.
 *
 * ELS_DIAG_SCRATCH is DERIVED, never passed. It is the "some probe is active"
 * umbrella the shared plumbing keys off; the per-probe capture code keys off
 * ELS_DIAG_PROBE. Passing ELS_DIAG_SCRATCH by hand is rejected below rather than
 * silently reserving the block for a probe that does not exist. */
#if defined(ELS_DIAG_SCRATCH) && !defined(ELS_DIAG_PROBE)
#error "ELS_DIAG_SCRATCH is derived, not passed. Use scripts/build.sh --diag=<probe>; see DIAG.md."
#endif

#ifdef ELS_DIAG_PROBE
/* A misspelled macro name expands to an undefined identifier, which the
 * preprocessor evaluates to 0 -- i.e. silently to "no probe" while the build
 * still calls itself diagnostic. Rejecting NONE explicitly is what turns that
 * into a compile error instead of a diagnostic build that measures nothing. */
#if ELS_DIAG_PROBE == ELS_DIAG_SCHEMA_NONE
#error "ELS_DIAG_PROBE is unset, misspelled, or NONE. Use scripts/build.sh --diag=<probe>; see DIAG.md."
#elif ELS_DIAG_PROBE == ELS_DIAG_SCHEMA_TAKEUP_SETTLE
#error "ELS_DIAG_SCHEMA_TAKEUP_SETTLE is RETIRED; use takeup-settle-v2. See DIAG.md."
#elif ELS_DIAG_PROBE == ELS_DIAG_SCHEMA_TAKEUP_SETTLE_V2
/* recognised */
#else
#error "unknown ELS_DIAG_PROBE. Register the schema id in Ramps.h and add it to this chain; see DIAG.md."
#endif
#define ELS_DIAG_SCRATCH 1
#endif

/* Why the capture stopped. The distinction matters: a capture that ran out of
 * buckets did not finish measuring, and its last bucket is a floor rather than
 * a result. */
#define ELS_DIAG_END_PULSE  1   /* servo drove again -- settling is over */
#define ELS_DIAG_END_WINDOW 2   /* ran out of buckets while still quiet-or-moving */

/* Probe capture state, held in rampsHandler_t. Deliberately generic and shared
 * by every probe rather than per-probe: it is two words of ISR-owned scratch,
 * and a per-probe union here would change rampsHandler_t's size from build to
 * build for no benefit. A probe needing more than this can keep its own statics
 * in its own header.
 *
 * 0 = idle, 1 = armed, 2 = capturing -- the states the existing probe uses; a
 * probe is free to mean something else by them. */
typedef struct {
  uint16_t state;
  uint32_t captureTick;   // ISR ticks since capture start
} elsDiagCtx_t;


typedef struct {
  int32_t delta;
  uint32_t oldPosition;
  uint32_t position;
  int32_t scaledDelta;
  int32_t error;
} deltaPosError_t;

typedef struct {
  uint32_t timerHandleSlot;            // init-only: index into ramps_timer_handles[]; held as a 4-byte slot id (not a pointer) so the modbus wire layout is identical on STM32 and 64-bit emulator hosts
  int32_t position;                    // READ-ONLY (firmware-owned): absolute encoder position, updated by ISR
  int32_t speed;                       // READ-ONLY (firmware-owned): encoder speed (counts/s), updated by updateSpeedTask
  int32_t syncRatioNum, syncRatioDen;  // SW write: sync ratio numerator/denominator (output steps per input count)
  uint16_t syncEnable;                 // SW write: 0 = sync disabled, non-zero = sync enabled for this scale
  int16_t scaleDir;                    // SW write: ±1, default +1 (no inversion); applied to encoder delta in ISR
} input_t;

extern TIM_HandleTypeDef *ramps_timer_handles[SCALES_COUNT];

typedef struct {
  float maxSpeed;              // SW write: maximum step rate (steps/s); clamped to 100000 by firmware
  float currentSpeed;          // READ-ONLY (firmware-owned): live ramp speed, managed by ramp algorithm
  float jogSpeed;              // SW write: jog target speed (steps/s); negative = reverse
  float acceleration;          // SW write: ramp acceleration (steps/s²)
  int32_t stepsToGo;          // SW write: remaining steps for indexing move; firmware decrements toward zero
  uint32_t destinationSteps;  // SW write: absolute destination step count for indexing mode
  uint32_t currentSteps;      // READ-ONLY (firmware-owned): step counter incremented/decremented by ISR
  uint32_t desiredSteps;      // READ-ONLY (firmware-owned): accumulated target steps, driven by sync/ramp
  int16_t servoDir;           // SW write: ±1, default +1 (no inversion); applied to DIR pin in ISR
} servo_t;

typedef struct {
  uint32_t servoCurrent;               // READ-ONLY (firmware-owned): mirror of servo.currentSteps, updated by updateSpeedTask
  uint32_t servoDesired;               // READ-ONLY (firmware-owned): mirror of servo.desiredSteps, updated by updateSpeedTask
  uint32_t stepsToGo;                  // READ-ONLY (firmware-owned): mirror of servo.stepsToGo, updated by ramp algorithm
  float servoSpeed;                    // READ-ONLY (firmware-owned): output step rate (steps/100ms), updated by servoEnableTask
  int32_t scaleCurrent[SCALES_COUNT];  // READ-ONLY (firmware-owned): mirror of scales[i].position, updated by ISR
  int32_t scaleSpeed[SCALES_COUNT];    // READ-ONLY (firmware-owned): mirror of scales[i].speed, updated by updateSpeedTask
  uint32_t cycles;                     // READ-ONLY (firmware-owned): ISR execution time in CPU cycles, updated by updateSpeedTask
  uint32_t executionInterval;          // READ-ONLY (firmware-owned): ISR interval in CPU cycles, updated by ISR
  uint16_t servoMode;                  // SW write: 0=disabled, 1=sync/index (also set by firmware), 2=jog
} fastData_t;

typedef struct {
  uint16_t enable;            // SW write: 1 = enable ELS stop feature
  uint16_t scaleIndex;        // SW write: which scale (0–3) is the position reference (Z axis)
  int32_t  stopPosition;      // SW write: threshold in encoder counts
  int16_t  stopDirection;     // SW write: 1 = stop when pos >= threshold, -1 = stop when pos <= threshold
  uint16_t active;            // bidirectional: firmware sets to 1 when triggered; SW writes 0 to resume
  float    threadPitchSteps;  // SW write: leadscrew steps per thread pitch (float); 0.0f = turning (no correction)
  int32_t  hysteresis;        // SW write: encoder counts carriage must retract before re-enabling; 0 = no hysteresis
  float    zCountsPerPitch;   // SW write: Z scale encoder counts per thread pitch; 0.0f = correction disabled
   uint32_t backlashSteps;     // SW write: leadscrew backlash takeup magnitude in servo steps; direction derived from sign(syncRatioNum) × sign(threadPitchSteps × zCountsPerPitch); 0 = takeup disabled
  int32_t  latchedZ;          // READ-ONLY (firmware-owned): scales[scaleIndex].position at first trigger of the job
  int32_t  latchedSpindle;    // READ-ONLY (firmware-owned): scales[0].position at first trigger of the job
  uint16_t referenceLatched;  // READ-ONLY (firmware-owned): 0 until first trigger captures the reference, 1 thereafter; reset on enable 0→1
  uint16_t takeupPending;     // READ-ONLY (firmware-owned): 1 while the post-resume backlash takeup move is executing
  float    lastIdealAdvance;  // READ-ONLY (firmware-owned): last resume's deltaSpindle × syncRatioNum / syncRatioDen
  float    lastActualAdvance; // READ-ONLY (firmware-owned): last resume's deltaZ × threadPitchSteps / zCountsPerPitch
  float    lastPhaseError;    // READ-ONLY (firmware-owned): last resume's idealAdvance − actualAdvance (pre-modulo)
  float    lastCorrection;    // READ-ONLY (firmware-owned): last resume's correction added to stepsToGo (post-modulo)
  /* --- Backlash calibration + closed-loop take-up confirmation. APPENDED AT THE
   * TAIL of elsStop_t, which is itself the last member of rampsSharedData_t, so
   * every pre-existing Modbus register offset is unchanged. uint16s are grouped
   * ahead of the 32-bit fields so the block packs with ZERO padding, and the
   * next feature's registers append behind it the same way (reserved order per
   * the auto-start plan: re-sync latchCommand/latchSeq, then the auto-start
   * block). Algorithm, units, and the physics that constrain all of this live in
   * Core/Inc/els_backlash_cal.h — read that before changing anything here. */
  uint16_t protocolVersion;       // READ-ONLY (firmware-owned): register-layout version, starts at 1. Bump whenever this struct changes; reflex-ui checks it at connect so a map mismatch names itself instead of surfacing as garbled reads
  uint16_t calCommand;            // bidirectional: SW writes 1 to request a calibration run; FIRMWARE CLEARS IT on consume. This is the atomic hand-off. SW must NOT poll it for completion — it clears the instant the ISR picks it up, long before the run finishes. Edge-detect calSeq instead
  uint16_t calSeq;                // READ-ONLY (firmware-owned): increments once per finished run, success OR failure. Monotonic, so a host polling at Modbus rates cannot alias a fast run
  uint16_t calResult;             // READ-ONLY (firmware-owned): outcome of the run counted by calSeq. ELS_CAL_* in els_backlash_cal.h; 0 = OK
  uint16_t takeupResult;          // READ-ONLY (firmware-owned): outcome of the last take-up. ELS_CAL_*/ELS_TAKEUP_* in els_backlash_cal.h; 0 = OK. Replaces a binary fault flag so "carriage never moved" and "never reached target" stay distinguishable
  uint16_t takeupSeq;             // READ-ONLY (firmware-owned): increments once per take-up outcome; lets SW tell completed-normally from host-cleared, which takeupPending alone cannot
  int32_t  calMeasured[3];        // READ-ONLY (firmware-owned): lash measured at each of the 3 reversals, in servo steps. The HOST judges whether the spread is acceptable — measurement lives here, policy lives in the UI
  int32_t  calCeilingSteps;       // SW write: per-leg hard ceiling in servo steps. Driving this far without Z moving IS the open-half-nut / uncoupled failure. MACHINE-SPECIFIC; size it comfortably past the largest credible lash
  int32_t  calMotionThreshCounts; // SW write: Z scale counts that count as real motion. MACHINE-SPECIFIC — ~2 counts on elspi (200 counts/mm, so 1 count ≈ 2.5 servo steps); emulator is 400 counts/mm. 0 disables detection and FAILS CLOSED (never confirms) — deliberate: an unconfigured threshold must refuse, not wave everything through
  int32_t  lastTakeupZDelta;      // READ-ONLY (firmware-owned): signed Z counts moved across the last take-up, projected onto the take-up direction. NEGATIVE means the carriage moved the WRONG way — a distinct fault signature from "didn't move"
  int32_t  takeupThreshCounts;    // READ-ONLY (firmware-DERIVED, not operator-set): Z counts the last take-up had to move to be confirmed. Derived from (backlashSteps - mean(calMeasured)) via elsTakeupConfirmThreshold(); falls back to calMotionThreshCounts with no calibration on file or in turning mode. Published so the UI can say "moved 3, needed 4" instead of just refusing
  /* --- DIAGNOSTIC SCRATCHPAD — RESERVED, AND NEVER MEANINGFUL IN A BASELINE ---
   * A fixed 64-register (128-byte) block for temporary firmware-side
   * instrumentation, so a throwaway probe never has to change the register
   * LAYOUT and never has to bump protocolVersion again.
   *
   * The block is reserved UNCONDITIONALLY — it is part of the map in every
   * build, which is what makes its offset stable — but every WRITE to it is
   * compiled out unless ELS_DIAG_SCRATCH is defined. In a release build the
   * whole block reads zero. That makes "no probe in a real baseline" a property
   * of the binary rather than a promise in a document: release builds omit the
   * flag, so the writes do not exist.
   *
   * WHY protocolVersion CANNOT GUARD THIS, AND WHAT DOES. The point of
   * reserving the block is that adding a probe does NOT change the layout — so
   * the version deliberately does not bump, and nothing would otherwise tell a
   * reader that diagTrace[] meant one thing last build and something else this
   * build. That is a channel which hands back a plausible number with the wrong
   * meaning, which is worse than no channel. diagSchema closes it: it names the
   * probe currently compiled in, 0 means "nothing here", and ANY reader MUST
   * check it and refuse to interpret a value it does not recognise. Same
   * "names itself instead of surfacing as garbled reads" property that
   * protocolVersion gives the map as a whole.
   *
   * READ IT ON DEMAND, NEVER IN THE POLL LOOP. 64 registers is ~12 ms of extra
   * serial time per read at 115200 baud, against ~29 ms for the entire map —
   * a permanent 40% tax on every poll cycle, for a block that is empty in every
   * production build. reflex-ui reads this only when a diagnostic view is open.
   */
  uint16_t diagSchema;         // READ-ONLY (firmware-owned): identifies the probe compiled into the block. 0 = none; do NOT interpret anything below it. Never assume a schema you did not read
  uint16_t diagSeq;            // READ-ONLY (firmware-owned): increments once per COMPLETED capture. Edge-detect this; there is deliberately no "capture in progress" register to poll
  uint16_t diagBucketTicks;    // READ-ONLY (firmware-owned): ISR ticks summed into each diagTrace bucket. PUBLISHED so the host never has to assume the ISR rate — the repo has disagreed with itself about that rate by 10x
  uint16_t diagBucketCount;    // READ-ONLY (firmware-owned): populated diagTrace entries, for the same reason
  int32_t  diagSettleTicks;    // READ-ONLY (firmware-owned): ticks from capture start to the LAST tick that saw nonzero dZ. THE measurement ELS_SLIP_SETTLE_TICKS is a guess at — meaningful in v2, where the capture stops before the pass starts
  int32_t  diagNetCounts;      // READ-ONLY (firmware-owned): signed Z counts summed across the capture
  int16_t  diagTrace[ELS_DIAG_TRACE_BUCKETS];  // READ-ONLY (firmware-owned): per-bucket SIGNED sum of dZ. Signed rather than magnitude on purpose — encoder dither cancels, real motion does not, which is exactly the distinction a quiescence test needs and the reason to prefer net displacement over summed |dZ|
  uint16_t diagCaptureTicks;   // READ-ONLY (firmware-owned): ticks the capture actually ran, i.e. how long the servo stayed silent after the take-up. Distinct from diagSettleTicks, which is when Z last MOVED
  uint16_t diagEndReason;      // READ-ONLY (firmware-owned): ELS_DIAG_END_*. A window-full capture did not finish measuring; treat its tail as a floor, not a result
  uint16_t diagReserved[4];    // pads the block to a fixed 128 bytes so its size never depends on which probe is in it
} elsStop_t;

typedef struct {
  uint32_t executionInterval;          // READ-ONLY (firmware-owned): ISR period in CPU cycles (current - previous timestamp)
  uint32_t executionIntervalPrevious;  // READ-ONLY (firmware-owned): DWT timestamp of previous ISR entry
  uint32_t executionIntervalCurrent;   // READ-ONLY (firmware-owned): DWT timestamp of current ISR entry
  uint32_t executionCycles;            // READ-ONLY (firmware-owned): ISR wall-clock duration in CPU cycles
  servo_t servo;
  input_t scales[SCALES_COUNT];
  fastData_t fastData;
  elsStop_t elsStop;
} rampsSharedData_t;


typedef struct {
  // Modbus shared data
  rampsSharedData_t shared;

  // STM32 Related
  TIM_HandleTypeDef *synchroRefreshTimer;
  UART_HandleTypeDef *modbusUart;

  deltaPosError_t scalesDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSyncDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSpeed[SCALES_COUNT];
  deltaPosError_t rampsDeltaPos;
  uint32_t servoPreviousDirection;
  uint16_t elsStopPreviousActive;
  uint16_t elsStopPreviousEnable;
  int32_t  elsStopTakeupTargetSteps;  // servo.currentSteps target value at end of post-resume backlash takeup
  int32_t  elsStopTakeupSign;         // direction of the takeup move (+1/-1); completion is a crossing test, not exact equality
  int32_t  elsStopSettleCount;        // ticks elapsed since takeup commanded-complete; dwell before phase-correction Z snapshot
  uint16_t elsStopHysteresisCleared;  // 1 once the axis has cleared stopPosition by >= elsStop.hysteresis counts; cleared when firmware latches active = 1
  int32_t  elsStopTakeupZStart;       // scales[elsStop.scaleIndex].position captured at takeup INITIATION; the baseline the Z confirmation gate measures against
  int32_t  elsStopTakeupZSign;        // +1/-1: sign the Z scale should move in for this takeup; sign(signedTakeup) x droSign. Only the magnitude gates completion — the sign turns lastTakeupZDelta into a wrong-way diagnostic
  int32_t  elsStopTakeupTicks;        // ISR ticks since takeup initiation; backstop against a takeup that never reaches target (see ELS_TAKEUP_TIMEOUT_TICKS)
  uint16_t elsStopTakeupLatched;      // 1 once the Z confirmation window has closed on an unconfirmed takeup; further Z motion can no longer release the gate (see ELS_TAKEUP_CONFIRM_WINDOW_TICKS)
  elsCalCtx_t elsCal;                 // backlash calibration run state; non-Modbus, the ISR owns it entirely (els_backlash_cal.h)
  /* Motion attribution for the Z confirmation gate: which of the Z counts seen
   * during a take-up arrived while the servo was actually driving. Non-Modbus,
   * ISR-owned, reset at take-up initiation (els_slip.h).
   *
   * Deliberately NOT published over Modbus yet. elsStop_t's tail has a reserved
   * append order (re-sync, then auto-start) and the attributed/unattributed
   * split is only worth a protocolVersion bump once reflex-ui has something to
   * say with it — "moved 20 counts, none of them ours" is a much better refusal
   * message than the current one, and that is a UI change, not a firmware one. */
  elsSlipAccum_t elsSlip;
  /* Diagnostic probe capture state. Non-Modbus, ISR-owned, and untouched by a
   * release build -- the no-op entry points in els_diag.h ignore it.
   *
   * UNCONDITIONAL, and LAST IN THE STRUCT, both on purpose. Unconditional so
   * every call site in Ramps.c can pass &data->diag with no #ifdef; last so
   * that carrying it in a release build cannot shift the offset of any field
   * above it. It is zero-initialised handler RAM, so it lives in .bss and never
   * reaches the .bin -- which is why this refactor left the release image
   * byte-identical. Keep it here at the tail. */
  elsDiagCtx_t diag;
} rampsHandler_t;

extern modbusHandler_t RampsModbusData;

void RampsStart(rampsHandler_t *rampsData);

void SynchroRefreshTimerIsr(rampsHandler_t *data);

_Noreturn void updateSpeedTask(void *argument);

_Noreturn void userLedTask(__attribute__((unused)) void *argument);

_Noreturn void servoEnableTask(void *argument);

//static void timServoEnableOnCallback(xTimerHandle pxTimer);
//static void timServoEnableOffCallback(xTimerHandle pxTimer);

/* LAST, and it has to be. els_diag.h's entry points take elsStop_t* and
 * elsDiagCtx_t*, so it cannot be included until both exist -- which is why this
 * sits at the foot of the header rather than up with the other includes. */
#include "els_diag.h"

#endif