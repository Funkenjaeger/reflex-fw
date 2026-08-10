/*
 * ELS slip / motion-attribution accumulator, extracted as pure functions so it
 * can be unit tested in isolation (no HAL / struct deps). Used by
 * SynchroRefreshTimerIsr() in Ramps.c and by the els_slip unit tests.
 *
 * WHY THIS EXISTS
 * ---------------
 * els_backlash_cal.h answered "did Z move?". This answers "did Z move BECAUSE
 * WE MOVED IT?", which is a strictly different question and the one the
 * 2026-08-08 hardware finding actually asked.
 *
 * The take-up confirmation gate is a PURE ENDPOINT COMPARISON: Z now against a
 * baseline captured once at take-up initiation (Ramps.c, elsStopTakeupZStart).
 * It has no notion of WHEN, inside that span, the motion arrived, or whether the
 * servo was issuing pulses at the time. So an operator nudging the carriage by
 * hand with the half-nut open reads identically to drivetrain-coupled motion,
 * and on 2026-08-08 that released a correctly-withheld gate on the real machine.
 * Bounding the span (ELS_TAKEUP_CONFIRM_WINDOW_TICKS) shrank the exposure to
 * ~250 ms but did not change its shape: any Z motion from any source inside the
 * window is still accepted as proof.
 *
 * The fix is a TIME DIMENSION. Bucket each tick's Z delta by whether the servo
 * had recently issued a step pulse:
 *
 *   attributed    - dZ arriving while the servo was driving, or within a settle
 *                   horizon of its last pulse. This is the only evidence the
 *                   gate is allowed to count.
 *   unattributed  - dZ arriving with no recent commanded pulse. Something moved
 *                   the carriage and it was not us. Diagnostic only; it never
 *                   confirms anything, and it is deliberately NOT treated as a
 *                   fault either (see the honesty note below).
 *
 * WHAT THIS DOES NOT DO -- READ THIS BEFORE QUOTING THE PARAGRAPH ABOVE
 * --------------------------------------------------------------------
 * A hand nudge that lands INSIDE the settle horizon of a real pulse is
 * physically indistinguishable from genuine inertial settle, using Z and
 * commanded steps alone. That is not an implementation gap to be closed later;
 * it is what the settle horizon is FOR. There is no third signal here to break
 * the tie -- no leadscrew encoder, no nut sensor, no carriage accelerometer.
 *
 * So this NARROWS the attack window from "any time in ~250 ms" to "within
 * settleTicks of an actual pulse". That is a real reduction and it is the whole
 * benefit. It is NOT a proof of impossibility, and it must never be described as
 * one. If someone later wants that proof, it needs a sensor, not a rewrite of
 * this file.
 *
 * WHY THE LOWER-BOUND PROPERTY SURVIVES
 * -------------------------------------
 * els_backlash_cal.h:"THIS IS A LOWER BOUND ONLY. NEVER TURN IT INTO A WINDOW."
 * That constraint is preserved here BY CONSTRUCTION, not by convention:
 * elsSlipConfirmed() is a `>=` floor test with no upper limit, so a well-seated
 * drivetrain that moves the carriage the entire commanded distance still
 * confirms. Attribution only ever REMOVES evidence from the total; it can never
 * turn excess motion into a fault.
 *
 * THE SETTLE HORIZON HAS A HARD FLOOR, AND IT IS NOT A TUNING PREFERENCE
 * ---------------------------------------------------------------------
 * Step pulses are PACED, not continuous: the ISR emits at most one pulse every
 * `servoCycles` ticks (Ramps.c, servoCyclesCounter). At any take-up speed slower
 * than one step per tick, MOST ticks inside a perfectly healthy drive burst emit
 * no pulse at all. If settleTicks is shorter than that pacing period, the
 * horizon punches holes THROUGH THE MIDDLE OF THE DRIVE and genuine coupled
 * motion lands in `unattributed` -- a healthy machine that refuses to start.
 *
 * That failure is silent, speed-dependent, and would look exactly like a
 * mechanical fault. elsSlipSettleTicks() exists to make it unreachable: it
 * floors the horizon at the live pulse period so a wrong constant cannot cause
 * it. Call it; do not pass a raw constant to elsSlipTick().
 *
 * UNITS AND HOSTS -- THREE WAYS TO TUNE THIS WRONG
 * -----------------------------------------------
 * 1. settleTicks is a count of ISR TICKS, not milliseconds. Firmware constants
 *    assume the ~100 kHz TIM9 rate (Ramps.c). The emulator's real-time physics
 *    server drives the same ISR at its own isr_rate_hz (emulator/src/main.cpp
 *    documents 10 kHz, i.e. 10x slower), so a horizon chosen by watching
 *    wall-clock behaviour in serve mode is 10x wrong on hardware. Tick COUNTS
 *    are portable; seconds are not. (The fixture tests call the ISR in a tight
 *    loop with no pacing at all, so wall-clock time is meaningless there.)
 * 2. Z resolution is machine config, not a constant: elspi is 200 counts/mm,
 *    the emulator's reference config is 400 (els_backlash_cal.h). settleTicks is
 *    a TIME quantity and is therefore resolution-independent -- but any COUNTS
 *    threshold eyeballed off emulator output is 2x wrong on the lathe. Keep
 *    thresholds derived from config the way elsTakeupConfirmThreshold() does.
 * 3. The horizon must comfortably exceed ELS_SETTLE_TICKS, the post-take-up
 *    dwell, because the gate's FIRST evaluation happens that many ticks after
 *    the last pulse. A horizon shorter than the dwell rejects the inertial
 *    settle it was built to accept.
 *
 * The horizon itself is an UNMEASURED PARAMETER. See ELS_SLIP_SETTLE_TICKS in
 * Ramps.c: it is a starting value, not a commissioned one, and the number that
 * belongs there can only come off the real machine.
 */
#ifndef ELS_SLIP_H
#define ELS_SLIP_H

#include <stdint.h>
#include <stdbool.h>

/* "No pulse has ever been seen." Distinct from a large finite age so the first
 * tick of a take-up cannot attribute motion that predates the first pulse. */
#define ELS_SLIP_NEVER_DRIVEN INT32_MAX

/* Per-take-up (or, for a cumulative consumer, per-job) motion attribution state.
 *
 * Integer by construction. Each field is an exact sum of exact integer per-tick
 * deltas, so no rounding can accumulate across a window -- the same discipline
 * els_backlash_cal.h states for elsCalTakeupCommand(), and the reason the ELS
 * makes a no-drift guarantee at all. The one float in this area
 * (zCountsPerPitch/threadPitchSteps, els_backlash_cal.h) is applied ONCE at
 * decision time by elsTakeupConfirmThreshold(), never per tick.
 *
 * int64_t, not int32_t, and deliberately so. The take-up gate's own worst case
 * fits in 32 bits with headroom (per-tick dZ is bounded by the int16 cast in
 * Ramps.c's scale-refresh loop, so ~32767 x 25000 ticks ~= 8.2e8 against a 2.1e9
 * ceiling) -- but that headroom is an arithmetic bound on a type, not a measured
 * bound on plausible encoder counts, and the ELS auto-start consumer accumulates
 * CUMULATIVELY with no window to bound it at all. A wrap here would not fail
 * loudly; it would silently produce a confirmation. 64-bit adds are free on this
 * part relative to the cost of being wrong about that. */
typedef struct {
  int64_t attributedZCounts;    /* Z counts seen while recently driven -- the ONLY evidence the gate counts */
  int64_t unattributedZCounts;  /* Z counts seen with no recent pulse -- diagnostic; never confirms */
  int64_t sumServoSteps;        /* signed commanded steps issued since reset */
  int32_t ticksSinceLastPulse;  /* ISR ticks since the last nonzero dServo; ELS_SLIP_NEVER_DRIVEN until the first */
  uint16_t active;              /* 1 while accumulating; set by elsSlipReset() */
} elsSlipAccum_t;

/* Begin (or restart) accumulation. Callers reset at the point the thing being
 * attributed begins -- take-up initiation for the gate; far less often, or
 * never, for a cumulative consumer. */
static inline void elsSlipReset(elsSlipAccum_t *a)
{
  a->attributedZCounts   = 0;
  a->unattributedZCounts = 0;
  a->sumServoSteps       = 0;
  a->ticksSinceLastPulse = ELS_SLIP_NEVER_DRIVEN;
  a->active              = 1;
}

/* The settle horizon actually in force this tick.
 *
 * configuredTicks is the mechanical settle allowance (ELS_SLIP_SETTLE_TICKS).
 * servoCyclePeriod is the LIVE pulse pacing (Ramps.c's servoCycles), which
 * varies with servo.maxSpeed and is written at runtime by updateSpeedTask.
 *
 * Flooring at the pacing period is a CORRECTNESS requirement, not padding: see
 * the header note. Consecutive pulses sit exactly servoCyclePeriod ticks apart,
 * so the largest age reachable mid-burst is servoCyclePeriod - 1; the +1 covers
 * the one-tick lag between a pulse and the encoder delta it produces becoming
 * visible to the next tick's scale refresh.
 *
 * Note what this costs at very low take-up speeds: a large servoCycles drags the
 * horizon out until attribution approaches the un-attributed behaviour it
 * replaced. That degradation is honest and bounded (the gate still closes at
 * ELS_TAKEUP_CONFIRM_WINDOW_TICKS) -- and the alternative, refusing to confirm a
 * healthy slow take-up, is strictly worse. */
static inline int32_t elsSlipSettleTicks(int32_t configuredTicks,
                                         int32_t servoCyclePeriod)
{
  int32_t floorTicks = (servoCyclePeriod > 0) ? (servoCyclePeriod + 1) : 1;
  return (configuredTicks > floorTicks) ? configuredTicks : floorTicks;
}

/* One ISR tick.
 *
 * MUST be called at a point where BOTH deltas describe the SAME tick. In
 * Ramps.c that is after the scale-refresh loop (which computes this tick's dZ)
 * AND after step-pulse generation (which produces this tick's dServo) -- i.e.
 * near the end of the ISR, NOT from inside the take-up gate block, which runs
 * before both and would silently correlate last tick's Z against this tick's
 * pulse.
 *
 * dZ           this tick's raw Z-scale delta in encoder counts, already signed
 *              by scaleDir (Ramps.c, scalesDeltaPos[].delta x scales[].scaleDir).
 * dServoSteps  this tick's signed commanded step delta: -1, 0 or +1. Zero on
 *              every tick that emitted no pulse, which at any real take-up speed
 *              is most of them.
 * settleTicks  from elsSlipSettleTicks(). Do not pass a bare constant. */
static inline void elsSlipTick(elsSlipAccum_t *a, int32_t dZ,
                               int32_t dServoSteps, int32_t settleTicks)
{
  if (dServoSteps != 0) {
    a->sumServoSteps      += dServoSteps;
    a->ticksSinceLastPulse = 0;
  } else if (a->ticksSinceLastPulse < ELS_SLIP_NEVER_DRIVEN) {
    a->ticksSinceLastPulse++;   /* saturates; never wraps into "recently driven" */
  }

  if (a->ticksSinceLastPulse <= settleTicks) {
    a->attributedZCounts += dZ;
  } else {
    a->unattributedZCounts += dZ;
  }
}

/* The take-up gate's decision: has ATTRIBUTED motion reached the floor?
 *
 * Deliberately the same shape and the same fail-closed convention as
 * elsZMotionSeen() (els_backlash_cal.h), which this replaces at the gate:
 * magnitude only, `>=`, and threshCounts <= 0 means "never confirm" so an
 * uncommissioned machine refuses rather than waving everything through.
 *
 * POLARITY-FREE, and that is a deliberate scope boundary, not an oversight.
 * elsZMotionSeen() is polarity-free for the reason in els_backlash_cal.h, and
 * this change is about WHEN motion arrived, not which way it went. Making the
 * gate reject wrong-way motion is a separate, defensible strictness increase
 * with its own failure mode -- a miscomputed droSign would make a healthy
 * machine refuse forever -- so it is not smuggled in here. The signed
 * projection stays available for diagnosis (elsStop.lastTakeupZDelta, and
 * elsSlipAttributedAlong() below) so the evidence for making that change later
 * is collected either way. */
static inline bool elsSlipConfirmed(const elsSlipAccum_t *a, int32_t threshCounts)
{
  if (threshCounts <= 0) return false;         /* fail closed */
  int64_t d = a->attributedZCounts;
  if (d < 0) d = -d;
  return d >= (int64_t)threshCounts;
}

/* Attributed motion projected onto the direction we drove, in Z counts.
 * Negative means the carriage moved the WRONG way while we were driving it --
 * distinct from "didn't move" and from "moved, but not because of us". Mirrors
 * elsZMovedAlong()'s convention so the two diagnostics read the same way.
 * Saturates into int32_t for publication; the accumulator itself stays 64-bit. */
static inline int32_t elsSlipAttributedAlong(const elsSlipAccum_t *a, int32_t droSign)
{
  int64_t v = a->attributedZCounts * (int64_t)droSign;
  if (v >  INT32_MAX) return INT32_MAX;
  if (v <  INT32_MIN) return INT32_MIN;
  return (int32_t)v;
}

#endif /* ELS_SLIP_H */
