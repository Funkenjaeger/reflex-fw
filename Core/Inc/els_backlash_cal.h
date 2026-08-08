/*
 * ELS leadscrew backlash calibration + take-up confirmation, extracted as pure
 * functions so they can be unit tested in isolation (no HAL / struct deps).
 * Used by elsCalUpdate() in Ramps.c and by the els_backlash_cal permutation
 * tests.
 *
 * WHY THIS EXISTS
 * ---------------
 * The backlash take-up was 100% open loop: completion was a crossing test
 * against servo.currentSteps — the firmware's own count of pulses SENT, never
 * motion OBSERVED. If the half-nut is open, the servo is disabled, or the
 * coupling slipped, currentSteps crosses the target on schedule and the
 * firmware reports a completed take-up into thin air. applyPhaseCorrection then
 * snapshots a Z from a drivetrain that was never coupled and the pass indexes
 * into the wrong groove. ARCHITECTURE.md "Limits" names this hole and says
 * there is no sensor to warn against it. The Z scale IS that sensor.
 *
 * THE PART THAT IS EASY TO GET WRONG
 * ----------------------------------
 * "Z must move backlashSteps worth of counts" is WRONG and wrong in the unsafe
 * direction — it would refuse every correctly configured take-up. The whole
 * purpose of the take-up is to drive the leadscrew ACROSS the lash window, and
 * motion spent inside that window moves the nut, not the carriage.
 *
 * So for a take-up sized AT OR BELOW the true lash, the carriage may not move
 * at all, and "carriage didn't move" cannot then distinguish a good take-up
 * from an open half-nut. That is the regime the OLD open-loop take-up lived in
 * — backlashSteps hand-entered against an unknown lash — and it is why the
 * naive predicate above fails.
 *
 * READ THE NEXT PARAGRAPH BEFORE QUOTING THE ONE ABOVE. That is NOT the regime
 * this module creates. Once calibrated, the commanded take-up EXCEEDS the true
 * lash by construction, so a correct take-up MUST move the carriage, by a
 * knowable amount:
 *
 *     carriage motion = commanded - true lash = detection distance + margin
 *
 * Both terms are deliberate and neither is optional. The measurement already
 * reads high by the detection distance (you cannot see motion until it
 * registers on the scale), and elsCalTakeupCommand() adds margin on top. On
 * elspi (1 Z count ~= 2.52 servo steps, 2-count threshold, 20% / 10-step
 * margin) that is ~6 Z counts of carriage motion at a 0.05 mm lash rising to
 * ~10 counts at 0.20 mm — 3-5x over the detection threshold.
 *
 * This is what lets the confirmation gate return a POSITIVE answer at all. If
 * the "may not move" claim were true of a calibrated take-up, the gate could
 * only ever withhold. A calibrated take-up that produces no carriage motion
 * means something is genuinely wrong.
 *
 * The discriminator only exists if you deliberately command PAST the expected
 * lash and watch for motion on the far side. That is what calibration does:
 *
 *   - The step count at which Z FIRST MOVES after a reversal IS the backlash.
 *   - Observed Z motion IS the proof the nut is engaged.
 *   - The "no motion ever" case is simply the failure branch of the
 *     measurement's success criterion.
 *
 * Hence: measure it, then always command measured + margin, and always require
 * confirmation. Never trim the take-up toward the minimum.
 *
 * WHY MOTION DETECTION HERE IS POLARITY-FREE
 * ------------------------------------------
 * els_phase.h has to care about DRO polarity (droSign = stopDirection *
 * cuttingDir) because it compares spindle-derived and carriage-derived
 * advances, which can run opposite. Calibration does not: during a reversal the
 * carriage is mechanically STATIONARY while the lash is traversed, and the only
 * motion it is then capable of is in the new drive direction. So any |dZ| past
 * the threshold means the lash was crossed, whichever way the scale counts.
 * This also removes a config dependency — calibration is machine-level and must
 * work before any thread geometry has been entered.
 *
 * RESOLUTION, AND WHY THE MARGIN HAS A FLOOR (elspi numbers)
 * ---------------------------------------------------------
 * Leadscrew 8 TPI / 1600 steps per rev = 0.00198 mm per servo step.
 * Z scale 200 counts/mm = 0.005 mm per count. So ONE Z COUNT IS ~2.5 SERVO
 * STEPS. A motion threshold of 2 counts — about the floor before quantization
 * noise — therefore carries a ~5-step blind zone. Against a typical lash of
 * 0.05-0.20 mm (25-100 steps) that is ~5% uncertainty at the coarse end but
 * ~20% at the fine end, i.e. at a small lash a flat 20% margin is roughly the
 * measurement uncertainty itself and is not really margin at all. The take-up
 * command is therefore max(pct, floor) — see elsCalTakeupCommand(). The
 * emulator runs different geometry (400 counts/mm, 4000 counts/rev), which is
 * exactly why none of these numbers may be baked in as constants.
 */
#ifndef ELS_BACKLASH_CAL_H
#define ELS_BACKLASH_CAL_H

#include <stdint.h>
#include <stdbool.h>

/* Number of measured reversals per calibration run. Three is what makes the
 * consistency check meaningful; the host decides whether the spread is
 * acceptable (policy lives in the UI, measurement lives here). */
#define ELS_CAL_CYCLES 3

/* elsStop.calResult / elsStop.takeupResult codes. 0 is success everywhere so a
 * zeroed struct reads as "nothing wrong yet". Frozen protocol constants —
 * mirrored in reflex-ui devices.py comments. */
#define ELS_CAL_OK             0u
#define ELS_CAL_ERR_ENABLED    1u  /* refused: elsStop.enable != 0 (a job is live) */
#define ELS_CAL_ERR_SERVOMODE  2u  /* refused: fastData.servoMode != 1 */
#define ELS_CAL_ERR_CONFIG     3u  /* refused: ceiling or motion threshold unset */
#define ELS_CAL_ERR_NO_MOTION  4u  /* drove the full ceiling, Z never moved — half-nut open? */
#define ELS_CAL_ERR_ABORTED    5u  /* enable asserted, or servoMode left 1, mid-run */

/* elsStop.takeupResult additionally uses: */
#define ELS_TAKEUP_ERR_UNCONFIRMED 4u /* deliberately shares NO_MOTION: same physical cause */
#define ELS_TAKEUP_ERR_TIMEOUT     6u /* take-up never reached its commanded target */

/* Calibration phases. Values are internal (not published over Modbus); the
 * host sees calResult + calSeq, not this. */
typedef enum {
  ELS_CAL_IDLE = 0,
  ELS_CAL_SEAT,     /* drive one way until Z moves: establishes a known flank */
  ELS_CAL_MEASURE,  /* reverse, count steps until Z moves: that count IS the lash */
  ELS_CAL_RESEAT,   /* final unmeasured seat, cutting direction */
  ELS_CAL_DONE,
  ELS_CAL_FAILED
} elsCalPhase_t;

typedef struct {
  uint16_t phase;                      /* elsCalPhase_t */
  uint16_t cycle;                      /* 0..ELS_CAL_CYCLES-1 */
  int32_t  driveSign;                  /* +1/-1 currently commanded direction */
  int32_t  stepsRef;                   /* servo.currentSteps when the leg armed */
  int32_t  zRef;                       /* Z scale position when the leg armed */
  int32_t  measured[ELS_CAL_CYCLES];   /* lash per reversal, in servo steps */
  int32_t  prevSteps;                  /* servo.currentSteps last tick; used to detect the reversal pulse */
  uint16_t armed;                      /* 1 once the leadscrew is actually moving the commanded way */
  uint16_t result;                     /* ELS_CAL_* */
} elsCalCtx_t;

/* What the caller should do to the servo this tick. The pure layer decides;
 * the ISR actuates. driveSign == 0 means "stop commanding motion". */
typedef struct {
  int32_t driveSign;    /* direction to command, or 0 to hold */
  bool    startPhase;   /* true on the tick a new drive burst should be issued */
  bool    finished;     /* run is over (check ctx->result) */
} elsCalAction_t;

/* Motion test. Polarity-free by construction (see header note). Shared with the
 * per-pass take-up confirmation, which asks the same physical question.
 *
 * threshCounts <= 0 is treated as "never confirmed" rather than "always
 * confirmed": an unconfigured threshold must fail closed, because the whole
 * point of this module is refusing to proceed on unobserved motion. */
static inline bool elsZMotionSeen(int32_t zNow, int32_t zRef, int32_t threshCounts)
{
  if (threshCounts <= 0) return false;
  int32_t d = zNow - zRef;
  if (d < 0) d = -d;
  return d >= threshCounts;
}

/* Signed Z motion projected onto the direction we drove, in Z counts. Negative
 * means the carriage moved the WRONG way — a real fault signature (miswired
 * scale direction, or something driving the carriage that isn't us), distinct
 * from "didn't move". Reported as elsStop.lastTakeupZDelta for diagnosis.
 *
 * droSign is els_phase.h's quantity: how a cutting-direction servo move changes
 * the DRO. Callers that do not have valid thread geometry should pass +1 and
 * treat only the magnitude as meaningful. */
static inline int32_t elsZMovedAlong(int32_t zNow, int32_t zRef,
                                     int32_t driveSign, int32_t droSign)
{
  return (zNow - zRef) * driveSign * droSign;
}

/* The take-up command derived from a measured lash.
 *
 * Always measured + margin, never trimmed toward the minimum. The margin is
 * max(pctNum/pctDen, floorSteps) for the reason in the header note: a flat
 * percentage collapses into the measurement's own quantization uncertainty at
 * small lash, so a floor expressed in steps keeps real margin there. Integer
 * math throughout — this feeds a step count, and the ELS's no-drift guarantee
 * rests on never introducing float rounding into step domain arithmetic. */
static inline int32_t elsCalTakeupCommand(int32_t measuredSteps,
                                          int32_t pctNum, int32_t pctDen,
                                          int32_t floorSteps)
{
  if (measuredSteps <= 0) return 0;
  int32_t pctMargin = (pctDen != 0) ? (measuredSteps * pctNum) / pctDen : 0;
  int32_t margin    = (pctMargin > floorSteps) ? pctMargin : floorSteps;
  if (margin < 0) margin = 0;
  return measuredSteps + margin;
}

/* Spread of a completed measurement set, in servo steps (max - min). The host
 * decides what spread is acceptable; this only computes it, so the same number
 * is available to firmware diagnostics and to the UI's accept/reject gate
 * without two implementations disagreeing. */
static inline int32_t elsCalSpread(const int32_t *measured, int32_t n)
{
  if (n <= 0) return 0;
  int32_t lo = measured[0], hi = measured[0];
  for (int32_t i = 1; i < n; i++) {
    if (measured[i] < lo) lo = measured[i];
    if (measured[i] > hi) hi = measured[i];
  }
  return hi - lo;
}

static inline int32_t elsCalMean(const int32_t *measured, int32_t n)
{
  if (n <= 0) return 0;
  int32_t sum = 0;
  for (int32_t i = 0; i < n; i++) sum += measured[i];
  return sum / n;
}

/* Mean, but 0 unless EVERY leg produced a real measurement.
 *
 * A run that measured two reversals and then lost the carriage leaves a zero in
 * the set; averaging that silently halves the apparent lash. Callers deriving a
 * threshold from the measurement need "is there a trustworthy calibration on
 * file" to be a single unambiguous answer, so an incomplete set reports as no
 * calibration rather than as a small one. */
static inline int32_t elsCalMeanValid(const int32_t *measured, int32_t n)
{
  if (n <= 0) return 0;
  int32_t sum = 0;
  for (int32_t i = 0; i < n; i++) {
    if (measured[i] <= 0) return 0;
    sum += measured[i];
  }
  return sum / n;
}

/* Fraction of the EXPECTED carriage motion a take-up must produce to be
 * confirmed. One half: enough to reject a take-up that barely twitched when it
 * should have moved several counts, while leaving generous room for
 * quantization, lash asymmetry between directions, and a machine whose lash has
 * drifted somewhat since it was calibrated. Raising this toward 1 trades
 * false-accepts for false-refusals, and a false refusal stops a working
 * machine — so it stays conservative. */
#define ELS_TAKEUP_CONFIRM_NUM 1
#define ELS_TAKEUP_CONFIRM_DEN 2

/* Per-pass take-up confirmation threshold, in Z counts.
 *
 * calMotionThreshCounts is the smallest motion the SCALE CAN RESOLVE. That is
 * the right number for a calibration leg, where it sets the measurement bias.
 * It is the WRONG number for confirming a take-up, and using it there is a much
 * weaker test than the physics supports: a calibrated take-up is entitled to
 * move the carriage a knowable amount, because the command deliberately exceeds
 * the lash.
 *
 *     motion_steps  = commanded - trueLash
 *     trueLash      ~= measuredMean - detectionDistance
 *  => motion_counts  = (commanded - measuredMean) * zPerStep + motionThreshCounts
 *
 * (the detection distance cancels into the +motionThreshCounts term, which is
 * why this needs no separate estimate of it). On elspi that is ~6 Z counts at a
 * 0.05 mm lash rising to ~10 at 0.20 mm, against a bare floor of 2 — so a
 * take-up that moves 2 counts when it should have moved 7 currently PASSES.
 * That is a real signature: partial half-nut engagement, a slipping nut, a
 * coupling on its way out.
 *
 * THIS IS A LOWER BOUND ONLY. NEVER TURN IT INTO A WINDOW.
 * The carriage moves (commanded - lash still to traverse), and how much lash is
 * still to traverse depends on where the nut was sitting when the take-up
 * started. The formula above assumes the worst case — nut against the far wall,
 * so the full lash must be crossed — which is the MINIMUM motion a healthy
 * take-up can produce. A nut already partway across, or already against the
 * take-up's own wall, moves the carriage further, up to the entire commanded
 * distance. All of those are healthy. An upper bound would reject them, and
 * would reject exactly the cases where the drivetrain was already well seated.
 * Motion far in excess of expectation is not evidence of a fault here.
 *
 * FALLS BACK to the bare floor whenever the derivation is not trustworthy — no
 * complete calibration on file, turning mode (zero thread geometry), or a
 * hand-entered backlashSteps at or below the measurement. Falling back
 * reproduces the previous behaviour exactly rather than inventing a threshold
 * out of numbers that do not mean what this formula assumes. In particular an
 * UNCALIBRATED machine must not become un-runnable because commanded minus zero
 * looks like an enormous margin.
 *
 * A non-positive motionThreshCounts is returned unchanged so the fail-closed
 * path (elsZMotionSeen never confirms) is preserved end to end. */
static inline int32_t elsTakeupConfirmThreshold(int32_t commandedSteps,
                                                int32_t measuredMeanSteps,
                                                float threadPitchSteps,
                                                float zCountsPerPitch,
                                                int32_t motionThreshCounts)
{
  if (motionThreshCounts <= 0)  return motionThreshCounts;   /* fail closed, unchanged */
  if (measuredMeanSteps  <= 0)  return motionThreshCounts;   /* no calibration on file */
  if (threadPitchSteps == 0.0f || zCountsPerPitch == 0.0f) {
    return motionThreshCounts;                                /* turning: no geometry */
  }

  int32_t marginSteps = commandedSteps - measuredMeanSteps;
  if (marginSteps <= 0) return motionThreshCounts;            /* commanded at/below measured */

  float zPerStep = zCountsPerPitch / threadPitchSteps;
  if (zPerStep < 0.0f) zPerStep = -zPerStep;

  float expected  = (float)marginSteps * zPerStep + (float)motionThreshCounts;
  int32_t thresh  = (int32_t)((expected * (float)ELS_TAKEUP_CONFIRM_NUM)
                              / (float)ELS_TAKEUP_CONFIRM_DEN);
  return (thresh > motionThreshCounts) ? thresh : motionThreshCounts;
}

/* Begin a run. cuttingDir is the direction a cut feeds the leadscrew; the run
 * ENDS seated in that direction so the machine is left with lash loaded on the
 * same side a pass starts from, rather than in an unknown state. */
static inline void elsCalStart(elsCalCtx_t *ctx, int32_t cuttingDir,
                               int32_t currentSteps, int32_t zPos)
{
  ctx->phase     = ELS_CAL_SEAT;
  ctx->cycle     = 0;
  ctx->driveSign = (cuttingDir >= 0) ? 1 : -1;
  ctx->stepsRef  = currentSteps;
  ctx->zRef      = zPos;
  ctx->prevSteps = currentSteps;
  ctx->armed     = 0;
  ctx->result    = ELS_CAL_OK;
  for (int32_t i = 0; i < ELS_CAL_CYCLES; i++) ctx->measured[i] = 0;
}

/* One tick of the calibration state machine.
 *
 * stepsRemaining is servo.stepsToGo: the caller commands driveSign*ceiling at
 * each phase start and this drains toward 0. Reaching 0 without confirmed
 * motion means we drove the entire ceiling and the carriage never responded —
 * the open-half-nut / uncoupled case, which is a hard failure, not a retry.
 *
 * Note the ordering: motion is checked BEFORE exhaustion. A carriage that moves
 * on the very last commanded step is a successful measurement, not a failure. */
static inline elsCalAction_t elsCalTick(elsCalCtx_t *ctx,
                                        int32_t currentSteps, int32_t zPos,
                                        int32_t stepsRemaining,
                                        int32_t motionThreshCounts)
{
  elsCalAction_t act = { 0, false, false };
  int32_t prevSteps = ctx->prevSteps;
  ctx->prevSteps = currentSteps;

  switch (ctx->phase) {
    case ELS_CAL_IDLE:
    case ELS_CAL_DONE:
    case ELS_CAL_FAILED:
      act.finished = (ctx->phase != ELS_CAL_IDLE);
      return act;

    case ELS_CAL_SEAT:
    case ELS_CAL_MEASURE:
    case ELS_CAL_RESEAT: {
      /* ---- ARMING: do not start measuring until the leadscrew has ACTUALLY
       * reversed ------------------------------------------------------------
       * Commanding a reversal does not reverse the servo. It decelerates
       * through the ramp first, and while it does, currentSteps keeps moving
       * the OLD way and so does the carriage. If the leg baselined zRef at the
       * command instant, that residual motion crosses the threshold within a
       * few steps and the leg "completes" having measured the deceleration
       * transient rather than the lash. (Measured 6 steps against a true lash
       * of 60 before this existed — see els_takeup_confirm_test.)
       *
       * The lash traversal in the new direction begins exactly when
       * currentSteps first moves in the new direction, so that is where both
       * baselines are taken. Steps spent decelerating are correctly excluded:
       * they did not cross any lash. */
      if (!ctx->armed) {
        /* Arm on the first PULSE in the new direction, comparing against LAST
         * TICK rather than against the phase-start count. The distinction is
         * load-bearing: the ramp overshoots past the reversal command while it
         * decelerates, so a phase-start baseline would require currentSteps to
         * travel all the way back before arming — and that return travel is
         * lash traversal, which would then be excluded from the measurement.
         * (Observed: legs 2 and 3 read 4 steps against a true lash of 60.)
         * The first pulse in the new direction is exactly where lash traversal
         * begins, so baseline both refs there. */
        if ((currentSteps - prevSteps) * ctx->driveSign >= 1) {
          ctx->armed    = 1;
          ctx->stepsRef = prevSteps;
          ctx->zRef     = zPos;
        } else {
          if (stepsRemaining == 0) {
            /* Drove a whole ceiling and the servo never even moved the
             * commanded way — not a lash measurement, a dead drivetrain. */
            ctx->phase   = ELS_CAL_FAILED;
            ctx->result  = ELS_CAL_ERR_NO_MOTION;
            act.finished = true;
            return act;
          }
          act.driveSign = ctx->driveSign;
          return act;
        }
      }

      bool moved = elsZMotionSeen(zPos, ctx->zRef, motionThreshCounts);

      if (moved) {
        if (ctx->phase == ELS_CAL_MEASURE) {
          int32_t steps = currentSteps - ctx->stepsRef;
          if (steps < 0) steps = -steps;
          ctx->measured[ctx->cycle] = steps;
          ctx->cycle++;
        }

        if (ctx->phase == ELS_CAL_RESEAT) {
          ctx->phase   = ELS_CAL_DONE;
          act.finished = true;
          return act;
        }

        /* Reverse for the next leg. After the last measurement, the final leg
         * is an unmeasured re-seat back into the cutting direction. */
        ctx->driveSign = -ctx->driveSign;
        ctx->phase     = (ctx->cycle >= ELS_CAL_CYCLES) ? ELS_CAL_RESEAT : ELS_CAL_MEASURE;
        ctx->stepsRef  = currentSteps;
        ctx->zRef      = zPos;
        ctx->armed     = 0;          /* re-arm: the servo has not reversed yet */
        act.driveSign  = ctx->driveSign;
        act.startPhase = true;
        return act;
      }

      if (stepsRemaining == 0) {
        ctx->phase   = ELS_CAL_FAILED;
        ctx->result  = ELS_CAL_ERR_NO_MOTION;
        act.finished = true;
        return act;
      }

      act.driveSign = ctx->driveSign;   /* keep going */
      return act;
    }

    default:
      ctx->phase   = ELS_CAL_FAILED;
      ctx->result  = ELS_CAL_ERR_CONFIG;
      act.finished = true;
      return act;
  }
}

#endif /* ELS_BACKLASH_CAL_H */
