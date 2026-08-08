/*
 * Unit tests for the pure backlash-calibration / take-up-confirmation layer
 * (Core/Inc/els_backlash_cal.h). No HAL, no ISR, no emulator physics — this
 * exercises the decision logic directly, which is the whole reason it was
 * extracted as pure functions.
 *
 * WHAT THIS FILE IS GUARDING
 * --------------------------
 * The take-up used to be 100% open loop: completion was a crossing test against
 * servo.currentSteps, the firmware's own count of pulses SENT. An open half-nut
 * produced a confidently "completed" take-up and a phase correction computed
 * from an uncoupled drivetrain. The gate these functions implement is the fix,
 * and the properties below are the ones that make it correct rather than merely
 * present:
 *
 *   - an unconfigured motion threshold must FAIL CLOSED, not wave everything
 *     through (a permissive default here silently restores the original bug);
 *   - a leg that finds motion on its very last commanded step is a SUCCESS, not
 *     a failure — the ordering of the motion check against the exhaustion check
 *     is load-bearing;
 *   - the take-up command is measured + max(pct, floor), never trimmed toward
 *     the minimum, because at small lash a flat percentage collapses into the
 *     measurement's own quantization uncertainty (see els_backlash_cal.h).
 *
 * Mutation-tested: see the MUTATION notes on the individual cases for the exact
 * edit that must make each one fail. A test that cannot fail is worse than no
 * test, because it reads as coverage.
 */
#include <cstdio>
#include <cstdlib>

#include "els_backlash_cal.h"

static int failures = 0;

static void check(bool cond, const char *what) {
    printf("   %-72s %s\n", what, cond ? "ok" : "FAIL");
    if (!cond) failures++;
}

static void checkEq(int32_t got, int32_t want, const char *what) {
    bool ok = (got == want);
    printf("   %-72s %s (got %d, want %d)\n", what, ok ? "ok" : "FAIL",
           (int)got, (int)want);
    if (!ok) failures++;
}

/* ------------------------------------------------------------------------- *
 * A tiny simulated drivetrain with lash.
 *
 * Models exactly the property that makes this problem hard: while the leadscrew
 * traverses the lash window the carriage does NOT move, and only once the nut
 * reaches a wall does Z follow. `coupled = false` models an open half-nut — the
 * leadscrew turns forever and Z never moves.
 * ------------------------------------------------------------------------- */
struct Drivetrain {
    int32_t lashSteps;        /* true backlash, in servo steps */
    int32_t stepsPerZCount;   /* servo steps per Z encoder count (elspi ~2.5) */
    bool    coupled = true;

    int32_t currentSteps = 0; /* pulses issued */
    int32_t z            = 0; /* Z scale counts */
    int32_t nutPos       = 0; /* position within [0, lashSteps] */
    int32_t carriage     = 0; /* carriage position in servo-step equivalents */

    void advance(int32_t dir) {
        currentSteps += dir;
        if (!coupled) return;
        nutPos += dir;
        if (nutPos > lashSteps) { carriage += (nutPos - lashSteps); nutPos = lashSteps; }
        if (nutPos < 0)         { carriage += nutPos;               nutPos = 0; }
        z = carriage / stepsPerZCount;
    }
};

/* Drive a full calibration run against the model. Returns ticks consumed. */
static int32_t runCal(elsCalCtx_t &ctx, Drivetrain &dt,
                      int32_t ceiling, int32_t threshCounts,
                      int32_t cuttingDir, int32_t maxTicks = 2000000)
{
    elsCalStart(&ctx, cuttingDir, dt.currentSteps, dt.z);
    int32_t stepsToGo = ctx.driveSign * ceiling;

    for (int32_t t = 0; t < maxTicks; t++) {
        elsCalAction_t act = elsCalTick(&ctx, dt.currentSteps, dt.z,
                                        stepsToGo, threshCounts);
        if (act.finished) return t;
        if (act.startPhase) stepsToGo = act.driveSign * ceiling;

        if (stepsToGo != 0) {                  /* the indexing ramp, simplified */
            int32_t dir = (stepsToGo > 0) ? 1 : -1;
            dt.advance(dir);
            stepsToGo -= dir;
        }
    }
    return -1;   /* did not terminate */
}

int main() {
    printf("=== ELS backlash calibration / take-up confirmation ===\n\n");

    /* ---------------- Motion test: the fail-closed property -------------- */
    printf("-- elsZMotionSeen --\n");
    check(elsZMotionSeen(100, 90, 5),  "10 counts of motion clears a 5-count threshold");
    check(!elsZMotionSeen(100, 98, 5), "2 counts does not clear a 5-count threshold");
    check(elsZMotionSeen(90, 100, 5),  "motion is magnitude-only (negative dZ counts)");
    /* MUTATION: change the `threshCounts <= 0` guard to `return true` and this
     * fails. That mutation is precisely the original open-loop behaviour, so
     * this single assertion is the regression guard for the whole feature. */
    check(!elsZMotionSeen(100, 0, 0),  "UNCONFIGURED threshold (0) NEVER confirms - fails closed");
    check(!elsZMotionSeen(100, 0, -1), "negative threshold also fails closed");

    /* ---------------- Signed projection ---------------------------------- */
    printf("\n-- elsZMovedAlong --\n");
    checkEq(elsZMovedAlong(110, 100, 1, 1),   10, "moved with the drive direction is positive");
    checkEq(elsZMovedAlong(90, 100, 1, 1),   -10, "moved AGAINST the drive direction is negative (wrong-way fault)");
    checkEq(elsZMovedAlong(90, 100, -1, 1),   10, "reversed drive flips the projection");
    checkEq(elsZMovedAlong(90, 100, 1, -1),   10, "inverted DRO polarity flips the projection");

    /* ---------------- Take-up command: measured + max(pct, floor) -------- */
    printf("\n-- elsCalTakeupCommand (20%% margin, 10-step floor) --\n");
    checkEq(elsCalTakeupCommand(100, 20, 100, 10), 120, "coarse lash: 20%% dominates (100 -> 120)");
    /* At 25 steps (~0.05 mm on elspi) a flat 20% is 5 steps — about the
     * measurement's own blind zone at a 2-count threshold. The floor is what
     * keeps real margin there.
     * MUTATION: drop the floor term (return measured + pctMargin) -> 30, fails. */
    checkEq(elsCalTakeupCommand(25, 20, 100, 10),   35, "fine lash: FLOOR dominates (25 -> 35, not 30)");
    checkEq(elsCalTakeupCommand(50, 20, 100, 10),   60, "crossover point: pct == floor");
    checkEq(elsCalTakeupCommand(0, 20, 100, 10),     0, "no measurement yields no command");
    checkEq(elsCalTakeupCommand(-5, 20, 100, 10),    0, "negative measurement yields no command");

    /* ---------------- Derived take-up confirmation threshold -------------- */
    printf("\n-- elsTakeupConfirmThreshold (expected motion, not the bare floor) --\n");
    {
        /* elspi-shaped geometry: zPerStep = zCountsPerPitch/threadPitchSteps.
         * 1 Z count ~= 2.52 servo steps, so zPerStep ~= 0.397. */
        const float TPS = 533.333f, ZCP = 211.67f;   /* 211.67/533.333 = 0.3969 */

        /* lash 60 -> measured ~65, commanded 78, margin 13.
         * expected = 13*0.3969 + 2 = 7.2 counts; half of that = 3. */
        checkEq(elsTakeupConfirmThreshold(78, 65, TPS, ZCP, 2), 3,
                "calibrated: demands a fraction of expected motion, not the floor");

        /* Bigger lash -> bigger margin -> a proportionally stricter demand. */
        checkEq(elsTakeupConfirmThreshold(127, 106, TPS, ZCP, 2), 5,
                "coarser lash raises the demand");

        /* MUTATION: return motionThreshCounts unconditionally and every
         * assertion in this block collapses to 2 — which is precisely the weak
         * test this function exists to replace. */

        /* ---- fallbacks: each must reproduce the bare floor exactly ---- */
        checkEq(elsTakeupConfirmThreshold(78, 0, TPS, ZCP, 2), 2,
                "NO CALIBRATION on file falls back to the floor");
        checkEq(elsTakeupConfirmThreshold(78, 65, 0.0f, ZCP, 2), 2,
                "turning mode (no thread geometry) falls back to the floor");
        checkEq(elsTakeupConfirmThreshold(78, 65, TPS, 0.0f, 2), 2,
                "zero zCountsPerPitch falls back to the floor");
        checkEq(elsTakeupConfirmThreshold(60, 65, TPS, ZCP, 2), 2,
                "commanded at/below measured falls back to the floor");
        /* The uncalibrated case is the load-bearing fallback: commanded minus
         * zero looks like an enormous margin, and without the guard a machine
         * that has never been calibrated would become un-runnable. */
        checkEq(elsTakeupConfirmThreshold(78, 0, TPS, ZCP, 0), 0,
                "unconfigured floor stays 0 so the fail-closed path is preserved");
    }

    printf("\n-- elsCalMeanValid (an incomplete run is NOT a small calibration) --\n");
    {
        int32_t good[3] = {64, 65, 66};
        int32_t partial[3] = {64, 65, 0};
        checkEq(elsCalMeanValid(good, 3), 65, "complete set averages");
        /* MUTATION: use elsCalMean here and a half-finished run reports a
         * plausible-but-small lash, which then inflates the derived threshold. */
        checkEq(elsCalMeanValid(partial, 3), 0, "incomplete set reports NO calibration");
    }

    /* ---------------- Spread / mean --------------------------------------- */
    printf("\n-- elsCalSpread / elsCalMean --\n");
    { int32_t m[3] = {100, 104, 98};
      checkEq(elsCalSpread(m, 3), 6,   "spread is max-min");
      checkEq(elsCalMean(m, 3),  100,  "mean of a tight set"); }
    { int32_t m[3] = {100, 220, 98};
      checkEq(elsCalSpread(m, 3), 122, "an outlier shows up as a large spread"); }

    /* ---------------- The state machine: a healthy machine ---------------- */
    printf("\n-- calibration run, coupled drivetrain (true lash = 100 steps) --\n");
    {
        elsCalCtx_t ctx{};
        Drivetrain dt{100, 3, true};
        int32_t ticks = runCal(ctx, dt, /*ceiling*/ 400, /*thresh*/ 2, /*cuttingDir*/ 1);

        check(ticks > 0, "run terminates");
        checkEq(ctx.result, ELS_CAL_OK, "result is OK");
        checkEq(ctx.phase, ELS_CAL_DONE, "ends in DONE");
        printf("      measured = %d, %d, %d (true lash 100, threshold 2 counts = 6 steps)\n",
               (int)ctx.measured[0], (int)ctx.measured[1], (int)ctx.measured[2]);
        for (int i = 0; i < ELS_CAL_CYCLES; i++) {
            char buf[96];
            snprintf(buf, sizeof buf, "measurement %d recovers the lash within quantization", i);
            /* Detection needs the carriage to move threshCounts, so each
             * measurement reads slightly HIGH by the detection distance. That
             * bias is real and is exactly why the take-up adds margin rather
             * than trusting the number. */
            check(ctx.measured[i] >= 100 && ctx.measured[i] <= 100 + 2 * 3 + 2, buf);
        }
        checkEq(elsCalSpread(ctx.measured, 3) <= 2, 1, "the three measurements agree (spread <= 2)");
    }

    /* ---------------- The state machine: an OPEN HALF-NUT ---------------- */
    printf("\n-- calibration run, UNCOUPLED drivetrain (open half-nut) --\n");
    {
        elsCalCtx_t ctx{};
        Drivetrain dt{100, 3, false};        /* leadscrew turns, carriage never moves */
        int32_t ticks = runCal(ctx, dt, 400, 2, 1);

        check(ticks > 0, "run terminates rather than hanging");
        /* MUTATION: make elsZMotionSeen permissive, or check exhaustion before
         * motion, and this becomes ELS_CAL_OK with garbage measurements — the
         * exact silent-wrong-answer this whole feature exists to prevent. */
        checkEq(ctx.result, ELS_CAL_ERR_NO_MOTION, "result is NO_MOTION, not OK");
        checkEq(ctx.phase, ELS_CAL_FAILED, "ends in FAILED");
        checkEq(ctx.cycle, 0, "no measurement was recorded");
    }

    /* ---------------- Ordering: motion on the final commanded step ------- */
    printf("\n-- motion detected on the LAST commanded step --\n");
    {
        /* Ceiling sized so the carriage crosses the detection threshold on the
         * very last step of the first measured reversal. If elsCalTick checked
         * exhaustion before motion, this legitimate machine would be condemned.
         * MUTATION: move the `stepsRemaining == 0` block above the `moved`
         * block -> NO_MOTION, fails. */
        elsCalCtx_t ctx{};
        Drivetrain dt{100, 3, true};
        elsCalStart(&ctx, 1, dt.currentSteps, dt.z);

        const int32_t ceiling = 106;       /* 100 lash + 6 steps = 2 Z counts */
        int32_t stepsToGo = ctx.driveSign * ceiling;
        int32_t guard = 0;
        bool sawFailure = false;
        while (guard++ < 100000) {
            elsCalAction_t act = elsCalTick(&ctx, dt.currentSteps, dt.z, stepsToGo, 2);
            if (act.finished) { sawFailure = (ctx.result != ELS_CAL_OK); break; }
            if (act.startPhase) stepsToGo = act.driveSign * ceiling;
            if (stepsToGo != 0) {
                int32_t dir = (stepsToGo > 0) ? 1 : -1;
                dt.advance(dir);
                stepsToGo -= dir;
            }
        }
        check(!sawFailure, "a machine that responds on the last step is NOT condemned");
        checkEq(ctx.result, ELS_CAL_OK, "result is OK");
    }

    /* ---------------- Threshold of 0 poisons a whole run ----------------- */
    printf("\n-- calibration with an unconfigured threshold --\n");
    {
        elsCalCtx_t ctx{};
        Drivetrain dt{100, 3, true};       /* perfectly healthy machine */
        int32_t ticks = runCal(ctx, dt, 400, /*thresh*/ 0, 1);
        check(ticks > 0, "run terminates");
        checkEq(ctx.result, ELS_CAL_ERR_NO_MOTION,
                "healthy machine + unconfigured threshold REFUSES (fails closed)");
    }

    /* ---------------- Both cutting-direction polarities ------------------ */
    printf("\n-- both cutting directions --\n");
    for (int32_t dir = -1; dir <= 1; dir += 2) {
        elsCalCtx_t ctx{};
        Drivetrain dt{80, 3, true};
        int32_t ticks = runCal(ctx, dt, 400, 2, dir);
        char buf[96];
        snprintf(buf, sizeof buf, "cuttingDir=%+d completes with OK", (int)dir);
        check(ticks > 0 && ctx.result == ELS_CAL_OK, buf);
        snprintf(buf, sizeof buf, "cuttingDir=%+d measures ~80 steps", (int)dir);
        check(ctx.measured[0] >= 80 && ctx.measured[0] <= 92, buf);
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
