/*
 * Unit tests for the pure motion-attribution layer (Core/Inc/els_slip.h). No
 * HAL, no ISR, no emulator physics — this drives the accumulator one tick at a
 * time, which is the only way to express the tick patterns that matter.
 *
 * WHY THIS FILE EXISTS ALONGSIDE els_takeup_confirm_test
 * -----------------------------------------------------
 * That file proves the WIRING: that the ISR feeds this accumulator fresh deltas
 * at the right point and that the gate acts on the result. It cannot prove the
 * timing properties, because the emulator's lash model moves the carriage
 * INSTANTANEOUSLY with the pulse — every Z count it produces lands one tick
 * after a pulse, so every settle horizon down to 1 tick behaves identically
 * there. A real drivetrain does not do that; it is the entire reason a settle
 * horizon exists.
 *
 * So the horizon's two failure modes are only reachable here:
 *
 *   - TOO LONG:  an uncorrelated shove gets credited to the servo. This is the
 *                2026-08-08 hardware defect, and it is the failure everyone
 *                thinks about.
 *   - TOO SHORT: a HEALTHY machine refuses to start. Step pulses are paced, so
 *                at any real take-up speed most ticks of a perfectly good drive
 *                burst emit nothing. A horizon shorter than that pacing punches
 *                holes through the middle of the drive and genuine coupled
 *                motion is discarded as "not ours". Silent, speed-dependent,
 *                and it looks exactly like a mechanical fault.
 *
 * The second is the one with no natural alarm — nothing in a refusal says
 * "because the constant was smaller than servoCycles" — so elsSlipSettleTicks()
 * exists to make it unreachable, and the cases below are what prove that guard
 * is load-bearing rather than decorative.
 *
 * Mutation-tested: see the MUTATION note on each case for the exact edit that
 * must make it fail. A test that cannot fail is worse than no test, because it
 * reads as coverage.
 */
#include <cstdio>
#include <cstdint>

#include "els_slip.h"

static int failures = 0;

static void check(bool cond, const char *what) {
    printf("   %-72s %s\n", what, cond ? "ok" : "FAIL");
    if (!cond) failures++;
}

static void checkEq(int64_t got, int64_t want, const char *what) {
    bool ok = (got == want);
    printf("   %-72s %s (got %lld, want %lld)\n", what, ok ? "ok" : "FAIL",
           (long long)got, (long long)want);
    if (!ok) failures++;
}

/* A paced drive burst, the way the ISR actually emits one: a pulse every
 * `period` ticks and nothing in between. `zPerTick` is delivered on the tick
 * given by `zLagTicks` after each pulse, which is where the model gets to say
 * how promptly the carriage responds. */
static void driveBurst(elsSlipAccum_t *a, int32_t pulses, int32_t period,
                       int32_t zPerPulse, int32_t zLagTicks, int32_t settleTicks)
{
    for (int32_t p = 0; p < pulses; p++) {
        for (int32_t t = 0; t < period; t++) {
            int32_t dServo = (t == 0) ? 1 : 0;
            int32_t dZ     = (t == zLagTicks) ? zPerPulse : 0;
            elsSlipTick(a, dZ, dServo, settleTicks);
        }
    }
}

static void coast(elsSlipAccum_t *a, int32_t ticks, int32_t settleTicks)
{
    for (int32_t t = 0; t < ticks; t++) elsSlipTick(a, 0, 0, settleTicks);
}

int main() {
    printf("=== ELS motion attribution (pure layer) ===\n\n");

    /* ---------------- Nothing is credited before the first pulse ------ */
    printf("-- motion before the first pulse is never attributed --\n");
    {
        /* MUTATION: initialise ticksSinceLastPulse to 0 instead of
         * ELS_SLIP_NEVER_DRIVEN in elsSlipReset() and this goes red. A take-up
         * would then start out crediting itself with whatever the carriage was
         * already doing, which at a stop-and-resume is exactly the operator
         * repositioning it by hand. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        for (int i = 0; i < 100; i++) elsSlipTick(&a, 5, 0, 1000);

        checkEq(a.attributedZCounts, 0, "no pulse yet, so nothing is our doing");
        checkEq(a.unattributedZCounts, 500, "...and the motion is still counted, as unattributed");
        check(!elsSlipConfirmed(&a, 2), "a take-up cannot confirm on it");
    }

    /* ---------------- The 2026-08-08 defect, at tick resolution ------- */
    printf("\n-- an uncorrelated shove long after the last pulse is not evidence --\n");
    {
        /* MUTATION: drop the `ticksSinceLastPulse <= settleTicks` test in
         * elsSlipTick() (attribute everything) and this goes red. This is the
         * hardware defect in one assertion. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        driveBurst(&a, /*pulses*/ 30, /*period*/ 4, /*zPerPulse*/ 1, /*lag*/ 1, /*settle*/ 100);
        checkEq(a.attributedZCounts, 30, "the drive burst itself is fully attributed");

        coast(&a, 5000, 100);            /* servo long since stopped */
        elsSlipTick(&a, 200, 0, 100);    /* the handwheel */
        coast(&a, 10, 100);

        checkEq(a.unattributedZCounts, 200, "the shove is seen");
        checkEq(a.attributedZCounts, 30, "...and credited to nobody");
        check(!elsSlipConfirmed(&a, 100), "200 counts of hand motion does not confirm a 100-count floor");
        check(elsSlipConfirmed(&a, 30), "...while the 30 counts we actually drove still do");
    }

    /* ---------------- Genuine settle still counts -------------------- */
    printf("\n-- motion still arriving shortly after the last pulse IS attributed --\n");
    {
        /* The carriage does not stop dead when the servo does. A horizon that
         * rejected this would be an instant latch, which is the other unsafe
         * extreme (see ELS_TAKEUP_CONFIRM_WINDOW_TICKS in Ramps.c).
         *
         * MUTATION: change `<=` to `<` in elsSlipTick()'s horizon test and the
         * boundary case below goes red. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        driveBurst(&a, 10, 4, 1, 1, /*settle*/ 100);
        coast(&a, 60, 100);              /* inertia/compliance, well inside */
        elsSlipTick(&a, 7, 0, 100);
        checkEq(a.attributedZCounts, 17, "late-but-settling motion counts");

        /* The boundary, pinned from both sides. Neither assertion alone
         * distinguishes `<=` from `<` — only the pair does. */
        elsSlipAccum_t b;
        elsSlipReset(&b);
        elsSlipTick(&b, 0, 1, 100);      /* pulse: age 0 */
        coast(&b, 99, 100);              /* ages 1..99 */
        elsSlipTick(&b, 3, 0, 100);      /* age 100 — exactly ON the horizon */
        checkEq(b.attributedZCounts, 3, "exactly ON the horizon still counts (<=, not <)");

        elsSlipAccum_t c;
        elsSlipReset(&c);
        elsSlipTick(&c, 0, 1, 100);      /* pulse: age 0 */
        coast(&c, 100, 100);             /* ages 1..100 */
        elsSlipTick(&c, 3, 0, 100);      /* age 101 — one tick past */
        checkEq(c.attributedZCounts, 0, "one tick past the horizon is already outside it");
        checkEq(c.unattributedZCounts, 3, "and lands in the unattributed bucket");
    }

    /* ---------------- The horizon floor: a healthy machine must start - */
    printf("\n-- a paced drive burst is attributed WHOLE, not sampled --\n");
    {
        /* This is the too-short failure. Pulses every 50 ticks, and the carriage
         * responds late within each pulse interval — quantization alone does
         * this: one Z count is ~2.5 servo steps on elspi, so counts arrive on
         * their own schedule, not the pulse train's.
         *
         * MUTATION: make elsSlipSettleTicks() return configuredTicks unchanged
         * (drop the floor) and this goes red — a healthy, correctly coupled
         * take-up stops confirming, at low speed only. Nothing else in the suite
         * catches that; the ISR fixture cannot, because its lash model responds
         * instantly. */
        const int32_t period    = 50;
        const int32_t configured = 10;    /* deliberately far below the pacing */
        const int32_t settle     = elsSlipSettleTicks(configured, period);

        checkEq(settle, period + 1, "the horizon is floored at the live pulse pacing");

        elsSlipAccum_t a;
        elsSlipReset(&a);
        driveBurst(&a, /*pulses*/ 20, period, /*zPerPulse*/ 1, /*lag*/ period - 1, settle);

        checkEq(a.attributedZCounts, 20, "every count of a healthy burst is credited");
        checkEq(a.unattributedZCounts, 0, "none of it is thrown away as 'not ours'");
        check(elsSlipConfirmed(&a, 11), "so the take-up confirms, as it must");
    }

    printf("\n-- the floor never SHORTENS a deliberately generous horizon --\n");
    {
        checkEq(elsSlipSettleTicks(1000, 50), 1000, "configured horizon wins when it is larger");
        checkEq(elsSlipSettleTicks(1000, 0),  1000, "servoCycles == 0 (boot window) does not zero it");
        checkEq(elsSlipSettleTicks(0, 0),     1,    "and a zero horizon still admits the pulse tick itself");
    }

    /* ---------------- Fail closed, and stay a lower bound ------------- */
    printf("\n-- confirmation conventions inherited from elsZMotionSeen --\n");
    {
        /* MUTATION: change `threshCounts <= 0` to `< 0` in elsSlipConfirmed()
         * and the first case goes red — an uncommissioned machine would silently
         * return to open-loop behaviour, which is the original defect. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        driveBurst(&a, 100, 2, 1, 1, 1000);

        check(!elsSlipConfirmed(&a, 0), "an unconfigured threshold FAILS CLOSED");
        check(!elsSlipConfirmed(&a, -5), "so does a negative one");
        check(elsSlipConfirmed(&a, 100), "exactly at the floor confirms (>=, not >)");
        check(elsSlipConfirmed(&a, 4),
              "and far past it still confirms — LOWER BOUND, never a window");
    }

    printf("\n-- attribution is polarity-free, the diagnostic is not --\n");
    {
        /* elsSlipConfirmed() inherits elsZMotionSeen()'s magnitude-only rule on
         * purpose (els_backlash_cal.h explains why detection is polarity-free);
         * elsSlipAttributedAlong() is what makes wrong-way motion legible.
         *
         * MUTATION: drop the abs() in elsSlipConfirmed() and the first case goes
         * red for a machine whose Z scale simply counts the other way. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        driveBurst(&a, 40, 2, -1, 1, 1000);      /* carriage running the other way */

        check(elsSlipConfirmed(&a, 40), "magnitude confirms regardless of scale polarity");
        checkEq(elsSlipAttributedAlong(&a, +1), -40, "...and the signed view says WRONG WAY");
        checkEq(elsSlipAttributedAlong(&a, -1),  40, "...which flips with droSign, as it must");
    }

    /* ---------------- Width: the reason these are int64_t ------------- */
    printf("\n-- the accumulator survives a cumulative consumer --\n");
    {
        /* The take-up gate's own worst case fits in 32 bits. ELS auto-start
         * (6a63f3e3) does not: it accumulates cumulatively with no window to
         * bound it. A wrap would not fail loudly — it would produce a confident
         * confirmation with the wrong sign.
         *
         * MUTATION: narrow attributedZCounts to int32_t and this goes red. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        a.ticksSinceLastPulse = 0;
        for (int i = 0; i < 200000; i++) elsSlipTick(&a, 30000, 1, 1000);

        checkEq(a.attributedZCounts, (int64_t)200000 * 30000,
                "6e9 counts accumulate exactly, well past the int32 ceiling");
        checkEq(a.sumServoSteps, 200000, "and the step sum tracks alongside it");
        check(elsSlipAttributedAlong(&a, 1) == INT32_MAX,
              "the published 32-bit view saturates rather than wrapping");
    }

    /* ---------------- The age counter cannot wrap into "recent" ------- */
    printf("\n-- an undriven accumulator never ages back into 'recently driven' --\n");
    {
        /* MUTATION: remove the `< ELS_SLIP_NEVER_DRIVEN` guard on the increment
         * in elsSlipTick() and ticksSinceLastPulse overflows INT32_MAX — signed
         * overflow, undefined behaviour, and the plausible outcome is a wrap to
         * a small age, i.e. a machine that starts attributing hand motion again
         * after sitting idle. */
        elsSlipAccum_t a;
        elsSlipReset(&a);
        coast(&a, 1000, 100);
        checkEq(a.ticksSinceLastPulse, ELS_SLIP_NEVER_DRIVEN,
                "the never-driven age saturates instead of counting up");
        elsSlipTick(&a, 500, 0, 100);
        checkEq(a.attributedZCounts, 0, "so nothing is ever credited to a servo that never pulsed");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
