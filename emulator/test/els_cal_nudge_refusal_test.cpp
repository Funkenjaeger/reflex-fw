/*
 * ISR-LEVEL regression: a calibration leg must NOT be satisfied by carriage
 * motion that the servo did not produce.
 *
 * WHAT THIS PINS, AND WHY IT IS A SEPARATE FILE FROM ITS SIBLING
 * -------------------------------------------------------------
 * els_cal_isr_attribution_test.cpp pins the case attribution CANNOT decide: a
 * nudge landing INSIDE the settle horizon of a real pulse. Its verdict is that
 * the gap is still open, which is honest but has an awkward property -- that
 * verdict holds both with and without the 2026-08-10 attribution fix, because
 * the exploit it drives is the one the settle horizon deliberately cannot see
 * (els_slip.h, "WHAT THIS DOES NOT DO"). The same is true of the pure-layer
 * KNOWN GAP case in els_backlash_cal_test.cpp.
 *
 * Mutation-verified 2026-08-13: reverting elsCalTick()'s decision from
 * elsSlipConfirmed() back to the bare endpoint test elsZMotionSeen() -- i.e.
 * undoing the fix that separates "did Z move?" from "did Z move BECAUSE WE
 * MOVED IT?" -- left the VERDICT of every pre-existing target intact. Seven of
 * the eight stayed fully green; the eighth (els_cal_isr_attribution_test) went
 * red only on a fixture-sanity line, and for a reason that is a side effect
 * rather than a statement: the mutated leg completes on the nudge one tick
 * sooner, which clears elsCal.armed and so stops Ramps.c ticking the
 * accumulator the sanity line inspects. Its actual verdict still passed. So the
 * fix had no executable statement of what it bought.
 *
 * This file states the part that IS decidable, and is the target whose VERDICT
 * dies when that line is reverted: motion arriving when the servo has NOT
 * pulsed for longer than the settle horizon is not evidence for a leg, and a
 * leg offered only that evidence must stay where it is.
 *
 * THE PAIR IS THE POINT (same convention as els_takeup_confirm_test.cpp)
 * ---------------------------------------------------------------------
 * The SAME shove -- same magnitude, same rig, same leg, same phase, delivered
 * at the same point after arming -- is run in two conditions that differ in
 * exactly one respect, whether the servo was driving when it arrived:
 *
 *   A (must confirm)  drive live: the shove is indistinguishable from the
 *                     inertial settle of a real pulse, and crediting it is what
 *                     the settle horizon is FOR. A healthy slow machine depends
 *                     on this arm passing.
 *   B (must refuse)   drive quiet past the horizon: nothing the servo did can
 *                     account for the motion, so it must not complete the leg.
 *
 * Arm A is not decoration. A test that simply ignored all shoves would satisfy
 * arm B vacuously; A is what forces B's refusal to be caused by the drive
 * context rather than by the shove being invisible to the rig.
 *
 * WHY A REFUSED LEG MATTERS MORE THAN IT LOOKS
 * --------------------------------------------
 * A MEASURE leg fooled by a shove writes a calMeasured[] that is not a lash at
 * all -- it is however far the leadscrew happened to have turned by the time
 * someone leaned on the carriage. That number feeds elsTakeupConfirmThreshold()
 * (els_backlash_cal.h), which is the standard the take-up gate is judged against
 * on every cut thereafter. Poisoning the measurement poisons the gate that was
 * built to catch the same class of fault, which is why the leg is judged here on
 * measured[]/cycle and not on how the overall run eventually terminates.
 *
 * SCOPE -- what this does NOT cover
 * ---------------------------------
 * It does not cover the `&& data->elsCal.armed` gate on Ramps.c's calibration
 * attribution-tick block, and no target in this suite does -- re-verified
 * 2026-08-13 by deleting that clause and watching all 9 targets stay green.
 * That is a property of the code rather than a hole here: elsCalTick() calls
 * elsSlipReset() at the instant a leg arms, so everything the accumulator could
 * collect before arming is discarded, and nothing reads it in between. Dropping
 * the gate is currently unobservable through elsCal's own outputs, so it is
 * defense in depth rather than behavior, and NO test can be written against it
 * without first giving the pre-arm accumulator an observable consumer. Do not
 * read this file's green as cover for that line.
 */

extern "C" {
#include "Ramps.h"
#include "Scales.h"
#include "emulator_state.h"
}

#include <cstdio>
#include <cstdint>
#include <cstring>

extern "C" {

GPIO_TypeDef    emu_gpioa, emu_gpiob, emu_gpioc;
RCC_TypeDef     emu_rcc;
DWT_Type        emu_dwt;
CoreDebug_Type  emu_coreDebug;
EmulatorHardwareState emu_hw;

void emu_log_trace(const char *fmt, ...) { (void)fmt; }
void emu_log_event(const char *fmt, ...) { (void)fmt; }

void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p; (void)i; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) {
    (void)p; (void)pin; (void)s;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *p, uint16_t pin) { (void)p; (void)pin; }

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t ch) {
    (void)h; (void)ch; return HAL_OK;
}
HAL_StatusTypeDef initScaleTimer(TIM_HandleTypeDef *h) { (void)h; return HAL_OK; }

void ModbusInit(modbusHandler_t *m)  { (void)m; }
void ModbusStart(modbusHandler_t *m) { (void)m; }

osStatus_t osDelay(uint32_t ticks) { (void)ticks; return osOK; }
osThreadId_t osThreadNew(osThreadFunc_t f, void *arg, const osThreadAttr_t *a) {
    (void)f; (void)arg; (void)a; return nullptr;
}

extern uint16_t servoCycles;
extern uint16_t servoCyclesCounter;

} /* extern "C" */

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

/* ------------------------------------------------------------------ */
/* Lash + Rig: same shape as els_cal_isr_attribution_test.cpp / the take-up
 * file's fixture. Duplicated rather than shared for the reason stated there --
 * each ISR-level target stays independently buildable and its diff surface
 * stays its own. */

static const int32_t Z_CLEAR = 1000;

/* The shove, in Z counts. ONE constant used by both arms of the pair: if these
 * ever diverge the pair stops being a controlled comparison. Well past the
 * 2-count motion threshold either way. */
static const int32_t SHOVE_COUNTS = 20;

/* How long the servo must sit silent before a shove counts as unattributed.
 * ELS_SLIP_SETTLE_TICKS is private to Ramps.c, so this is deliberately ~20x the
 * value in force there (1000 ticks as of 2026-08-13) rather than derived from
 * it. It is not load-bearing on its own: the "shove landed UNATTRIBUTED"
 * fixture-sanity assertion below is what proves the wait was actually long
 * enough, and it goes RED -- loudly, before the verdict is read -- if the
 * horizon is ever retuned past this. An inadequate wait cannot make the verdict
 * pass for the wrong reason. */
static const int32_t QUIET_TICKS = 20000;

struct Lash {
    int32_t lashSteps      = 60;
    int32_t stepsPerZCount = 3;
    bool    coupled        = true;

    int32_t nutPos   = 0;
    int32_t carriage = 0;
    int32_t prevSteps = 0;

    int32_t follow(int32_t currentSteps, int32_t zBase) {
        int32_t d = currentSteps - prevSteps;
        prevSteps = currentSteps;
        if (!coupled) return zBase + carriage / stepsPerZCount;
        while (d > 0) { nutPos++; if (nutPos > lashSteps) { nutPos = lashSteps; carriage++; } d--; }
        while (d < 0) { nutPos--; if (nutPos < 0)         { nutPos = 0;         carriage--; } d++; }
        return zBase + carriage / stepsPerZCount;
    }
};

struct Rig {
    rampsHandler_t     data;
    TIM_TypeDef        tim[SCALES_COUNT];
    TIM_HandleTypeDef  htim[SCALES_COUNT];
    int32_t            spindleCnt = 0;
    int32_t            zCnt = Z_CLEAR;
    Lash               lash;
    int32_t            zBase = 0;

    void init(int32_t motionThresh, bool coupled, int32_t lashSteps = 60,
              int32_t ceilingSteps = 400) {
        std::memset(&data, 0, sizeof(data));
        std::memset(tim,  0, sizeof(tim));
        std::memset(htim, 0, sizeof(htim));
        spindleCnt = 0;
        zCnt       = Z_CLEAR;
        lash = Lash{};
        lash.lashSteps = lashSteps;
        lash.coupled   = coupled;
        servoCycles        = 1;
        servoCyclesCounter = 0;

        for (int i = 0; i < SCALES_COUNT; i++) {
            htim[i].Instance = &tim[i];
            ramps_timer_handles[i] = &htim[i];
            data.shared.scales[i].timerHandleSlot = (uint32_t)i;
            data.shared.scales[i].scaleDir     = 1;
            data.shared.scales[i].syncRatioNum = 1;
            data.shared.scales[i].syncRatioDen = 100;
            data.shared.scales[i].syncEnable   = 0;
        }
        data.shared.scales[0].syncRatioNum = -2;
        data.shared.scales[0].syncRatioDen = 15;
        data.shared.scales[0].syncEnable   = 1;

        data.shared.servo.maxSpeed     = 100000.0f;
        data.shared.servo.acceleration = 50000.0f;
        data.shared.servo.servoDir     = 1;
        data.shared.fastData.servoMode = 1;

        data.shared.elsStop.scaleIndex            = 1;
        data.shared.elsStop.threadPitchSteps      = 533.333f;
        data.shared.elsStop.zCountsPerPitch       = 846.667f;
        data.shared.elsStop.hysteresis            = 500;
        data.shared.elsStop.calMotionThreshCounts = motionThresh;
        data.shared.elsStop.calCeilingSteps       = ceilingSteps;
        data.shared.elsStop.enable                = 0;

        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        data.scalesDeltaPos[0].position = spindleCnt;
        data.scalesDeltaPos[1].position = zCnt;
        data.shared.scales[0].position  = spindleCnt;
        data.shared.scales[1].position  = zCnt;
        data.scalesSyncDeltaPos[0].oldPosition = spindleCnt;
        data.scalesSyncDeltaPos[1].oldPosition = zCnt;

        zBase = zCnt;
        lash.prevSteps = 0;
        lash.carriage  = 0;
    }

    /* One ISR tick. Z is whatever the lash model produces from the servo's own
     * pulses, offset by whatever the operator has shoved into zBase. The spindle
     * keeps turning throughout and is irrelevant to the measurement: Ramps.c
     * suspends sync outright while a calibration is running (see the
     * "Sync paused ... while a backlash CALIBRATION is running" gate), so it can
     * neither help nor poison a leg. */
    void step() {
        spindleCnt += 1;
        zCnt = lash.follow((int32_t)data.shared.servo.currentSteps, zBase);
        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;
        SynchroRefreshTimerIsr(&data);
    }

    void steps(int32_t n) { for (int32_t i = 0; i < n; i++) step(); }

    /* Operator pushing the carriage by hand: Z moves with no servo pulse behind
     * it. Deliberately independent of the lash model -- the nut does not move,
     * which is what makes this a shove rather than a drive. */
    void shove(int32_t zCounts) { zBase += zCounts; }

    /* Silence the servo WITHOUT ending the leg.
     *
     * maxSpeed 0 with currentSpeed 0 makes updateIndexingPosition() take neither
     * the accelerate branch (currentSpeed < maxSpeed is false) nor the
     * decelerate branch (stepsToGo < stopDistance is false, stopDistance being
     * 0), so positionIncrement stays 0: no pulses, and -- the part that matters
     * -- servo.stepsToGo is not drained, so elsCalTick() never sees the
     * exhaustion that would fail the leg out from under the test. The leg simply
     * sits armed and undecided, which is exactly the state a stalled drive
     * leaves it in. */
    void silenceServo() {
        data.shared.servo.maxSpeed     = 0.0f;
        data.shared.servo.currentSpeed = 0.0f;
    }

    /* Run the leg forward until it arms, i.e. until the servo has actually
     * pulsed in the commanded direction and elsCalTick() has baselined the
     * attribution window. Returns false if it never got there. */
    bool runToArmed(int32_t maxTicks = 5000) {
        for (int32_t i = 0; i < maxTicks && !data.elsCal.armed; i++) step();
        return data.elsCal.armed != 0;
    }

    /* Run until a step pulse has landed SINCE the leg armed.
     *
     * Not the same as armed, and the difference is load-bearing for this test.
     * elsCalTick() arms one tick AFTER the pulse it arms on (it compares
     * currentSteps against LAST tick's), and elsSlipReset() then sets
     * ticksSinceLastPulse to ELS_SLIP_NEVER_DRIVEN -- deliberately not a large
     * finite age, so nothing that predates the leg can be credited to it. Pulses
     * early in an acceleration ramp are hundreds of ticks apart, so for that
     * whole span the accumulator credits NOTHING, and a shove delivered there is
     * unattributed no matter what the servo is doing. Both arms of the pair
     * therefore run to here first: it is the earliest state in which "the drive
     * is live" is true of the accumulator and not just of the servo. */
    bool runToDriveCredited(int32_t maxTicks = 5000) {
        for (int32_t i = 0; i < maxTicks && data.elsCal.slip.ticksSinceLastPulse != 0; i++)
            step();
        return data.elsCal.slip.ticksSinceLastPulse == 0;
    }

    bool runToPhase(uint16_t phase, int32_t maxTicks = 60000) {
        for (int32_t i = 0; i < maxTicks && data.elsCal.phase != phase; i++) step();
        return data.elsCal.phase == phase;
    }

    void startRun() {
        data.shared.elsStop.calCommand = 1;
        step();                    /* request intake tick: starts the run */
    }
};

/* Advance a healthy coupled rig to a MEASURE leg that has armed AND is being
 * actively driven -- the single state both arms of the pair are judged from, so
 * that the only thing separating them downstream is whether the drive then goes
 * quiet. Returns false if the rig could not get there, which is a harness
 * failure, not a verdict.
 *
 * The leg is reached the honest way: SEAT is completed by the servo genuinely
 * crossing the lash, and MEASURE arms on the real reversal. Nothing is poked
 * into elsCal directly. */
static bool armMeasureLeg(Rig &rig) {
    rig.init(/*motionThresh*/ 2, /*coupled*/ true, /*lashSteps*/ 60);
    rig.startRun();
    if (rig.data.elsCal.phase != ELS_CAL_SEAT) return false;
    if (!rig.runToArmed())                     return false;   /* SEAT arms */
    if (!rig.runToPhase(ELS_CAL_MEASURE))      return false;   /* SEAT completes on real lash */
    if (!rig.runToArmed())                     return false;   /* MEASURE arms on the reversal */
    if (!rig.runToDriveCredited())             return false;   /* a pulse has landed since */
    return true;
}

int main() {
    printf("=== ELS calibration leg: unattributed carriage motion must not satisfy it ===\n\n");
    printf("Drives data->elsCal through the real SynchroRefreshTimerIsr().\n");
    printf("The SAME %d-count shove is delivered to the SAME armed MEASURE leg\n",
           (int)SHOVE_COUNTS);
    printf("in two conditions; only 'was the servo driving?' differs.\n\n");

    /* ---------------------------------------------------------------- */
    /* Positive control: the rig can complete a calibration at all.       */
    printf("-- control: a healthy coupled drivetrain completes and measures the true lash --\n");
    {
        Rig rig;
        rig.init(/*motionThresh*/ 2, /*coupled*/ true, /*lashSteps*/ 60);
        rig.startRun();
        uint16_t seq0 = rig.data.shared.elsStop.calSeq;
        for (int i = 0; i < 200000 && rig.data.shared.elsStop.calSeq == seq0; i++) rig.step();

        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_OK,
                "calResult OK on a healthy coupled drivetrain");
        printf("      measured = %d, %d, %d (true lash 60)\n",
               (int)rig.data.shared.elsStop.calMeasured[0],
               (int)rig.data.shared.elsStop.calMeasured[1],
               (int)rig.data.shared.elsStop.calMeasured[2]);
        for (int i = 0; i < ELS_CAL_CYCLES; i++) {
            int32_t m = rig.data.shared.elsStop.calMeasured[i];
            check(m >= 55 && m <= 75, "measurement recovers the true lash (rig is honest)");
        }
    }

    /* ---------------------------------------------------------------- */
    /* ARM A -- drive live. The shove MUST be credited.                   */
    printf("\n-- ARM A: shove arrives while the servo is DRIVING -> leg confirms --\n");
    printf("   (indistinguishable from the inertial settle of a real pulse, and\n");
    printf("    crediting it is what the settle horizon exists to do -- a healthy\n");
    printf("    slow machine depends on this arm passing)\n");
    {
        Rig rig;
        check(armMeasureLeg(rig), "reached an armed, actively-driven MEASURE leg (harness sanity)");

        uint16_t cycle0 = rig.data.elsCal.cycle;
        checkEq(rig.data.elsCal.phase, ELS_CAL_MEASURE, "leg is in MEASURE");
        checkEq(cycle0, 0, "no measurement recorded yet");

        rig.shove(SHOVE_COUNTS);
        rig.steps(5);

        printf("      after shove+5 ticks: phase=%d cycle=%d attributed=%lld unattributed=%lld"
               " ticksSinceLastPulse=%d\n",
               (int)rig.data.elsCal.phase, (int)rig.data.elsCal.cycle,
               (long long)rig.data.elsCal.slip.attributedZCounts,
               (long long)rig.data.elsCal.slip.unattributedZCounts,
               (int)rig.data.elsCal.slip.ticksSinceLastPulse);

        check(rig.data.elsCal.slip.attributedZCounts != 0,
              "the shove was credited as ATTRIBUTED (servo had pulsed recently)");
        checkEq((int32_t)rig.data.elsCal.slip.unattributedZCounts, 0,
                "...and nothing landed in the unattributed bucket");
        checkEq(rig.data.elsCal.cycle, cycle0 + 1,
                "the leg CONFIRMED on it: a measurement was recorded");
    }

    /* ---------------------------------------------------------------- */
    /* ARM B -- drive quiet. The shove MUST be refused. This is the        */
    /* assertion the mutation kills.                                      */
    printf("\n-- ARM B: identical shove, servo SILENT past the settle horizon -> leg refuses --\n");
    {
        Rig rig;
        check(armMeasureLeg(rig), "reached an armed, actively-driven MEASURE leg (harness sanity)");

        uint16_t phase0    = rig.data.elsCal.phase;
        uint16_t cycle0    = rig.data.elsCal.cycle;
        int32_t  stepsRef0 = rig.data.elsCal.stepsRef;
        checkEq(phase0, ELS_CAL_MEASURE, "leg is in MEASURE");
        checkEq(cycle0, 0, "no measurement recorded yet");

        /* Stall the drive, let it coast to a stop, then sit silent. */
        rig.silenceServo();
        rig.steps(10);
        int32_t stepsAtSilence = (int32_t)rig.data.shared.servo.currentSteps;
        int32_t toGoAtSilence  = rig.data.shared.servo.stepsToGo;
        rig.steps(QUIET_TICKS);

        checkEq((int32_t)rig.data.shared.servo.currentSteps, stepsAtSilence,
                "servo issued NO pulses during the quiet period (fixture sanity)");
        check(rig.data.shared.servo.stepsToGo == toGoAtSilence
              && toGoAtSilence != 0,
              "the leg is still live and undecided: stepsToGo intact, not exhausted");
        checkEq(rig.data.elsCal.phase, phase0,
                "silence alone did not move the leg");

        /* THE SHOVE -- byte-identical to arm A's. */
        rig.shove(SHOVE_COUNTS);
        rig.steps(5);

        printf("      after shove+5 ticks: phase=%d cycle=%d attributed=%lld unattributed=%lld"
               " ticksSinceLastPulse=%d\n",
               (int)rig.data.elsCal.phase, (int)rig.data.elsCal.cycle,
               (long long)rig.data.elsCal.slip.attributedZCounts,
               (long long)rig.data.elsCal.slip.unattributedZCounts,
               (int)rig.data.elsCal.slip.ticksSinceLastPulse);

        /* Fixture sanity FIRST, and it must be loud. If the shove never reached
         * the accumulator, or reached it inside the settle horizon because
         * QUIET_TICKS was no longer long enough, the verdict below would pass
         * for a reason that has nothing to do with attribution. These two say
         * "the evidence exists, and it is the RIGHT KIND of evidence" before the
         * verdict is allowed to mean anything. */
        checkEq((int32_t)rig.data.elsCal.slip.unattributedZCounts, SHOVE_COUNTS,
                "the shove landed and was bucketed UNATTRIBUTED (fixture sanity)");
        checkEq((int32_t)rig.data.elsCal.slip.attributedZCounts, 0,
                "nothing was credited to the servo, which issued no pulses");

        /* THE VERDICT. Under the pre-2026-08-10 bare endpoint test
         * (elsZMotionSeen(zPos, ctx->zRef, ...)) every one of these flips: the
         * leg advances out of MEASURE, re-baselines stepsRef, and writes a
         * measured[] that is not a lash. */
        checkEq(rig.data.elsCal.cycle, cycle0,
                "VERDICT: no measurement was recorded from unattributed motion");
        checkEq(rig.data.elsCal.measured[cycle0], 0,
                "VERDICT: measured[] was not written");
        checkEq(rig.data.elsCal.phase, phase0,
                "VERDICT: the leg did not advance");
        checkEq(rig.data.elsCal.stepsRef, stepsRef0,
                "VERDICT: the leg's step baseline was not re-taken");
        checkEq((int32_t)rig.data.shared.elsStop.calSeq, 0,
                "VERDICT: the run did not publish a result off the back of a shove");
    }

    /* ---------------------------------------------------------------- */
    /* Selectivity control: a sub-threshold shove delivered with the drive */
    /* LIVE must not confirm either -- proves arm A's pass is the          */
    /* threshold being cleared, not "any shove while driving confirms".    */
    printf("\n-- control: sub-threshold shove with the drive live does NOT confirm --\n");
    {
        Rig rig;
        rig.init(/*motionThresh*/ 10, /*coupled*/ true, /*lashSteps*/ 60);
        rig.startRun();
        check(rig.runToArmed(), "SEAT leg armed (harness sanity)");
        if (!rig.runToPhase(ELS_CAL_MEASURE)) check(false, "reached MEASURE (harness sanity)");
        check(rig.runToArmed(), "MEASURE leg armed (harness sanity)");
        check(rig.runToDriveCredited(), "MEASURE leg is being actively driven (harness sanity)");

        uint16_t cycle0 = rig.data.elsCal.cycle;
        rig.shove(3);                 /* below calMotionThreshCounts = 10 */
        rig.steps(5);
        checkEq(rig.data.elsCal.cycle, cycle0,
                "3 counts does not clear a 10-count threshold, driving or not");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
