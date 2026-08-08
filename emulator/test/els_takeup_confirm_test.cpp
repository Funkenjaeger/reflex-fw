/*
 * ISR-LEVEL tests for the closed-loop backlash take-up confirmation and the
 * calibration driver. Drives the real SynchroRefreshTimerIsr() against a
 * simulated drivetrain with lash.
 *
 * WHY THIS EXISTS ALONGSIDE els_backlash_cal_test
 * -----------------------------------------------
 * That test covers the pure decision layer (els_backlash_cal.h). This one
 * covers the wiring: that the ISR actually captures the Z baseline at take-up
 * initiation, actually withholds completion when the carriage does not respond,
 * actually publishes result/seq codes, and that the enable escape hatch really
 * releases a withheld take-up. None of that is exercised by the pure tests, and
 * none of it was exercised by els_stop_resume_relatch_test either — that
 * scenario runs exactly ONE ISR pass after the resume and asserts on take-up
 * INITIATION, so the completion path had no coverage at all before this file.
 *
 * THE PROPERTY THAT MATTERS MOST
 * ------------------------------
 * The take-up must NOT report complete just because the firmware finished
 * issuing the pulses it decided to issue. If the half-nut is open, currentSteps
 * crosses the target exactly on schedule and the old code called
 * applyPhaseCorrection on a Z snapshot from a drivetrain that was never
 * coupled. The coupled/uncoupled pair below is what distinguishes a real gate
 * from a gate-shaped comment.
 */

extern "C" {
#include "Ramps.h"
#include "Scales.h"
#include "emulator_state.h"
}

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>

/* ------------------------------------------------------------------ */
/* Stubs for every Ramps.c external (same set as the relatch test)      */
/* ------------------------------------------------------------------ */
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
    printf("   %-70s %s\n", what, cond ? "ok" : "FAIL");
    if (!cond) failures++;
}

static void checkEq(int32_t got, int32_t want, const char *what) {
    bool ok = (got == want);
    printf("   %-70s %s (got %d, want %d)\n", what, ok ? "ok" : "FAIL",
           (int)got, (int)want);
    if (!ok) failures++;
}

/* ------------------------------------------------------------------ */
/* Fixture: the relatch rig plus a lash model driven by real step pulses */
/* ------------------------------------------------------------------ */

static const int32_t SPINDLE_PER_PASS = 40;
static const int32_t Z_CLEAR          = 1000;
static const int32_t Z_PAST           = -5;
static const int32_t Z_STOP_POS       = 0;
static const int16_t Z_STOP_DIR       = -1;
static const float   SENTINEL         = 1.0e30f;

/* Lash: the leadscrew traverses `lashSteps` before the carriage moves at all.
 * `coupled = false` is an open half-nut — pulses go out, Z never responds. */
struct Lash {
    int32_t lashSteps      = 60;
    int32_t stepsPerZCount = 3;
    bool    coupled        = true;

    int32_t nutPos   = 0;
    int32_t carriage = 0;
    int32_t prevSteps = 0;

    /* Follow the servo's actual pulse output (currentSteps), not anything the
     * test decides — that is what makes this a wiring test. */
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
    bool               lashDriven = false;   /* once true, Z follows the servo */
    int32_t            zBase = 0;

    void init(uint32_t backlashSteps, int32_t motionThresh, bool coupled = true,
              int32_t lashSteps = 60) {
        std::memset(&data, 0, sizeof(data));
        std::memset(tim,  0, sizeof(tim));
        std::memset(htim, 0, sizeof(htim));
        spindleCnt = 0;
        zCnt       = Z_CLEAR;
        lash = Lash{};
        lash.lashSteps = lashSteps;
        lash.coupled   = coupled;
        lashDriven = false;
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

        data.shared.elsStop.scaleIndex           = 1;
        data.shared.elsStop.stopPosition         = Z_STOP_POS;
        data.shared.elsStop.stopDirection        = Z_STOP_DIR;
        data.shared.elsStop.threadPitchSteps     = 533.333f;
        data.shared.elsStop.zCountsPerPitch      = 846.667f;
        data.shared.elsStop.backlashSteps        = backlashSteps;
        data.shared.elsStop.hysteresis           = 500;
        data.shared.elsStop.calMotionThreshCounts = motionThresh;
        data.shared.elsStop.calCeilingSteps      = 400;
        data.shared.elsStop.enable               = 0;

        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        data.scalesDeltaPos[0].position = spindleCnt;
        data.scalesDeltaPos[1].position = zCnt;
        data.shared.scales[0].position  = spindleCnt;
        data.shared.scales[1].position  = zCnt;
        data.scalesSyncDeltaPos[0].oldPosition = spindleCnt;
        data.scalesSyncDeltaPos[1].oldPosition = zCnt;
    }

    void step(int32_t zTarget) {
        spindleCnt += SPINDLE_PER_PASS;
        zCnt        = zTarget;
        tim[0].CNT  = (uint32_t)spindleCnt;
        tim[1].CNT  = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;
        SynchroRefreshTimerIsr(&data);
    }

    /* Hand Z over to the lash model: from here Z is whatever the servo's own
     * pulses produce through the drivetrain. */
    void beginLashDriven() {
        lashDriven = true;
        zBase = zCnt;
        lash.prevSteps = (int32_t)data.shared.servo.currentSteps;
        lash.carriage = 0;
        /* Seat the nut against the wall the take-up drives AWAY from, so the
         * take-up must traverse the full lash before the carriage moves. That is
         * the worst case and the only one that exercises the thing under test.
         * Starting at nutPos = 0 (the take-up direction's own wall) makes the
         * carriage move on the very first step, so every take-up "confirms"
         * regardless of lash — a fixture that cannot fail. Cutting direction
         * here is negative (syncRatioNum < 0), so the far wall is lashSteps.
         * Mirrors the emulator's own half-nut model, which deliberately drops
         * the nut on the opposite wall for the same reason. */
        lash.nutPos = lash.lashSteps;
    }

    /* Move the carriage WITHOUT the servo driving it — an operator pushing it
     * with the half-nut open. This is the degree of freedom the production
     * emulator still lacks (see reflex-fw todo.md), and its absence is why the
     * hardware defect below was unreachable by test: the model could express
     * "coupled" and "never moves", but not "moving for a reason that isn't us". */
    void nudgeCarriage(int32_t zCounts) {
        zBase += zCounts;
    }

    void stepDriven() {
        spindleCnt += SPINDLE_PER_PASS;
        zCnt = lash.follow((int32_t)data.shared.servo.currentSteps, zBase);
        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;
        SynchroRefreshTimerIsr(&data);
    }

    /* Arm a job, trigger the stop, then resume — the point at which the
     * firmware initiates a backlash take-up. */
    void armAndTrigger() {
        step(Z_CLEAR);
        data.shared.elsStop.enable = 1;
        for (int i = 0; i < 3; i++) step(Z_CLEAR);
        step(Z_PAST);
        step(Z_PAST);
        step(Z_CLEAR);                    /* retract clear so the resume edge lands */

        /* Let the pulse generator catch up before resuming. Sync is gated while
         * active == 1, so desiredSteps is static and currentSteps closes the
         * gap. Without this the take-up inherits a large pulse BACKLOG and the
         * carriage moves much further than the take-up commanded, which would
         * mask exactly the under-motion this file tests for.
         *
         * The backlog is a FIXTURE artifact, not firmware behaviour: this rig
         * advances the spindle 40 counts per tick against a 2/15 ratio (~5.3
         * steps/tick) while the generator emits 1 pulse/tick, so desiredSteps
         * runs away. On a real machine sync is gated for the whole retract and
         * the operator takes seconds to press Cut, so the servo is long settled. */
        for (int i = 0; i < 20000
             && data.shared.servo.currentSteps != data.shared.servo.desiredSteps; i++) {
            step(Z_CLEAR);
        }

        data.shared.elsStop.lastIdealAdvance = SENTINEL;
        data.shared.elsStop.active = 0;   /* SW resume */
    }
};

/* ------------------------------------------------------------------ */

int main() {
    printf("=== ELS take-up Z confirmation + calibration (ISR level) ===\n\n");

    /* ---------------- Coupled: take-up completes -------------------- */
    printf("-- coupled drivetrain: take-up confirms and phase correction runs --\n");
    {
        Rig rig;
        rig.init(/*backlashSteps*/ 90, /*motionThresh*/ 2, /*coupled*/ true, /*lash*/ 60);
        rig.armAndTrigger();
        rig.step(Z_CLEAR);                       /* initiates the take-up */
        check(rig.data.shared.elsStop.takeupPending == 1, "take-up initiated");
        rig.beginLashDriven();

        for (int i = 0; i < 60000 && rig.data.shared.elsStop.takeupPending; i++)
            rig.stepDriven();

        checkEq(rig.data.shared.elsStop.takeupPending, 0, "take-up completed");
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_CAL_OK, "takeupResult is OK");
        check(rig.data.shared.elsStop.takeupSeq >= 1, "takeupSeq incremented");
        check(rig.data.shared.elsStop.lastIdealAdvance != SENTINEL,
              "applyPhaseCorrection ran");
        check(rig.data.shared.elsStop.lastTakeupZDelta != 0,
              "lastTakeupZDelta recorded real motion");
    }

    /* ---------------- Uncoupled: THE point of the feature ------------ */
    printf("\n-- OPEN HALF-NUT: take-up is withheld, correction never runs --\n");
    {
        /* MUTATION: delete the elsZMotionSeen() branch in the take-up block (i.e.
         * always complete) and every assertion here flips — which is precisely
         * the pre-fix behaviour this file exists to pin. */
        Rig rig;
        rig.init(90, 2, /*coupled*/ false, 60);
        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        check(rig.data.shared.elsStop.takeupPending == 1, "take-up initiated");
        rig.beginLashDriven();

        for (int i = 0; i < 60000; i++) rig.stepDriven();

        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "takeupResult reports UNCONFIRMED");
        check(rig.data.shared.elsStop.lastIdealAdvance == SENTINEL,
              "applyPhaseCorrection did NOT run on an uncoupled drivetrain");
        checkEq(rig.data.shared.elsStop.takeupSeq, 1,
                "takeupSeq reported the outcome ONCE, not once per tick");

        /* The pass ABORTS back to the stopped state once the confirmation
         * window closes. Holding the machine was the earlier behaviour and it
         * was worse than useless: the only way out was the enable toggle, which
         * clears referenceLatched on re-engage — so recovering from a FALSE
         * alarm cost the operator their thread phase reference. */
        checkEq(rig.data.shared.elsStop.takeupPending, 0, "not holding the machine");
        checkEq(rig.data.shared.elsStop.active, 1, "back to stopped-at-shoulder");
        checkEq(rig.data.shared.elsStop.referenceLatched, 1,
                "phase reference preserved: retry is free");
    }

    /* ---------------- Partial engagement: moves, but not enough ------ */
    /* The pair below is the whole point of the derived threshold. BOTH cases
     * clear the bare 2-count detection floor, so both would have been confirmed
     * under a floor-only rule. Only the second is actually healthy.
     *
     * Geometry is made self-consistent here: the rig's default registers encode
     * the emulator's zPerStep while the lash model advances 1 Z count per 3
     * servo steps, so zCountsPerPitch is set to match the model. Otherwise the
     * threshold would be derived from geometry the simulated drivetrain does
     * not obey. */
    /* Numbers are chosen with generous separation on BOTH sides of the
     * threshold. The ramp overshoots slightly past the take-up target, and each
     * overshoot step is extra carriage motion, so a case sitting one or two
     * counts from the boundary measures the overshoot rather than the guard.
     * calMeasured 65, commanded 105 (margin 40), zPerStep 0.5, floor 2
     *   => expected 40*0.5 + 2 = 22 counts, threshold 11. */
    printf("\n-- PARTIAL ENGAGEMENT: carriage moves, but less than it should --\n");
    {
        Rig rig;
        rig.init(/*backlashSteps*/ 105, /*motionThresh*/ 2, /*coupled*/ true,
                 /*true lash*/ 95);                    /* worn / partly engaged */
        rig.lash.stepsPerZCount = 2;
        rig.data.shared.elsStop.zCountsPerPitch =
            rig.data.shared.elsStop.threadPitchSteps / 2.0f;
        for (int i = 0; i < 3; i++) rig.data.shared.elsStop.calMeasured[i] = 65;

        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        for (int i = 0; i < 60000; i++) rig.stepDriven();

        /* commanded 105 - true lash 95 = 10 steps = 5 Z counts. Comfortably
         * clears the bare 2-count floor; nowhere near the derived 11. */
        checkEq(rig.data.shared.elsStop.takeupThreshCounts, 11,
                "threshold derived from expected motion, not the bare floor");
        check(rig.data.shared.elsStop.lastTakeupZDelta > 2,
              "carriage DID move, and would have cleared the bare 2-count floor");
        check(rig.data.shared.elsStop.lastTakeupZDelta < 11,
              "...but moved less than a calibrated take-up should");
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "REFUSED - moved less than a calibrated take-up should");
        checkEq(rig.data.shared.elsStop.active, 1, "aborted back to stopped");
        check(rig.data.shared.elsStop.lastIdealAdvance == SENTINEL,
              "no phase correction on a partially engaged take-up");
    }

    printf("\n-- positive control: same setup, healthy lash --\n");
    {
        Rig rig;
        rig.init(/*backlashSteps*/ 105, /*motionThresh*/ 2, /*coupled*/ true,
                 /*true lash*/ 65);                    /* what it was calibrated at */
        rig.lash.stepsPerZCount = 2;
        rig.data.shared.elsStop.zCountsPerPitch =
            rig.data.shared.elsStop.threadPitchSteps / 2.0f;
        for (int i = 0; i < 3; i++) rig.data.shared.elsStop.calMeasured[i] = 65;

        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.takeupPending; i++)
            rig.stepDriven();

        checkEq(rig.data.shared.elsStop.takeupThreshCounts, 11, "same derived threshold");
        checkEq(rig.data.shared.elsStop.takeupPending, 0,
                "CONFIRMED - a healthy lash clears the stricter demand");
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_CAL_OK, "takeupResult OK");
    }

    /* The threshold is a LOWER BOUND, never a window. Where the nut sits when a
     * take-up starts decides how much lash is left to traverse, so a well-seated
     * drivetrain moves the carriage MUCH further than the worst-case estimate --
     * up to the entire commanded distance. All of that is healthy. */
    printf("\n-- nut already seated: carriage moves far more, still confirmed --\n");
    {
        Rig rig;
        rig.init(/*backlashSteps*/ 105, /*motionThresh*/ 2, /*coupled*/ true,
                 /*true lash*/ 65);
        rig.lash.stepsPerZCount = 2;
        rig.data.shared.elsStop.zCountsPerPitch =
            rig.data.shared.elsStop.threadPitchSteps / 2.0f;
        for (int i = 0; i < 3; i++) rig.data.shared.elsStop.calMeasured[i] = 65;

        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        rig.lash.nutPos = 0;      /* already against the take-up's own wall */
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.takeupPending; i++)
            rig.stepDriven();

        check(rig.data.shared.elsStop.lastTakeupZDelta > 11,
              "moved far beyond the expected minimum");
        checkEq(rig.data.shared.elsStop.takeupPending, 0,
                "CONFIRMED - excess motion is not a fault (lower bound, not a window)");
    }

    /* ---------------- The confirmation WINDOW ------------------------ */
    /* Regression pair for the 2026-08-08 hardware finding: a correctly withheld
     * take-up was released by the operator nudging the carriage by hand with the
     * half-nut open, because the gate re-evaluated forever against the original
     * baseline and accepted Z motion from any source at any time.
     *
     * The fix is a bounded window, not an instant latch — the carriage does not
     * stop dead when the servo does, so genuinely late motion must still count. */
    printf("\n-- late motion INSIDE the window still confirms (inertia/compliance) --\n");
    {
        Rig rig;
        rig.init(90, 2, /*coupled*/ false, 60);   /* nut open: no driven motion */
        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();

        /* Run until the gate has evaluated and withheld. */
        for (int i = 0; i < 60000
             && rig.data.shared.elsStop.takeupResult != ELS_TAKEUP_ERR_UNCONFIRMED; i++)
            rig.stepDriven();
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "withheld first");
        check(rig.data.elsStopTakeupLatched == 0, "window still open at this point");

        rig.nudgeCarriage(20);          /* well past the threshold */
        /* Several ticks: the gate runs near the TOP of the ISR and the scale
         * positions are refreshed further down, so it sees the previous tick's
         * Z. One tick is not enough for injected motion to become visible. */
        for (int i = 0; i < 5; i++) rig.stepDriven();
        checkEq(rig.data.shared.elsStop.takeupPending, 0,
                "motion arriving while the window is open CONFIRMS");
    }

    printf("\n-- late motion AFTER the window does NOT confirm (the HW defect) --\n");
    {
        /* MUTATION: remove the elsStopTakeupLatched guard (i.e. re-evaluate
         * forever, the original behaviour) and this test goes green while the
         * machine goes back to accepting a hand-pushed carriage as proof the
         * half-nut is engaged. */
        Rig rig;
        rig.init(90, 2, /*coupled*/ false, 60);
        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();

        /* Run well past ELS_SETTLE_TICKS + ELS_TAKEUP_CONFIRM_WINDOW_TICKS. */
        for (int i = 0; i < 40000; i++) rig.stepDriven();
        checkEq(rig.data.elsStopTakeupLatched, 1, "window has closed");

        /* The pass is ABORTED back to the stopped state, not held. */
        checkEq(rig.data.shared.elsStop.takeupPending, 0, "no longer holding the machine");
        checkEq(rig.data.shared.elsStop.active, 1, "back to stopped-at-shoulder");
        checkEq(rig.data.shared.elsStop.referenceLatched, 1,
                "thread phase reference PRESERVED - a retry must be free");
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "and it still says why");

        rig.nudgeCarriage(200);         /* a big shove, far past any threshold */
        for (int i = 0; i < 200; i++) rig.stepDriven();
        check(rig.data.shared.elsStop.lastIdealAdvance == SENTINEL,
              "a hand-pushed carriage never triggers the phase correction");

        /* Retry: pressing Cut again clears the warning and starts a fresh
         * take-up, with no reset ritual in between. */
        rig.data.shared.elsStop.active = 0;
        rig.stepDriven();
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_CAL_OK,
                "the warning self-clears when the next cut is initiated");
    }

    /* ---------------- Unconfigured threshold fails closed ------------ */
    printf("\n-- unconfigured motion threshold (0) on a HEALTHY machine --\n");
    {
        /* A permissive default here would silently restore the original
         * open-loop behaviour on every machine that had not been commissioned. */
        Rig rig;
        rig.init(90, /*motionThresh*/ 0, /*coupled*/ true, 60);
        rig.armAndTrigger();
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        for (int i = 0; i < 60000; i++) rig.stepDriven();

        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "healthy machine + threshold 0 is REFUSED (fails closed)");
        checkEq(rig.data.shared.elsStop.active, 1, "aborted back to stopped");
        check(rig.data.shared.elsStop.lastIdealAdvance == SENTINEL,
              "no phase correction on an unconfirmed take-up");
    }

    /* ---------------- Calibration refusals --------------------------- */
    printf("\n-- calibration refuses unless the machine is in a safe state --\n");
    {
        Rig rig;
        rig.init(90, 2, true, 60);
        rig.data.shared.elsStop.enable = 1;           /* a job is live */
        rig.data.shared.elsStop.calCommand = 1;
        rig.step(Z_CLEAR);
        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_ERR_ENABLED,
                "refuses while elsStop.enable is set");
        checkEq(rig.data.shared.elsStop.calCommand, 0,
                "calCommand consumed and cleared by the firmware");
        check(rig.data.shared.elsStop.calSeq >= 1, "refusal still bumps calSeq");
    }
    {
        Rig rig;
        rig.init(90, 2, true, 60);
        rig.data.shared.fastData.servoMode = 0;       /* steps would never drain */
        rig.data.shared.elsStop.calCommand = 1;
        rig.step(Z_CLEAR);
        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_ERR_SERVOMODE,
                "refuses when servoMode != 1");
    }
    {
        Rig rig;
        rig.init(90, /*motionThresh*/ 0, true, 60);
        rig.data.shared.elsStop.calCommand = 1;
        rig.step(Z_CLEAR);
        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_ERR_CONFIG,
                "refuses when the motion threshold is unconfigured");
    }

    /* ---------------- Calibration measures the lash ------------------ */
    printf("\n-- calibration run against a coupled drivetrain (lash = 60 steps) --\n");
    {
        Rig rig;
        rig.init(0, /*motionThresh*/ 2, /*coupled*/ true, /*lash*/ 60);
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        rig.data.shared.elsStop.calCommand = 1;

        uint16_t seq0 = rig.data.shared.elsStop.calSeq;
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.calSeq == seq0; i++)
            rig.stepDriven();

        check(rig.data.shared.elsStop.calSeq == (uint16_t)(seq0 + 1), "run finished");
        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_OK, "calResult is OK");
        printf("      measured = %d, %d, %d (true lash 60)\n",
               (int)rig.data.shared.elsStop.calMeasured[0],
               (int)rig.data.shared.elsStop.calMeasured[1],
               (int)rig.data.shared.elsStop.calMeasured[2]);
        for (int i = 0; i < 3; i++) {
            char buf[80];
            snprintf(buf, sizeof buf, "measurement %d is within quantization of 60", i);
            int32_t m = rig.data.shared.elsStop.calMeasured[i];
            check(m >= 60 && m <= 60 + 3 * 3, buf);
        }
    }

    /* ---------------- Calibration on an open half-nut ---------------- */
    printf("\n-- calibration run with the half-nut OPEN --\n");
    {
        Rig rig;
        rig.init(0, 2, /*coupled*/ false, 60);
        rig.step(Z_CLEAR);
        rig.beginLashDriven();
        rig.data.shared.elsStop.calCommand = 1;

        uint16_t seq0 = rig.data.shared.elsStop.calSeq;
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.calSeq == seq0; i++)
            rig.stepDriven();

        check(rig.data.shared.elsStop.calSeq == (uint16_t)(seq0 + 1),
              "run terminates rather than driving forever");
        checkEq(rig.data.shared.elsStop.calResult, ELS_CAL_ERR_NO_MOTION,
                "calResult is NO_MOTION");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
