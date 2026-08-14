/*
 * ISR-LEVEL tests for the backlash-CALIBRATION leg's motion attribution --
 * the sibling of els_takeup_confirm_test.cpp's take-up coverage, but for
 * data->elsCal instead of the take-up's elsStop/elsSlip path.
 *
 * WHY THIS FILE EXISTS (task 6a7c455e8f08a317e962b42a)
 * -----------------------------------------------------
 * Mutation testing on 2026-08-12 established that dropping the take-up-style
 * `armed` gate around the calibration leg's ISR wiring changed nothing across
 * the whole 7-target suite, for a structural reason: NO test target drove
 * data->elsCal through the real production ISR (SynchroRefreshTimerIsr) at
 * all in a way that exercises a mid-leg hand nudge. els_backlash_cal_test.cpp
 * covers the pure decision layer (elsCalTick) against a mock Drivetrain that
 * ticks its own attribution accumulator by hand -- useful, but it is not the
 * ISR wiring, and a wiring bug (wrong tick placement, wrong gating condition,
 * a field never actually connected) is invisible to it by construction.
 *
 * This file is the ISR-level analogue: it links the real Core/Src/Ramps.c and
 * drives SynchroRefreshTimerIsr() directly, the same fixture shape as
 * els_takeup_confirm_test.cpp's Rig/Lash pair (duplicated here rather than
 * shared, to keep this target buildable/runnable standalone and to keep the
 * take-up file's own diff surface untouched).
 *
 * REQUIRES the 2026-08-10 staged patch
 * (~/ol-work/scratch/2026-08-10/els-calibration-slip-attribution.patch)
 * applied on top of feat/els-slip-attribution. Without it, Core/Inc/
 * els_backlash_cal.h has no elsCalCtx_t.slip field and Core/Src/Ramps.c has
 * no attribution-tick block for the calibration leg at all -- this file will
 * not compile against vanilla d4835c4. That absence is itself the point: the
 * calibration path's ISR-side attribution wiring is patch-only, unmerged
 * anywhere, and until now had zero test coverage in either state.
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
/* Same lash + Rig shape as els_takeup_confirm_test.cpp. Comments trimmed to
 * what differs for calibration; see that file for the full rationale. */

static const int32_t Z_CLEAR = 1000;

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

        data.shared.elsStop.scaleIndex           = 1;
        data.shared.elsStop.threadPitchSteps     = 533.333f;
        data.shared.elsStop.zCountsPerPitch      = 846.667f;
        data.shared.elsStop.hysteresis           = 500;
        data.shared.elsStop.calMotionThreshCounts = motionThresh;
        data.shared.elsStop.calCeilingSteps      = ceilingSteps;
        data.shared.elsStop.enable               = 0;

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

    /* Every tick from the very first: Z is whatever the lash model produces
     * from the servo's own pulses (or, if uncoupled, frozen at zBase until a
     * nudge moves it) -- there is no separate "hand it to the lash" step
     * because calibration legs have no pre-drive settle phase to skip past. */
    void stepDriven() {
        spindleCnt += 1;   /* spindle keeps turning; irrelevant to calibration */
        zCnt = lash.follow((int32_t)data.shared.servo.currentSteps, zBase);
        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;
        SynchroRefreshTimerIsr(&data);
    }

    /* Operator pushing the carriage by hand -- uncoupled from any servo pulse. */
    void nudgeCarriage(int32_t zCounts) {
        zBase += zCounts;
    }
};

/* ------------------------------------------------------------------ */

int main() {
    printf("=== ELS calibration-leg motion attribution (ISR level) ===\n\n");
    printf("Requires the 08-10 patch (elsCalCtx_t.slip / Ramps.c calibration\n");
    printf("attribution-tick block). Drives data->elsCal through the real\n");
    printf("SynchroRefreshTimerIsr(), not a fixture-simulated drivetrain.\n\n");

    /* ---------------- Positive control: coupled leg measures correctly --- */
    printf("-- coupled drivetrain: calibration leg completes and measures the true lash --\n");
    {
        Rig rig;
        rig.init(/*motionThresh*/ 2, /*coupled*/ true, /*lashSteps*/ 60);
        rig.data.shared.elsStop.calCommand = 1;
        rig.stepDriven();   /* request intake tick: starts the leg */

        checkEq(rig.data.elsCal.phase, /*ELS_CAL_SEAT=1*/ 1, "leg started (phase SEAT)");

        uint16_t seq0 = rig.data.shared.elsStop.calSeq;
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.calSeq == seq0; i++)
            rig.stepDriven();

        checkEq(rig.data.shared.elsStop.calResult, /*ELS_CAL_OK=0*/ 0,
                "calResult OK on a healthy coupled drivetrain");
        printf("      measured = %d, %d, %d (true lash 60)\n",
               (int)rig.data.shared.elsStop.calMeasured[0],
               (int)rig.data.shared.elsStop.calMeasured[1],
               (int)rig.data.shared.elsStop.calMeasured[2]);
        for (int i = 0; i < 3; i++) {
            int32_t m = rig.data.shared.elsStop.calMeasured[i];
            check(m >= 55 && m <= 70, "measurement is close to the true lash (sanity: harness works)");
        }
        check(rig.data.elsCal.slip.attributedZCounts != 0,
              "attribution accumulator saw real servo-driven Z motion (nonzero, sign per direction)");
    }

    /* ---------------- THE GAP: hand nudge mid-leg, half-nut open ---------- */
    /* The relevant unit of harm is ONE LEG, not the whole 3-cycle run: a
     * single MEASURE cycle fooled by a nudge writes a bogus calMeasured[i],
     * which is exactly what elsTakeupConfirmThreshold() consumes downstream
     * (see els_backlash_cal.h). So the check below is on the FIRST phase
     * transition a nudge can reach, not on overall run completion -- a run
     * that later times out on a LATER, un-nudged cycle is not evidence the
     * first cycle's poisoned measurement didn't happen. */
    printf("\n-- OPEN HALF-NUT + hand nudge WHILE the leg is actively driving --\n");
    printf("   (this is the mid-drive-nudge exploit the 08-10 patch's own\n");
    printf("    'KNOWN GAP' fixture-level test documents; here it is driven\n");
    printf("    through the REAL ISR for the first time, and judged on the\n");
    printf("    SINGLE phase transition the nudge can reach)\n");
    {
        Rig rig;
        rig.init(/*motionThresh*/ 2, /*coupled*/ false, /*lashSteps*/ 60);
        rig.data.shared.elsStop.calCommand = 1;
        rig.stepDriven();

        checkEq(rig.data.elsCal.phase, 1, "leg started (phase SEAT)");

        /* Run until armed (first pulse in the commanded direction credited) --
         * this is the earliest point elsSlipReset() has baselined the window,
         * matching the patch's own gating discipline. */
        for (int i = 0; i < 1000 && !rig.data.elsCal.armed; i++) rig.stepDriven();
        check(rig.data.elsCal.armed == 1, "leg armed (harness sanity)");

        int32_t stepsRefAtArm = rig.data.elsCal.stepsRef;

        /* Nudge while the leadscrew is STILL actively turning toward the
         * ceiling -- an open half-nut means it will keep pulsing the whole
         * leg, so "mid-drive" is not a narrow window here, it is the entire
         * leg duration. This mirrors the take-up file's uncoupled+nudge case
         * but WITHOUT a post-drive dwell, because calibration legs have none. */
        rig.nudgeCarriage(20);   /* well past calMotionThreshCounts=2 */
        for (int i = 0; i < 5; i++) rig.stepDriven();

        printf("      after nudge+5 ticks: phase=%d armed=%d attributedZCounts=%lld"
               " unattributedZCounts=%lld ticksSinceLastPulse=%d\n",
               (int)rig.data.elsCal.phase, (int)rig.data.elsCal.armed,
               (long long)rig.data.elsCal.slip.attributedZCounts,
               (long long)rig.data.elsCal.slip.unattributedZCounts,
               (int)rig.data.elsCal.slip.ticksSinceLastPulse);

        /* Assert the nudge was actually seen (either bucket) before trusting
         * the verdict -- same discipline els_takeup_confirm_test.cpp uses. */
        check(rig.data.elsCal.slip.attributedZCounts != 0
              || rig.data.elsCal.slip.unattributedZCounts != 0,
              "the nudge was seen by the attribution accumulator (fixture sanity)");

        /* THE VERDICT: did the SEAT phase transition to MEASURE off the back
         * of the nudge alone (no real coupling exists in this rig at all)?
         * That transition is what baselines stepsRef/zRef for a MEASURE leg
         * against a nudge instant instead of true lash contact -- exactly the
         * poisoned-measurement mechanism the task describes. */
        bool phaseAdvancedOnNudge = (rig.data.elsCal.phase != 1 /* no longer SEAT */)
                                     || (rig.data.elsCal.stepsRef != stepsRefAtArm);
        printf("   %-70s %s\n",
               "VERDICT: nudge alone advanced the leg past SEAT (gap OPEN at ISR level)",
               phaseAdvancedOnNudge ? "CONFIRMED" : "did NOT reproduce -- gap may be closed");
        check(phaseAdvancedOnNudge,
              "GAP CONFIRMED AT ISR LEVEL: an unattributed hand nudge alone "
              "advanced a calibration leg past its arming phase");

        /* Drain the rest of the run for the record -- not the primary
         * assertion, just context on what the run does after the poisoned
         * transition (this rig has no further coupling or nudges, so the
         * later cycles legitimately time out; that does not undo the harm
         * already done to the first leg's baseline). */
        for (int i = 0; i < 60000 && rig.data.shared.elsStop.calSeq == 0; i++)
            rig.stepDriven();
        printf("      final: calResult=%d (0=OK, 4=NO_MOTION) calMeasured=[%d,%d,%d]\n",
               (int)rig.data.shared.elsStop.calResult,
               (int)rig.data.shared.elsStop.calMeasured[0],
               (int)rig.data.shared.elsStop.calMeasured[1],
               (int)rig.data.shared.elsStop.calMeasured[2]);
    }

    /* ---------------- Negative control: sub-threshold nudge -------------- */
    /* Proves the harness/assertion above is not vacuously true regardless of
     * what happens -- a nudge that never crosses calMotionThreshCounts must
     * NOT advance the phase. If this ever goes red, the test above is not
     * measuring what it claims to. */
    printf("\n-- negative control: nudge BELOW the motion threshold does not advance the leg --\n");
    {
        Rig rig;
        rig.init(/*motionThresh*/ 10, /*coupled*/ false, /*lashSteps*/ 60);
        rig.data.shared.elsStop.calCommand = 1;
        rig.stepDriven();
        for (int i = 0; i < 1000 && !rig.data.elsCal.armed; i++) rig.stepDriven();
        check(rig.data.elsCal.armed == 1, "leg armed (harness sanity)");

        int32_t phaseAtArm = rig.data.elsCal.phase;
        rig.nudgeCarriage(3);     /* below threshCounts=10 */
        for (int i = 0; i < 5; i++) rig.stepDriven();

        checkEq(rig.data.elsCal.phase, phaseAtArm,
                "sub-threshold nudge does NOT advance the phase (selectivity control)");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
