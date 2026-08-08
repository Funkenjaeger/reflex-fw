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
        lash.nutPos = 0;
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

        checkEq(rig.data.shared.elsStop.takeupPending, 1,
                "take-up STILL PENDING - sync stays gated");
        checkEq(rig.data.shared.elsStop.takeupResult, ELS_TAKEUP_ERR_UNCONFIRMED,
                "takeupResult reports UNCONFIRMED");
        check(rig.data.shared.elsStop.lastIdealAdvance == SENTINEL,
              "applyPhaseCorrection did NOT run on an uncoupled drivetrain");
        checkEq(rig.data.shared.elsStop.takeupSeq, 1,
                "takeupSeq reported the outcome ONCE, not once per tick");

        /* Escape hatch: without it, failing closed would be unrecoverable. */
        rig.data.shared.elsStop.enable = 0;
        rig.stepDriven();
        checkEq(rig.data.shared.elsStop.takeupPending, 0,
                "enable 1->0 releases the withheld take-up");
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

        checkEq(rig.data.shared.elsStop.takeupPending, 1,
                "healthy machine + threshold 0 is WITHHELD (fails closed)");
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
