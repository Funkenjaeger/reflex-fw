/*
 * KNOWN-FAILING REPRODUCTION — ELS stop resume re-latch defect.
 *
 * WHAT THIS TEST CLAIMS
 * ---------------------
 * When the host software resumes a threading job by writing
 * `elsStop.active = 0` over Modbus WITHOUT first retracting the carriage
 * clear of `elsStop.stopPosition`, the firmware silently skips the entire
 * resume sequence (backlash takeup AND phase correction). The machine
 * simply re-stops and looks like nothing happened.
 *
 * WHY (static reading of Core/Src/Ramps.c, confirmed by execution here)
 * --------------------------------------------------------------------
 * Within one ISR pass, two things happen IN THIS ORDER:
 *
 *   1. Ramps.c ~403 (inside the per-scale loop, so EARLIER in the pass):
 *          if (!shared->elsStop.active && shared->elsStop.enable) {
 *            int32_t refPos = shared->scales[scaleIndex].position;
 *            shouldStop = (stopDirection >= 0) ? (refPos >= stopPosition)
 *                                              : (refPos <= stopPosition);
 *            if (shouldStop) shared->elsStop.active = 1;   // RE-LATCH
 *          }
 *      `active` is the ONLY gate. There is no retract-distance requirement.
 *      `elsStop.hysteresis` (Ramps.h:102, documented as "encoder counts
 *      carriage must retract before re-enabling") is declared and NEVER
 *      READ anywhere in the firmware — a repo-wide case-insensitive search
 *      for "hyster" over all C/C++/header sources yields exactly one hit:
 *      that declaration itself.
 *
 *   2. Ramps.c ~455 (after the loop, so LATER in the SAME pass):
 *          if (data->elsStopPreviousActive && !shared->elsStop.active) {
 *            ...backlash takeup / applyPhaseCorrection...
 *          }
 *      with `data->elsStopPreviousActive = shared->elsStop.active;` at
 *      Ramps.c ~479.
 *
 * So if Z is still at/past the threshold when SW clears `active`, step 1
 * re-latches `active = 1` before step 2 ever looks at it. The 1->0 edge is
 * consumed inside the pass and never becomes observable. takeupPending is
 * never set; applyPhaseCorrection is never called.
 *
 * TEST SHAPE (defect case + passing control)
 * ------------------------------------------
 * Both scenarios are byte-for-byte identical except for ONE number: the Z
 * encoder position at the moment SW writes active = 0.
 *
 *   DEFECT  case: Z left at Z_PAST   (-5 counts, i.e. past a stop at 0)
 *   CONTROL case: Z retracted to Z_CLEAR (+1000 counts, clear of the stop)
 *
 * The CONTROL is what makes the red result meaningful: it proves the
 * harness CAN observe the resume path running. Without it a red test would
 * only prove the harness is broken.
 *
 * This test drives the REAL Core/Src/Ramps.c ISR (SynchroRefreshTimerIsr)
 * directly, with local stubs for the shim/HAL/Modbus/RTOS externals — the
 * same "stub the externals, don't link hal_shim.c" pattern used by
 * els_halfnut_test.cpp and els_boot_delta_test.cpp. Nothing under Core/ is
 * modified.
 *
 * EXPECTED RESULT: this target FAILS (exit 1) until the resume path is
 * fixed. It is registered in CTest deliberately, as a reproduction.
 *
 * Build/run: `els_stop_resume_relatch_test` CTest target (see CMakeLists).
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
/* Shim / HAL / RTOS / Modbus stubs — Ramps.c's complete external set.  */
/* (nm -u on the firmware build's Ramps.c.o: HAL_GPIO_{Init,WritePin,   */
/*  TogglePin}, HAL_TIM_{Base_Start_IT,Encoder_Start}, Modbus{Init,     */
/*  Start}, emu_{coreDebug,dwt,gpioa,gpiob,hw,log_trace},               */
/*  initScaleTimer, osDelay, osThreadNew.)                              */
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

/* Ramps.c globals the ISR divides by. servoCycles is 0 at startup and is
 * normally set by updateSpeedTask (Ramps.c:601); the ISR's
 *   servoCyclesCounter = (servoCyclesCounter + 1) % servoCycles;   (Ramps.c:513)
 * divides by it, so a test that calls the ISR without an RTOS must seed it or
 * take SIGFPE. emulator/src/main.cpp does the same thing (servoCycles = 1). */
extern uint16_t servoCycles;
extern uint16_t servoCyclesCounter;

} /* extern "C" */

/* ------------------------------------------------------------------ */
/* Fixture                                                             */
/* ------------------------------------------------------------------ */

/* Geometry copied from the emulator's own ELS scenario (emulator/src/main.cpp):
 * 12 TPI thread on an 8 TPI leadscrew, spindle (scale 0) drives sync, Z DRO
 * (scale 1) is read-only, carriage feeds -Z toward the shoulder. */
static const int32_t SPINDLE_COUNTS_PER_PASS = 40;    /* arbitrary, just needs to move */
static const int32_t Z_CLEAR       = 1000;            /* comfortably clear of the stop */
static const int32_t Z_PAST        = -5;              /* just past the stop */
static const int32_t Z_STOP_POS    = 0;
static const int16_t Z_STOP_DIR    = -1;              /* stop when Z <= stopPosition */
static const float   SENTINEL      = 1.0e30f;         /* nothing real writes this */

struct Rig {
    rampsHandler_t     data;
    TIM_TypeDef        tim[SCALES_COUNT];
    TIM_HandleTypeDef  htim[SCALES_COUNT];
    int32_t            spindleCnt;
    int32_t            zCnt;

    void init(uint32_t backlashSteps) {
        std::memset(&data, 0, sizeof(data));
        std::memset(tim,  0, sizeof(tim));
        std::memset(htim, 0, sizeof(htim));
        spindleCnt = 0;
        zCnt       = Z_CLEAR;
        servoCycles        = 1;   /* one step pulse per ISR tick, as main.cpp does */
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

        /* Spindle drives sync; Z is a read-only DRO. Matches main.cpp:85-86. */
        data.shared.scales[0].syncRatioNum = -2;
        data.shared.scales[0].syncRatioDen = 15;
        data.shared.scales[0].syncEnable   = 1;
        data.shared.scales[1].syncEnable   = 0;

        data.shared.servo.maxSpeed     = 100000.0f;
        data.shared.servo.acceleration = 50000.0f;
        data.shared.servo.servoDir     = 1;
        data.shared.fastData.servoMode = 1;   /* sync/index */

        data.shared.elsStop.scaleIndex       = 1;
        data.shared.elsStop.stopPosition     = Z_STOP_POS;
        data.shared.elsStop.stopDirection    = Z_STOP_DIR;
        data.shared.elsStop.threadPitchSteps = 533.333f;
        data.shared.elsStop.zCountsPerPitch  = 846.667f;
        data.shared.elsStop.backlashSteps    = backlashSteps;
        data.shared.elsStop.hysteresis       = 500;  /* SW asks for 500 counts of retract.
                                                      * Firmware never reads this field —
                                                      * setting it here documents that even
                                                      * a configured hysteresis changes
                                                      * nothing. Read-only use; the field
                                                      * itself is untouched in Core/. */
        data.shared.elsStop.enable           = 0;

        /* Seed the raw-counter shadow so the first pass sees a zero delta on
         * every channel except the ones we deliberately move. */
        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        data.scalesDeltaPos[0].position = spindleCnt;
        data.scalesDeltaPos[1].position = zCnt;
        data.shared.scales[0].position  = spindleCnt;
        data.shared.scales[1].position  = zCnt;
        data.scalesSyncDeltaPos[0].oldPosition = spindleCnt;
        data.scalesSyncDeltaPos[1].oldPosition = zCnt;
    }

    /* One ISR pass. Advances the spindle; Z goes wherever the caller says. */
    void step(int32_t zTarget) {
        spindleCnt += SPINDLE_COUNTS_PER_PASS;
        zCnt        = zTarget;
        tim[0].CNT  = (uint32_t)spindleCnt;
        tim[1].CNT  = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;    /* ~10 us at 100 MHz */
        SynchroRefreshTimerIsr(&data);
    }
};

/* ------------------------------------------------------------------ */
/* Scenario                                                            */
/* ------------------------------------------------------------------ */

struct Outcome {
    bool    armedOk;              /* preconditions held before the resume write */
    int32_t zAtResume;
    uint16_t activeBeforeWrite;
    uint16_t prevActiveBeforeWrite;
    uint16_t activeAfterPass;
    uint16_t takeupPendingAfterPass;
    int32_t stepsToGoBefore;
    int32_t stepsToGoAfter;
    float   lastIdealAdvanceAfter;
    bool    phaseCorrectionRan;
    bool    takeupRan;
};

/*
 * zAtResume: where the Z DRO sits when SW writes active = 0.
 *   Z_PAST  -> the defect scenario (no retract)
 *   Z_CLEAR -> the control       (retracted clear of the threshold)
 *
 * Everything else about the two runs is identical.
 */
static Outcome runResumeScenario(int32_t zAtResume, uint32_t backlashSteps,
                                 bool moveStopPosBeforeResume = false,
                                 int32_t newStopPos = 0) {
    Rig rig;
    rig.init(backlashSteps);
    Outcome o{};
    o.zAtResume = zAtResume;

    /* --- Arm a fresh job: enable rising edge resets referenceLatched --- */
    rig.step(Z_CLEAR);                       /* settle, enable still 0 */
    rig.data.shared.elsStop.enable = 1;
    for (int i = 0; i < 3; i++) rig.step(Z_CLEAR);

    bool armedClean = (rig.data.shared.elsStop.active == 0);

    /* --- Feed the carriage past the stop and let the firmware latch. ---
     * The trigger test lives inside the per-scale loop and only runs for
     * scales with syncEnable != 0 (scale 0), so it reads the Z position
     * captured on the PREVIOUS pass: two passes are needed for the latch. */
    rig.step(Z_PAST);
    rig.step(Z_PAST);

    bool latched = (rig.data.shared.elsStop.active == 1)
                && (rig.data.shared.elsStop.referenceLatched == 1);

    /* --- One held pass. This is where the carriage is (or is not) moved. ---
     * `active` is still 1 here, so the trigger test is skipped entirely and
     * this pass only updates the Z DRO. Identical in both scenarios apart
     * from the Z target. */
    rig.step(zAtResume);

    o.activeBeforeWrite     = rig.data.shared.elsStop.active;
    o.prevActiveBeforeWrite = rig.data.elsStopPreviousActive;
    o.armedOk = armedClean && latched
             && (o.activeBeforeWrite == 1)
             && (o.prevActiveBeforeWrite == 1)
             && (rig.data.shared.elsStop.takeupPending == 0)
             && (rig.data.shared.scales[1].position == zAtResume);

    /* Isolation variant: move the threshold out from under an UNMOVED
     * carriage, so the trigger test evaluates false while Z is byte-identical
     * to the defect run. Used to prove the re-latch is the sole cause. */
    if (moveStopPosBeforeResume) rig.data.shared.elsStop.stopPosition = newStopPos;

    /* --- SW resumes: write active = 0 over Modbus (register write). --- */
    rig.data.shared.elsStop.active = 0;

    o.stepsToGoBefore = rig.data.shared.servo.stepsToGo;
    rig.data.shared.elsStop.lastIdealAdvance = SENTINEL;

    /* --- The very next ISR pass. Everything hinges on this one. --- */
    rig.step(zAtResume);

    o.activeAfterPass        = rig.data.shared.elsStop.active;
    o.takeupPendingAfterPass = rig.data.shared.elsStop.takeupPending;
    o.stepsToGoAfter         = rig.data.shared.servo.stepsToGo;
    o.lastIdealAdvanceAfter  = rig.data.shared.elsStop.lastIdealAdvance;

    o.takeupRan           = (o.takeupPendingAfterPass == 1);
    o.phaseCorrectionRan  = (o.lastIdealAdvanceAfter != SENTINEL);
    return o;
}

static void report(const char *label, const Outcome &o) {
    printf("      z@resume=%d active(before write)=%u prevActive=%u -> "
           "active(after pass)=%u takeupPending=%u stepsToGo %d->%d "
           "lastIdealAdvance=%s\n",
           (int)o.zAtResume, (unsigned)o.activeBeforeWrite,
           (unsigned)o.prevActiveBeforeWrite, (unsigned)o.activeAfterPass,
           (unsigned)o.takeupPendingAfterPass,
           (int)o.stepsToGoBefore, (int)o.stepsToGoAfter,
           o.phaseCorrectionRan ? "overwritten (correction ran)"
                                : "still SENTINEL (correction did NOT run)");
    (void)label;
}

int main() {
    int failures = 0;

    printf("=== ELS stop resume: 1->0 edge vs. same-pass re-latch ===\n");
    printf("stopPosition=%d stopDirection=%d  Z_PAST=%d  Z_CLEAR=%d\n\n",
           (int)Z_STOP_POS, (int)Z_STOP_DIR, (int)Z_PAST, (int)Z_CLEAR);

    /* ---------------- Backlash-takeup branch (backlashSteps != 0) ------- */
    const uint32_t BACKLASH = 300;

    printf("-- backlash takeup branch (elsStop.backlashSteps = %u) --\n", BACKLASH);

    Outcome ctlTakeup = runResumeScenario(Z_CLEAR, BACKLASH);
    printf("[%s] CONTROL (retracted before resume): preconditions armed\n",
           ctlTakeup.armedOk ? "PASS" : "FAIL");
    if (!ctlTakeup.armedOk) failures++;
    printf("[%s] CONTROL: expected takeupPending == 1, observed %u\n",
           ctlTakeup.takeupRan ? "PASS" : "FAIL",
           (unsigned)ctlTakeup.takeupPendingAfterPass);
    report("control", ctlTakeup);
    if (!ctlTakeup.takeupRan) failures++;

    Outcome defTakeup = runResumeScenario(Z_PAST, BACKLASH);
    printf("[%s] DEFECT (no retract before resume): preconditions armed\n",
           defTakeup.armedOk ? "PASS" : "FAIL");
    if (!defTakeup.armedOk) failures++;
    printf("[%s] DEFECT: expected takeupPending == 1, observed %u\n",
           defTakeup.takeupRan ? "PASS" : "FAIL",
           (unsigned)defTakeup.takeupPendingAfterPass);
    report("defect", defTakeup);
    if (!defTakeup.takeupRan) failures++;

    /* ---------------- Inline phase-correction branch (backlashSteps == 0) */
    printf("\n-- inline phase-correction branch (elsStop.backlashSteps = 0) --\n");

    Outcome ctlPhase = runResumeScenario(Z_CLEAR, 0);
    printf("[%s] CONTROL (retracted before resume): preconditions armed\n",
           ctlPhase.armedOk ? "PASS" : "FAIL");
    if (!ctlPhase.armedOk) failures++;
    printf("[%s] CONTROL: expected applyPhaseCorrection to run, observed %s\n",
           ctlPhase.phaseCorrectionRan ? "PASS" : "FAIL",
           ctlPhase.phaseCorrectionRan ? "it ran" : "it did NOT run");
    report("control", ctlPhase);
    if (!ctlPhase.phaseCorrectionRan) failures++;

    Outcome defPhase = runResumeScenario(Z_PAST, 0);
    printf("[%s] DEFECT (no retract before resume): preconditions armed\n",
           defPhase.armedOk ? "PASS" : "FAIL");
    if (!defPhase.armedOk) failures++;
    printf("[%s] DEFECT: expected applyPhaseCorrection to run, observed %s\n",
           defPhase.phaseCorrectionRan ? "PASS" : "FAIL",
           defPhase.phaseCorrectionRan ? "it ran" : "it did NOT run");
    report("defect", defPhase);
    if (!defPhase.phaseCorrectionRan) failures++;

    /* ---------------- Isolation control -------------------------------- */
    /* Same Z as the DEFECT run (-5, never retracted). The ONLY change is that
     * stopPosition is moved out from under the carriage right before the
     * resume write, so the Ramps.c:403 trigger test evaluates false. If this
     * one runs the resume path, the carriage position is NOT what blocks it —
     * the same-pass re-latch is. */
    printf("\n-- isolation control: Z UNMOVED (%d), threshold moved to %d --\n",
           (int)Z_PAST, (int)(Z_PAST - 1000));
    Outcome isoTakeup = runResumeScenario(Z_PAST, BACKLASH, true, Z_PAST - 1000);
    printf("[%s] ISOLATION: expected takeupPending == 1, observed %u\n",
           isoTakeup.takeupRan ? "PASS" : "FAIL",
           (unsigned)isoTakeup.takeupPendingAfterPass);
    report("isolation", isoTakeup);
    if (!isoTakeup.takeupRan) failures++;

    /* ---------------- Diagnosis line ----------------------------------- */
    printf("\n-- mechanism --\n");
    bool relatched = (defTakeup.activeAfterPass == 1) && (defPhase.activeAfterPass == 1);
    printf("    defect runs ended with elsStop.active = %u (re-latched by the\n"
           "    Ramps.c:403 trigger test, which runs EARLIER in the same pass\n"
           "    than the Ramps.c:455 1->0 edge test) : %s\n",
           (unsigned)defTakeup.activeAfterPass,
           relatched ? "re-latch CONFIRMED" : "no re-latch observed");
    printf("    control runs ended with elsStop.active = %u\n",
           (unsigned)ctlTakeup.activeAfterPass);

    printf("\n=== %s === (%d failing assertion%s)\n",
           failures == 0 ? "ALL PASS" : "FAILURES",
           failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
