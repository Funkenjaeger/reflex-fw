/*
 * ISR-level tests for the manual reference latch (interactive re-sync to an
 * existing thread).
 *
 * WHAT THE FEATURE IS
 * -------------------
 * The first-trigger auto-latch captures (latchedSpindle, latchedZ) at the end
 * of the first cutting pass only because that is a moment when lash state is
 * KNOWN. The manual latch is the SAME capture at an operator-chosen point on an
 * existing thread, requested via elsStop.latchCommand and consumed atomically
 * by the ISR. Everything downstream (takeup, applyPhaseCorrection, resume) is
 * unchanged and unaware of which producer latched the reference.
 *
 * CONTRACT UNDER TEST (Ramps.c, latchCommand block after the enable edges)
 * -----------------------------------------------------------------------
 *  1. ACCEPT: with enable == 1, a pending latchCommand is cleared in one ISR
 *     pass; latchedZ/latchedSpindle capture the scale positions coherently
 *     (both from the same tick — the values as of the START of the consuming
 *     pass, i.e. the previous tick's encoder read, ~10 us stale at the 100 kHz
 *     ISR rate and physically irrelevant for a hand-stopped spindle);
 *     referenceLatched is set; latchSeq increments exactly once.
 *  2. SUPPRESS: a manual latch sets referenceLatched, which is the SAME guard
 *     the first-trigger auto-latch checks — so the first stop trigger of the
 *     job must NOT overwrite the operator's reference.
 *  3. REFUSE: with enable == 0 the command is consumed WITHOUT a latchSeq
 *     increment and without touching the reference. The missing ack IS the
 *     refusal; a reference latched outside a job would be wiped by the next
 *     enable 0->1 edge anyway.
 *  4. CONSUME DOWNSTREAM: the resume path's phase correction measures
 *     deltaSpindle from the MANUAL latch point, proving the operator's datum —
 *     not the stop trigger — anchors the thread.
 *  5. MECHANISM NOT POLICY: a second latch while referenceLatched == 1
 *     overwrites the reference and acks. Fresh-job-only is HOST policy; the
 *     firmware provides the same latch every time it is asked.
 *
 * Drives the REAL Core/Src/Ramps.c ISR with the same external-stub strategy as
 * els_stop_resume_relatch_test.cpp. Fixture geometry is copied from there.
 *
 * Mutation-tested 2026-08-08: each case carries a MUTATION note naming the
 * code change that defeats it. All four were applied one at a time, the listed
 * failure counts observed, and the mutation reverted:
 *   M1 drop latchSeq++                  -> 3 failures (cases 1, 2, 5)
 *   M2 drop referenceLatched = 1        -> 4 failures (cases 1, 2x2, 4)
 *   M3 latch regardless of enable       -> 3 failures (case 3)
 *   M4 capture scales[0].position+1000  -> 3 failures (cases 1, 4, 5)
 * M4 also found a defect in THIS FILE's first draft: case 4 computed its
 * expected value from latchedSpindle itself, so the skew cancelled and the
 * case passed under M4. It now records the datum independently.
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

/* ------------------------------------------------------------------ */
/* Fixture — geometry copied from els_stop_resume_relatch_test.cpp     */
/* ------------------------------------------------------------------ */

static const int32_t SPINDLE_COUNTS_PER_PASS = 40;
static const int32_t Z_CLEAR    = 1000;
static const int32_t Z_PAST     = -5;
static const int32_t Z_STOP_POS = 0;
static const int16_t Z_STOP_DIR = -1;
static const float   SENTINEL   = 1.0e30f;
static const float   SYNC_NUM   = -2.0f;
static const float   SYNC_DEN   = 15.0f;

struct Rig {
    rampsHandler_t     data;
    TIM_TypeDef        tim[SCALES_COUNT];
    TIM_HandleTypeDef  htim[SCALES_COUNT];
    int32_t            spindleCnt;
    int32_t            zCnt;

    void init(uint32_t backlashSteps, int32_t hysteresisCounts = 500) {
        std::memset(&data, 0, sizeof(data));
        std::memset(tim,  0, sizeof(tim));
        std::memset(htim, 0, sizeof(htim));
        spindleCnt = 0;
        zCnt       = Z_CLEAR;
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

        data.shared.scales[0].syncRatioNum = (int32_t)SYNC_NUM;
        data.shared.scales[0].syncRatioDen = (int32_t)SYNC_DEN;
        data.shared.scales[0].syncEnable   = 1;
        data.shared.scales[1].syncEnable   = 0;

        data.shared.servo.maxSpeed     = 100000.0f;
        data.shared.servo.acceleration = 50000.0f;
        data.shared.servo.servoDir     = 1;
        data.shared.fastData.servoMode = 1;

        data.shared.elsStop.scaleIndex       = 1;
        data.shared.elsStop.stopPosition     = Z_STOP_POS;
        data.shared.elsStop.stopDirection    = Z_STOP_DIR;
        data.shared.elsStop.threadPitchSteps = 533.333f;
        data.shared.elsStop.zCountsPerPitch  = 846.667f;
        data.shared.elsStop.backlashSteps    = backlashSteps;
        data.shared.elsStop.hysteresis       = hysteresisCounts;
        data.shared.elsStop.enable           = 0;

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
        emu_dwt.CYCCNT += 1000;
        SynchroRefreshTimerIsr(&data);
    }

    /* Arm a fresh job and settle: enable rising edge resets referenceLatched. */
    void armJob() {
        step(Z_CLEAR);
        data.shared.elsStop.enable = 1;
        for (int i = 0; i < 3; i++) step(Z_CLEAR);
    }
};

static int failures = 0;

static void check(bool ok, const char *what) {
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

int main() {
    printf("=== ELS manual reference latch (latchCommand/latchSeq) ===\n\n");

    /* ---------------- 1. ACCEPT ---------------------------------------- */
    /* MUTATION: in the Ramps.c latch block, skip the latchSeq++ (ack never
     * sent) -> the seq assertion fails here and in case 5. Assign latchedZ
     * from scales[0] instead of scales[scaleIndex] -> the capture assertions
     * fail (spindle and Z fixture positions differ by construction). */
    printf("-- 1. accept: latch inside a job --\n");
    {
        Rig rig;
        rig.init(0);
        rig.armJob();
        check(rig.data.shared.elsStop.referenceLatched == 0, "armed with no reference yet");

        int32_t sBefore = rig.data.shared.scales[0].position;
        int32_t zBefore = rig.data.shared.scales[1].position;
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);

        check(rig.data.shared.elsStop.latchCommand == 0, "command cleared in one pass");
        check(rig.data.shared.elsStop.latchSeq == 1, "latchSeq acked exactly once");
        check(rig.data.shared.elsStop.referenceLatched == 1, "referenceLatched set");
        check(rig.data.shared.elsStop.latchedSpindle == sBefore,
              "latchedSpindle = spindle as of start of consuming pass (coherent capture)");
        check(rig.data.shared.elsStop.latchedZ == zBefore,
              "latchedZ = Z as of start of consuming pass (coherent capture)");
    }

    /* ---------------- 2. SUPPRESS the auto-latch ----------------------- */
    /* MUTATION: delete `referenceLatched = 1` from the latch block -> the
     * trigger's `if (!referenceLatched)` capture runs at the first stop and
     * both unchanged-assertions fail (spindle advances every pass, so the
     * auto-latched value can never equal the manual one). */
    printf("\n-- 2. suppress: first stop trigger must not overwrite --\n");
    {
        Rig rig;
        rig.init(0);
        rig.armJob();
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);
        int32_t sManual = rig.data.shared.elsStop.latchedSpindle;
        int32_t zManual = rig.data.shared.elsStop.latchedZ;

        /* Feed past the stop; trigger reads Z captured on the PREVIOUS pass,
         * so two passes are needed for the latch (see relatch test). */
        rig.step(Z_PAST);
        rig.step(Z_PAST);
        check(rig.data.shared.elsStop.active == 1, "stop trigger fired");
        check(rig.data.shared.elsStop.latchedSpindle == sManual,
              "latchedSpindle untouched by the trigger");
        check(rig.data.shared.elsStop.latchedZ == zManual,
              "latchedZ untouched by the trigger");
        check(rig.data.shared.elsStop.latchSeq == 1, "no spurious ack from the trigger");
    }

    /* ---------------- 3. REFUSE outside a job -------------------------- */
    /* MUTATION: hoist the capture out of the `enable != 0` guard (consume and
     * latch regardless) -> all four assertions fail: seq acks, the reference
     * appears, and referenceLatched goes 1 with no job to own it. */
    printf("\n-- 3. refuse: latch with enable == 0 consumes without ack --\n");
    {
        Rig rig;
        rig.init(0);
        rig.step(Z_CLEAR);   /* settle, enable stays 0 */
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);

        check(rig.data.shared.elsStop.latchCommand == 0, "command still consumed (not left pending)");
        check(rig.data.shared.elsStop.latchSeq == 0, "no latchSeq ack — the missing edge IS the refusal");
        check(rig.data.shared.elsStop.referenceLatched == 0, "no reference latched");
        check(rig.data.shared.elsStop.latchedSpindle == 0
              && rig.data.shared.elsStop.latchedZ == 0, "latched pair untouched");
    }

    /* ---------------- 4. CONSUME DOWNSTREAM ---------------------------- */
    /* The resume path must measure deltaSpindle from the MANUAL latch point.
     * backlashSteps = 0 selects the inline phase-correction branch so
     * lastIdealAdvance is published on the resume pass itself (the takeup
     * branch defers it past a settle dwell — separate machinery, covered by
     * els_takeup_confirm_test).
     * MUTATION (M4): make the latch block capture scales[0].position + 1000
     * into latchedSpindle -> the ideal-advance assertion fails by exactly
     * 1000 * SYNC_NUM/SYNC_DEN while the sentinel control still passes,
     * proving the assertion tracks the operator's datum and not merely "some
     * correction ran". Only holds because sManual is recorded independently —
     * see the comment at its capture. */
    printf("\n-- 4. consume: phase correction measures from the manual datum --\n");
    {
        Rig rig;
        rig.init(0);            /* inline correction branch */
        rig.armJob();
        /* Record the datum INDEPENDENTLY of the firmware's latch registers —
         * the value the latch is supposed to capture is the spindle position
         * as of the start of the consuming pass. Reading latchedSpindle back
         * would make this case self-consistent under a capture-skew mutation
         * (expected and actual both inherit the skew, which cancels), and it
         * did exactly that when first tried. */
        int32_t sManual = rig.data.shared.scales[0].position;
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);

        rig.step(Z_PAST);       /* feed to the stop */
        rig.step(Z_PAST);
        check(rig.data.shared.elsStop.active == 1, "stopped at the shoulder");

        rig.step(Z_CLEAR);      /* retract clear (hysteresis = 500 < 1000) */

        rig.data.shared.elsStop.lastIdealAdvance = SENTINEL;
        rig.data.shared.elsStop.active = 0;    /* SW resume */
        rig.step(Z_CLEAR);

        check(rig.data.shared.elsStop.lastIdealAdvance != SENTINEL,
              "phase correction ran on resume (sentinel overwritten)");
        float expected = (float)(rig.data.shared.scales[0].position - sManual)
                         * SYNC_NUM / SYNC_DEN;
        check(std::fabs(rig.data.shared.elsStop.lastIdealAdvance - expected) < 0.01f,
              "idealAdvance measured from the manual latch point");
    }

    /* ---------------- 5. MECHANISM NOT POLICY -------------------------- */
    /* MUTATION: guard the latch block with `referenceLatched == 0` -> the
     * overwrite assertions fail (seq stuck at 1, reference stuck at the first
     * capture). Fresh-job-only lives in the HOST wizard, deliberately. */
    printf("\n-- 5. re-latch overwrites: firmware is mechanism, host is policy --\n");
    {
        Rig rig;
        rig.init(0);
        rig.armJob();
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);
        int32_t sFirst = rig.data.shared.elsStop.latchedSpindle;

        for (int i = 0; i < 4; i++) rig.step(Z_CLEAR);   /* spindle moves on */

        int32_t sBefore = rig.data.shared.scales[0].position;
        rig.data.shared.elsStop.latchCommand = 1;
        rig.step(Z_CLEAR);

        check(rig.data.shared.elsStop.latchSeq == 2, "second latch acked");
        check(rig.data.shared.elsStop.latchedSpindle == sBefore
              && rig.data.shared.elsStop.latchedSpindle != sFirst,
              "reference overwritten by the second latch");
    }

    printf("\n=== %s === (%d failing assertion%s)\n",
           failures == 0 ? "ALL PASS" : "FAILURES",
           failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
