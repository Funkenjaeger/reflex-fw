/*
 * POSITIVE CONTROL — backlash-takeup Z confirmation gate.
 *
 * WHAT THIS TEST CLAIMS
 * ---------------------
 * Before this gate, the firmware declared the post-resume backlash takeup
 * "complete" on a PURE COMMANDED-STEP-COUNT CROSSING TEST: servo.currentSteps
 * vs. elsStopTakeupTargetSteps, a target the firmware itself assigned moments
 * earlier. That confirms only that the firmware finished issuing the pulses it
 * chose to issue. With the half-nut open, the servo disabled, or the coupling
 * slipped, currentSteps crosses the target exactly on schedule, takeup reports
 * complete, and applyPhaseCorrection snapshots a Z from a drivetrain that never
 * moved. ARCHITECTURE.md ("Limits") names this hole and states there is no
 * sensor to warn against it.
 *
 * The Z scale is that sensor. It is already sampled every ISR tick; only the
 * comparison was missing. Core/Src/Ramps.c now measures Z motion across the
 * takeup, projects it onto the takeup direction, and refuses to report the
 * takeup complete when it falls short of elsStop.takeupMinZCounts.
 *
 * WHY THE THRESHOLD IS A PARAMETER, NOT backlashSteps
 * ---------------------------------------------------
 * "Z must have moved backlashSteps worth of counts" is the wrong predicate and
 * wrong in the unsafe direction — it would refuse every correctly configured
 * takeup. The takeup's whole job is to drive the leadscrew ACROSS the lash
 * window, and motion spent inside the window moves the nut, not the carriage.
 * The legitimate range for a healthy takeup is the whole interval
 *     [ 0 , backlashSteps * zCountsPerPitch / threadPitchSteps ]
 * so 0 is a legal reading for a healthy machine AND the reading for a machine
 * coupled to nothing. The firmware cannot separate them from geometry it holds.
 * What makes it decidable is an operator choice: size backlashSteps larger than
 * the measured lash on purpose, and set takeupMinZCounts to the counts that
 * deliberate margin must produce. That is machine-specific — this emulator runs
 * 400 counts/mm on Z against a 4000 counts/rev spindle, the real lathe (elspi)
 * is 200 counts/mm and 6144 PPR — so the number lives in a Modbus register and
 * is set at the lathe, never baked in here.
 *
 * TEST SHAPE
 * ----------
 * Z is modelled as a function of the leadscrew: across the takeup the carriage
 * moves `coupling` x the geometric Z-counts-per-leadscrew-step ratio. One knob,
 * nothing else differs between the cases.
 *
 *   COUPLED   coupling = 1.0  -> Z moves ~476 counts -> takeup COMPLETES
 *   DECOUPLED coupling = 0.0  -> Z does not move     -> takeup REFUSED
 *   SHORT     coupling = 0.2  -> Z moves ~95 counts  -> takeup REFUSED
 *   DISABLED  coupling = 0.0, takeupMinZCounts = 0   -> takeup COMPLETES
 *   RELEASE   refused, then enable 1->0              -> hold released
 *
 * DECOUPLED and SHORT are the cases today's firmware gets wrong: without the
 * gate both report a completed takeup and run applyPhaseCorrection. They must
 * be RED against pre-gate code and GREEN with it.
 *
 * DISABLED is the backward-compatibility pin: an unconfigured takeupMinZCounts
 * must not change how an existing machine runs (same precedent as
 * elsStop.hysteresis == 0). It also states plainly what the pre-gate firmware
 * did in the DECOUPLED situation — proceed regardless.
 *
 * RELEASE matters because the gate FAILS CLOSED. A hold with no exit would be a
 * worse defect than the one being fixed.
 *
 * Drives the REAL Core/Src/Ramps.c ISR directly, stubbing every Ramps.c
 * external — the same "stub the externals, don't link hal_shim.c" pattern used
 * by els_stop_resume_relatch_test.cpp. Nothing under Core/ is modified by this
 * test.
 *
 * EMULATOR-PROVEN ONLY: no servo dynamics, no Modbus timing, no metal. The Z
 * coupling here is an algebraic stand-in for a drivetrain, not a model of one.
 * takeupMinZCounts must be set at the machine from observed lastTakeupZDelta.
 *
 * Build/run: `els_takeup_zconfirm_test` CTest target (see CMakeLists).
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
/* Shim / HAL / RTOS / Modbus stubs — Ramps.c's complete external set. */
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
/* Fixture — geometry copied from emulator/src/main.cpp's ELS scenario */
/* ------------------------------------------------------------------ */

static const int32_t SPINDLE_COUNTS_PER_PASS = 40;
static const int32_t Z_CLEAR    = 1000;      /* clear of the stop, where the operator resumes */
static const int32_t Z_PAST     = -5;        /* just past the stop */
static const int32_t Z_STOP_POS = 0;
static const int16_t Z_STOP_DIR = -1;        /* stop when Z <= stopPosition */
static const int32_t HYST       = 500;       /* retract counts demanded before re-arming */
static const float   SENTINEL   = 1.0e30f;   /* nothing real writes this */

static const float THREAD_PITCH_STEPS = 533.333f;   /* leadscrew steps per cut pitch */
static const float Z_COUNTS_PER_PITCH = 846.667f;   /* 2.1167 mm * 400 counts/mm */

/* Z encoder counts per leadscrew step when the drivetrain is rigidly coupled. */
static const double Z_PER_LS_STEP = (double)Z_COUNTS_PER_PITCH / (double)THREAD_PITCH_STEPS;

static const uint32_t BACKLASH = 300;        /* leadscrew steps of commanded takeup */

/* Z counts a fully coupled takeup produces: 300 * 1.5875 = 476.25 */
static const int32_t FULL_COUPLED_Z = (int32_t)((double)BACKLASH * Z_PER_LS_STEP);

/* Operator-set threshold. Deliberately well inside (0, FULL_COUPLED_Z): it
 * represents the Z motion the margin between backlashSteps and the machine's
 * real lash is expected to produce. */
static const int32_t MIN_Z_COUNTS = 200;

struct Rig {
    rampsHandler_t     data;
    TIM_TypeDef        tim[SCALES_COUNT];
    TIM_HandleTypeDef  htim[SCALES_COUNT];
    int32_t            spindleCnt;
    int32_t            zCnt;

    /* Z model: zCnt = zBase + (currentSteps - stepBase) * Z_PER_LS_STEP * coupling.
     * coupling = 1.0 is a rigid drivetrain; 0.0 is a leadscrew turning against an
     * open half-nut. parkZ() re-baselines so the arming phase can place Z by fiat
     * exactly as els_stop_resume_relatch_test.cpp does. */
    double  coupling;
    int32_t zBase;
    int32_t stepBase;

    void init() {
        std::memset(&data, 0, sizeof(data));
        std::memset(tim,  0, sizeof(tim));
        std::memset(htim, 0, sizeof(htim));
        spindleCnt = 0;
        zCnt       = Z_CLEAR;
        coupling   = 0.0;
        zBase      = Z_CLEAR;
        stepBase   = 0;
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
        data.shared.scales[1].syncEnable   = 0;

        data.shared.servo.maxSpeed     = 100000.0f;
        /* High acceleration purely to keep the 300-step takeup inside a short
         * bounded tick loop; it does not interact with the gate. */
        data.shared.servo.acceleration = 5000000.0f;
        data.shared.servo.servoDir     = 1;
        data.shared.fastData.servoMode = 1;

        data.shared.elsStop.scaleIndex       = 1;
        data.shared.elsStop.stopPosition     = Z_STOP_POS;
        data.shared.elsStop.stopDirection    = Z_STOP_DIR;
        data.shared.elsStop.threadPitchSteps = THREAD_PITCH_STEPS;
        data.shared.elsStop.zCountsPerPitch  = Z_COUNTS_PER_PITCH;
        data.shared.elsStop.backlashSteps    = BACKLASH;
        data.shared.elsStop.hysteresis       = HYST;
        data.shared.elsStop.takeupMinZCounts = 0;    /* per-case */
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

    /* Place the carriage and re-baseline the Z model there. */
    void parkZ(int32_t z) {
        zBase    = z;
        stepBase = (int32_t)data.shared.servo.currentSteps;
    }

    /* One ISR pass. Spindle always turns; Z follows the leadscrew per `coupling`. */
    void step() {
        spindleCnt += SPINDLE_COUNTS_PER_PASS;
        int32_t dSteps = (int32_t)data.shared.servo.currentSteps - stepBase;
        zCnt = zBase + (int32_t)llround((double)dSteps * Z_PER_LS_STEP * coupling);
        tim[0].CNT = (uint32_t)spindleCnt;
        tim[1].CNT = (uint32_t)zCnt;
        emu_dwt.CYCCNT += 1000;   /* ~10 us at 100 MHz */
        SynchroRefreshTimerIsr(&data);
    }
};

/* ------------------------------------------------------------------ */
/* Scenario                                                            */
/* ------------------------------------------------------------------ */

struct Outcome {
    bool     armedOk;
    bool     takeupStarted;
    bool     takeupCompleted;      /* takeupPending went 1 -> 0 within the tick budget */
    bool     phaseCorrectionRan;
    uint16_t takeupFault;
    int32_t  lastTakeupZDelta;
    int32_t  zTravelled;           /* raw Z counts moved across the takeup */
    int32_t  ticksToSettle;
    bool     releasedByDisable;    /* only meaningful when exerciseRelease is set */
};

/*
 * coupling:        drivetrain coupling across the takeup (1.0 rigid, 0.0 open)
 * minZCounts:      elsStop.takeupMinZCounts (0 disables the gate)
 * exerciseRelease: after the run, drop elsStop.enable and check the hold clears
 */
static Outcome runTakeupScenario(double coupling, int32_t minZCounts,
                                 bool exerciseRelease = false) {
    Rig rig;
    rig.init();
    rig.data.shared.elsStop.takeupMinZCounts = minZCounts;

    Outcome o{};

    /* --- Arm a fresh job. Z placed by fiat (coupling still 0). --- */
    rig.parkZ(Z_CLEAR);
    rig.step();
    rig.data.shared.elsStop.enable = 1;
    for (int i = 0; i < 3; i++) rig.step();

    bool armedClean = (rig.data.shared.elsStop.active == 0);

    /* --- Feed past the stop and let the firmware latch (two passes: the
     * trigger test reads the Z captured on the previous pass). --- */
    rig.parkZ(Z_PAST);
    rig.step();
    rig.step();

    bool latched = (rig.data.shared.elsStop.active == 1)
                && (rig.data.shared.elsStop.referenceLatched == 1);

    /* --- Operator retracts clear of the threshold (still by fiat). --- */
    rig.parkZ(Z_CLEAR);
    rig.step();

    o.armedOk = armedClean && latched
             && (rig.data.shared.elsStop.active == 1)
             && (rig.data.elsStopPreviousActive == 1)
             && (rig.data.shared.elsStop.takeupPending == 0)
             && (rig.data.shared.scales[1].position == Z_CLEAR);

    /* --- From here the carriage is coupled (or not) to the leadscrew. --- */
    rig.coupling = coupling;
    rig.parkZ(Z_CLEAR);

    rig.data.shared.elsStop.lastIdealAdvance = SENTINEL;

    /* --- SW resumes: write active = 0 over Modbus. --- */
    rig.data.shared.elsStop.active = 0;
    rig.step();

    o.takeupStarted = (rig.data.shared.elsStop.takeupPending == 1);
    int32_t zAtTakeupStart = rig.data.shared.scales[1].position;

    /* --- Run the takeup out. Bounded budget: the 300-step move plus the
     * ELS_SETTLE_TICKS dwell completes far inside this; a refused takeup
     * simply never clears takeupPending and burns the whole budget. --- */
    const int32_t TICK_BUDGET = 400000;
    int32_t t = 0;
    for (; t < TICK_BUDGET; t++) {
        rig.step();
        if (rig.data.shared.elsStop.takeupPending == 0) break;
    }
    o.takeupCompleted  = (rig.data.shared.elsStop.takeupPending == 0);
    o.ticksToSettle    = t;
    o.takeupFault      = rig.data.shared.elsStop.takeupFault;
    o.lastTakeupZDelta = rig.data.shared.elsStop.lastTakeupZDelta;
    o.zTravelled       = rig.data.shared.scales[1].position - zAtTakeupStart;
    o.phaseCorrectionRan = (rig.data.shared.elsStop.lastIdealAdvance != SENTINEL);

    /* --- Escape hatch: dropping enable must release a withheld takeup. --- */
    if (exerciseRelease) {
        rig.data.shared.elsStop.enable = 0;
        rig.step();
        o.releasedByDisable = (rig.data.shared.elsStop.takeupPending == 0)
                           && (rig.data.shared.elsStop.takeupFault == 0);
    }
    return o;
}

static void report(const Outcome &o) {
    printf("      started=%d completed=%d fault=%u lastTakeupZDelta=%d "
           "zTravelled=%d ticks=%d phaseCorrection=%s\n",
           (int)o.takeupStarted, (int)o.takeupCompleted, (unsigned)o.takeupFault,
           (int)o.lastTakeupZDelta, (int)o.zTravelled, (int)o.ticksToSettle,
           o.phaseCorrectionRan ? "RAN" : "did NOT run");
}

static int check(const char *label, bool ok, const char *expectation) {
    printf("[%s] %s: %s\n", ok ? "PASS" : "FAIL", label, expectation);
    return ok ? 0 : 1;
}

int main() {
    int failures = 0;

    printf("=== ELS backlash takeup: Z confirmation gate ===\n");
    printf("backlashSteps=%u  zCountsPerLeadscrewStep=%.4f  fully-coupled Z ~= %d counts\n",
           (unsigned)BACKLASH, Z_PER_LS_STEP, (int)FULL_COUPLED_Z);
    printf("takeupMinZCounts=%d (operator-set; NOT derived from backlashSteps)\n\n",
           (int)MIN_Z_COUNTS);

    /* ---------------- COUPLED: the carriage really moves ---------------- */
    printf("-- COUPLED (coupling = 1.0): carriage tracks the leadscrew --\n");
    Outcome coupled = runTakeupScenario(1.0, MIN_Z_COUNTS);
    failures += check("COUPLED", coupled.armedOk, "preconditions armed");
    failures += check("COUPLED", coupled.takeupStarted, "takeup started (takeupPending == 1)");
    failures += check("COUPLED", coupled.takeupCompleted,
                      "takeup reports COMPLETE (takeupPending cleared)");
    failures += check("COUPLED", coupled.takeupFault == 0, "takeupFault == 0");
    failures += check("COUPLED", coupled.phaseCorrectionRan,
                      "applyPhaseCorrection RAN");
    failures += check("COUPLED", coupled.lastTakeupZDelta >= MIN_Z_COUNTS,
                      "lastTakeupZDelta cleared the operator threshold");
    /* Geometry sanity check on the observed figure. The measured delta runs
     * slightly ABOVE the pure geometric 300 * 1.5875 = 476 counts, and the
     * excess is fully accounted for:
     *
     *   +8.5 counts  sync leaked in the RESUME pass. The per-scale sync block
     *                runs earlier in the ISR than the 1->0 edge block that
     *                initiates the takeup, so one pass of sync
     *                (40 spindle counts * -2/15 = -5.33 leadscrew steps) lands
     *                in desiredSteps before elsStopTakeupZStart is captured.
     *                Sync is gated for the rest of the takeup, so it leaks once.
     *   +2.3 counts  ~1.4 leadscrew steps of servo overshoot: completion is a
     *                CROSSING test, and the servo keeps closing on desiredSteps
     *                through the ELS_SETTLE_TICKS dwell.
     *
     * 476.25 + 8.47 + 2.3 = 487. The band below is that derivation, not a
     * number fitted to the output. */
    bool zNear = (coupled.lastTakeupZDelta >= FULL_COUPLED_Z - 8)
              && (coupled.lastTakeupZDelta <= FULL_COUPLED_Z + 24);
    failures += check("COUPLED", zNear,
                      "lastTakeupZDelta in [geometric-8, geometric+24] "
                      "(sync leak + crossing overshoot, derived above)");
    report(coupled);

    /* ---------------- DECOUPLED: the defect case ------------------------ */
    /* Commanded steps are issued and cross the target exactly as in the COUPLED
     * run. The ONLY difference is that no metal moved. Pre-gate firmware reports
     * this takeup complete and runs applyPhaseCorrection on a meaningless Z. */
    printf("\n-- DECOUPLED (coupling = 0.0): leadscrew turns, carriage does not --\n");
    Outcome decoupled = runTakeupScenario(0.0, MIN_Z_COUNTS);
    failures += check("DECOUPLED", decoupled.armedOk, "preconditions armed");
    failures += check("DECOUPLED", decoupled.takeupStarted,
                      "takeup started (takeupPending == 1)");
    failures += check("DECOUPLED", !decoupled.takeupCompleted,
                      "takeup REFUSED (takeupPending still 1)");
    failures += check("DECOUPLED", decoupled.takeupFault == 1, "takeupFault == 1");
    failures += check("DECOUPLED", !decoupled.phaseCorrectionRan,
                      "applyPhaseCorrection did NOT run");
    failures += check("DECOUPLED", decoupled.lastTakeupZDelta < MIN_Z_COUNTS,
                      "lastTakeupZDelta below the operator threshold");
    report(decoupled);

    /* ---------------- SHORT: moved, but not enough ---------------------- */
    /* Proves the gate tests a MAGNITUDE, not merely "Z != 0". A slipping or
     * partially engaged drivetrain is the realistic version of this. */
    printf("\n-- SHORT (coupling = 0.2): carriage moves ~%d counts, threshold %d --\n",
           (int)(FULL_COUPLED_Z / 5), (int)MIN_Z_COUNTS);
    Outcome shortMove = runTakeupScenario(0.2, MIN_Z_COUNTS);
    failures += check("SHORT", shortMove.armedOk, "preconditions armed");
    failures += check("SHORT", shortMove.takeupStarted,
                      "takeup started (takeupPending == 1)");
    failures += check("SHORT", shortMove.lastTakeupZDelta > 0,
                      "carriage DID move (rules out a dead Z channel)");
    failures += check("SHORT", !shortMove.takeupCompleted,
                      "takeup REFUSED (takeupPending still 1)");
    failures += check("SHORT", shortMove.takeupFault == 1, "takeupFault == 1");
    failures += check("SHORT", !shortMove.phaseCorrectionRan,
                      "applyPhaseCorrection did NOT run");
    report(shortMove);

    /* ---------------- GATE DISABLED: backward-compatibility pin --------- */
    /* Identical to DECOUPLED except takeupMinZCounts = 0. This is what the
     * pre-gate firmware did in every case: proceed on the commanded-step
     * crossing alone. An unconfigured register must not change machine
     * behavior (same contract as elsStop.hysteresis == 0). */
    printf("\n-- GATE DISABLED (coupling = 0.0, takeupMinZCounts = 0) --\n");
    Outcome ungated = runTakeupScenario(0.0, 0);
    failures += check("DISABLED", ungated.takeupStarted,
                      "takeup started (takeupPending == 1)");
    failures += check("DISABLED", ungated.takeupCompleted,
                      "takeup reports COMPLETE even with zero Z motion "
                      "(pre-gate behavior preserved)");
    failures += check("DISABLED", ungated.takeupFault == 0, "takeupFault == 0");
    failures += check("DISABLED", ungated.phaseCorrectionRan,
                      "applyPhaseCorrection RAN");
    failures += check("DISABLED", ungated.lastTakeupZDelta == 0,
                      "lastTakeupZDelta still reports the real (zero) motion");
    report(ungated);

    /* ---------------- RELEASE: fail-closed must be recoverable ---------- */
    printf("\n-- RELEASE (refused takeup, then elsStop.enable 1->0) --\n");
    Outcome released = runTakeupScenario(0.0, MIN_Z_COUNTS, true);
    failures += check("RELEASE", !released.takeupCompleted,
                      "takeup was withheld before the release");
    failures += check("RELEASE", released.releasedByDisable,
                      "dropping enable clears takeupPending and takeupFault");
    report(released);

    /* ---------------- Mechanism ---------------------------------------- */
    printf("\n-- mechanism --\n");
    printf("    COUPLED and DECOUPLED issue the SAME commanded step count and both\n"
           "    cross elsStopTakeupTargetSteps on schedule; the commanded-crossing\n"
           "    test cannot tell them apart. Observed Z: %d vs %d counts.\n",
           (int)coupled.lastTakeupZDelta, (int)decoupled.lastTakeupZDelta);

    printf("\nEMULATOR-PROVEN ONLY: no servo dynamics, no Modbus timing, no metal.\n"
           "takeupMinZCounts must be set at the machine from observed lastTakeupZDelta.\n");

    printf("\n=== %s === (%d failing assertion%s)\n",
           failures == 0 ? "ALL PASS" : "FAILURES",
           failures, failures == 1 ? "" : "s");
    return failures == 0 ? 0 : 1;
}
