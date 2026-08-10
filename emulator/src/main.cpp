/*
 * Lathe Emulator - Main entry point.
 *
 * Initializes the firmware data structures, starts the physics model,
 * transport layer, and ISR thread, then runs the dashboard.
 */

#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

#include "config.h"
#include "physics.h"
#include "transport.h"
#include "dashboard.h"

extern "C" {
#include "emulator_state.h"
#include "stm32f4xx_hal.h"
#include "Ramps.h"

/* Firmware externs */
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim9, htim11;
extern UART_HandleTypeDef huart1;
extern void emu_update_timer_counters(void);
}

static std::atomic<bool> g_running{true};

static void signalHandler(int sig) {
    (void)sig;
    g_running.store(false);
}

/*
 * Scripted ELS multi-pass threading scenario (EMU_SCENARIO=els_backlash).
 *
 * Reproduces the hardware "pass-1 vs pass-2..N thread-phase offset that tracks
 * the configured backlash" bug entirely in firmware logic. Drives the firmware
 * ISR through cut -> stop -> retract -> resume (takeup + applyPhaseCorrection)
 * for several passes, varying elsStop.backlashSteps, and logs the spindle phase
 * at each stop plus the firmware's phase-correction diagnostics. If the spindle
 * phase at stop changes with backlash, the bug reproduces.
 *
 * Geometry: 8 TPI thread (pitch 3.175mm). syncRatio 1/5 => 4000 spindle counts
 * (1 rev) -> 800 servo steps -> 1 pitch. zCountsPerPitch = 3.175mm * 400 c/mm.
 * Sign relationship matches hardware: syncRatioNum>0 with stopDirection=-1
 * (carriage feeds -Z toward the shoulder, spindle turned negative).
 */
static void runElsScenario(LathePhysics *physics, rampsHandler_t *rampsData,
                           const EmuConfig &cfg) {
    using namespace std::chrono;
    rampsSharedData_t *sh = &rampsData->shared;
    auto ms = [](int n){ std::this_thread::sleep_for(milliseconds(n)); };
    const int CPR = cfg.spindle_counts_per_rev;          // 4000

    auto phase = [&](int32_t p){ int32_t m = p % CPR; if (m < 0) m += CPR; return m; };

    // --- ELS geometry ---
    // Sign convention matches hardware's verified-correct takeup direction:
    // cuttingDir = sign(syncRatioNum) (tps*zcpp>0, no flip) must equal the
    // physical feed direction. The carriage feeds -Z toward the shoulder
    // (stopDirection=-1). Feed leadscrew sign = sign(RPM)*scaleDir*sign(num).
    // With num=-1 and RPM=+150 (scaleDir=+1): feed = -1 (-Z toward shoulder)
    // AND cuttingDir = -1 == feed -> takeup loads the correct (cutting) wall.
    // (The earlier num=+1/RPM=-150 config gave cuttingDir=+1 != -Z feed, so the
    //  takeup loaded the WRONG wall -- an artifact, not the hardware bug.)
    // Cut a 12 TPI thread (pitch 25.4/12 = 2.1167mm) on the 8 TPI (3.175mm)
    // leadscrew, matching the user's real lathe. Crucially the cut pitch is
    // INCOMMENSURATE with the leadscrew/half-nut snap grid (3.175mm), so the
    // half-nut snap-to-grid no longer folds away mod cut-pitch (the earlier
    // 1-pitch==1-leadscrew-rev config hid that). leadscrew = 251.97 steps/mm
    // (800 steps / 3.175mm), so cut pitch = 2.1167mm * 251.97 = 533.33 servo
    // steps/pitch and 4000 * 2/15 = 533.33 steps/rev => syncRatio 2/15.
    // num<0 keeps cuttingDir == the -Z feed direction (see sign note below).
    sh->scales[0].syncRatioNum = -2;
    sh->scales[0].syncRatioDen = 15;      // 4000 * 2/15 = 533.33 steps/rev = 1 cut pitch
    sh->scales[0].syncEnable   = 1;
    sh->scales[1].syncEnable   = 0;       // Z DRO read-only
    sh->elsStop.scaleIndex       = 1;
    sh->elsStop.threadPitchSteps = 533.333f;   // servo steps per 12 TPI cut pitch
    sh->elsStop.zCountsPerPitch  = 846.667f;    // 2.1167mm * 400 counts/mm
    sh->elsStop.stopDirection    = -1;    // stop when Z <= stopPosition
    sh->fastData.servoMode = 1;           // indexing/sync pulses enabled

    const int   PITCH_STEPS = 533;
    const int   FEED_PITCHES = 6;
    const int   FEED_STEPS  = PITCH_STEPS * FEED_PITCHES;   // servo steps per cut

    // backlash settings (servo steps) to test. Default sweeps a range (all
    // exceed the 0.6mm=~151-step lash, the valid operating regime) so a fixed-
    // resume run maps thread-phase vs backlash. Override with EMU_BACKLASH_STEPS
    // ("a,b,c,..."). One pass per value (fixed resume makes each repeatable).
    int backlash[32] = { 160, 230, 300, 370, 440, 510, 580, 650 };
    int NPASS = 8;
    if (const char *bls = getenv("EMU_BACKLASH_STEPS")) {
        NPASS = 0;
        const char *s = bls;
        while (*s && NPASS < 32) {
            backlash[NPASS++] = atoi(s);
            while (*s && *s != ',') s++;
            if (*s == ',') s++;
        }
    }

    // RampsStart() hardcodes maxSpeed=720/accel=120 — too slow for the 10kHz
    // emulator ISR (real pulse rate = isr_rate/servoCycles). Override here so
    // servoCycles=1 (one pulse per tick, ~10k steps/s) and retracts ramp fast.
    extern uint16_t servoCycles;
    sh->servo.maxSpeed     = 100000.0f;
    sh->servo.acceleration = 50000.0f;
    servoCycles = 1;

    printf("\n=== ELS backlash scenario: pitch=%d steps, feed=%d steps ===\n",
           PITCH_STEPS, FEED_STEPS);
    printf("DEBUG maxSpeed=%.0f servoCycles=%u servoMode=%d\n",
           (double)sh->servo.maxSpeed, (unsigned)servoCycles, (int)sh->fastData.servoMode);
    fflush(stdout);

    // Spin spindle so the carriage feeds in -Z (toward the shoulder).
    // Keep RPM low enough that the servo step rate (~10k/s) follows the sync
    // demand (RPM/60 * pitchSteps). At 150 RPM that's 2000 steps/s << 10k/s.
    // Spindle RPM (EMU_RPM, default 150). The emulator servo emits 1 step/tick
    // (servoCycles=1 at the 10kHz ISR = 10k steps/s) vs hardware's 100kHz, so the
    // unsynced phase-correction jog takes finite time during which the free-running
    // spindle turns. Lowering RPM shrinks that — a probe for the step-rate artifact.
    double rpm = 150.0;
    if (const char *r = getenv("EMU_RPM")) rpm = atof(r);
    physics->setTargetRPM(rpm);
    ms(800);

    // Fresh job: pulse enable to reset referenceLatched.
    sh->elsStop.enable = 0; ms(50);
    sh->elsStop.referenceLatched = 0;
    sh->elsStop.active = 0;

    // Establish the shoulder relative to the current carriage position.
    int32_t startZ = sh->scales[1].position;
    double  startMM = physics->getCarriageMM();
    sh->elsStop.stopPosition = startZ - (int32_t)(FEED_PITCHES * sh->elsStop.zCountsPerPitch);
    printf("startZ=%d stopPosition=%d\n", (int)startZ, (int)sh->elsStop.stopPosition);
    fflush(stdout);

    for (int p = 0; p < NPASS && g_running.load(); p++) {
        sh->elsStop.backlashSteps = (uint32_t)backlash[p];
        sh->elsStop.enable = 1;

        // Optionally resume at a FIXED spindle phase each pass (EMU_FIX_RESUME_PHASE).
        // applyPhaseCorrection is supposed to make the stop phase independent of the
        // resume spindle angle; pinning the resume phase isolates whether the
        // observed pass-to-pass noise is resume-angle dependent (a correction leak)
        // or comes from elsewhere (snap, lash, geometry).
        static const char *fixEnv = getenv("EMU_FIX_RESUME_PHASE");
        if (fixEnv != nullptr) {
            int32_t targetPh = atoi(fixEnv);            // 0..CPR
            int wg = 0;
            while (phase(sh->scales[0].position) != targetPh
                   && g_running.load() && wg < 20000) { ms(1); wg++; }
        }

        // Resume / start cut: active 1->0. For p>0 this triggers takeup+resync.
        int32_t zResume   = sh->scales[1].position;
        int32_t curResume = (int32_t)sh->servo.currentSteps;
        double  carResume = physics->getCarriageMM();
        int     prevTkp   = (int)sh->elsStop.takeupPending;
        bool    armLogged = false;
        int32_t zArm = zResume, curArm = curResume; double carArm = carResume;
        double  lashArm = physics->getBacklashOffsetMM();
        printf("  [resume p%d] spindlePhase=%d/%d z=%d lash=%.3f leadscrewFmodPitch=%.4f\n",
               p, (int)phase(sh->scales[0].position), CPR,
               (int)zResume, physics->getBacklashOffsetMM(),
               fmod(physics->getLeadscrewPositionMM(), 1270.0 / 400.0));
        fflush(stdout);
        sh->elsStop.active = 0;

        // Wait for the carriage to feed to the shoulder (firmware sets active=1).
        int guard = 0;
        while (sh->elsStop.active == 0 && g_running.load() && guard < 12000) {
            ms(2); guard++;
            int tkpNow = (int)sh->elsStop.takeupPending;
            if (!prevTkp && tkpNow && !armLogged) {        // takeup armed (0->1)
                zArm = sh->scales[1].position; curArm = (int32_t)sh->servo.currentSteps;
                carArm = physics->getCarriageMM();
                lashArm = physics->getBacklashOffsetMM(); armLogged = true;
            }
            if (prevTkp && !tkpNow) {                       // takeup+dwell done (1->0)
                int32_t cdir = (sh->scales[0].syncRatioNum > 0) ? 1 : -1;
                if (sh->elsStop.threadPitchSteps * sh->elsStop.zCountsPerPitch < 0.0f)
                    cdir = -cdir;
                printf("  [TAKEUP p%d] cuttingDir=%+d dCur=%+d dZ=%+d dCarriage=%+.4fmm  "
                       "lashWall=%.3f->%.3f (0=-wall,%.2f=+wall) target=%d cur=%d takeup=%d steps\n",
                       p, (int)cdir, (int)sh->servo.currentSteps - curArm,
                       (int)sh->scales[1].position - zArm,
                       physics->getCarriageMM() - carArm,
                       lashArm, physics->getBacklashOffsetMM(), cfg.z_backlash_mm,
                       (int)rampsData->elsStopTakeupTargetSteps,
                       (int)sh->servo.currentSteps, backlash[p]);
                fflush(stdout);
            }
            prevTkp = tkpNow;
            if ((guard % 500) == 0) {
                printf("  [wait p%d] sp=%d z=%d des=%d cur=%d stg=%d tkp=%d tgt=%d\n",
                       p, (int)sh->scales[0].position, (int)sh->scales[1].position,
                       (int)sh->servo.desiredSteps, (int)sh->servo.currentSteps,
                       (int)sh->servo.stepsToGo, tkpNow,
                       (int)rampsData->elsStopTakeupTargetSteps);
                fflush(stdout);
            }
        }
        if (sh->elsStop.active == 0) {
            printf("pass %d: TIMEOUT (z=%d stopPos=%d)\n", p,
                   (int)sh->scales[1].position, (int)sh->elsStop.stopPosition);
            break;
        }

        int32_t spAtStop = sh->scales[0].position;
        int32_t zAtStop = sh->scales[1].position;
        printf("PASS %d  backlash=%d  spindlePhase@stop=%d/%d  zStop=%d  "
               "ideal=%.1f actual=%.1f phaseErr=%.1f corr=%.1f  lashStop=%.3f carr=%.4f ls=%.4f\n",
               p, backlash[p], (int)phase(spAtStop), CPR, (int)zAtStop,
               (double)sh->elsStop.lastIdealAdvance,
               (double)sh->elsStop.lastActualAdvance,
               (double)sh->elsStop.lastPhaseError,
               (double)sh->elsStop.lastCorrection,
               physics->getBacklashOffsetMM(), physics->getCarriageMM(),
               physics->getLeadscrewPositionMM());
        fflush(stdout);

        // Manual half-nut retract (replicates the operator's stop-only flow):
        // open the half-nut, hand-move the carriage back to start (the servo
        // holds — sync is paused while active=1), then close the half-nut,
        // which snaps the carriage to the nearest leadscrew grid and engages
        // (leadscrew stationary). The servo is NOT commanded during retract.
        physics->requestHalfNutToggle();                 // disengage
        ms(60);
        physics->moveCarriageTo(startMM);                // hand-move back
        guard = 0;
        while ((physics->isZMoveTargetActive()
                || std::abs(physics->getCarriageMM() - startMM) > 0.02)
               && g_running.load() && guard < 8000) { ms(2); guard++; }
        ms(100);
        physics->requestHalfNutToggle();                 // re-engage (snap to grid)
        guard = 0;
        while (physics->getHalfNutState() != LathePhysics::ENGAGED
               && g_running.load() && guard < 3000) { ms(2); guard++; }
        ms(200);  // settle
        printf("  [retract done p%d] z=%d hn=%d (startZ=%d)\n", p,
               (int)sh->scales[1].position, (int)physics->getHalfNutState(),
               (int)startZ);
        fflush(stdout);
    }

    printf("=== scenario done ===\n");
    fflush(stdout);
    g_running.store(false);
}

/*
 * Physics-server mode (EMU_SCENARIO=serve): keep the Modbus transport alive and
 * run only the MECHANICAL physics — spindle rotation and the manual half-nut
 * retract — while leaving ALL elsStop registers (geometry, backlash, enable,
 * active, stopPosition) to be driven by the real reflex-ui host over Modbus.
 * This is the integration harness: reflex-ui owns the ELS workflow; the emulator
 * owns the lathe physics. On each new ELS stop (els_last_stop_seq increments),
 * the emulator performs the operator's manual retract (open half-nut, hand-move
 * the carriage back to the home position, re-engage), then waits for the host to
 * start the next cut. Spindle RPM is kept low by default (EMU_RPM) so the
 * emulator's 10kHz/1-step-per-tick servo doesn't introduce the step-rate phase
 * artifact that masqueraded as the bug in firmware-only runs.
 *
 * Retract home: the carriage position at startup (the host must place stop_z on
 * the cutting side of it). Override the retract distance via EMU_RETRACT_MM.
 */
/*
 * Serve-mode stdin command channel: gives system tests a way to drive the
 * machine's MANUAL degrees of freedom mid-test without inventing a Modbus
 * register for each one. One command per line, whitespace-separated:
 *
 *   x move <mm>         -> physics->moveCrossSlideTo(mm)
 *   x jog <-1|0|1>      -> physics->jogCrossSlide(dir)
 *   z move <mm>         -> physics->moveCarriageTo(mm)
 *   z jog <-1|0|1>      -> physics->jogCarriage(dir)
 *   halfnut open        -> physics->setHalfNutEngaged(false)
 *   halfnut close       -> physics->setHalfNutEngaged(true)
 *
 * WHY THE Z AND HALFNUT COMMANDS EXIST
 * ------------------------------------
 * Together they are the third machine state the emulator could not previously
 * represent: UNCOUPLED BUT MOVING ANYWAY. The model had exactly two worlds --
 * coupled (Z follows the leadscrew) and uncoupled (Z never moves) -- while a
 * real lathe has a third, because the operator can push the carriage with the
 * half-nut open. That is not a corner case here: the whole ELS stop/resume
 * model is built on hand-cranking between passes.
 *
 * Its absence is why the 2026-08-08 take-up gate defect shipped with passing
 * tests. A fixture that cannot express a failure produces tests that agree with
 * the code and are wrong together. See reflex-fw/todo.md.
 *
 * The z branch adds NO new coupling logic: LathePhysics::moveCarriageTo() /
 * jogCarriage() already refuse to move the carriage while the half-nut is
 * ENGAGED, so "only valid while the nut is open" is enforced inside the physics
 * model, not here.
 *
 * halfnut takes an explicit STATE (open/close), not a toggle, and the
 * distinction is load-bearing rather than cosmetic: a toggle issued while an
 * engage is waiting for phase alignment CANCELS it, so a toggle-shaped command
 * means different things depending on state the test cannot see. setHalfNutEngaged()
 * is idempotent; `halfnut close` never means "give up".
 *
 * Runs on its own thread (same pattern as isrThreadFunc) because
 * runPhysicsServer's loop can block for several seconds inside the
 * auto-retract and must never be the thing blocked on stdin. Unknown or
 * malformed lines are logged and skipped; EOF quietly ends the thread. The
 * thread is started detached, so it is never joined -- fine, since it makes
 * no further use of *physics after returning.
 */
static void stdinCommandThreadFunc(LathePhysics *physics) {
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream iss(line);
        std::string noun, verb;
        iss >> noun >> verb;

        if (noun == "halfnut") {
            if (verb == "open" || verb == "close") {
                physics->setHalfNutEngaged(verb == "close");
                emu_log_event("stdin cmd: halfnut %s", verb.c_str());
            } else {
                emu_log_event("stdin cmd: malformed 'halfnut' verb '%s' "
                              "(want open|close) (line: \"%s\")",
                              verb.c_str(), line.c_str());
            }
            continue;
        }

        if (noun != "x" && noun != "z") {
            emu_log_event("stdin cmd: unknown axis '%s' (line: \"%s\")",
                          noun.c_str(), line.c_str());
            continue;
        }

        if (verb == "move") {
            double mm;
            if (iss >> mm) {
                if (noun == "x") physics->moveCrossSlideTo(mm);
                else             physics->moveCarriageTo(mm);
                emu_log_event("stdin cmd: %s move %.4f", noun.c_str(), mm);
            } else {
                emu_log_event("stdin cmd: malformed '%s move' (line: \"%s\")",
                              noun.c_str(), line.c_str());
            }
        } else if (verb == "jog") {
            int dir;
            if ((iss >> dir) && (dir == -1 || dir == 0 || dir == 1)) {
                if (noun == "x") physics->jogCrossSlide(dir);
                else             physics->jogCarriage(dir);
                emu_log_event("stdin cmd: %s jog %d", noun.c_str(), dir);
            } else {
                emu_log_event("stdin cmd: malformed '%s jog' (line: \"%s\")",
                              noun.c_str(), line.c_str());
            }
        } else {
            emu_log_event("stdin cmd: unknown verb '%s' (line: \"%s\")", verb.c_str(), line.c_str());
        }
    }
    emu_log_event("stdin cmd: EOF, command channel closed");
}

static void runPhysicsServer(LathePhysics *physics, rampsHandler_t *rampsData,
                             const EmuConfig &cfg) {
    using namespace std::chrono;
    rampsSharedData_t *sh = &rampsData->shared;
    auto ms = [](int n){ std::this_thread::sleep_for(milliseconds(n)); };
    const int CPR = cfg.spindle_counts_per_rev;
    auto phase = [&](int32_t p){ int32_t m = p % CPR; if (m < 0) m += CPR; return m; };

    // Servo capable of following; low spindle RPM avoids the step-rate artifact.
    extern uint16_t servoCycles;
    sh->servo.maxSpeed     = 100000.0f;
    sh->servo.acceleration = 50000.0f;
    servoCycles = 1;

    double rpm = 30.0;
    if (const char *r = getenv("EMU_RPM")) rpm = atof(r);
    physics->setTargetRPM(rpm);

    double homeMM = physics->getCarriageMM();
    if (const char *h = getenv("EMU_RETRACT_MM")) homeMM = atof(h);

    // EMU_NO_AUTO_RETRACT: skip the simulated operator hand-retract on each ELS
    // stop. Use this to test reflex-ui's OWN retract workflow (stop+retract /
    // wizard modes), where the host drives the servo retract with the half-nut
    // kept engaged. Default (unset) = simulate the manual retract (stop-only).
    bool noAutoRetract = getenv("EMU_NO_AUTO_RETRACT") != nullptr;

    printf("=== PHYSICS SERVER: rpm=%.1f homeMM=%.4f auto_retract=%d (reflex-ui drives ELS over Modbus) ===\n",
           rpm, homeMM, (int)!noAutoRetract);
    printf("    spindle running; %s on each ELS stop.\n",
           noAutoRetract ? "host-driven retract (no auto-retract)"
                         : "manual half-nut retract");
    fflush(stdout);

    // X-axis command channel: system tests write "x move <mm>" / "x jog <dir>"
    // lines to our stdin. Own thread so this loop's occasional multi-second
    // blocking (auto-retract, below) never stalls command delivery.
    std::thread(stdinCommandThreadFunc, physics).detach();

    uint32_t prevSeq = emu_hw.els_last_stop_seq;
    while (g_running.load()) {
        ms(5);
        uint32_t seq = emu_hw.els_last_stop_seq;
        if (seq == prevSeq) continue;
        prevSeq = seq;

        int32_t spStop = emu_hw.els_last_stop_spindle;
        printf("STOP #%u  spindlePhase@stop=%d/%d  z=%d  carr=%.4f ls=%.4f\n",
               (unsigned)seq, (int)phase(spStop), CPR,
               (int)sh->scales[1].position, physics->getCarriageMM(),
               physics->getLeadscrewPositionMM());
        fflush(stdout);

        // Let the host observe active=1 and settle its FSM into 'stopped'.
        ms(400);

        if (noAutoRetract) {
            // Host (reflex-ui) drives the retract via the servo with the
            // half-nut kept engaged; the emulator does nothing but log.
            printf("  [stop #%u] host-driven retract; half-nut left engaged\n",
                   (unsigned)seq);
            fflush(stdout);
            continue;
        }

        // Manual retract: open half-nut, hand-move carriage home, re-engage.
        // Park on a lattice-aligned position so the re-engage snap is a no-op
        // and legacy stationary re-engagements stay deterministic. Computed
        // per stop: the lattice offset shifts every pass (takeup/correction
        // move the leadscrew), but it is frozen between the stop trigger and
        // re-engage (sync is gated while active=1, released by the host only
        // after we re-engage).
        physics->requestHalfNutToggle();
        ms(80);
        double alignedHome = physics->nearestGridPositionMM(homeMM);
        physics->moveCarriageTo(alignedHome);
        int guard = 0;
        while ((physics->isZMoveTargetActive()
                || std::abs(physics->getCarriageMM() - alignedHome) > 0.02)
               && g_running.load() && guard < 8000) { ms(2); guard++; }
        ms(120);
        physics->requestHalfNutToggle();
        guard = 0;
        while (physics->getHalfNutState() != LathePhysics::ENGAGED
               && g_running.load() && guard < 3000) { ms(2); guard++; }
        printf("  [retract done #%u] z=%d carr=%.4f hn=%d — ready for next cut\n",
               (unsigned)seq, (int)sh->scales[1].position,
               physics->getCarriageMM(), (int)physics->getHalfNutState());
        fflush(stdout);
    }
    printf("=== physics server stopped ===\n");
    fflush(stdout);
}

/*
 * ISR thread: ticks the physics model and calls the firmware's
 * SynchroRefreshTimerIsr at the configured rate.
 */
static void isrThreadFunc(LathePhysics *physics, rampsHandler_t *rampsData, int rate_hz) {
    auto interval = std::chrono::microseconds(1000000 / rate_hz);
    auto next_tick = std::chrono::steady_clock::now();

    /* Simulate the DWT cycle counter. At "100 MHz", each ISR tick at 10kHz = 10000 cycles. */
    uint32_t cycles_per_tick = 100000000U / (uint32_t)rate_hz;

    /* Track previous step pin state for edge detection */
    int prev_step_pin = 0;

    while (g_running.load()) {
        next_tick += interval;

        /* Advance DWT cycle counter to simulate the inter-ISR interval */
        emu_hw.dwt_cyccnt += cycles_per_tick;
        emu_dwt.CYCCNT = emu_hw.dwt_cyccnt;

        /* Advance physics */
        double dt = 1.0 / (double)rate_hz;
        physics->tick(dt, &rampsData->shared);

        /* Call firmware ISR.
         * The ISR reads DWT->CYCCNT at start and end to measure execution
         * time, but since emu_dwt.CYCCNT is a plain variable both reads
         * return the same value → executionCycles=0. We measure the real
         * wall-clock execution time and patch the result after. */
        SynchroRefreshTimerIsr(rampsData);

        /* Overwrite executionCycles with a realistic estimate.
         * The real ISR takes ~5-10µs on the MCU = 500-1000 cycles at 100MHz.
         * Use a fixed reasonable value since we can't measure sub-µs on host. */
        rampsData->shared.executionCycles = 600;

        /* Detect STEP rising edge and feed into physics */
        if (emu_hw.step_pin && !prev_step_pin) {
            int dir = emu_hw.dir_pin ? 1 : -1;
            physics->onStepPulse(dir);
        }
        prev_step_pin = emu_hw.step_pin;

        /* Increment HAL tick (~1ms per tick, approximate) */
        static int tick_divider = 0;
        tick_divider++;
        if (tick_divider >= rate_hz / 1000) {
            tick_divider = 0;
            HAL_IncTick();
        }

        /* Sleep until next tick */
        std::this_thread::sleep_until(next_tick);
    }
}

int main(int argc, char *argv[]) {
    printf("=== Lathe Emulator ===\n");

    /* Load configuration */
    EmuConfig cfg;
    std::string config_path = "config/lathe.toml";
    if (argc > 1) config_path = argv[1];
    loadConfig(config_path, cfg);

    /* Scripted scenario mode: force the carriage coupled to the leadscrew so
     * the firmware's sync/takeup drives it without manual half-nut engagement. */
    const char *scenario = getenv("EMU_SCENARIO");
    if (scenario != nullptr) {
        cfg.z_half_nut_engaged = true;
        /* The emulator ISR runs at isr_rate_hz (10kHz), 10x slower than the
         * 100kHz the servoCycles math assumes, so the real pulse rate is
         * isr_rate/servoCycles. Set max_speed high enough that servoCycles=1
         * (one pulse per ISR tick = ~10k steps/s) and a brisk acceleration so
         * the servo can follow the spindle and retract quickly. */
        cfg.servo_max_speed   = 100000;
        cfg.servo_acceleration = 50000;
        /* Wide Z travel + a high start so several pitches of feed fit without
         * hitting the carriage limits. */
        cfg.z_min_mm     = -500.0;
        cfg.z_max_mm     =  500.0;
        cfg.z_initial_mm =  100.0;
        /* Realistic non-trivial lash by default (~0.6mm, the measured lathe
         * value). All takeup values tested should EXCEED this (the valid
         * operating regime). Override with EMU_LASH_MM. */
        const char *bl = getenv("EMU_LASH_MM");
        cfg.z_backlash_mm = bl ? atof(bl) : 0.6;
    }

    /* Initialize emulator hardware state */
    memset(&emu_hw, 0, sizeof(emu_hw));
    pthread_mutex_init(&emu_hw.mutex, nullptr);

    /* Set TC flag so Modbus sendTxBuffer busy-wait completes immediately */
    extern USART_TypeDef emu_usart1;
    emu_usart1.SR = USART_SR_TC;

    /* Initialize physics model */
    LathePhysics physics(cfg);

    /* Initialize the firmware's rampsHandler_t struct */
    static rampsHandler_t rampsData;
    memset(&rampsData, 0, sizeof(rampsData));

    /* Wire timer handles to scales (same mapping as firmware's main.c). */
    extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
    TIM_HandleTypeDef *htims[SCALES_COUNT] = { &htim1, &htim2, &htim3, &htim4 };
    for (int i = 0; i < SCALES_COUNT; i++) {
        rampsData.shared.scales[i].timerHandleSlot = (uint32_t)i;
        ramps_timer_handles[i] = htims[i];
    }

    /* Set synchro timer and UART handles */
    rampsData.synchroRefreshTimer = &htim9;
    rampsData.modbusUart = &huart1;

    /* Apply config defaults to servo */
    rampsData.shared.servo.maxSpeed = (float)cfg.servo_max_speed;
    rampsData.shared.servo.acceleration = (float)cfg.servo_acceleration;

    /* Default sync ratios */
    for (int i = 0; i < SCALES_COUNT; i++) {
        rampsData.shared.scales[i].syncRatioNum = 1;
        rampsData.shared.scales[i].syncRatioDen = 100;
    }

    /* Initialize transport */
    Transport transport(cfg);
    transport.start();

    /* Initialize the firmware (Modbus, tasks, etc.) */
    printf("Starting firmware initialization...\n");
    RampsStart(&rampsData);
    printf("Firmware initialized.\n");

    /* NOTE: the scaleDir/servoDir REGISTERS keep their canonical +1 firmware
     * defaults (RampsStart) here. They represent the HOST's (reflex-ui's)
     * direction canonicalization, which the UI writes on connect from its
     * Reverse toggles. The `*_scale_dir`/`servo_dir` config keys instead drive
     * the PHYSICAL wiring signs in the physics model (see physics.cpp) — the
     * thing those toggles must cancel. Preloading the registers from the same
     * config double-applied the sign (self-cancelling in dashboard mode), so we
     * no longer do it. In dashboard/standalone mode (no host) an inverted wiring
     * therefore shows as an inverted DRO — the honest uncommissioned-machine view.
     */

    /* Initialize servoCycles to avoid division by zero on first ISR tick.
     * On real hardware updateSpeedTask() sets this within 50ms of boot,
     * but the emulator's ISR thread starts immediately.
     * Formula from updateSpeedTask: clock_freq / maxSpeed
     * clock_freq = 100MHz / (Prescaler+1) / (Period+1) = 100000
     */
    {
        extern uint16_t servoCycles;
        float clock_freq = 100000000.0f /
            ((float)rampsData.synchroRefreshTimer->Init.Prescaler + 1) /
            (float)(rampsData.synchroRefreshTimer->Init.Period + 1);
        float period = (cfg.servo_max_speed > 0) ? floorf(clock_freq / (float)cfg.servo_max_speed) : 138.0f;
        if (period < 1.0f) period = 1.0f;
        if (period > 65535.0f) period = 65535.0f;
        servoCycles = (uint16_t)period;
    }

    /* Install signal handler */
    signal(SIGINT, signalHandler);

    /* Start ISR thread */
    std::thread isrThread(isrThreadFunc, &physics, &rampsData, cfg.isr_rate_hz);

    emu_log_event("Emulator started");

    if (scenario != nullptr && strcmp(scenario, "serve") == 0) {
        /* Integration harness: physics only; reflex-ui drives ELS over Modbus. */
        runPhysicsServer(&physics, &rampsData, cfg);
    } else if (scenario != nullptr) {
        /* Scripted scenario instead of the interactive dashboard. */
        runElsScenario(&physics, &rampsData, cfg);
    } else {
        /* Run dashboard on main thread */
        Dashboard dashboard(cfg, physics, transport, rampsData.shared);
        emu_log_event("PTY: %s", transport.getPtyPath().c_str());
        if (cfg.tcp_enabled) emu_log_event("TCP: port %d", cfg.tcp_port);
        dashboard.run();
    }

    /* Shutdown */
    g_running.store(false);
    if (isrThread.joinable()) isrThread.join();
    transport.stop();

    printf("Emulator stopped.\n");
    return 0;
}
