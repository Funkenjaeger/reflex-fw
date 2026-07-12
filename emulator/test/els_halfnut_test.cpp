/*
 * Half-nut engagement physics tests (LathePhysics).
 *
 * Exercises the engagement lattice directly: the moving/stationary gate must
 * be driven by step-pulse activity (NOT the firmware's servo.currentSpeed,
 * which is pinned to 0 during pure ELS sync), a moving engage must fire only
 * when the carriage sits on the leadscrew thread lattice (once per rev — the
 * old phase check double-counted the leadscrew offset and fired twice per rev
 * at off-lattice positions), and a moving engage must seat the residual onto
 * the exact lattice with the lash wall opposite the physical drive direction.
 * The retained stationary snap must WARN when it teleports the carriage more
 * than the phase tolerance, and drops the nut at an arbitrary position within
 * the lash window.
 *
 * Links physics.cpp + config.cpp with local stubs for the shim externals
 * (emu_hw / emu_log_event / emu_update_timer_counters) — hal_shim.c is NOT
 * linked. Pulses are fed in the production order used by isrThreadFunc:
 * tick(dt) first, then onStepPulse().
 *
 * Build/run: compiled as the `els_halfnut_test` CTest target (see CMakeLists).
 */
#include "physics.h"

extern "C" {
#include "emulator_state.h"
#include "Ramps.h"
}

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>

/* --- Shim stubs (physics.cpp's only externals) --- */

EmulatorHardwareState emu_hw;

static std::vector<std::string> g_log;

extern "C" void emu_log_event(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_log.push_back(buf);
}

extern "C" void emu_update_timer_counters(void) {}

static bool logContains(const char *needle) {
    for (const auto &s : g_log)
        if (s.find(needle) != std::string::npos) return true;
    return false;
}

/* --- Fixture --- */

/* Reference lathe.toml geometry (ctor defaults differ — mm_per_step matters):
 * 8 TPI leadscrew -> grid = 3.175 mm; 0.00396875 mm/step -> exactly 800
 * steps per grid; lash 0.635 mm -> exactly 160 steps. */
static const double GRID = 25.4 / 8.0;             /* 3.175 mm */
static const double STEP = 0.00396875;             /* mm per leadscrew step */
static const int    STEPS_PER_GRID = 800;
static const double LASH = 0.635;                  /* = 160 steps */
static const double DT   = 1e-4;                   /* 10 kHz ISR tick */
static const double TOL_FRAC = 0.02;               /* must match PHASE_TOL_FRAC */

static EmuConfig makeConfig(double z_initial, int servo_dir = 1) {
    EmuConfig cfg;                       /* pure-assignment defaults */
    cfg.leadscrew_tpi = 8.0;
    cfg.leadscrew_mm_per_step = STEP;
    cfg.z_encoder_counts_per_mm = 400.0;
    cfg.z_backlash_mm = LASH;
    cfg.z_min_mm = -500.0;
    cfg.z_max_mm = 500.0;
    cfg.z_initial_mm = z_initial;
    cfg.z_half_nut_engaged = false;
    cfg.servo_dir = servo_dir;
    cfg.spindle_initial_rpm = 0.0;
    return cfg;
}

/* Zeroed firmware shared state passed to tick(). servo.currentSpeed == 0 is
 * the load-bearing part: during a pure sync cut the ramp variable reads 0, so
 * any gate consulting it would wrongly classify the leadscrew as stationary. */
static rampsSharedData_t g_shared;

static void ticks(LathePhysics &p, int n) {
    for (int i = 0; i < n; i++) p.tick(DT, &g_shared);
}

/* Sync-shaped drive: `ticks_per_pulse` ticks, then one step pulse. Returns
 * after `npulses` pulses or as soon as `stopWhenEngaged` sees ENGAGED
 * (checked post-tick, pre-pulse — the tick is where engagement is decided).
 * Returns the number of pulses actually delivered. */
static int pulseTrain(LathePhysics &p, int dir, int npulses,
                      int ticks_per_pulse, bool stopWhenEngaged) {
    for (int i = 0; i < npulses; i++) {
        for (int t = 0; t < ticks_per_pulse; t++) {
            p.tick(DT, &g_shared);
            if (stopWhenEngaged && p.getHalfNutState() == LathePhysics::ENGAGED)
                return i;
        }
        p.onStepPulse(dir);
    }
    return npulses;
}

/* Establish step activity so the gate reads "moving" BEFORE the engage
 * request (a request on a fresh sim correctly takes the stationary snap
 * path). Advances the leadscrew by WARMUP_PULSES * STEP — account for it in
 * any travel prediction. */
static const int WARMUP_PULSES = 8;
static void warmGate(LathePhysics &p, int dir) {
    pulseTrain(p, dir, WARMUP_PULSES, 3, false);
}

static int failures = 0;
#define CHECK(cond, ...) do { \
    if (cond) { printf("  [PASS] " __VA_ARGS__); printf("\n"); } \
    else { failures++; printf("  [FAIL] " __VA_ARGS__); printf("\n"); } \
} while (0)

/* T1: gate must be driven by pulse activity, not servo.currentSpeed.
 * Sync-shaped pulses with currentSpeed == 0 must count as "moving": the
 * engage request must enter ENGAGING (phase-match wait), not instantly
 * snap-engage, and the carriage must not teleport. */
static void testGateUsesPulseActivity() {
    printf("T1: moving gate driven by step pulses (currentSpeed == 0)\n");
    LathePhysics p(makeConfig(10.0 * GRID + 0.3 * GRID));
    g_log.clear();
    double z0 = p.getCarriageMM();

    pulseTrain(p, +1, 3, 80, false);        /* establish pulse activity */
    p.requestHalfNutToggle();
    pulseTrain(p, +1, 5, 80, false);        /* far from alignment (0.3 rev away) */

    CHECK(p.getHalfNutState() == LathePhysics::ENGAGING,
          "engage during sync pulses -> ENGAGING (state=%d)", (int)p.getHalfNutState());
    CHECK(std::abs(p.getCarriageMM() - z0) < 1e-12,
          "carriage not teleported (z=%.6f)", p.getCarriageMM());
    CHECK(!logContains("snap"), "no stationary-snap log entry");
}

/* T2: moving engage fires only at the lattice crossing — once per leadscrew
 * rev — and seats the carriage exactly on-lattice. For a start offset of
 * f*GRID and +dir drive, the crossing is at f*GRID of leadscrew travel; the
 * tolerance opens the window (f - TOL_FRAC)*GRID early. The old double-count
 * bug fired at f/2*GRID (off-lattice) instead. */
static void testEngagesOnLatticeOncePerRev() {
    printf("T2: engages at the unique lattice crossing, exactly on-lattice\n");
    const double fs[] = {0.1, 0.25, 0.4, 0.6, 0.75, 0.9};
    for (double f : fs) {
        LathePhysics p(makeConfig(10.0 * GRID + f * GRID));
        g_log.clear();
        warmGate(p, +1);
        /* Warm-up advanced the leadscrew: remaining phase distance shrinks. */
        double fr = f - (double)WARMUP_PULSES / STEPS_PER_GRID;
        p.requestHalfNutToggle();
        int pulses = pulseTrain(p, +1, 2 * STEPS_PER_GRID, 3, true);
        double travel = pulses * STEP;
        double lo = (fr - TOL_FRAC) * GRID - STEP;
        double hi = fr * GRID;
        CHECK(p.getHalfNutState() == LathePhysics::ENGAGED,
              "f=%.2f: engaged", f);
        CHECK(travel >= lo && travel <= hi,
              "f=%.2f: travel %.4f in [%.4f, %.4f] (old bug: %.4f)",
              f, travel, lo, hi, f / 2.0 * GRID);
        double snapped = p.nearestGridPositionMM(p.getCarriageMM());
        CHECK(std::abs(p.getCarriageMM() - snapped) < 1e-9,
              "f=%.2f: carriage seated on-lattice (dz=%.2e)",
              f, p.getCarriageMM() - snapped);
    }
}

/* T3: moving engage sets the lash wall OPPOSITE the physical drive direction
 * (worst case: the full lash transient runs before the carriage moves).
 * backlash_offset == 0 is the -wall, so +drive -> 0.0, -drive -> LASH.
 * The servo_sign=-1 variant catches storing raw DIR instead of phys_dir. */
static void testLashWallBothDirections() {
    struct Case { int servo_dir, dir; double wall; const char *name; };
    const Case cases[] = {
        {+1, +1, 0.0,  "phys + (dir=+1, sign=+1) -> -wall"},
        {+1, -1, LASH, "phys - (dir=-1, sign=+1) -> +wall"},
        {-1, -1, 0.0,  "phys + (dir=-1, sign=-1) -> -wall"},
    };
    printf("T3: lash wall opposite drive direction, full transient\n");
    for (const Case &c : cases) {
        int phys = c.dir * c.servo_dir;
        LathePhysics p(makeConfig(10.0 * GRID + 0.5 * GRID, c.servo_dir));
        warmGate(p, c.dir);
        p.requestHalfNutToggle();
        pulseTrain(p, c.dir, 2 * STEPS_PER_GRID, 3, true);
        CHECK(p.getHalfNutState() == LathePhysics::ENGAGED, "%s: engaged", c.name);
        CHECK(std::abs(p.getBacklashOffsetMM() - c.wall) < 1e-12,
              "%s: lash=%.4f (want %.4f)", c.name, p.getBacklashOffsetMM(), c.wall);

        /* Full lash transient: 160 steps of travel before the carriage moves.
         * Half-step slack around the boundary pulse (float accumulation). */
        double z0 = p.getCarriageMM();
        pulseTrain(p, c.dir, 155, 3, false);
        CHECK(std::abs(p.getCarriageMM() - z0) < STEP / 2.0,
              "%s: carriage still parked after 155 lash steps", c.name);
        pulseTrain(p, c.dir, 10, 3, false);
        double expect = phys * (165 * STEP - LASH);
        CHECK(std::abs((p.getCarriageMM() - z0) - expect) < 1e-6,
              "%s: moved %.5f after 165 steps (want %.5f)",
              c.name, p.getCarriageMM() - z0, expect);
    }
}

/* T4: stationary engage still snaps immediately (never waits for rotation),
 * WARNs when the snap displaces more than the tolerance, stays silent when
 * near-lattice, and drops the nut at a varying position within the lash. */
static void testStationarySnapWarning() {
    printf("T4: stationary snap warning + random lash drop-in\n");
    {
        LathePhysics p(makeConfig(10.0 * GRID + 0.4 * GRID));
        g_log.clear();
        ticks(p, 5);                        /* no pulses ever: stationary */
        p.requestHalfNutToggle();
        ticks(p, 2);
        CHECK(p.getHalfNutState() == LathePhysics::ENGAGED, "instant snap engage");
        CHECK(logContains("WARN"), "0.4*grid displacement logged a WARN");
        double snapped = p.nearestGridPositionMM(p.getCarriageMM());
        CHECK(std::abs(p.getCarriageMM() - snapped) < 1e-9, "snapped on-lattice");
        CHECK(p.getBacklashOffsetMM() >= 0.0 && p.getBacklashOffsetMM() <= LASH,
              "drop-in lash %.4f within [0, %.3f]", p.getBacklashOffsetMM(), LASH);
    }
    {
        LathePhysics p(makeConfig(10.0 * GRID + 0.005 * GRID));
        g_log.clear();
        ticks(p, 5);
        p.requestHalfNutToggle();
        ticks(p, 2);
        CHECK(p.getHalfNutState() == LathePhysics::ENGAGED, "near-lattice engage");
        CHECK(!logContains("WARN"), "no WARN within tolerance");
        CHECK(logContains("ENGAGED (snap"), "normal snap-engage log present");
    }
    {
        /* Drop-in position varies across engagements (PRNG draws). */
        LathePhysics p(makeConfig(10.0 * GRID));
        ticks(p, 5);
        std::vector<double> draws;
        for (int i = 0; i < 5; i++) {
            p.requestHalfNutToggle();       /* engage */
            ticks(p, 2);
            draws.push_back(p.getBacklashOffsetMM());
            p.requestHalfNutToggle();       /* disengage */
            ticks(p, 2);
        }
        bool varied = false;
        for (double d : draws)
            if (std::abs(d - draws[0]) > 1e-6) varied = true;
        CHECK(varied, "drop-in position varies across engagements");
    }
}

/* T5: ENGAGING falls back to the snap path when pulses pause longer than the
 * activity window (e.g. an ELS stop fires mid-request) — the request
 * completes via snap instead of hanging, with the WARN if displaced. */
static void testEngagingDecaysToSnap() {
    printf("T5: mid-ENGAGING pulse pause decays to snap engage\n");
    LathePhysics p(makeConfig(10.0 * GRID + 0.6 * GRID));
    g_log.clear();
    warmGate(p, +1);
    p.requestHalfNutToggle();
    pulseTrain(p, +1, 100, 3, true);        /* well short of the ~456-pulse crossing */
    CHECK(p.getHalfNutState() == LathePhysics::ENGAGING, "still ENGAGING mid-approach");
    ticks(p, 1100);                          /* 0.11 s > activity window */
    CHECK(p.getHalfNutState() == LathePhysics::ENGAGED, "completed via snap");
    CHECK(logContains("WARN"), "snap from 0.475*grid displacement WARNed");
}

/* T6: nearestGridPositionMM — on-lattice, within half a grid, correct with
 * negative leadscrew positions (fmod sign handling). */
static void testNearestGridPosition() {
    printf("T6: nearestGridPositionMM lattice math\n");
    {
        LathePhysics p(makeConfig(0.0));    /* leadscrew at 0: lattice = k*GRID */
        double r = p.nearestGridPositionMM(7.0);
        CHECK(std::abs(r - 2.0 * GRID) < 1e-9, "ls=0: 7.0 -> %.4f (want %.4f)", r, 2.0 * GRID);
        r = p.nearestGridPositionMM(-7.0);
        CHECK(std::abs(r - (-2.0 * GRID)) < 1e-9, "ls=0: -7.0 -> %.4f (want %.4f)", r, -2.0 * GRID);
    }
    {
        /* Drive the leadscrew negative: offset must stay in [0, GRID). */
        LathePhysics p(makeConfig(0.0));
        pulseTrain(p, -1, 300, 2, false);   /* ls = -1.190625 mm */
        double ls = p.getLeadscrewPositionMM();
        CHECK(ls < 0.0, "leadscrew driven negative (%.4f)", ls);
        for (double target : {-10.0, -0.5, 0.0, 4.2, 100.0}) {
            double r = p.nearestGridPositionMM(target);
            CHECK(std::abs(r - target) <= GRID / 2.0 + 1e-9,
                  "target %.1f -> %.4f within grid/2", target, r);
            /* On-lattice: (r - ls) must be an integer number of grids. */
            double phase = std::fmod((r - ls) / GRID, 1.0);
            if (phase < 0.0) phase += 1.0;
            double d = std::abs(phase - std::round(phase));
            CHECK(d < 1e-9, "target %.1f -> on-lattice (phase err %.2e)", target, d);
        }
    }
}

/* T7: a toggle during ENGAGING cancels the pending engage (operator releases
 * the lever before it drops in): back to DISENGAGED, carriage untouched, and
 * no engagement later fires on its own. */
static void testCancelDuringEngaging() {
    printf("T7: toggle during ENGAGING cancels\n");
    LathePhysics p(makeConfig(10.0 * GRID + 0.6 * GRID));
    g_log.clear();
    double z0 = p.getCarriageMM();
    warmGate(p, +1);
    p.requestHalfNutToggle();
    pulseTrain(p, +1, 50, 3, true);
    CHECK(p.getHalfNutState() == LathePhysics::ENGAGING, "in ENGAGING");
    p.requestHalfNutToggle();                /* cancel */
    CHECK(p.getHalfNutState() == LathePhysics::DISENGAGED, "cancelled to DISENGAGED");
    CHECK(logContains("CANCELLED"), "cancel logged");
    pulseTrain(p, +1, 2 * STEPS_PER_GRID, 2, true);  /* through several crossings */
    CHECK(p.getHalfNutState() == LathePhysics::DISENGAGED, "no latent engage");
    CHECK(std::abs(p.getCarriageMM() - z0) < 1e-12, "carriage untouched");
}

int main() {
    memset(&g_shared, 0, sizeof(g_shared));  /* servo.currentSpeed == 0 */
    printf("=== half-nut engagement physics tests ===\n");
    testGateUsesPulseActivity();
    testEngagesOnLatticeOncePerRev();
    testLashWallBothDirections();
    testStationarySnapWarning();
    testEngagingDecaysToSnap();
    testNearestGridPosition();
    testCancelDuringEngaging();
    printf("=== %s (%d failures) ===\n", failures == 0 ? "ALL PASS" : "FAILURES", failures);
    return failures == 0 ? 0 : 1;
}
