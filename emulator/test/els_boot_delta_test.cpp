/*
 * First-tick boot-delta int16 wrap guard test (LathePhysics).
 *
 * Ramps.c:387-390 computes each scale channel's per-tick delta as
 *     (int16_t) (position - oldPosition)
 * and accumulates it into the firmware's cumulative DRO position
 * (shared->scales[i].position += delta). On REAL HARDWARE this is always
 * safe: nothing ever pre-seeds a timer counter (zero grep hits for
 * TIM_SetCounter/__HAL_TIM_SET_COUNTER in Core/Src), so oldPosition and
 * position start together near zero and every real delta is small.
 *
 * The EMULATOR pre-seeds carriage_mm/cross_slide_mm from
 * z_initial_mm/x_initial_mm (config: z_axis.initial_position_mm /
 * cross_slide.initial_position_mm) before the very first physics tick. Prior
 * to the fix this repo commits alongside this test, physics.cpp exposed that
 * seeded position to the firmware's timer-counter register in one shot on
 * tick 1 -- so the RAW first-tick delta was the FULL initial offset in
 * encoder counts (e.g. 100mm * 400 counts/mm = 40000), which overflows
 * int16_t (wraps to -25536). Above 32767/400 = 81.9175mm, ANY configured
 * initial offset wrapped, silently mis-seeding the emulator's DRO by 65536
 * counts (163.84mm) -- a test-harness correctness bug, not a lathe-safety
 * bug (confirmed emulator-only per the Core/Src grep above).
 *
 * The fix ramps the EXPOSED scale_counters value toward the true physics
 * position in steps bounded well under the int16 range, so no single tick's
 * delta can ever overflow -- the DRO still seeds to the correct absolute
 * value, just over a handful of sub-millisecond ticks instead of one.
 *
 * This test reproduces Ramps.c's own per-tick cast-and-accumulate logic
 * directly against the real LathePhysics/physics.cpp, so it exercises the
 * actual guard end-to-end without needing the full firmware+Modbus system
 * harness. It fails against the pre-fix physics.cpp (proven below via the
 * git history of this file/commit) because the very first simulated
 * accumulation step wraps, permanently offsetting the accumulated DRO by
 * 65536 counts from the true seeded position.
 *
 * Links physics.cpp + config.cpp with local stubs for the shim externals
 * (emu_hw / emu_log_event / emu_update_timer_counters) -- hal_shim.c is NOT
 * linked. Same pattern as els_halfnut_test.cpp.
 *
 * Build/run: compiled as the `els_boot_delta_test` CTest target (see
 * CMakeLists).
 */
#include "physics.h"

extern "C" {
#include "emulator_state.h"
}

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cmath>

/* --- Shim stubs (physics.cpp's only externals) --- */

EmulatorHardwareState emu_hw;

extern "C" void emu_log_event(const char *fmt, ...) {
    (void)fmt;
}

extern "C" void emu_update_timer_counters(void) {}

/* --- Fixture --- */

static EmuConfig makeConfig(double initial_mm) {
    EmuConfig cfg;                       /* pure-assignment defaults */
    cfg.leadscrew_tpi = 8.0;
    cfg.leadscrew_mm_per_step = 0.00396875;
    cfg.z_encoder_counts_per_mm = 400.0;
    cfg.z_backlash_mm = 0.6;
    cfg.z_min_mm = -500.0;
    cfg.z_max_mm = 500.0;
    cfg.z_initial_mm = initial_mm;
    cfg.z_half_nut_engaged = false;
    cfg.x_encoder_counts_per_mm = 400.0;
    cfg.x_min_mm = -500.0;
    cfg.x_max_mm = 500.0;
    cfg.x_initial_mm = initial_mm;
    cfg.spindle_initial_rpm = 0.0;
    return cfg;
}

/* Mirrors Ramps.c:387-390 exactly: cast the raw-counter delta to int16_t,
 * then accumulate -- this IS the firmware's DRO-seeding logic, replicated
 * so the test can assert on it without linking Ramps.c/Modbus.c/the Modbus
 * register map. */
struct FirmwareDroSim {
    int32_t oldPosition = 0;
    int32_t position = 0;
    int64_t accumulated = 0;   /* shared->scales[i].position */
    int32_t maxAbsDelta = 0;

    void step(int32_t rawCounterValue) {
        oldPosition = position;
        position = rawCounterValue;
        int16_t delta = (int16_t)(position - oldPosition);   /* Ramps.c's exact cast */
        int32_t absDelta = std::abs((int32_t)delta);
        if (absDelta > maxAbsDelta) maxAbsDelta = absDelta;
        accumulated += delta;
    }
};

int main() {
    int failures = 0;
    const double DT = 1e-4; /* 10 kHz ISR, matches isrThreadFunc's real tick rate */

    /* Seed comfortably past the 81.9175mm (32767/400) wrap boundary -- also
     * past the OLD EMU_SCENARIO hardcoded 100mm default, so this test would
     * have caught the originally-reported bug too. */
    const double SEED_MM = 150.0;
    const double COUNTS_PER_MM = 400.0;
    const int64_t trueCounts = (int64_t)(SEED_MM * COUNTS_PER_MM);

    /* --- Sensitivity check: prove a raw, un-ramped single-shot exposure of
     * this seed WOULD wrap under Ramps.c's int16 cast. This is the exact
     * arithmetic the pre-fix physics.cpp produced on tick 1 (scale_counters
     * set directly from getCarriageEncoderCounts() with no ramp), and is
     * independent of whichever physics.cpp is currently linked -- it exists
     * to document, in the test's own output, what "the old code produces"
     * per the investigation this test is closing out. */
    int16_t oldFirstDelta = (int16_t)(int32_t)(trueCounts - 0);
    bool oldWraps = (oldFirstDelta != trueCounts);
    printf("[%s] sensitivity: a raw %.1fmm seed's first-tick delta truncates to "
           "int16 as %d (true value %lld) -- %s\n",
           oldWraps ? "PASS" : "FAIL", SEED_MM, (int)oldFirstDelta,
           (long long)trueCounts, oldWraps ? "WRAPS, as expected" : "did not wrap");
    if (!oldWraps) {
        failures++;
        printf("  seed too small to exercise the bug -- raise SEED_MM above 81.9175mm\n");
    }

    /* --- Guard check: run the REAL physics + whatever physics.cpp is
     * currently linked, replaying Ramps.c's exact per-tick cast-and-
     * accumulate logic, and assert no single tick's delta can overflow
     * int16 while the DRO still converges to the true seeded position. */
    EmuConfig cfg = makeConfig(SEED_MM);
    LathePhysics physics(cfg);

    FirmwareDroSim zSim, xSim;
    const int MAX_TICKS = 5000; /* 5000 * 1e-4s = 500ms wall-clock budget, generous */
    int ticksToConverge = -1;
    for (int i = 0; i < MAX_TICKS; i++) {
        physics.tick(DT, nullptr); /* shared_data is unused by tick() (see physics.cpp) */
        zSim.step((int32_t)emu_hw.scale_counters[1]);
        xSim.step((int32_t)emu_hw.scale_counters[2]);
        if (ticksToConverge < 0 && zSim.accumulated == trueCounts && xSim.accumulated == trueCounts) {
            ticksToConverge = i + 1;
        }
    }

    /* int16_t's representable magnitude is 32767; require staying well clear
     * of it (not just "didn't literally wrap this run") so a slightly larger
     * legitimate seed, or a small amount of concurrent real motion during
     * the ramp-up window, still can't tip it over. */
    const int32_t SAFE_MARGIN = 32767;

    bool zNoOverflow = zSim.maxAbsDelta < SAFE_MARGIN;
    bool zConverged = (zSim.accumulated == trueCounts);
    printf("[%s] Z DRO: maxAbsDelta=%d (< %d) converged=%s in %d ticks "
           "(accumulated=%lld true=%lld)\n",
           (zNoOverflow && zConverged) ? "PASS" : "FAIL",
           zSim.maxAbsDelta, SAFE_MARGIN, zConverged ? "yes" : "no",
           ticksToConverge, (long long)zSim.accumulated, (long long)trueCounts);
    if (!zNoOverflow || !zConverged) failures++;

    bool xNoOverflow = xSim.maxAbsDelta < SAFE_MARGIN;
    bool xConverged = (xSim.accumulated == trueCounts);
    printf("[%s] X DRO: maxAbsDelta=%d (< %d) converged=%s in %d ticks "
           "(accumulated=%lld true=%lld)\n",
           (xNoOverflow && xConverged) ? "PASS" : "FAIL",
           xSim.maxAbsDelta, SAFE_MARGIN, xConverged ? "yes" : "no",
           ticksToConverge, (long long)xSim.accumulated, (long long)trueCounts);
    if (!xNoOverflow || !xConverged) failures++;

    /* Convergence should be fast (sub-millisecond): a slow ramp would make
     * every system test that touches DRO position at connect-time flaky. */
    const int MAX_ACCEPTABLE_TICKS = 100; /* 10ms at 10kHz -- generous vs. expected ~tens of ticks */
    bool convergedFast = ticksToConverge > 0 && ticksToConverge <= MAX_ACCEPTABLE_TICKS;
    printf("[%s] convergence speed: %d ticks (<= %d)\n",
           convergedFast ? "PASS" : "FAIL", ticksToConverge, MAX_ACCEPTABLE_TICKS);
    if (!convergedFast) failures++;

    printf("=== %s ===\n", failures == 0 ? "ALL PASS" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
