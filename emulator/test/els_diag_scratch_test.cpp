/*
 * Diagnostic scratchpad: the probe must not leak into an unflagged build.
 *
 * THE RULE THIS ENFORCES. No build that reaches dev-staging, dev or main may
 * define ELS_DIAG_SCRATCH. That rule is meant to be structural rather than
 * documentary -- release builds omit the flag, so the writes do not exist and
 * diagSchema reads 0 -- and this test is what makes the claim checkable instead
 * of merely stated.
 *
 * WHY THE INIT PATH IS THE WHOLE ATTACK SURFACE, and why that is not a gap.
 * The capture code cannot leak on its own: its working state (diagState,
 * diagCaptureTick) lives inside the same #ifdef in rampsHandler_t, so deleting
 * the guard around the capture block does not produce a quiet release build
 * that records data -- it produces a compile error. The one place a leak can
 * happen SILENTLY is RampsStart(), which sets diagSchema in both configurations
 * via #ifdef/#else. Get that wrong and a release build advertises a probe it
 * does not have, which is the failure diagSchema exists to prevent.
 *
 * DELIBERATELY TWO-SIDED. Asserting only "reads 0 without the flag" would pass
 * just as happily against a build where the flag does nothing at all -- a check
 * that cannot distinguish "correctly suppressed" from "wired up wrong" is not
 * worth running. So the flagged build asserts the opposite: schema present, and
 * the trace geometry actually published. Both halves run in CI, because CI
 * builds the emulator suite in the default configuration and this file compiles
 * into either.
 *
 * The reserved BLOCK is unconditional in both configurations by design -- that
 * is what keeps the register offsets stable -- so its presence is not what is
 * under test here. Its size is pinned by reflex-ui's cross-repo register
 * contract test instead.
 *
 * Build/run: `els_diag_scratch_test` CTest target (see CMakeLists).
 */
extern "C" {
#include "Ramps.h"
#include "Scales.h"
#include "emulator_state.h"
}

#include <cstdio>
#include <cstdint>
#include <cstring>

/* Stubs for every Ramps.c external. Same set as els_takeup_confirm_test. */
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

} /* extern "C" */

static int failures = 0;

static void check(bool ok, const char *what)
{
    printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) failures++;
}

int main(void)
{
    static rampsHandler_t data;
    static TIM_TypeDef       tim[SCALES_COUNT];
    static TIM_HandleTypeDef htim[SCALES_COUNT];

    /* Poison the whole handler first. Zeroing it would let a MISSING write pass
     * as a correct zero -- the test would then be incapable of telling
     * "explicitly set to 0" from "never touched", which is exactly the
     * distinction it exists to make. */
    std::memset(&data, 0xA5, sizeof(data));
    std::memset(tim,  0, sizeof(tim));
    std::memset(htim, 0, sizeof(htim));
    for (int i = 0; i < SCALES_COUNT; i++) {
        htim[i].Instance = &tim[i];
        ramps_timer_handles[i] = &htim[i];
        /* Required INPUT to RampsStart -- it indexes ramps_timer_handles by
         * this, so it cannot stay poisoned. Everything the test actually asserts
         * on (the diag block) is left at 0xA5. */
        data.shared.scales[i].timerHandleSlot = (uint32_t)i;
    }
    data.modbusUart = nullptr;

    RampsStart(&data);

    /* Guards the one-time bump. Anything appended to elsStop_t after this must
     * move it again, and reflex-ui's ELS_PROTOCOL_VERSION with it. */
    check(data.shared.elsStop.protocolVersion == 2,
          "protocolVersion is 2 (scratchpad map)");

#ifdef ELS_DIAG_SCRATCH
    /* Pinned to a SPECIFIC schema, not "any nonzero". A probe revision changes
     * what the block's fields mean, and reflex-ui mirrors those meanings -- so
     * bumping it should cost somebody a deliberate edit here rather than sliding
     * through green. This caught the v1 -> v2 bump on 2026-08-16. */
    /* Asserted against the LITERAL wire value, deliberately, not against
     * ELS_DIAG_PROBE. Ramps.c now publishes `diagSchema = ELS_DIAG_PROBE`, so
     * checking those two agree is a tautology that would hold no matter what
     * number came out -- a check structurally incapable of failing. The literal
     * is the number reflex-ui mirrors and refuses when unrecognised, so pinning
     * it here is what makes a renumbering cost a deliberate edit on both sides.
     *
     * The #else is not decoration: it makes "added a probe, wrote no assertions
     * for it" a BUILD failure rather than a target that runs and checks nothing.
     * A new probe must land its own arm here. */
#if ELS_DIAG_PROBE == ELS_DIAG_SCHEMA_TAKEUP_SETTLE_V2
    check(data.shared.elsStop.diagSchema == 2,
          "take-up settle v2 publishes wire schema 2");
    check(data.shared.elsStop.diagBucketCount == ELS_DIAG_TRACE_BUCKETS,
          "take-up settle v2 publishes its bucket count");
#else
#error "this probe has no assertions in els_diag_scratch_test.cpp -- add an arm above"
#endif
    check(data.shared.elsStop.diagEndReason == 0,
          "end reason starts cleared (no stale verdict beside a fresh trace)");
    check(data.shared.elsStop.diagBucketTicks > 0,
          "flagged build publishes bucket width (host must not assume the ISR rate)");
    printf("=== %s (probe build, schema %u) ===\n",
           failures == 0 ? "ALL PASS" : "FAILURES",
           (unsigned)data.shared.elsStop.diagSchema);
#else
    /* THE RELEASE ASSERTION. A non-zero schema here means a probe leaked into a
     * build that must not carry one -- and because protocolVersion does not move
     * for a probe change, nothing downstream would catch it. */
    check(data.shared.elsStop.diagSchema == 0,
          "release build advertises NO probe (diagSchema == 0)");
    check(data.shared.elsStop.diagBucketTicks == 0,
          "release build publishes no bucket width");
    check(data.shared.elsStop.diagBucketCount == 0,
          "release build publishes no bucket count");
    printf("=== %s (release build) ===\n",
           failures == 0 ? "ALL PASS" : "FAILURES");
#endif

    return failures == 0 ? 0 : 1;
}
