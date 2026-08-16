/* Diagnostic probe dispatch.
 *
 * Ramps.c calls five entry points -- elsDiagInit, elsDiagArm,
 * elsDiagCaptureStart, elsDiagCapturing, elsDiagTick -- UNCONDITIONALLY, with no #ifdef at any
 * call site. This header decides what they are:
 *
 *   - probe selected  -> the probe's own header supplies them
 *   - no probe        -> empty inlines here, plus the one thing a release build
 *                        genuinely must do: advertise that it carries no probe
 *
 * WHY NO #ifdef AT THE CALL SITES. The hooks sit at specific instants in
 * the ISR tick and cannot move, so they are permanent features of the control
 * flow whether or not a probe is compiled in. Expressing them as calls that
 * compile to nothing keeps the ISR readable as continuous motion control while
 * leaving the instants marked -- and, more usefully, leaves the DO-NOT-MOVE
 * constraint attached to something a reader can see. Bare #ifdef blocks made
 * those instants look optional; they are not.
 *
 * Cost in a release build: no code. Verified by comparing the release image
 * before and after this refactor -- .text IDENTICAL (36028 bytes) and .data
 * identical; every call compiles to nothing. .bss grows 8 bytes for the
 * always-present elsDiagCtx_t, which shifts subsequent RAM addresses and is
 * therefore NOT bit-identical -- the .bin differs only in relocated .word
 * constants plus one commutative operand swap the compiler chose
 * (vfma.f32 s14,s12,s15 -> s14,s15,s12). Same instructions, same count.
 *
 * ADDING A PROBE: see DIAG.md. Implement all five entry points in
 * els_diag_<name>.h and add an arm to the dispatch below. The entry points are
 * a fixed contract precisely so Ramps.c never learns which probe it is calling.
 *
 * DELIBERATELY NOT A SHARED TRACE FRAMEWORK. This file is dispatch and no-ops,
 * nothing more. There is exactly one probe today, and factoring out "common"
 * trace machinery from a single example would be inventing a seam rather than
 * finding one. When a second probe lands, whatever genuinely repeats between
 * them can move here -- see the open loop.
 */
#ifndef ELS_DIAG_H
#define ELS_DIAG_H

#include <stdint.h>
#include <stdbool.h>

/* Included from Ramps.h AFTER elsStop_t and elsDiagCtx_t are defined; the entry
 * points below take both. elsDiagCtx_t lives in Ramps.h with the rest of the
 * handler state rather than here, because it is ISR-owned handler RAM and its
 * placement at the END of rampsHandler_t is what keeps the release image
 * unchanged -- unused .bss does not reach the .bin. */

#ifdef ELS_DIAG_PROBE

#if ELS_DIAG_PROBE == ELS_DIAG_SCHEMA_TAKEUP_SETTLE_V2
#include "els_diag_takeup_settle.h"
#else
/* Unreachable: Ramps.h rejects an unrecognised probe long before here. Kept so
 * that a probe registered in Ramps.h but never dispatched fails LOUDLY, rather
 * than silently linking a build with no probe entry points at all. */
#error "ELS_DIAG_PROBE is registered but has no header in els_diag.h -- see DIAG.md"
#endif

#else  /* ---------------- release: no probe ---------------- */

/* diagSchema is set in EVERY configuration, not merely left at whatever the
 * struct was initialised with. It is the ONLY thing that tells a reader what the
 * rest of the block means, so "no probe" has to be stated rather than inferred:
 * a reader that finds 0 must not interpret the block. The other two advertise
 * the trace geometry and would be equally misleading if stale. */
static inline void elsDiagInit(elsDiagCtx_t *ctx, elsStop_t *stop) {
  (void)ctx;
  stop->diagSchema      = 0;
  stop->diagBucketTicks = 0;
  stop->diagBucketCount = 0;
}

static inline void elsDiagArm(elsDiagCtx_t *ctx, elsStop_t *stop) {
  (void)ctx; (void)stop;
}

/* Constant false, which is what lets the ISR's guarded block -- including the
 * dZ/dServo arithmetic in its condition-free path -- vanish entirely from a
 * release build rather than merely being skipped at runtime. */
static inline bool elsDiagCapturing(const elsDiagCtx_t *ctx) {
  (void)ctx;
  return false;
}

static inline void elsDiagCaptureStart(elsDiagCtx_t *ctx) {
  (void)ctx;
}

static inline void elsDiagTick(elsDiagCtx_t *ctx, elsStop_t *stop,
                               int32_t dZ, int32_t dServo) {
  (void)ctx; (void)stop; (void)dZ; (void)dServo;
}

#endif /* ELS_DIAG_PROBE */

#endif /* ELS_DIAG_H */
