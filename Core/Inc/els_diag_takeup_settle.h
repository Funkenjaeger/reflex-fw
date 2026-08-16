/* Take-up settle trace, v2 -- diagnostic probe implementation.
 *
 * ONE PROBE PER HEADER, and every probe implements the SAME four entry points
 * (elsDiagInit / elsDiagArm / elsDiagCaptureStart / elsDiagCapturing /
 * elsDiagTick). els_diag.h
 * decides which header provides them, or supplies no-ops when no probe is
 * selected. See DIAG.md for the pattern and for what this probe measures.
 *
 * Included ONLY via els_diag.h, and only when this probe is the selected one.
 * Nothing here is compiled into a release build.
 *
 * WHAT THIS FILE DOES NOT OWN: where it is called from. The bodies live here,
 * but each call site in Ramps.c sits at one specific instant in the ISR tick and
 * cannot be moved -- this probe measures WHEN things happen within a tick, so a
 * relocated hook measures something else. Each function below records the
 * instant it must be called at, because that constraint is no longer visible
 * from the code around it now that the bodies have moved out.
 */
#ifndef ELS_DIAG_TAKEUP_SETTLE_H
#define ELS_DIAG_TAKEUP_SETTLE_H

#include <stdint.h>
#include <stdbool.h>

/* WHAT THIS PROBE EXISTS TO MEASURE -- the two numbers Ramps.c otherwise GUESSES:
 *
 *   1. How long the carriage actually keeps moving after the last step pulse.
 *      That is what ELS_SLIP_SETTLE_TICKS is supposed to be, and it had never
 *      been measured -- it cannot be derived from the emulator, whose lash model
 *      moves the carriage instantaneously with the pulse, so it has no settle
 *      behaviour to observe at all.
 *   2. Whether Z ever goes properly QUIET, or dithers indefinitely under spindle
 *      vibration. That sets the tolerance band for any future "has the carriage
 *      stopped" test, and a band chosen without it could make a healthy machine
 *      refuse every pass.
 *
 * The capture is a DECIMATED trace, not a raw one: each bucket is the SIGNED sum
 * of dZ over ELS_DIAG_BUCKET_TICKS. Signed matters -- dither around a fixed
 * position cancels while genuine drift accumulates, the same reason a quiescence
 * test must use net displacement rather than summed |dZ|. Summing in the ISR is
 * what makes it affordable: 50 buckets covers 50 ms, where a raw per-tick trace
 * of the same span would be 5000 samples and fit nowhere.
 *
 * v2, after the 2026-08-16 machine run. v1 started at commanded-take-up
 * completion and then ran unconditionally for its whole window -- so it kept
 * recording after the gate confirmed and the PASS STARTED, and most of every
 * trace was the carriage traversing under sync at a flat ~1.9 counts/ms. That is
 * not a settle tail; a settle tail decays. diagSettleTicks was worse than
 * useless: it reported the last nonzero dZ, which once the pass is running is
 * always "just before the window closed" (measured: 4954-4995 out of 5000).
 *
 * v2 ends the capture the moment the servo issues its next step pulse -- exactly
 * the boundary between settling and being driven again -- which makes every
 * field mean what its name says. Buckets dropped 100 ticks -> 10 so the
 * sub-millisecond tail is resolvable: v1 showed 3-6 whole buckets of ZERO at 1 ms
 * each, which bounds the settle below 3 ms but cannot see inside it.
 *
 * RESULT (2026-08-16): the carriage stops dead. Zero dither, settle at the
 * floor. The question is answered; this probe is retained as the worked example
 * for writing the next one. Schema ids live in Ramps.h -- they are part of the
 * register contract reflex-ui mirrors, not a detail of this file. */

/* Bucket width in ISR ticks. ~0.097 ms at the measured 103 kHz ISR rate.
 * PROBE-SPECIFIC: it belongs to this probe's trace geometry, not to the
 * scratchpad, which is why it moved here from Ramps.c with the rest of the
 * probe. The firmware PUBLISHES it in diagBucketTicks so no reader has to know
 * it -- see the note in els_diag.py about not baking rates into the host. */
#define ELS_DIAG_BUCKET_TICKS 10

/* Startup. Publishes which probe is compiled in and its trace geometry, then
 * clears the block.
 *
 * diagSchema comes straight from the selection macro rather than a literal: the
 * register's whole job is to say which probe a reader is looking at, so a
 * hardcoded schema here could disagree with the probe actually compiled in --
 * the one lie this field must never be able to tell.
 *
 * The clears are explicit rather than left to BSS. A host that connects before
 * the first take-up must see "no capture yet", not whatever was in RAM -- and on
 * this part BSS happens to be zero, which would make the invariant look held
 * while nothing actually enforced it. */
static inline void elsDiagInit(elsDiagCtx_t *ctx, elsStop_t *stop) {
  stop->diagSchema       = ELS_DIAG_PROBE;
  stop->diagBucketTicks  = ELS_DIAG_BUCKET_TICKS;
  stop->diagBucketCount  = ELS_DIAG_TRACE_BUCKETS;
  stop->diagCaptureTicks = 0;
  stop->diagEndReason    = 0;
  stop->diagSettleTicks  = 0;
  stop->diagNetCounts    = 0;
  ctx->state             = 0;
  ctx->captureTick       = 0;
}

/* CALL AT TAKE-UP INITIATION, before any pulses are issued.
 *
 * Arms the trace and clears it HERE rather than when the capture starts. That
 * tick already does a pile of one-off work, so a 50-entry clear costs nothing
 * measurable, whereas clearing at capture start would put it on the tick where
 * the take-up completes -- the one tick whose timing this probe exists to
 * measure. DO NOT MOVE THE CALL SITE.
 *
 * The outcome fields are cleared too, so a reader that catches the block
 * mid-flight can never see the PREVIOUS capture's verdict beside this one's
 * trace. */
static inline void elsDiagArm(elsDiagCtx_t *ctx, elsStop_t *stop) {
  ctx->state             = 1;
  ctx->captureTick       = 0;
  stop->diagSettleTicks  = 0;
  stop->diagNetCounts    = 0;
  stop->diagCaptureTicks = 0;
  stop->diagEndReason    = 0;
  for (int b = 0; b < ELS_DIAG_TRACE_BUCKETS; b++) {
    stop->diagTrace[b] = 0;
  }
}

/* CALL ON THE FIRST TICK AT WHICH THE COMMANDED MOTION IS COMPLETE -- the last
 * step pulse, and therefore t=0 for the settle question. This fires BEFORE the
 * ELS_SETTLE_TICKS dwell and before the gate's first evaluation, so the trace
 * covers the whole of both. */
static inline void elsDiagCaptureStart(elsDiagCtx_t *ctx) {
  if (ctx->state == 1) {
    ctx->state       = 2;
    ctx->captureTick = 0;
  }
}

/* Is a capture in flight? Must stay TRIVIAL -- one field compare, no memory
 * beyond the context.
 *
 * It exists so the ISR can skip computing elsDiagTick's arguments on ticks that
 * would discard them. dZ costs two array indexings and a multiply, dServo a
 * subtract; as function arguments they are evaluated before the callee can
 * early-return, so without this guard every tick pays for them whether or not a
 * capture is running. That cost landed in the ISR this probe exists to
 * TIME, which makes it self-defeating rather than merely wasteful -- measured at
 * +128 bytes of ISR when this guard was missing. */
static inline bool elsDiagCapturing(const elsDiagCtx_t *ctx) {
  return ctx->state == 2;
}

/* CALL ONCE PER ISR TICK WHILE elsDiagCapturing(), at the point where dZ is
 * THIS tick's delta.
 *
 * IT ENDS ON THE SERVO'S NEXT PULSE, and that is the whole correction over v1.
 * v1 was deliberately not gated on takeupPending, on the reasoning that the
 * interesting part is what Z does AFTER the gate decides. True, but it ran on
 * unconditionally afterwards -- and what comes after the decision is the PASS,
 * so most of every v1 trace was the carriage traversing under sync at a flat
 * ~1.9 counts/ms. Flat is the tell: a settle tail decays.
 *
 * The next commanded pulse is the exact boundary between "still settling from
 * the take-up" and "being driven again", and it does not depend on which
 * firmware state machine happens to clear which flag first. dServo is checked
 * BEFORE accumulating, so the driving tick itself is never counted as settle. */
static inline void elsDiagTick(elsDiagCtx_t *ctx, elsStop_t *stop,
                               int32_t dZ, int32_t dServo) {
  if (ctx->state != 2) {
    return;
  }

  if (dServo != 0) {
    stop->diagCaptureTicks = (uint16_t)ctx->captureTick;
    stop->diagEndReason    = ELS_DIAG_END_PULSE;
    ctx->state = 0;
    stop->diagSeq++;   /* publish LAST: the seq bump is the ack that the block is complete and readable */
  } else {
    uint32_t bucket = ctx->captureTick / ELS_DIAG_BUCKET_TICKS;
    if (bucket < (uint32_t)ELS_DIAG_TRACE_BUCKETS) {
      /* int16 per bucket. Per-tick dZ is bounded by the int16 cast in the
       * scale-refresh loop and a settle tail is single-digit counts, so a
       * 10-tick bucket has orders of magnitude of headroom. A probe is not
       * worth saturation logic in a 100 kHz ISR. */
      stop->diagTrace[bucket] = (int16_t)(stop->diagTrace[bucket] + dZ);
    }

    stop->diagNetCounts += dZ;
    if (dZ != 0) {
      stop->diagSettleTicks = (int32_t)ctx->captureTick;
    }

    ctx->captureTick++;
    if (ctx->captureTick >= (uint32_t)ELS_DIAG_TRACE_BUCKETS * ELS_DIAG_BUCKET_TICKS) {
      /* Ran out of buckets before the servo moved again. The capture did NOT
       * finish measuring: report that rather than letting a truncated trace
       * pass for a complete one. */
      stop->diagCaptureTicks = (uint16_t)ctx->captureTick;
      stop->diagEndReason    = ELS_DIAG_END_WINDOW;
      ctx->state = 0;
      stop->diagSeq++;
    }
  }
}

#endif /* ELS_DIAG_TAKEUP_SETTLE_H */
