# Diagnostic probes

Temporary firmware instrumentation, and the rules that keep it from leaking into
a build that runs a lathe.

A **probe** is a block of measurement code compiled in only on request. It writes
to the 64-register *diagnostic scratchpad* reserved at the tail of `elsStop_t` —
reserved in **every** build, so its offset never moves, but written only in a
diagnostic build. In a release build the whole block reads zero.

```bash
./scripts/build.sh --diag=takeup-settle-v2     # build with one probe
./scripts/flash.sh --diag=takeup-settle-v2     # build it and flash it
./scripts/build.sh --diag                      # lists what is available
```

Never on `dev-staging`, `dev` or `main`.

## One probe at a time

Every probe writes the same 64 registers. Two compiled in together would
interleave their fields and produce a capture that looks well-formed and means
nothing.

This is enforced by shape, not by rule. Selection is a single macro holding a
single value — `-DELS_DIAG_PROBE=ELS_DIAG_SCHEMA_<NAME>` — so "two probes at
once" is not a state the build system can express. There is no flag combination
that produces it, and nothing to remember.

The surrounding guards in `Core/Inc/Ramps.h` reject the near misses, all at
compile time:

| You pass | Result |
|---|---|
| nothing | release build, scratchpad reads zero |
| a registered probe | that probe, and only that probe |
| `ELS_DIAG_SCRATCH` by hand | **compile error** — it is derived, never passed |
| a misspelled macro name | **compile error** — would otherwise expand to `0` and silently build a "diagnostic" image carrying no probe |
| a retired probe | **compile error**, naming its replacement |
| an unregistered id | **compile error** |

The misspelling case is the one worth understanding. An undefined identifier
evaluates to `0` in the preprocessor, so `-DELS_DIAG_PROBE=ELS_DIAG_SCHEMA_TAKUP_SETTLE_V2`
would quietly mean "no probe" while the build still called itself diagnostic —
you would flash it, capture nothing, and have no error anywhere to explain why.
Rejecting `NONE` explicitly is what turns that into a build failure.

## Knowing what is running

`elsStop.diagSchema` names the probe compiled in; `0` means none. **A reader must
check it and refuse any id it does not recognise.** `protocolVersion`
deliberately does *not* move when a probe changes — the whole point of reserving
the block is that adding a probe does not change the layout — so `diagSchema` is
the only thing standing between a reader and a plausible number with the wrong
meaning.

reflex-ui mirrors these ids and logs the schema at connect. `scripts/flash.sh`
records the probe in `~/firmware/flashed.json` alongside the git revision, so a
capture pulled later can be traced to what was measuring it.

Schema ids are a wire contract: **append only, never renumber.** A retired id is
never reissued to a different probe — a stale reader that still recognises an old
id must not silently accept new data under it.

## Code layout — the pattern

Probes follow a fixed shape so that adding one is mechanical and `Ramps.c` never
learns which probe it is calling.

| File | Role |
|---|---|
| `Core/Inc/els_diag.h` | dispatch. Includes the selected probe's header, or supplies no-op entry points. **Always** included, from the foot of `Ramps.h`. |
| `Core/Inc/els_diag_<name>.h` | one probe. Its state machine, its constants, its trace geometry. |
| `Core/Inc/Ramps.h` | the scratchpad registers and schema ids (register contract), plus `elsDiagCtx_t` |
| `Core/Src/Ramps.c` | **call sites only** — no probe logic, and no `#ifdef` |

Naming: files `els_diag_*.h`, functions `elsDiag*`, matching the existing
`els_slip.h` / `els_backlash_cal.h` modules this pattern is modelled on.

**Every probe implements the same five entry points.** That fixed contract is
what keeps `Ramps.c` probe-agnostic:

| Entry point | Called at |
|---|---|
| `elsDiagInit(ctx, stop)` | `RampsStart`. Publish `diagSchema` + geometry, clear the block |
| `elsDiagArm(ctx, stop)` | take-up initiation, before any pulses |
| `elsDiagCaptureStart(ctx)` | first tick at which commanded motion is complete |
| `elsDiagCapturing(ctx)` | cheap predicate — see below |
| `elsDiagTick(ctx, stop, dZ, dServo)` | once per ISR tick while capturing |

**`elsDiagCapturing` must stay trivial, and the ISR must call it before
computing `elsDiagTick`'s arguments.** C evaluates arguments before the callee
can early-return, so an unguarded call makes every tick pay for `dZ` (two array
indexings and a multiply) and `dServo` (a subtract) whether or not a capture is
running. That cost lands in the ISR a timing probe exists to measure, which
makes it self-defeating rather than merely wasteful. Measured: **+128 bytes of
ISR** when the guard was missing, versus 20 bytes *smaller* than the pre-refactor
baseline with it.

**Call-site placement is not the probe's to choose.** The bodies live in the
probe header; the instants do not. Each hook sits at one specific point in the
tick — `elsDiagArm` clears at initiation precisely because the capture-start tick
is the one being measured — and a relocated hook measures something else. The
entry-point comments carry that constraint, because it is no longer visible from
the code surrounding the call.

**Release cost is no code.** The no-op entry points, `elsDiagCapturing` returning
a constant `false`, let the whole guarded block vanish. Verified by comparing the
release image across the extraction: `.text` identical at 36028 bytes, `.data`
identical, and the entire instruction-level difference is one commutative
operand swap the compiler chose (`vfma.f32 s14,s12,s15` → `s14,s15,s12`). `.bss`
grows 8 bytes for the always-present `elsDiagCtx_t`, which relocates later RAM
addresses — so the image is functionally identical, not bit-identical.

`elsDiagCtx_t` sits **last** in `rampsHandler_t` on purpose: carrying it in a
release build then cannot shift the offset of any field above it.

This is dispatch and no-ops, **not a shared trace framework**. With one probe in
existence, factoring out "common" trace machinery would be inventing a seam
rather than finding one. When a second probe lands, whatever genuinely repeats
can move into `els_diag.h` — that is the open loop, not an oversight.

## Registry

### `takeup-settle-v2` — schema 2 — **one-off, retained as a worked example**

Measures how long the carriage keeps moving after the ELS take-up finishes, to
put a number on `ELS_SLIP_SETTLE_TICKS`, which until then was a guess.

Capture starts when the take-up completes and ends at the servo's next pulse
(`ELS_DIAG_END_PULSE`) or when the buckets run out (`ELS_DIAG_END_WINDOW`). A
window-full capture *did not finish measuring* — treat its tail as a floor, not a
result.

| Field | Meaning |
|---|---|
| `diagSeq` | increments once per completed capture; edge-detect it, there is deliberately no "in progress" register |
| `diagTrace[50]` | per-bucket **signed** sum of Z counts — signed so encoder dither cancels and real motion does not |
| `diagBucketTicks` | ISR ticks per bucket, published so the host never assumes the ISR rate (this repo has disagreed with itself about that rate by 10×) |
| `diagBucketCount` | populated bucket count |
| `diagSettleTicks` | ticks from capture start to the last tick that saw motion — **the measurement** |
| `diagNetCounts` | signed Z counts across the whole capture |
| `diagCaptureTicks` | how long the capture ran; distinct from `diagSettleTicks`, which is when Z last *moved* |
| `diagEndReason` | `ELS_DIAG_END_*` |

**Result (2026-08-16):** the carriage stops dead — zero dither, settle measured
at the floor. The question it existed to answer is answered.

It is kept rather than deleted because it is the reference implementation for
writing another one, and because the geometry it publishes (`diagBucketTicks`,
`diagBucketCount`) is the pattern every trace-shaped probe should copy. Expect
most probes to be like this one: used once, then inert.

### `takeup-settle` — schema 1 — **RETIRED**

Superseded by v2, which ends the capture at the servo's next pulse instead of a
fixed window. Its id is burned and will not be reused. Selecting it is a compile
error that names the replacement.

## Adding a probe

1. **Register the schema id** in `Core/Inc/Ramps.h`, appending after the highest
   existing id. Never renumber. `scripts/lib/diag.sh` parses these defines, so
   the new probe becomes selectable as `--diag=<lowercase-hyphenated-name>`
   automatically — there is no second list to update.
2. **Add an arm to the `#error` chain** in `Ramps.h` so the id is recognised.
   Without it the build refuses, by design.
3. **Write the probe** in `Core/Inc/els_diag_<name>.h`, implementing all five
   entry points, and add an arm to the dispatch `#if` in `els_diag.h` so it gets
   included. Nothing goes in `Ramps.c`: the call sites are already there and are
   probe-agnostic. See "Code layout" above — especially the `elsDiagCapturing`
   guard, which is load-bearing for ISR cost.
4. **Add a test target** in `emulator/CMakeLists.txt` mirroring
   `els_diag_scratch_takeup_settle_v2_test`, and an assertion arm in
   `emulator/test/els_diag_scratch_test.cpp`. Both are load-bearing: the target
   is what compiles your probe at all, and the test file's `#else #error` makes
   "added a probe, wrote no assertions" a build failure. Pin the **literal** wire
   value, not `ELS_DIAG_PROBE` — asserting the published schema equals the macro
   that set it is a tautology that cannot fail.
5. **Mirror it in reflex-ui**, in all the places — mirroring the id alone is not
   enough, and the failure is silent until you are standing at the lathe:
   - `reflex/utils/devices.py` — the schema constant.
   - `reflex/fsms/els_diag.py` — add it to `KNOWN_SCHEMAS`. **This is the one
     that bites.** The recorder refuses any schema outside that set, so a probe
     registered in firmware but missing here makes the UI log *"firmware reports
     diagSchema=N, which this UI does not recognise"* and go dormant. The
     firmware is fine, the flash is fine, and nothing is recorded.
   - `SCHEMAS_WITH_END_REASON` in the same file, if the probe publishes
     `diagEndReason`.

   Nothing cross-checks these two registries: the register-map contract test
   compares layout (field sets, offsets, types, total size) and says nothing
   about schema ids, so a probe added on one side and forgotten on the other
   passes CI green. Until that check exists, this step is the check.
6. **Document it here**, including whether it is one-off or durable, and what its
   result was once you have one.

Until 2026-08-16 the diagnostic half of the leak-guard test was never compiled by
anything — the comment claimed both halves existed while only the release target
was built. Step 4 is why that cannot recur: a probe with no target is a probe
nothing tests.

## Retiring a probe

Mark the schema line `/* RETIRED -- see <replacement> */`. The scripts parse that
comment and drop it from the selectable list, and the `#error` chain should name
the replacement. Leave the id in place forever.

Retiring a probe does **not** mean deleting its code. A little residue is fine as
long as it cannot activate in a release build — which the guards above already
guarantee. Delete it when it stops being a useful example, not on a schedule.
