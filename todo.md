# reflex-fw — TODO / follow-ups

Tracking file for deferred work, known workarounds, and follow-ups. Add items here
instead of leaving bare `TODO:` comments in code.

---

## Repo admin

### Switch default branch back to `main` once the first release is pushed
- **Why:** `hard-fork` (the old default + fork-import branch) has been retired. The
  default branch is temporarily set to **`dev`** so day-to-day work has a sensible
  default without touching `main` (a push to `main` triggers the release pipeline,
  and we want the first release to be solid first).
- **Do when:** the first real release is ready to ship from `main`.
- **How:** `gh repo edit --default-branch main`, then `git remote set-head origin main`.
- Lineage at the time of the switch: `main` ──(+8)──► (old `hard-fork`) ──► `dev`
  (dev contains everything). Decide whether to advance `main` first.

## Modbus communication

### Modbus RX DMA — shelved, needs on-hardware debugging
- **Status:** the working fix is the **interrupt-mode overrun recovery** on branch
  `fix/modbus-uart-overrun-recovery` (commit `dc86a29`). It is validated: zero CRC
  errors through a real ELS *cutting* pass on the Pi. **This is the one to ship/merge.**
- **Root cause (for context):** Modbus RX ran byte-at-a-time in interrupt mode at the
  lowest NVIC priority (USART1 = 15) and was starved by the 100 kHz step-generation
  ISR (TIM9 → `SynchroRefreshTimerIsr`, NVIC prio 5). Starvation → USART overrun (ORE)
  → dropped bytes → CRC errors + connection flapping. The stopgap recovers from each
  overrun instead of leaving RX stalled until reconnect.
- **DMA attempt (does NOT work):** branch `fix/modbus-rx-dma` (commit `9cb4381`) wires
  USART1_RX to DMA2 Stream2 Ch4 and sets `xTypeHW = USART_HW_DMA`. On the bench the UI
  never connected (slave never responds), so RX DMA isn't delivering frames. Cause not
  found by code inspection — needs a logic analyzer on the RS-485 line and/or an SWD
  debugger to localize. Top suspects, in order:
  1. First-frame IDLE handling — `HAL_UARTEx_ReceiveToIdle_DMA` may need an explicit
     `__HAL_UART_CLEAR_IDLEFLAG` before arming, or IDLE-interrupt enable not taking;
     if the first frame never completes, the slave never responds.
  2. NORMAL-mode re-arm race in `HAL_UARTEx_RxEventCallback` between frame completion
     and re-arming the next receive.
  3. DMA2 Stream2/Ch4 mapping — correct per RM0383 for USART1_RX, but confirm on the
     scope that DMA activity actually occurs.
- **Decision:** DMA is an optional future optimization (prevents overruns vs. recovering
  from them). Only worth pursuing if much higher spindle RPM / step rates during
  threading start producing errors the stopgap can't keep up with — and only with a
  debugger attached, not blind.

### Optional: move Modbus TX to DMA
- TX still uses `HAL_UART_Transmit_IT` (`sendTxBuffer`, Modbus.c). The master tolerates
  inter-byte gaps within its read timeout, so this is fine today. If heavy-load TX-side
  corruption ever appears, move TX to DMA (USART1_TX = DMA2 Stream7 Ch4).

### Implement Modbus function `MB_FC_WRITE_MULTIPLE_COILS`
- `Core/Src/Modbus.c:884` — case is stubbed (`// TODO: implement "sending coils"`).
  Not currently needed by the UI; implement if/when coil writes are used.

### Optional: implement Modbus diagnostic accessors
- `Core/Inc/Modbus.h:271` — a block of original-library prototypes is left
  unimplemented (`getInCnt`/`getOutCnt`/`getErrCnt`/`getID`/`getState`/`getLastError`/
  `setID`/`setTxendPinOverTime`/`ModbusEnd`). The handler already tracks
  `u16InCnt`/`u16OutCnt`/`u16errCnt`, so exposing counters for diagnostics would be cheap
  if we ever want comms health metrics.

---

## Real-time / performance

### Reduce 100 kHz synchro-ISR CPU load
- `SynchroRefreshTimerIsr` (`Core/Src/Ramps.c`) runs every 10 µs (TIM9, 100 kHz) at
  NVIC prio 5 and does non-trivial work every tick (phase correction, elsStop
  bookkeeping, GPIO). This is what starved Modbus RX in the first place. Moving
  non-time-critical work (elsStop state bookkeeping, phase-correction math) out of the
  ISR into a normal FreeRTOS task would free CPU, cut the worst-case ISR duration, and
  reduce starvation pressure on everything else. Keep only the must-happen-now step
  pulse in the ISR.

---

## ELS shoulder stop

### Resume is silently skipped when the carriage hasn't retracted (KNOWN FAILING TEST)
- **Reproduction:** `emulator/test/els_stop_resume_relatch_test.cpp`, CTest target
  `els_stop_resume_relatch_test`. It is RED on purpose; 2 assertions fail. The
  same file contains passing controls, so a green run would mean the bug is fixed.
- If SW resumes by writing `elsStop.active = 0` while the reference axis is still
  at/past `elsStop.stopPosition`, the trigger test at `Core/Src/Ramps.c:403` (inside
  the per-scale loop, so EARLIER in the pass) re-latches `active = 1` before the
  1→0 edge test at `Core/Src/Ramps.c:455` runs. The edge is consumed within the
  pass, so the backlash takeup and `applyPhaseCorrection` are both silently
  skipped — `takeupPending` never sets and `stepsToGo` never moves. The machine
  just looks like it re-stopped.
- **FIXED 2026-08-07 in `aa07cff`.** `elsStop.hysteresis` (`Core/Inc/Ramps.h:102`)
  was declared and never read anywhere in the firmware, while reflex-ui actively
  wrote it — so the 800 it wrote for standalone-stop was a no-op and every stop
  behaved as tight/0. It is now read by the trigger gate.
  Both open questions were decided: retract is measured **from `stopPosition`**,
  and `hysteresis == 0` is an explicit **no-op gate** preserving prior behavior,
  which is what keeps reflex-ui's `0` writes for guided/retract/wizard modes safe.
  Implementation is one tracking field in `rampsHandler_t` plus ~9 lines inside the
  `Ramps.c:403-408` block; `Ramps.c:455` is untouched.
  Emulator ctest went 3/4 → 4/4: the two `a165e07` repro assertions now pass
  **unmodified**, and the gate was mutation-tested in both directions (forcing the
  cleared flag true fails all five new GATE assertions; setting `hysteresis = 0`
  restores the pre-fix outcome exactly).
  Consequence for the ELS auto-start plan: `hysteresis` is now live, so
  `armRetractCounts` must take a **fresh** register rather than reusing this one.
- **NOT proven on hardware.** The emulator has no servo dynamics, no Modbus timing
  and no metal. Do not treat 4/4 as a machine result.

---

## ELS backlash: closed-loop calibration + take-up confirmation (2026-08-08)

### Landed
- The take-up is no longer open loop. Completion now requires **confirmed Z-scale
  motion**, not just "the firmware finished issuing the pulses it decided to
  issue". Fails closed: no motion means `takeupPending` stays set, sync stays
  gated, and `applyPhaseCorrection()` does not run on an uncoupled drivetrain.
  Recovery is the `elsStop.enable` 1->0 escape hatch.
- `calCommand`-driven calibration measures the lash directly (three reversals,
  counting servo steps until Z moves). Host judges consistency and writes
  `backlashSteps = measured + max(20%, floor)`.
- `protocolVersion` register added and checked by reflex-ui at connect.
- `rampsSharedData_t` 264 -> 300 bytes; `KNOWN_ROOT_SIZE` moved in lockstep.
- Superseded local commit `d917641` (a takeup gate built around an operator-set
  `takeupMinZCounts` threshold rather than a measurement). Rewound; preserved on
  branch `archive/d917641-takeup-zconfirm` — its 470-line test is worth mining.

### Two defects found by the ISR-level tests, both real
- **Calibration must gate sync.** `scales[].syncEnable` is independent of
  `elsStop.enable`, so a turning spindle drives the leadscrew straight through
  the measurement. Guarding on `enable` alone was insufficient.
- **Legs must arm on the first pulse in the NEW direction**, not when the
  reversal is commanded. The ramp overshoots while decelerating and that return
  travel is lash traversal; baselining at the command instant measured the
  deceleration transient instead (6 steps against a true lash of 60).

### NOT proven on hardware
- Emulator + host tests only: no servo dynamics, no Modbus timing, no metal.
- The calibration run is the first thing in this codebase that commands
  **bidirectional carriage motion**. Bench it with the tool well clear before it
  ever sees a workpiece.
- `calCeilingSteps` and `calMotionThreshCounts` are machine-specific and have
  never been set on elspi. Defaults are sized for 200 counts/mm Z; the emulator
  is 400 counts/mm.
- Open question for the machine: the measurement reads HIGH by the detection
  distance (~2 Z counts ~= 5 servo steps on elspi). That bias is conservative
  and deliberate, but confirm the real magnitude on metal before tuning the
  margin floor down.

### Commission `ELS_SLIP_SETTLE_TICKS` on elspi (UNMEASURED PARAMETER)

Motion attribution (`Core/Inc/els_slip.h`) replaced the 250 ms confirmation
window as the thing that actually bounds the 2026-08-08 exposure. The number
that bound is now made of — `ELS_SLIP_SETTLE_TICKS` in `Ramps.c`, currently
**1000 ticks (~10 ms at the 100 kHz ISR rate)** — has never been measured on the
machine, and **cannot be measured in the emulator**: its lash model moves the
carriage instantaneously with the pulse, so it has no settle behaviour at all.
The value there satisfies the structural constraints (above `ELS_SETTLE_TICKS`,
above pulse pacing) and nothing more.

To commission it: run a real take-up at the take-up speed actually in use and
watch how long Z counts keep arriving after the last commanded pulse. Set the
horizon just above that. Tune it **down** from 1000 against a machine that still
confirms reliably — never up to make a refusal go away, since the horizon is
exactly the interval in which a hand nudge is still accepted as evidence.

Two unit traps, both live (full list in `els_slip.h`):
- **Ticks, not milliseconds.** The emulator's real-time serve loop drives the
  same ISR ~10x slower than hardware (`emulator/src/main.cpp`), so a horizon
  chosen by wall-clock there is 10x wrong on the lathe.
- **200 vs 400 counts/mm.** The horizon itself is a time quantity and so is
  resolution-independent, but any *counts* threshold eyeballed off emulator
  output is 2x wrong on elspi.

---

## Emulator

### Model manual carriage movement with the half-nut open (TEST-INFRASTRUCTURE GAP)

**Found by hardware, 2026-08-08. This gap is why a real defect in the take-up
confirmation gate was unreachable by any test.**

The emulator (and `els_takeup_confirm_test.cpp`'s `Lash` model) computes the
carriage position as a PURE FUNCTION of `servo.currentSteps`. There are exactly
two worlds: coupled, where Z moves when driven past lash, and uncoupled, where
Z NEVER moves. Reality has a third — **uncoupled but moving anyway**, because
the operator pushed the carriage with the half-nut open.

That third state is not a corner case here. Manual carriage movement with the
nut open is a first-class part of how this machine is used: the whole ELS stop
resume model is built on the operator hand-cranking between passes, and the
interactive re-sync feature (task 6a768a98) is entirely about it.

**What it cost:** the take-up confirmation gate tested "did Z move?" when it
needed to test "did Z move BECAUSE WE DROVE IT?". On hardware, a withheld
take-up was satisfied by the operator nudging the carriage by hand with the nut
open. No test could express that, because the model has no input for carriage
motion that the servo did not cause.

**Needed:**
- ~~The same degree of freedom in the unit-test `Lash` fixture~~ — DONE.
  `Rig::nudgeCarriage()` injects carriage motion the servo did not cause,
  independently of the serve-mode command channel.

  **DECIDED 2026-08-13 (Evan): keep `Rig::nudgeCarriage()` and `LathePhysics`
  separate. Do NOT promote it into the shared model.** This item previously
  recorded the *fact* of the split with no rationale, which is why it kept
  reading as unfinished. The rationale:

  `LathePhysics` already has a carriage-displacement input — `z move` / `z jog`
  via the serve-mode channel, added 2026-08-10 (see the entry below). Crucially
  `moveCarriageTo`/`jogCarriage` **refuse while the nut is ENGAGED**, and that
  refusal is deliberately part of the physics model: "only valid with the nut
  open" is a property of the machine, so it belongs where the machine is
  modelled.

  `Rig::nudgeCarriage()` is a bare `zBase += zCounts` on the ISR-level `Rig`
  with no such guard, and that is correct for what it is. An ISR-level harness
  must be able to inject states the physics model forbids — that is the whole
  point of testing what firmware does when the world misbehaves.

  So promotion has only two outcomes and both are bad: the promoted function
  keeps its lack of a guard, and the physics model acquires a hole that lets
  production paths move the carriage through an engaged nut; or it acquires the
  guard, and the ISR tests can no longer inject the uncoupled-but-moving case
  they exist to cover. They are not duplicates — they model different layers,
  and the missing piece was only ever this paragraph.

  Detail: journal 2026-08-13. **This decision is estate-level, not branch-level
  — the wiki/journal copy is authoritative; this note is a mirror that only
  reaches branches as they merge.**
- ~~Regression case: a withheld take-up must NOT be satisfied by hand motion~~ —
  DONE, and it is what pins motion attribution: `els_takeup_confirm_test.cpp`
  now runs an identical 20-count shove twice inside the same open window, once
  ~50 ticks after the last pulse (confirms — inertia) and once 5000 ticks after
  (refused — a handwheel). Mutation-proven selective: disabling attribution
  reddens the second and leaves the first green.
- STILL OPEN: **a calibration leg must not be satisfied by hand motion either.**
  `elsCalUpdate()` still uses the bare `elsZMotionSeen()` endpoint test, so a
  nudge during a measurement leg is indistinguishable from lash being crossed —
  the same defect shape, in the code path that produces the numbers the take-up
  gate is then judged against. `elsSlipTick()` is the primitive it needs; the
  accumulator is currently only fed while `takeupPending`.
- ~~An external carriage-displacement input to `LathePhysics`~~ — DONE
  2026-08-10. The serve-mode stdin channel (`emulator/src/main.cpp`) gained
  `z move <mm>` / `z jog <-1|0|1>` and `halfnut open` / `halfnut close`.
  Together they express the third state: **uncoupled but moving anyway**.
  `moveCarriageTo`/`jogCarriage` already refuse while the nut is ENGAGED, so
  the "only valid with the nut open" rule stays inside the physics model.

  `halfnut` takes an explicit STATE, not a toggle, and the distinction is
  load-bearing: a toggle issued while an engage is waiting for phase alignment
  CANCELS it, so a toggle-shaped command means different things depending on
  state a test cannot see. `LathePhysics::setHalfNutEngaged()` is idempotent;
  `els_halfnut_test` T8 pins all four transitions, and the naive
  "toggle if it disagrees" implementation reddens 6 of its assertions.

  Serve mode still force-engages the nut at boot for **any** `EMU_SCENARIO`
  value — deliberately left alone. A test that wants the nut open now says so,
  which keeps every existing scenario working unchanged.

- ~~SYSTEM-level regression~~ — DONE 2026-08-10, and it lives in the OTHER
  repo: `reflex-ui/tests/system/test_els_takeup_attribution.py`. Runs the real
  operator cycle (cut a pass → stop → retract → open the nut → press Cut) and
  shoves the carriage mid-window. Mutation-proven: reverting the firmware gate
  to the endpoint comparison reddens the refusal case while the coupled
  positive control stays green.

  Note for whoever moves this next: the take-up does NOT run on the first cut.
  The resume path needs `referenceLatched`, which the FIRMWARE sets at a real
  stop trigger — so a take-up always has a completed pass in front of it. That
  is also precisely the situation the 2026-08-08 defect occurred in.

**Design note for whoever does this:** a fixture that cannot express a failure
makes tests that agree with the code and are wrong together. The take-up gate
assumed coupling in order to test FOR coupling, and the fixture inherited the
same assumption, so both were self-consistent and both were wrong.


### Dashboard: show the real backlash offset
- `emulator/src/dashboard.cpp` Z-axis status line prints a hardcoded
  `backlash: 0.0` (`toDisplay(0.0)`) instead of the physics model's actual nut
  position. Wire it to `physics.getBacklashOffsetMM()` so lash-wall state is
  visible when eyeballing half-nut engagement / takeup behavior.

---

## Hardware workarounds (remove when HW is fixed)

### STEP duplicated on SPARE_2 (PA3) — PCB wiring workaround
- The current PCB has a wiring issue where the primary STEP pin isn't driven, so the
  firmware mirrors every STEP pulse onto SPARE_2 (PA3) as the actual step output. See
  `Core/Src/Ramps.c` `SynchroRefreshTimerIsr` (the paired `STEP_PIN` + `SPARE_2_PIN`
  writes) and pin defs in `Core/Inc/Ramps.h`.
- **Remove the SPARE_2 mirroring once the next HW revision drives STEP correctly**, and
  reclaim PA3 as a real spare.
