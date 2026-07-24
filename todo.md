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

## Emulator

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
