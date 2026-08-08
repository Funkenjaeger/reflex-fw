# AGENTS.md — Reflex Firmware

## Branching and Hardware Verification — READ FIRST

**This project drives a real lathe. The only complete test is on hardware, and
Evan runs that, not on demand.** The emulator and the test suite are good and
getting better, but they have repeatedly looked green while something real was
wrong — no servo dynamics, no Modbus timing, no metal. Emulator green is
evidence, never verification.

**Do NOT commit directly to `dev-staging`.** It is one step from a dev release
and everything on it is supposed to be hardware-verified.

- Work on a **feature branch**, or on **`integration`** when several changes are
  in flight and separate branches would just be overhead.
- `integration` / feature branch → `dev-staging` is merged **only after Evan has
  verified on hardware**. He does that merge, or explicitly asks for it.
- `dev-staging` → `dev` and `dev` → `main` are **Evan's alone**. Never do these.

**The one exception**, for changes that cannot affect machine behaviour and so
need no hardware run: documentation, comments, `todo.md`, tests, and
emulator-only code. Anything touching `Core/` is NOT clerical, however small it
looks or however well tested — `Core/Src/Ramps.c` is the ISR that moves the
machine.

If unsure whether a change qualifies, it does not. Put it on a branch and ask.

**Never push without being asked.** `origin` fans out to BOTH
`github.com/Funkenjaeger/reflex-fw` and `dserver:/mnt/git/reflex-fw.git`, so any
push writes two remotes at once. Note also that git only *fetches* from GitHub,
so there is no tracking ref for dserver and `--force-with-lease` cannot protect
it — a force-push needs an explicit
`--force-with-lease=<branch>:<expected-sha>` aimed at the dserver URL directly,
or it fails with "stale info" after GitHub has already moved.

Record hardware-verification points in `todo.md` so the next session knows what
has actually been proven on metal. Last verified: **2026-08-03 ~20:00**.

## Todo Tracking

When you encounter a task, follow-up item, or piece of work that should be tracked, add it to `todo.md` in the project root. This applies to:
- Deferred work discovered during development
- `TODO`/`FIXME` comments that appear in code or documentation
- Bugs or improvements identified during debugging
- Hardware workarounds that should be removed on a future board revision
- Any action item that won't be completed in the current session

Do NOT leave `TODO`/`FIXME` comments in code, documentation, or bash snippets — always route them to `todo.md` instead.

## Project Overview

Reflex Firmware is the STM32F411 firmware for a rotary controller board. It provides Digital Read Out (DRO) and single-axis control for CNC-style rotary tables, communicating with the host UI over RS-485/Modbus RTU.

## Sibling Repos

This project is tightly coupled with **reflex-ui**, the Python/Kivy host application.

- **Interface:** RS-485 Modbus RTU — the entire `rampsSharedData_t` struct is memory-mapped to Modbus holding registers
- **Version compatibility:** For released versions, matching major.minor implies UI↔FW compatibility. For dev branches, assume the latest commit on each repo's respective branch is compatible. Cross-repo changes affecting the Modbus register interface are called out in commit messages.
- **Finding the UI repo:** The reflex-ui repository may be cloned adjacent to this one. If you can't locate it, ask the user for the path. Once found, persist the location using whatever memory or persistence mechanism is available so you don't need to ask again.

## Agent Provisioning

This project has a sibling repository (reflex-ui) that agents may need to reference.
If your runtime supports workspace or permission configuration, grant read access to the sibling repo path.
For opencode, this means configuring `external_directory` permission in your project config to allow access to the reflex-ui repository.

## Building

### Firmware (STM32F411, arm-none-eabi toolchain)
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
Outputs: `build/reflex.elf`, `.hex`, `.bin`.

### Emulator (native, x86_64)
```bash
cmake -S emulator -B emulator/build
cmake --build emulator/build
```
Outputs: `emulator/build/lathe-emulator`.

### Always build both after code changes
After any changes to `Core/Src/Ramps.c`, `Core/Inc/Ramps.h`, or other firmware sources, build **both** targets and fix any issues before proceeding. Ensure **all build artifacts stay in `build/` directories** — nothing should leak into the repo root or `emulator/` directory.

## Architecture

### Key files
- `Core/Src/Ramps.c` — core motion control (ISR, indexing, jog, ELS)
- `Core/Inc/Ramps.h` — shared data structs, Modbus-mapped
- `Core/Src/Modbus.c` — Modbus RTU slave (addr 17)
- `Core/Src/Scales.c` — encoder timer init

### Concurrency
- TIM9 ISR (`SynchroRefreshTimerIsr`) handles all motion control at ~100 µs ticks
- FreeRTOS tasks handle Modbus, speed updates, motor enable
- `rampsSharedData_t` is the shared state, memory-mapped to Modbus registers

### Servo modes
- 0: disabled
- 1: indexing (trapezoidal ramp, sync-driven)
- 2: jogging (continuous speed)

### ELS (Electronic Limit Switch) threading
- `elsStop.enable = 1` starts a threading job
- Sync gates on/off via `elsStop.active` and `elsStop.takeupPending`
- Resume sequence (active 1→0): reset state → backlash takeup → phase correction → sync resume
- Phase correction folds to ±pitch/2, then constrained to cutting direction (Fix 3, 2026-06-16)
- Sync un-gates before correction move completes (Fix 4, 2026-06-16)
- `stepsToGo`/`currentSpeed` reset before takeup (Fix 1, 2026-06-16)

### Modbus register layout
The entire `rampsSharedData_t` struct is directly memory-mapped to Modbus holding registers. Adding fields shifts offsets and breaks host compatibility.

### ISR tick order (critical for timing)
1. Reset STEP pin
2. Update execution interval
3. Handle ELS enable/active state transitions
4. Check takeup completion → phase correction
5. Read encoders, compute sync deltas, check ELS trigger
6. Run indexing/jog ramp
7. Generate step pulses
8. Update servoCyclesCounter
