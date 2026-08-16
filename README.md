# Reflex Firmware

This repository contains the **firmware** for a digital controller board based on the **STM32F411** microcontroller. It provides Digital Read Out (DRO) and Electronic Leadscrew (ELS) functionality for lathes when integrated with the corresponding [Reflex UI software](https://github.com/Funkenjaeger/reflex-ui).

This firmware handles all necessary low-level real-time control functionality for operations including:
 - Jogging (with trapezoidal velocity profile)
 - Spindle-synchronized carriage feed for controlled feeding or threading operations (standard ELS functionality)
 - Automatic electronic stop, usable when feeding or threading
 - Electronic retract
 - Automatic phase re-sync to thread pitch between passes (allows free use of half nut between threading passes)

This firmware is based on the [rotary-controller-f4](https://github.com/bartei/rotary-controller-f4) project and as of the present version, remains compatible with the associated hardware.  
This project (along with the corresponding UI SW project) was hard forked from the original primarily due to natural divergence that followed from a focus on lathe use cases, where the original rotary-controller project was designed for CNC-style rotary table use cases.

---

## ⚙️ Features

* Utilizes **STM32CubeMX** for hardware configuration (.ioc file included)
* Modular firmware structure with FreeRTOS support
* Programmed over SWD with an ST‑Link V2
* Optimized for high-speed encoder + stepper/servo motor control
* Includes a native FW+lathe emulator for hardware-free testing with the Python GUI

---

## 🛠️ Build & Flash

Build and flash on **the machine with the ST-Link plugged into it**. For this
project that is the Pi that also runs the UI, which is perfectly capable of
compiling the firmware — and doing both in one place means the binary on the
target cannot be a different revision from the checkout in front of you.
`git rev-parse HEAD` there *is* what is flashed.

### Requirements

```bash
sudo apt install gcc-arm-none-eabi cmake build-essential openocd
```

Plus an ST-Link v2 on USB. The `openocd` package installs udev rules granting
the `plugdev` group access, so flashing needs no `sudo`.

### Build and flash

```bash
./scripts/flash.sh
```

That builds, flashes over SWD, and records what it did.

> **Power-cycle the controller after flashing.** A reset alone does not reliably
> start the new firmware on this board. openocd's `Verified OK` confirms the
> flash *contents*, not what the core is *executing* — so programming and
> verification both report success while the machine keeps running the previous
> firmware, silently and with no error anywhere. Confirm from the reflex-ui log
> that `Firmware register protocol version N (expected N)` matches what you
> flashed before believing it took.

```bash
./scripts/flash.sh --diag=NAME   # with a diagnostic probe compiled in
./scripts/flash.sh --dry-run     # everything except the write
./scripts/build.sh               # build only, no flashing
./scripts/build.sh --diag        # lists the available probes
```

**It rebuilds every time by default.** `--no-build` opts out. A stale binary is
the easiest mistake to make and the hardest to notice; rebuilding costs seconds.

**`--host NAME` builds here and flashes there** over SSH, for the case where the
probe host genuinely cannot build. It adds a copy and a checksum — a transfer
that can silently truncate is worth verifying before it is written to the
controller of a machine with moving parts. Prefer the local path: it makes that
whole failure mode, and the version ambiguity that comes with it, not exist.

**Release and diagnostic builds live in separate directories.** `build/` is the
release firmware; each `--diag=NAME` build gets its own directory, so the flag
can never depend on what the last `cmake` invocation happened to say. A
diagnostic build compiles in **one** measurement probe and must **never** reach
`dev-staging`, `dev` or `main`. At runtime the `elsStop.diagSchema` register says
which probe is running (`0` = none), and the UI logs it at connect.

Which probes exist, what they measure, how to add or retire one, and why only one
can be compiled in at a time: **[DIAG.md](DIAG.md)**. `./scripts/build.sh --diag`
with no name lists them.

**Every flash is recorded** in `~/firmware/flashed.json` on the probe host: UTC
timestamp, variant, git revision, whether the tree was dirty, and the ELF's MD5.
Working out what firmware was on this lathe once took an afternoon of forensics
across build-artifact timestamps; this makes it a lookup.

### Underneath

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c 'transport select swd' -c 'program build/reflex-fw.elf verify reset exit'
```

OpenOCD rather than `st-flash`: it takes the ELF directly (load addresses come
from the headers, so there is no `--format`/base-address to get wrong), it is
markedly more tolerant of ST-Link **clones**, and it is the same tool that would
drive a GPIO-bitbanged probe if the boards are ever respun without a dongle.

> Bitbanging SWD from a Raspberry Pi's GPIO used to be documented here via
> `raspberry.cfg`. It has been removed: that config uses OpenOCD's
> `bcm2835gpio` driver, which memory-maps the GPIO block on the SoC — and the
> Pi 5 moved GPIO onto the RP1 southbridge, so the driver has nothing to map and
> cannot work there at all. It was never used in practice. `raspberrypi5.cfg`
> holds an untested `linuxgpiod` equivalent for whenever the boards get respun;
> read its header before trusting it.

---

## 🖥️ Lathe Emulator

A native Linux emulator is included for hardware-free firmware testing. It compiles the real firmware sources (`Ramps.c`, `Modbus.c`, `Scales.c`, `UARTCallback.c`) against a HAL/FreeRTOS shim layer and simulates lathe physics — spindle with inertia, leadscrew, carriage with half-nut engagement, and cross-slide. The emulator exposes Modbus RTU via PTY pair and TCP socket so the unmodified Python GUI can connect as if talking to real hardware.

A two-pane ANSI terminal dashboard with sparklines provides live visualization, with keyboard controls for spindle RPM, manual axis movement, half-nut engagement, and more. All parameters are configurable via TOML file.

### Emulator Build & Run

```bash
cd emulator
cmake -B build
cmake --build build
./build/lathe-emulator config/lathe.toml
```

---

## 🔧 Hardware Configuration

* `.ioc` file for use with STM32CubeMX included
* Pin assignments for encoder, buttons, LEDs, SWD, etc. reviewed and tested
* Memory layout defined by `STM32F411CEUX_FLASH.ld` and `STM32F411CEUX_RAM.ld`

---

## 🧩 PCB & Schematic

Firmware integrates with hardware design available at:

* **Compatible PCB**: [rotary-controller-pcb](https://github.com/bartei/rotary-controller-pcb)

Together, they form a complete controller + UI system when paired with:

* [reflex-ui](https://github.com/Funkenjaeger/reflex-ui) — a Raspberry Pi Kivy-based DRO + control UI

---

## 📄 License

Licensed under MIT. See `LICENSE` for full terms.

