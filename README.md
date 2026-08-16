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

The ARM toolchain and the ST-Link usually live on **different machines** — the
Pi that runs the UI has the debug probe hanging off it, and the cross-compiler is
wherever you write code. `scripts/flash.sh` spans that gap so you do not have to
think about it.

### Requirements

**Build host:** CMake and an ARM toolchain (`arm-none-eabi-gcc`, `make`).
**Probe host:** an ST-Link v2 on USB and `openocd` (`sudo apt install openocd`);
passwordless SSH to it from the build host. The two may be the same machine.

### Build and flash

```bash
./scripts/flash.sh
```

That builds, copies, verifies the transfer, flashes over SWD, and records what it
did. Add `--diag` for the diagnostic variant, `--host NAME` to target something
other than `elspi`, `--dry-run` to do everything except the write.

```bash
./scripts/flash.sh --diag        # with the ELS settle-trace probe
./scripts/build.sh               # build only, no flashing
./scripts/build.sh --diag --clean
```

**It rebuilds every time by default.** That is deliberate: with the build on one
machine and the flash on another, a copy that is quietly one revision behind is
the easiest mistake to make and the hardest to notice. `--no-build` opts out.

**Two variants, two build directories.** `build/` is the release firmware.
`build-diag/` adds `-DELS_DIAG_SCRATCH`, compiling in the ELS settle-trace probe
— **never** put that on `dev-staging`, `dev` or `main`. They are separate
directories so the flag can never depend on what the last `cmake` invocation
happened to say. At runtime the `elsStop.diagSchema` register tells you which one
is running, and the UI logs it at connect.

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

