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
* Supports ST‑Link V2 and Raspberry Pi + OpenOCD programming
* Optimized for high-speed encoder + stepper/servo motor control
* Includes a native FW+lathe emulator for hardware-free testing with the Python GUI

---

## 🛠️ Build & Flash

### Requirements

* CMake & C/C++ toolchain (e.g. `arm-none-eabi-gcc`, `make`)
* ST-Link v2 or Raspberry Pi with OpenOCD

### Build

```bash
git clone https://github.com/Funkenjaeger/reflex-fw.git
cd reflex-fw
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Clean

```bash
rm -rf build
```

### Flash

* **ST‑Link V2**:

  ```bash
  st-flash --format ihex write build/reflex-fw.hex
  ```

* **Raspberry Pi + OpenOCD**:

  ```bash
  openocd -f ./raspberry.cfg
  ```

  The default `raspberry.cfg` configures SWD over GPIO pins 24/25 + GND. Ensure GND wiring is the **same length** as SWCLK/SWDIO for reliability. Modify the GPIO pins in `raspberry.cfg` if needed.

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

