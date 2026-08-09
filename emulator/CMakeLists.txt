cmake_minimum_required(VERSION 3.16)
project(lathe-emulator C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Firmware source directory
set(FW_DIR ${CMAKE_SOURCE_DIR}/..)

# --- Shim headers come FIRST so they override the real HAL/FreeRTOS ---
include_directories(
    ${CMAKE_SOURCE_DIR}/shim
    ${CMAKE_SOURCE_DIR}/src
    ${FW_DIR}/Core/Inc
)

# Preprocessor defines to match firmware expectations
add_definitions(
    -DSTM32F411xE
    -DUSE_HAL_DRIVER
    -DEMULATOR_BUILD
)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)

# --- Firmware C sources (compiled against our shim) ---
set(FW_SOURCES
    ${FW_DIR}/Core/Src/Ramps.c
    ${FW_DIR}/Core/Src/Modbus.c
    ${FW_DIR}/Core/Src/Scales.c
    ${FW_DIR}/Core/Src/UARTCallback.c
)

# --- Shim implementations ---
set(SHIM_SOURCES
    ${CMAKE_SOURCE_DIR}/shim/hal_shim.c
    ${CMAKE_SOURCE_DIR}/shim/freertos_shim.cpp
)

# --- Emulator C++ sources ---
set(EMU_SOURCES
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/physics.cpp
    ${CMAKE_SOURCE_DIR}/src/transport.cpp
    ${CMAKE_SOURCE_DIR}/src/dashboard.cpp
    ${CMAKE_SOURCE_DIR}/src/config.cpp
)

add_executable(lathe-emulator
    ${FW_SOURCES}
    ${SHIM_SOURCES}
    ${EMU_SOURCES}
)

target_link_libraries(lathe-emulator
    pthread
    m
)

# Allow C++ files to include C headers
set_source_files_properties(${FW_SOURCES} PROPERTIES LANGUAGE C)
set_source_files_properties(${SHIM_SOURCES} ${EMU_SOURCES} PROPERTIES LANGUAGE CXX)

# The firmware .c files must be compiled as C, but our shim headers
# use extern "C" guards so they work from both C and C++ translation units.
# Force C compilation for firmware sources specifically.
set_source_files_properties(${FW_SOURCES} ${CMAKE_SOURCE_DIR}/shim/hal_shim.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11"
)

# --- Unit tests ---
enable_testing()

# Pure ELS phase-correction permutation tests (no HAL/shim deps; just els_phase.h).
add_executable(els_phase_test ${CMAKE_SOURCE_DIR}/test/els_phase_test.cpp)
target_link_libraries(els_phase_test m)
add_test(NAME els_phase_test COMMAND els_phase_test)

# Backlash calibration / take-up confirmation: exercises the pure decision layer
# in Core/Inc/els_backlash_cal.h against a simulated drivetrain with lash.
# Header-only — no HAL, no firmware sources, no emulator physics.
add_executable(els_backlash_cal_test ${CMAKE_SOURCE_DIR}/test/els_backlash_cal_test.cpp)
target_include_directories(els_backlash_cal_test PRIVATE ${CMAKE_SOURCE_DIR}/../Core/Inc)
target_link_libraries(els_backlash_cal_test m)
add_test(NAME els_backlash_cal_test COMMAND els_backlash_cal_test)

# Half-nut engagement physics tests: exercise LathePhysics directly. Links
# physics.cpp + config.cpp (EmuConfig ctor); the test file stubs the shim
# externals (emu_hw, emu_log_event, emu_update_timer_counters), so hal_shim.c
# is deliberately NOT linked.
add_executable(els_halfnut_test ${CMAKE_SOURCE_DIR}/test/els_halfnut_test.cpp
    ${CMAKE_SOURCE_DIR}/src/physics.cpp
    ${CMAKE_SOURCE_DIR}/src/config.cpp)
target_link_libraries(els_halfnut_test m)
add_test(NAME els_halfnut_test COMMAND els_halfnut_test)

# First-tick boot-delta int16 wrap guard: exercises LathePhysics directly
# (same link shape as els_halfnut_test) and replays Ramps.c's own per-tick
# int16-cast-and-accumulate DRO logic against it, so it proves the guard
# without linking Ramps.c/Modbus.c or the Modbus register map.
add_executable(els_boot_delta_test ${CMAKE_SOURCE_DIR}/test/els_boot_delta_test.cpp
    ${CMAKE_SOURCE_DIR}/src/physics.cpp
    ${CMAKE_SOURCE_DIR}/src/config.cpp)
target_link_libraries(els_boot_delta_test m)
add_test(NAME els_boot_delta_test COMMAND els_boot_delta_test)

# KNOWN-FAILING REPRODUCTION: ELS stop resume re-latch defect.
# Drives the REAL Core/Src/Ramps.c ISR (SynchroRefreshTimerIsr) directly; the
# test file stubs every Ramps.c external (HAL GPIO/TIM, Modbus, CMSIS-RTOS,
# emu_hw/emu_log_trace/emu_dwt), so neither hal_shim.c nor Modbus.c is linked.
# Contains a defect case (resume without retracting) that FAILS on purpose and
# a control case (resume after retracting) that passes -- the pair is what
# distinguishes a real defect from a broken test. Expect this target RED until
# the resume path is fixed.
# ISR-level take-up confirmation + calibration driver. Same stub strategy as the
# relatch test (links Ramps.c, stubs every external), but drives the servo's own
# step pulses through a lash model so the Z scale responds the way a real
# drivetrain does. Covers the wiring the pure els_backlash_cal_test cannot:
# baseline capture, withholding, result/seq publication, the enable escape hatch.
add_executable(els_takeup_confirm_test
    ${CMAKE_SOURCE_DIR}/test/els_takeup_confirm_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_takeup_confirm_test m pthread)
add_test(NAME els_takeup_confirm_test COMMAND els_takeup_confirm_test)

add_executable(els_stop_resume_relatch_test
    ${CMAKE_SOURCE_DIR}/test/els_stop_resume_relatch_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_stop_resume_relatch_test m pthread)
add_test(NAME els_stop_resume_relatch_test COMMAND els_stop_resume_relatch_test)

# Manual reference latch (interactive re-sync to an existing thread). Same stub
# strategy as the relatch test: links the real Ramps.c, stubs every external.
# Covers the latchCommand/latchSeq command/ack contract: atomic consume,
# auto-latch suppression via referenceLatched, refusal (no ack) outside a job,
# and the resume path measuring from the manual datum.
add_executable(els_manual_latch_test
    ${CMAKE_SOURCE_DIR}/test/els_manual_latch_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_manual_latch_test m pthread)
add_test(NAME els_manual_latch_test COMMAND els_manual_latch_test)
