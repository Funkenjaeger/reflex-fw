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

# Motion attribution (Core/Inc/els_slip.h): which Z counts arrived while the
# servo was actually driving. Header-only, same shape as the cal test. Drives
# the accumulator tick by tick, which is the ONLY place the settle horizon's
# timing properties are reachable — the ISR fixture's lash model responds to a
# pulse instantly, so every horizon behaves identically there.
add_executable(els_slip_test ${CMAKE_SOURCE_DIR}/test/els_slip_test.cpp)
target_include_directories(els_slip_test PRIVATE ${CMAKE_SOURCE_DIR}/../Core/Inc)
target_link_libraries(els_slip_test m)
add_test(NAME els_slip_test COMMAND els_slip_test)

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

# ISR-level backlash-CALIBRATION motion attribution (task 6a7c455e). Drives
# data->elsCal through the real SynchroRefreshTimerIsr(), same stub strategy
# as els_takeup_confirm_test. REQUIRES the calibration slip-attribution
# infrastructure (elsCalCtx_t.slip, els_backlash_cal.h's elsSlipConfirmed()
# call, and this ISR's own calibration attribution-tick block) -- currently
# only present once the 2026-08-10 staged patch
# (els-calibration-slip-attribution.patch) is applied on top of this branch.
add_executable(els_cal_isr_attribution_test
    ${CMAKE_SOURCE_DIR}/test/els_cal_isr_attribution_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_cal_isr_attribution_test m pthread)
add_test(NAME els_cal_isr_attribution_test COMMAND els_cal_isr_attribution_test)

# ISR-level regression: a calibration leg must NOT be satisfied by carriage
# motion the servo did not produce. Sibling of els_cal_isr_attribution_test
# (same stub strategy, same Rig/Lash shape) but pinning the DECIDABLE half:
# that file drives the shove the settle horizon deliberately cannot see, so it
# passes with or without the attribution fix; this one drives a shove delivered
# after the servo has been silent past the horizon, which is the case
# elsSlipConfirmed() exists to refuse. Mutation-verified 2026-08-13: reverting
# els_backlash_cal.h's `moved` line to elsZMotionSeen() turns THIS target's
# VERDICT red, while leaving the verdict of every pre-existing target intact.
add_executable(els_cal_nudge_refusal_test
    ${CMAKE_SOURCE_DIR}/test/els_cal_nudge_refusal_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_cal_nudge_refusal_test m pthread)
add_test(NAME els_cal_nudge_refusal_test COMMAND els_cal_nudge_refusal_test)

# Diagnostic scratchpad leak guard. Compiles into EITHER configuration and
# asserts the opposite thing in each: a release build must advertise no probe
# (diagSchema == 0), a -DELS_DIAG_SCRATCH build must advertise one and publish
# its trace geometry. A one-sided test here would pass just as happily against a
# flag that does nothing, so both halves exist on purpose.
add_executable(els_diag_scratch_test
    ${CMAKE_SOURCE_DIR}/test/els_diag_scratch_test.cpp
    ${FW_DIR}/Core/Src/Ramps.c)
set_source_files_properties(${FW_DIR}/Core/Src/Ramps.c
    PROPERTIES LANGUAGE C COMPILE_FLAGS "-std=gnu11")
target_link_libraries(els_diag_scratch_test m pthread)
add_test(NAME els_diag_scratch_test COMMAND els_diag_scratch_test)
