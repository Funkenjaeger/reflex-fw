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
