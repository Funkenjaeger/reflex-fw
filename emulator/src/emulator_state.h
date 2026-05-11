/*
 * Emulator shared state.
 * Global state accessible from shim, physics, transport, and dashboard.
 */
#ifndef EMULATOR_STATE_H
#define EMULATOR_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declaration */
struct EmulatorState;

/*
 * GPIO event callback type.
 * Called from HAL_GPIO_WritePin shim whenever a pin changes state.
 */
typedef void (*gpio_write_cb_t)(void *GPIOx, uint16_t pin, int state);

/*
 * UART TX callback type.
 * Called when the firmware writes bytes via HAL_UART_Transmit_IT.
 */
typedef void (*uart_tx_cb_t)(const uint8_t *data, uint16_t size);

/*
 * Timer counter values written by the physics model,
 * read by the firmware via __HAL_TIM_GET_COUNTER.
 * Index 0-3 corresponds to TIM1-TIM4 (scales 0-3).
 */
typedef struct {
    /* Physics → firmware: virtual encoder counter values */
    volatile uint32_t scale_counters[4];

    /* Firmware → physics: GPIO pin states captured from step/dir/ena */
    volatile int step_pin;       /* PA0 rising edge = step pulse */
    volatile int dir_pin;        /* PB14: 1=forward, 0=reverse */
    volatile int ena_pin;        /* PB15: 0=enabled (active low) */
    volatile int spare2_pin;     /* PA3 */

    /* UART transport hooks */
    gpio_write_cb_t on_gpio_write;
    uart_tx_cb_t on_uart_tx;
    void *user_data;

    /* UART RX: transport calls this to inject a byte into the firmware */
    /* (we call HAL_UART_RxCpltCallback from the shim) */
    volatile int uart_rx_armed;      /* 1 = firmware has called HAL_UART_Receive_IT */
    uint8_t *uart_rx_buf;            /* pointer to firmware's dataRX byte */
    uint16_t uart_rx_size;

    /* Monotonic tick counter for HAL_GetTick */
    volatile uint32_t hal_tick;

    /* DWT cycle counter (incremented by ISR thread to simulate CPU cycles) */
    volatile uint32_t dwt_cyccnt;

    /* Mutex for shared data access */
    pthread_mutex_t mutex;

    /* Event log ring buffer for dashboard */
    char event_log[256][128];
    volatile int event_log_head;
    volatile int event_log_count;

    /* ELS-stop per-pass instrumentation, written atomically by Ramps.c the
     * instant the stop condition triggers. Lives outside rampsSharedData_t
     * so it is NOT part of the Modbus map — emulator-only debug aid for
     * verifying pass-to-pass spindle phase repeatability. */
    volatile int32_t  els_last_stop_spindle;   /* scales[0].position at trigger */
    volatile int32_t  els_last_stop_z;         /* scales[scaleIndex].position at trigger */
    volatile uint32_t els_last_stop_seq;       /* increments on every trigger; dashboard edge-detects this */

} EmulatorHardwareState;

/* Global instance */
extern EmulatorHardwareState emu_hw;

/* Add an event to the log (thread-safe). Writes to both the dashboard
 * ring buffer and the persistent file. */
void emu_log_event(const char *fmt, ...);

/* Append a trace line to the persistent log file only. Use for high-volume
 * diagnostic output (e.g. per-tick ISR samples) that would otherwise crowd
 * the dashboard's 256-entry ring. Thread-safe; no-op if file logging is
 * disabled. */
void emu_log_trace(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* EMULATOR_STATE_H */
