/**
 * Copyright © 2022 <Stefano Bertelli>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef THIRD_PARTY_RAMPS_H_
#define THIRD_PARTY_RAMPS_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "Modbus.h"
#include "Scales.h"

#define MODBUS_ADDRESS 17

#define STEP_PIN GPIO_PIN_0
#define STEP_GPIO_PORT GPIOA

#define DIR_PIN GPIO_PIN_14
#define DIR_GPIO_PORT GPIOB

#define ENA_PIN GPIO_PIN_15
#define ENA_GPIO_PORT GPIOB
#define ENA_DELAY_MS 500

#define USR_LED_Pin GPIO_PIN_12
#define USR_LED_GPIO_Port GPIOB

#define SPARE_1_PIN GPIO_PIN_1
#define SPARE_1_GPIO_PORT GPIOA

#define SPARE_2_PIN GPIO_PIN_3
#define SPARE_2_GPIO_PORT GPIOA

#define SPARE_3_PIN GPIO_PIN_4
#define SPARE_3_GPIO_PORT GPIOA


typedef struct {
  int32_t delta;
  uint32_t oldPosition;
  uint32_t position;
  int32_t scaledDelta;
  int32_t error;
} deltaPosError_t;

typedef struct {
  TIM_HandleTypeDef *timerHandle;      // init-only: set before RampsStart(); must not be written via Modbus
  int32_t position;                    // READ-ONLY (firmware-owned): absolute encoder position, updated by ISR
  int32_t speed;                       // READ-ONLY (firmware-owned): encoder speed (counts/s), updated by updateSpeedTask
  int32_t syncRatioNum, syncRatioDen;  // SW write: sync ratio numerator/denominator (output steps per input count)
  uint16_t syncEnable;                 // SW write: 0 = sync disabled, non-zero = sync enabled for this scale
} input_t;

typedef struct {
  float maxSpeed;              // SW write: maximum step rate (steps/s); clamped to 100000 by firmware
  float currentSpeed;          // READ-ONLY (firmware-owned): live ramp speed, managed by ramp algorithm
  float jogSpeed;              // SW write: jog target speed (steps/s); negative = reverse
  float acceleration;          // SW write: ramp acceleration (steps/s²)
  int32_t stepsToGo;          // SW write: remaining steps for indexing move; firmware decrements toward zero
  uint32_t destinationSteps;  // SW write: absolute destination step count for indexing mode
  uint32_t currentSteps;      // READ-ONLY (firmware-owned): step counter incremented/decremented by ISR
  uint32_t desiredSteps;      // READ-ONLY (firmware-owned): accumulated target steps, driven by sync/ramp
} servo_t;

typedef struct {
  uint32_t servoCurrent;               // READ-ONLY (firmware-owned): mirror of servo.currentSteps, updated by updateSpeedTask
  uint32_t servoDesired;               // READ-ONLY (firmware-owned): mirror of servo.desiredSteps, updated by updateSpeedTask
  uint32_t stepsToGo;                  // READ-ONLY (firmware-owned): mirror of servo.stepsToGo, updated by ramp algorithm
  float servoSpeed;                    // READ-ONLY (firmware-owned): output step rate (steps/100ms), updated by servoEnableTask
  int32_t scaleCurrent[SCALES_COUNT];  // READ-ONLY (firmware-owned): mirror of scales[i].position, updated by ISR
  int32_t scaleSpeed[SCALES_COUNT];    // READ-ONLY (firmware-owned): mirror of scales[i].speed, updated by updateSpeedTask
  uint32_t cycles;                     // READ-ONLY (firmware-owned): ISR execution time in CPU cycles, updated by updateSpeedTask
  uint32_t executionInterval;          // READ-ONLY (firmware-owned): ISR interval in CPU cycles, updated by ISR
  uint16_t servoMode;                  // SW write: 0=disabled, 1=sync/index (also set by firmware), 2=jog
} fastData_t;

typedef struct {
  uint16_t enable;            // SW write: 1 = enable ELS stop feature
  uint16_t scaleIndex;        // SW write: which scale (0–3) is the position reference
  int32_t  stopPosition;      // SW write: threshold in encoder counts
  int16_t  stopDirection;     // SW write: 1 = stop when pos >= threshold, -1 = stop when pos <= threshold
  uint16_t active;            // bidirectional: firmware sets to 1 when triggered; SW writes 0 to resume
  int32_t  accumulatedError;  // READ-ONLY (firmware-owned): sync steps withheld while stopped; reset on resume
  float    threadPitchSteps;  // SW write: leadscrew steps per thread pitch (float); 0.0f = turning (no correction)
  int32_t  hysteresis;        // SW write: encoder counts carriage must retract before re-enabling; 0 = no hysteresis
  int32_t  latchedSpindleEncoder;  // READ-ONLY (firmware-owned): scales[0].position at stop trigger; INT32_MIN = no stop yet
} elsStop_t;

typedef struct {
  uint32_t executionInterval;          // READ-ONLY (firmware-owned): ISR period in CPU cycles (current - previous timestamp)
  uint32_t executionIntervalPrevious;  // READ-ONLY (firmware-owned): DWT timestamp of previous ISR entry
  uint32_t executionIntervalCurrent;   // READ-ONLY (firmware-owned): DWT timestamp of current ISR entry
  uint32_t executionCycles;            // READ-ONLY (firmware-owned): ISR wall-clock duration in CPU cycles
  servo_t servo;
  input_t scales[SCALES_COUNT];
  fastData_t fastData;
  elsStop_t elsStop;
} rampsSharedData_t;


typedef struct {
  // Modbus shared data
  rampsSharedData_t shared;

  // STM32 Related
  TIM_HandleTypeDef *synchroRefreshTimer;
  UART_HandleTypeDef *modbusUart;

  deltaPosError_t scalesDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSyncDeltaPos[SCALES_COUNT];
  deltaPosError_t scalesSpeed[SCALES_COUNT];
  deltaPosError_t rampsDeltaPos;
  uint32_t servoPreviousDirection;
  uint16_t elsStopPreviousActive;
  uint32_t elsStopStepsAtStop;  // desiredSteps snapshot taken when elsStop.active is latched
} rampsHandler_t;

extern modbusHandler_t RampsModbusData;

void RampsStart(rampsHandler_t *rampsData);

void SynchroRefreshTimerIsr(rampsHandler_t *data);

_Noreturn void updateSpeedTask(void *argument);

_Noreturn void userLedTask(__attribute__((unused)) void *argument);

_Noreturn void servoEnableTask(void *argument);

//static void timServoEnableOnCallback(xTimerHandle pxTimer);
//static void timServoEnableOffCallback(xTimerHandle pxTimer);

#endif