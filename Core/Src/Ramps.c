/**
 * Copyright © 2024 <Stefano Bertelli>
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
 * LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <math.h>
#include "Ramps.h"
#include "Scales.h"
#include "els_phase.h"

/* Post-takeup settle dwell: after the backlash takeup reaches its commanded
 * target step count, the step/dir servo may still be closing following error /
 * settling. Snapshotting the Z DRO then yields a takeup-speed-dependent (hence
 * backlash-dependent) phase error. Hold position for this many ISR ticks before
 * sampling Z in applyPhaseCorrection. ISR runs at ~100 kHz (TIM9, 10 us/tick),
 * so 100000 ticks ~= 1 s. The settling hypothesis was REFUTED on hardware (the
 * 1 s dwell changed nothing) and in the emulator (the carriage genuinely doesn't
 * move during a lash-absorbed takeup, so there is nothing to settle), so this is
 * now a short, behaviour-neutral guard rather than a real settle window. Keep it
 * small so the emulator's bounded-guard scenario loop doesn't time out. */
#define ELS_SETTLE_TICKS 50

#ifdef EMULATOR_BUILD
#include "emulator_state.h"

/* Step_6 instrumentation: trace leadscrew step accounting from
 * applyPhaseCorrection completion to the next ELS stop trigger. Kept as a
 * permanent emulator-only debug aid for ELS phase regressions — the
 * per-pass `step6 #N start / t=... / end` log lines expose dCur, dDes,
 * dSpindle, dZ, direction-flutter counts, and per-direction step pulses,
 * which together fully describe the firmware's step accounting during the
 * window where phase is determined. See DEBUGGING.md for the original
 * investigation that motivated this instrumentation. */
static uint32_t emu_step6_active = 0;
static uint32_t emu_step6_pass = 0;
static uint32_t emu_step6_tick = 0;
static const uint32_t emu_step6_log_interval = 2000;  /* mid-cut sample period in ISR ticks */
static int32_t  emu_step6_start_current = 0;
static int32_t  emu_step6_start_desired = 0;
static int32_t  emu_step6_start_spindle = 0;
static int32_t  emu_step6_start_z = 0;
static uint32_t emu_step6_pos_pulses = 0;
static uint32_t emu_step6_neg_pulses = 0;
static uint32_t emu_step6_dir_flips = 0;
static int32_t  emu_step6_prev_change_sign = 0;
#endif


// This variable is the handler for the modbus communication
modbusHandler_t RampsModbusData;

uint16_t servoCycles = 0;
uint16_t servoCyclesCounter = 0;

/* Side-table of real TIM handles, indexed by input_t.timerHandleSlot.
 * The modbus-exposed input_t carries only a uint32_t slot id so the wire
 * layout is identical on STM32 (4-byte pointer) and 64-bit emulator hosts. */
TIM_HandleTypeDef *ramps_timer_handles[SCALES_COUNT] = {0};


//osThreadId_t userLedTaskHandler;
const osThreadAttr_t ledTaskAttributes = {
.name = "UpdateLedTask",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityLow,

};

//osThreadId_t updateSpeedTaskHandler;
const osThreadAttr_t speedTaskAttributes = {
.name = "updateSpeedTask",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityLow,
};

//osThreadId_t updateServoEnableTaskHandler;
const osThreadAttr_t servoEnableTaskAttributes = {
.name = "servoEnableTask",
.stack_size = 128 * 4,
.priority = (osPriority_t) osPriorityLow,
};


void configureOutputPin(GPIO_TypeDef *Port, uint16_t Pin) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Port, &GPIO_InitStruct);
}


void RampsStart(rampsHandler_t *rampsData) {
  rampsData->shared.servo.maxSpeed = 720;
  rampsData->shared.servo.acceleration = 120;
  rampsData->shared.servo.servoDir = 1;

  for (int i = 0; i < SCALES_COUNT; i++) {
    rampsData->shared.scales[i].syncRatioNum = 1;
    rampsData->shared.scales[i].syncRatioDen = 100;
    rampsData->shared.scales[i].scaleDir = 1;
  }

  rampsData->shared.elsStop.referenceLatched = 0;
  rampsData->shared.elsStop.takeupPending = 0;

  // Configure Pins
  configureOutputPin(DIR_GPIO_PORT, DIR_PIN);
  configureOutputPin(ENA_GPIO_PORT, ENA_PIN);
  configureOutputPin(STEP_GPIO_PORT, STEP_PIN);
  configureOutputPin(SPARE_1_GPIO_PORT, SPARE_1_PIN);
  configureOutputPin(SPARE_2_GPIO_PORT, SPARE_2_PIN);
  configureOutputPin(SPARE_3_GPIO_PORT, SPARE_3_PIN);

  // Configure tasks
  osThreadNew(userLedTask, rampsData, &ledTaskAttributes);
  osThreadNew(updateSpeedTask, rampsData, &speedTaskAttributes);
  osThreadNew(servoEnableTask, rampsData, &servoEnableTaskAttributes);


  // Initialize and start encoder timer, reset the sync flags
  for (int j = 0; j < SCALES_COUNT; ++j) {
    initScaleTimer(ramps_timer_handles[rampsData->shared.scales[j].timerHandleSlot]);
    HAL_TIM_Encoder_Start(ramps_timer_handles[rampsData->shared.scales[j].timerHandleSlot], TIM_CHANNEL_ALL);
  }

  // Enable debug cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Start Modbus
  RampsModbusData.uModbusType = MB_SLAVE;
  RampsModbusData.port = rampsData->modbusUart;
  RampsModbusData.u8id = MODBUS_ADDRESS;
  RampsModbusData.u16timeOut = 1000;
  RampsModbusData.EN_Port = NULL;
  RampsModbusData.u16regs = (uint16_t *) (&rampsData->shared);
  RampsModbusData.u16regsize = sizeof(rampsData->shared) / sizeof(uint16_t);
  RampsModbusData.xTypeHW = USART_HW;
  ModbusInit(&RampsModbusData);
  ModbusStart(&RampsModbusData);

  // Start synchro interrupt
  HAL_TIM_Base_Start_IT(rampsData->synchroRefreshTimer);
}

static inline void
deltaPositionAndError(int32_t currentValue, int32_t ratioNum, int32_t ratioDen, deltaPosError_t *data) {
  int32_t startValue = (int16_t) (currentValue - data->oldPosition) * ratioNum + data->error;
  data->oldPosition = currentValue;
  data->scaledDelta = (int32_t) (startValue / ratioDen);
  data->error = (int32_t) (startValue % ratioDen);
}

static inline void updateIndexingPosition(rampsHandler_t *data) {
  rampsSharedData_t *shared = &(data->shared);
  float interval = (float) shared->executionInterval / 100000000.0f;
  float stopDistance = (shared->servo.currentSpeed * shared->servo.currentSpeed / shared->servo.acceleration) / 2;

  // Accelerate Pos
  if (shared->servo.stepsToGo > 0) {
    if ((float)shared->servo.stepsToGo > stopDistance && shared->servo.currentSpeed < shared->servo.maxSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      // max speed
      if (shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = shared->servo.maxSpeed;
      }
    }

    // Decelerate Pos
    if ((float)shared->servo.stepsToGo < stopDistance) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed < 0) {
        shared->servo.currentSpeed = 0;
      }
    }
  }

  if (shared->servo.stepsToGo < 0) {
    // Accelerate Neg
    if (-(float)shared->servo.stepsToGo > stopDistance && -shared->servo.currentSpeed < shared->servo.maxSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (-shared->servo.currentSpeed > shared->servo.maxSpeed) {
        shared->servo.currentSpeed = -shared->servo.maxSpeed;
      }
    }

    // Decelerate Neg
    if (-(float)shared->servo.stepsToGo < stopDistance) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed > 0) {
        shared->servo.currentSpeed = 0;
      }
    }
  }

  if (shared->servo.stepsToGo == 0) {
    // Security measure, end of travel
    shared->servo.currentSpeed = 0;
  } else {
    int32_t positionIncrement = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) / 100000000);
    data->rampsDeltaPos.error = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) % 100000000);
    shared->servo.desiredSteps += positionIncrement;
    shared->servo.stepsToGo -= positionIncrement;
    shared->fastData.stepsToGo = shared->servo.stepsToGo;
  }
}

static inline void updateJogPosition(rampsHandler_t *data) {
  rampsSharedData_t *shared = &(data->shared);
  float interval = (float) shared->executionInterval / 100000000.0f;

  // Start Pos
  if (shared->servo.jogSpeed > 0) {
    if (shared->servo.currentSpeed < shared->servo.jogSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      // max speed
      if (shared->servo.currentSpeed > shared->servo.jogSpeed) {
        shared->servo.currentSpeed = shared->servo.jogSpeed;
      }
    }
  }

  // Start Neg
  if (shared->servo.jogSpeed < 0) {
    if (shared->servo.currentSpeed > shared->servo.jogSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed < shared->servo.jogSpeed) {
        shared->servo.currentSpeed = shared->servo.jogSpeed;
      }
    }
  }

  // Stop Pos/Neg
  if (shared->servo.currentSpeed > 0) {
    if (shared->servo.currentSpeed > shared->servo.jogSpeed) {
      shared->servo.currentSpeed -= shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed < 0) shared->servo.currentSpeed = 0;
    }
  }

  if (shared->servo.currentSpeed < 0) {
    if (shared->servo.currentSpeed < shared->servo.jogSpeed) {
      shared->servo.currentSpeed += shared->servo.acceleration * interval;
      if (shared->servo.currentSpeed > 0) shared->servo.currentSpeed = 0;
    }
  }

  int32_t positionIncrement = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) / 100000000);
  data->rampsDeltaPos.error = ((int32_t)((float)shared->servo.currentSpeed * (float)shared->executionInterval + (float)data->rampsDeltaPos.error) % 100000000);
  shared->servo.desiredSteps += positionIncrement;
}

/* ELS phase correction: jog the carriage to an integer multiple of thread
 * pitch above stopPosition, so the next sync return covers a whole number
 * of thread pitches and the trigger fires at the latched spindle phase.
 *
 * Two leadscrew-step quantities are compared:
 *   idealAdvance  = what pure sync would have done since latch
 *                   (spindle motion via syncRatio)
 *   actualAdvance = what the carriage actually did since latch
 *                   (Z motion via thread-pitch geometry)
 * Their difference, modulo pitch and folded to ±pitch/2, is the shortest
 * pre-cut jog that brings Z to thread-pitch alignment. Workflow-agnostic:
 * the carriage may have gotten to its current Z by any combination of
 * electronic retract, manual jog, half-nut snap, etc.
 *
 * See ARCHITECTURE.md → ELS Shoulder Stop for the conceptual model. */
static inline void applyPhaseCorrection(rampsSharedData_t *shared) {
  int32_t deltaSpindle =
    shared->scales[0].position - shared->elsStop.latchedSpindle;
  int32_t deltaZ =
    shared->scales[shared->elsStop.scaleIndex].position - shared->elsStop.latchedZ;

  /* Pure, unit-tested phase-correction math (els_phase.h). The phaseError sign
   * is derived from stopDirection*cuttingDir so the Z-stop lands in phase for
   * BOTH DRO/leadscrew polarities — using the fixed ideal-actual form made the
   * thread phase walk 2x the backlash on machines where they run opposite. */
  elsCorrResult_t r = elsComputePhaseCorrection(
      deltaSpindle, deltaZ,
      shared->scales[0].syncRatioNum, shared->scales[0].syncRatioDen,
      shared->elsStop.threadPitchSteps, shared->elsStop.zCountsPerPitch,
      shared->elsStop.stopDirection);

  shared->servo.stepsToGo += r.stepsToAdd;

  shared->elsStop.lastIdealAdvance  = r.idealAdvance;
  shared->elsStop.lastActualAdvance = r.actualAdvance;
  shared->elsStop.lastPhaseError    = r.phaseError;
  shared->elsStop.lastCorrection    = r.correction;

#ifdef EMULATOR_BUILD
  /* Arm step_6 trace: snapshot starting state so per-tick logs can report
   * deltas. emu_step6_pass identifies the pass we just left (the trigger seq
   * already incremented when this pass started). */
  emu_step6_pass             = emu_hw.els_last_stop_seq;
  emu_step6_tick             = 0;
  emu_step6_start_current    = (int32_t)shared->servo.currentSteps;
  emu_step6_start_desired    = (int32_t)shared->servo.desiredSteps;
  emu_step6_start_spindle    = shared->scales[0].position;
  emu_step6_start_z          = shared->scales[shared->elsStop.scaleIndex].position;
  emu_step6_pos_pulses       = 0;
  emu_step6_neg_pulses       = 0;
  emu_step6_dir_flips        = 0;
  emu_step6_prev_change_sign = 0;
  emu_step6_active           = 1;
  emu_log_trace("step6 #%u start cur=%d des=%d sp=%d z=%d stg=%d corr=%.1f",
                (unsigned)emu_step6_pass,
                (int)shared->servo.currentSteps,
                (int)shared->servo.desiredSteps,
                (int)shared->scales[0].position,
                (int)shared->scales[shared->elsStop.scaleIndex].position,
                (int)shared->servo.stepsToGo,
                (double)r.correction);
#endif
}

void SynchroRefreshTimerIsr(rampsHandler_t *data) {
  uint32_t start = DWT->CYCCNT;
  // Reset the step pin as soon as possible
  HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPARE_2_GPIO_PORT, SPARE_2_PIN, GPIO_PIN_RESET);
  rampsSharedData_t *shared = &(data->shared);
  shared->executionIntervalPrevious = shared->executionIntervalCurrent;
  shared->executionIntervalCurrent = DWT->CYCCNT;
  shared->executionInterval = shared->executionIntervalCurrent - shared->executionIntervalPrevious;
  shared->fastData.executionInterval = shared->executionInterval;

  // Reset reference latch on elsStop.enable rising edge (start of a new threading job)
  if (shared->elsStop.enable && !data->elsStopPreviousEnable) {
    shared->elsStop.referenceLatched = 0;
  }

  // Auto-clear active when enable is deasserted
  if (data->elsStopPreviousEnable && !shared->elsStop.enable) {
    shared->elsStop.active = 0;
  }

  data->elsStopPreviousEnable = shared->elsStop.enable;

  // Detect completion of post-resume backlash takeup move, dwell for the servo
  // to settle, then apply phase correction. takeupPending stays set during the
  // dwell so sync remains gated and the servo holds at the takeup target.
  if (shared->elsStop.takeupPending) {
    // Crossing test (not exact equality): the servo emits one step per servoCycle
    // while desiredSteps can advance several steps per tick, so currentSteps may
    // skip the exact target (esp. across the reversal pulse-skip). Treat the
    // takeup as complete once currentSteps reaches or passes the target in the
    // takeup direction.
    int32_t cur = (int32_t)shared->servo.currentSteps;
    bool takeupReached = (data->elsStopTakeupSign >= 0)
                         ? (cur >= data->elsStopTakeupTargetSteps)
                         : (cur <= data->elsStopTakeupTargetSteps);
    if (takeupReached) {
      if (data->elsStopSettleCount < ELS_SETTLE_TICKS) {
        data->elsStopSettleCount++;        // dwell after commanded-complete
      } else {
        data->elsStopSettleCount = 0;
        shared->elsStop.takeupPending = 0;
        applyPhaseCorrection(shared);       // snapshot Z only once settled
      }
    }
  }

  for (int i = 0; i < SCALES_COUNT; i++) {
    data->scalesDeltaPos[i].oldPosition = data->scalesDeltaPos[i].position;
    data->scalesDeltaPos[i].position = __HAL_TIM_GET_COUNTER(ramps_timer_handles[data->shared.scales[i].timerHandleSlot]);
    data->scalesDeltaPos[i].delta = (int16_t) (data->scalesDeltaPos[i].position - data->scalesDeltaPos[i].oldPosition);
    shared->scales[i].position += data->scalesDeltaPos[i].delta * shared->scales[i].scaleDir;

    // calculate delta for sync ratio configured for the current scale
    deltaPositionAndError(
      shared->scales[i].position,
      shared->scales[i].syncRatioNum,
      shared->scales[i].syncRatioDen,
      &data->scalesSyncDeltaPos[i]
    );

    // request motion only if sync is enabled
    if (shared->scales[i].syncEnable != 0) {
      // Check ELS stop trigger (only latch when not already active)
      if (!shared->elsStop.active && shared->elsStop.enable) {
        int32_t refPos = shared->scales[shared->elsStop.scaleIndex].position;
        bool shouldStop = (shared->elsStop.stopDirection >= 0)
                          ? (refPos >= shared->elsStop.stopPosition)
                          : (refPos <= shared->elsStop.stopPosition);
        if (shouldStop) {
          shared->elsStop.active = 1;
          if (!shared->elsStop.referenceLatched) {
            shared->elsStop.latchedZ         = shared->scales[shared->elsStop.scaleIndex].position;
            shared->elsStop.latchedSpindle   = shared->scales[0].position;
            shared->elsStop.referenceLatched = 1;
          }
#ifdef EMULATOR_BUILD
          /* Atomic per-pass latch for the emulator dashboard. Lives outside
           * the Modbus map (rampsSharedData_t) so the SW protocol is
           * unaffected. Sequence counter increments so the dashboard can
           * edge-detect new triggers without polling `active` itself. */
          emu_hw.els_last_stop_spindle = shared->scales[0].position;
          emu_hw.els_last_stop_z       = shared->scales[shared->elsStop.scaleIndex].position;
          emu_hw.els_last_stop_seq++;

          /* Close out the step_6 trace if armed. Reports cumulative deltas
           * + direction-flutter counters so the analysis can compare the
           * predicted step_6 motion against what actually happened. */
          if (emu_step6_active) {
            int32_t d_cur   = (int32_t)shared->servo.currentSteps - emu_step6_start_current;
            int32_t d_des   = (int32_t)shared->servo.desiredSteps - emu_step6_start_desired;
            int32_t d_sp    = shared->scales[0].position - emu_step6_start_spindle;
            int32_t d_z     = shared->scales[shared->elsStop.scaleIndex].position - emu_step6_start_z;
            int32_t syncErr = data->scalesSyncDeltaPos[shared->elsStop.scaleIndex].error;
            emu_log_trace("step6 #%u end t=%u dCur=%+d dDes=%+d syncE=%d dSp=%+d dZ=%+d flips=%u P+=%u P-=%u",
                          (unsigned)emu_step6_pass, (unsigned)emu_step6_tick,
                          (int)d_cur, (int)d_des, (int)syncErr,
                          (int)d_sp, (int)d_z,
                          (unsigned)emu_step6_dir_flips,
                          (unsigned)emu_step6_pos_pulses, (unsigned)emu_step6_neg_pulses);
            emu_step6_active = 0;
          }
#endif
        }
      }

      // Sync paused while stopped or while a backlash takeup is in progress
      if (!shared->elsStop.active && !shared->elsStop.takeupPending) {
        shared->servo.desiredSteps += data->scalesSyncDeltaPos[i].scaledDelta;
      }
    }

    // Update fastData current position
    shared->fastData.scaleCurrent[i] = shared->scales[i].position;
  }

  // Detect SW clearing elsStop.active (1→0): initiate backlash takeup, or apply correction inline
  if (data->elsStopPreviousActive && !shared->elsStop.active) {
    if (shared->elsStop.referenceLatched
        && shared->elsStop.threadPitchSteps != 0.0f
        && shared->elsStop.zCountsPerPitch  != 0.0f
        && shared->scales[0].syncRatioDen   != 0) {
      if (shared->elsStop.backlashSteps != 0u) {
        shared->servo.stepsToGo    = 0;
        shared->servo.currentSpeed = 0;
        data->elsStopSettleCount   = 0;
        int32_t cuttingDir = (shared->scales[0].syncRatioNum > 0) ? 1 : -1;
        if (shared->elsStop.threadPitchSteps * shared->elsStop.zCountsPerPitch < 0.0f) {
          cuttingDir = -cuttingDir;
        }
        int32_t signedTakeup = cuttingDir * (int32_t)shared->elsStop.backlashSteps;
        shared->servo.stepsToGo       += signedTakeup;
        data->elsStopTakeupTargetSteps = (int32_t)shared->servo.currentSteps + signedTakeup;
        data->elsStopTakeupSign        = (signedTakeup >= 0) ? 1 : -1;
        shared->elsStop.takeupPending  = 1;
      } else {
        applyPhaseCorrection(shared);
      }
    }
  }
  data->elsStopPreviousActive = shared->elsStop.active;

  if (shared->fastData.servoMode == 1) updateIndexingPosition(data);
  if (shared->fastData.servoMode == 2) updateJogPosition(data);

  if (shared->fastData.servoMode != 0 && servoCyclesCounter == 0) {
    int32_t change = (int32_t)(shared->servo.desiredSteps) - (int32_t)shared->servo.currentSteps;
    // generate pulses to reach desired position with the motor
    uint32_t direction = 1;

    if (change > 0) {
      direction = 1;
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, shared->servo.servoDir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    if (change < 0) {
      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_PIN, shared->servo.servoDir == 1 ? GPIO_PIN_RESET : GPIO_PIN_SET);
      direction = -1;
    }

    if (direction == data->servoPreviousDirection && change != 0) {
      HAL_GPIO_WritePin(STEP_GPIO_PORT, STEP_PIN, GPIO_PIN_SET);
      HAL_GPIO_WritePin(SPARE_2_GPIO_PORT, SPARE_2_PIN, GPIO_PIN_SET);
      shared->servo.currentSteps += direction;
#ifdef EMULATOR_BUILD
      if (emu_step6_active) {
        if (direction == 1) emu_step6_pos_pulses++;
        else                emu_step6_neg_pulses++;
      }
#endif
    }

    data->servoPreviousDirection = direction;
  }

  /* Divide-by-zero guard. The zero window is REACHABLE, not theoretical:
   * servoCycles is 0 from reset (Ramps.c:64) and its only writer is
   * updateSpeedTask (Ramps.c:613), but RampsStart() enables the 100 kHz TIM9
   * interrupt as its last act (HAL_TIM_Base_Start_IT, Ramps.c:165) and main.c
   * only reaches osKernelStart() afterwards, so this ISR runs before the
   * scheduler exists at all, and updateSpeedTask then sleeps osDelay(50) before
   * its first assignment. That is >5000 ISR ticks at 10 us with servoCycles == 0.
   * It goes unnoticed on hardware because Cortex-M4 UDIV-by-zero yields 0 with
   * DIV_0_TRP clear (never set here) and servoMode is still 0 so no pulses are
   * emitted; it is still C undefined behavior, and the emulator's x86 build
   * takes SIGFPE on this line. Substituting 0 reproduces the observed hardware
   * result without the UB. Nonzero servoCycles behaves exactly as before.
   * COUPLED WITH updateSpeedTask's newPeriod clamp (Ramps.c:610-613): that
   * clamp now floors at 1, not 0, so the boot window above is the only
   * remaining source of servoCycles == 0. This guard must stay regardless —
   * it is what makes that window survivable. */
  servoCyclesCounter = (servoCycles != 0) ? (servoCyclesCounter + 1) % servoCycles : 0;

#ifdef EMULATOR_BUILD
  /* Step_6 per-tick trace: flutter detection + periodic sample. End-of-tick
   * so post-emission state is captured. Direction flutter = sign flips of
   * (desiredSteps − currentSteps); high flip counts would support the
   * indexing↔sync interaction hypothesis (DEBUGGING.md #1). */
  if (emu_step6_active) {
    int32_t change_now = (int32_t)shared->servo.desiredSteps - (int32_t)shared->servo.currentSteps;
    int32_t cs = (change_now > 0) ? 1 : (change_now < 0 ? -1 : 0);
    if (cs != 0 && emu_step6_prev_change_sign != 0 && cs != emu_step6_prev_change_sign) {
      emu_step6_dir_flips++;
    }
    if (cs != 0) emu_step6_prev_change_sign = cs;

    emu_step6_tick++;
    if ((emu_step6_tick % emu_step6_log_interval) == 0) {
      int32_t d_cur   = (int32_t)shared->servo.currentSteps - emu_step6_start_current;
      int32_t d_des   = (int32_t)shared->servo.desiredSteps - emu_step6_start_desired;
      int32_t d_sp    = shared->scales[0].position - emu_step6_start_spindle;
      int32_t d_z     = shared->scales[shared->elsStop.scaleIndex].position - emu_step6_start_z;
      int32_t syncErr = data->scalesSyncDeltaPos[shared->elsStop.scaleIndex].error;
      emu_log_trace("step6 #%u t=%u dCur=%+d dDes=%+d syncE=%d dSp=%+d dZ=%+d flips=%u P+=%u P-=%u stg=%d",
                    (unsigned)emu_step6_pass, (unsigned)emu_step6_tick,
                    (int)d_cur, (int)d_des, (int)syncErr,
                    (int)d_sp, (int)d_z,
                    (unsigned)emu_step6_dir_flips,
                    (unsigned)emu_step6_pos_pulses, (unsigned)emu_step6_neg_pulses,
                    (int)shared->servo.stepsToGo);
    }
  }
#endif

  shared->executionCycles = DWT->CYCCNT - start;
}

_Noreturn void userLedTask(__attribute__((unused)) void *argument) {
  uint16_t oldInCnt = 0;
  uint8_t blinkInterval = 0;

  for (;;) {
    osDelay(50);
    blinkInterval = (blinkInterval + 1) % 10;
    if (blinkInterval == 0) {
      HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }

    if (oldInCnt != RampsModbusData.u16InCnt) {
      oldInCnt = RampsModbusData.u16InCnt;
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
      osDelay(25);
      HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    }
  }

}

const int32_t updateSpeedTaskTicks = 50;

_Noreturn void updateSpeedTask(void *argument) {
  rampsHandler_t *rampsData = (rampsHandler_t *) argument;

  for (;;) {

    // Update the current speed
    osDelay(updateSpeedTaskTicks);

    // Update fast access variables
    rampsData->shared.fastData.cycles = rampsData->shared.executionCycles;
    rampsData->shared.fastData.servoCurrent = rampsData->shared.servo.currentSteps;
    rampsData->shared.fastData.servoDesired = rampsData->shared.servo.desiredSteps;

    // If maximum speed has been changed, update the motor timer accordingly
    float clock_freq = 100000000.0f / ((float) rampsData->synchroRefreshTimer->Init.Prescaler + 1) /
                       (float) (rampsData->synchroRefreshTimer->Init.Period + 1);

    // Clamping value for max speed to the maximum allowed by the current timer refresh rate from the sync routine
    if (rampsData->shared.servo.maxSpeed > 100000) {
      rampsData->shared.servo.maxSpeed = 100000;
    }

    float newPeriod = floorf(clock_freq / rampsData->shared.servo.maxSpeed);
    if (newPeriod > (float) UINT16_MAX) {
      newPeriod = 65535;
    }
    if (newPeriod < 1) {
      // Never clamp to 0: a period of 0 has no meaningful semantics (it
      // would encode "too fast to subdivide" as an impossible state), and
      // the only thing that makes a zero survivable is the ISR divide-by-
      // zero guard at Ramps.c:513. Floor at 1 (the fastest representable
      // subdivision) so this writer stops relying on that guard.
      newPeriod = 1;
    }
    servoCycles = (uint16_t) newPeriod;

    for (int i = 0; i < SCALES_COUNT; i++) {
      // Update scale/spindle speed value
      deltaPositionAndError(
        rampsData->shared.scales[i].position,
        1000,
        updateSpeedTaskTicks,
        &rampsData->scalesSpeed[i]
      );
      rampsData->shared.scales[i].speed = rampsData->scalesSpeed[i].scaledDelta;
      rampsData->shared.fastData.scaleSpeed[i] = rampsData->scalesSpeed[i].scaledDelta;
    }
  }
}

_Noreturn void servoEnableTask(void *argument) {
  rampsHandler_t *rampsData = (rampsHandler_t *) argument;
  rampsSharedData_t *shared = (rampsSharedData_t *) &rampsData->shared;
  uint32_t previousPosition = 0;

  for (;;) {
    osDelay(100);

    bool anySyncMotionEnabled = false;
    for (int i = 0; i < SCALES_COUNT; i++) {
      anySyncMotionEnabled = anySyncMotionEnabled || (shared->scales[i].syncEnable != 0);
    }

    if (anySyncMotionEnabled && !shared->elsStop.active && rampsData->shared.fastData.servoMode != 2)
      rampsData->shared.fastData.servoMode = 1;

    rampsData->shared.fastData.servoSpeed = (float)(int32_t)(rampsData->shared.servo.currentSteps - previousPosition) * 10;
    previousPosition = rampsData->shared.servo.currentSteps;

    if (shared->fastData.servoMode != 0) HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_RESET);
    if (shared->fastData.servoMode == 0) HAL_GPIO_WritePin(ENA_GPIO_PORT, ENA_PIN, GPIO_PIN_SET);
  }
}