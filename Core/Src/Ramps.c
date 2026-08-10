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

/* Backstop for a backlash takeup that never reaches its commanded target. ISR
 * runs at ~100 kHz (TIM9, 10 us/tick), so this is ~5 s — far longer than any
 * legitimate takeup (tens to low hundreds of steps) even at a slow maxSpeed, and
 * deliberately generous because tripping it early would be worse than not having
 * it. It is a DIAGNOSTIC, not a control: it does not release the sync gate (see
 * the takeup block for why), it just names the failure so the UI can. The known
 * way in is servoMode != 1, where stepsToGo is never consumed at all. */
#define ELS_TAKEUP_TIMEOUT_TICKS 500000

/* How long the Z confirmation gate keeps LOOKING after the commanded take-up
 * motion has finished, before latching its verdict. ISR is ~100 kHz, so this is
 * ~250 ms.
 *
 * Neither extreme is safe. Re-evaluating FOREVER (the original behaviour) means
 * a withheld take-up is satisfied by the first Z motion from ANY source — on
 * hardware 2026-08-08 an operator nudging the carriage by hand, with the
 * half-nut open, released a correctly-withheld gate. But latching the instant
 * the move completes assumes the carriage stops dead when the servo does, and
 * it does not: there is real inertia and drivetrain compliance.
 *
 * Note the existing ELS_SETTLE_TICKS evidence does NOT cover this. That 1 s
 * dwell was tried when a take-up was fully absorbed by lash and moved the
 * carriage not at all, so there was nothing to settle. A take-up sized at
 * measured + margin deliberately DOES move the carriage, which is a different
 * regime and newer than that experiment.
 *
 * 250 ms sits well above any plausible mechanical settle at these speeds
 * (sub-millimetre moves, heavy carriage, high friction) and well below the time
 * it takes a person to reach a handwheel. It bounds the exposure; it does not
 * eliminate it — only correlating Z motion against commanded steps does that,
 * and that is the real fix (see todo.md).
 *
 * THAT FIX NOW EXISTS (els_slip.h), and it changed what this constant is FOR.
 * The window no longer carries the safety property — attribution does. What is
 * left here is purely RECOVERY LATENCY: how long the gate keeps re-evaluating a
 * late-but-genuine take-up before giving up and aborting the pass. It can stay
 * generous precisely because a hand nudge inside it no longer counts as
 * evidence. Shortening it would only make the machine give up sooner. */
#define ELS_TAKEUP_CONFIRM_WINDOW_TICKS 25000

/* Mechanical settle allowance for motion attribution: how long after a commanded
 * step pulse Z motion may still be credited to the servo rather than to whatever
 * else moved the carriage. This is the constant that replaces the 250 ms window
 * as the actual exposure bound, so it is the number an attacker — or an
 * unlucky operator — has to hit.
 *
 * THIS VALUE IS NOT COMMISSIONED. It is a starting point chosen to satisfy the
 * constraints below, not a measurement of elspi's drivetrain, and it CANNOT be
 * derived from the emulator: the emulator's lash model moves the carriage
 * instantaneously with the pulse, so it has no settle behaviour to measure at
 * all. Measuring it means watching real Z counts arrive after the last pulse of
 * a real take-up, on the machine, at the take-up speed actually in use.
 *
 * Constraints any replacement value must satisfy:
 *  - MUST exceed ELS_SETTLE_TICKS (50). The gate's first evaluation happens that
 *    many ticks after the last pulse; a shorter horizon rejects the inertial
 *    settle this whole mechanism exists to accept.
 *  - MUST exceed the live pulse pacing period (servoCycles), or genuine coupled
 *    motion mid-burst is discarded and a HEALTHY machine refuses to start. Not
 *    left to this constant — elsSlipSettleTicks() floors it at runtime — but a
 *    value below the pacing period means that floor is doing all the work and
 *    the number here is decorative.
 *  - Ticks, not milliseconds. At the ~100 kHz hardware ISR rate 1000 ticks is
 *    ~10 ms. The emulator's real-time serve loop runs the same ISR ~10x slower,
 *    so anything tuned by watching wall-clock there is 10x wrong here
 *    (els_slip.h has the full unit-trap list).
 *
 * Smaller is safer and only becomes unsafe in one direction: too small starts
 * refusing healthy take-ups. Tune it DOWN from here against a machine that still
 * confirms reliably, never up to make a refusal go away. */
#define ELS_SLIP_SETTLE_TICKS 1000

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
  /* Register-layout version. Bump this whenever elsStop_t / rampsSharedData_t
   * changes shape; reflex-ui reads it at connect so a firmware/UI map mismatch
   * reports itself by name instead of surfacing as garbled register reads. */
  rampsData->shared.elsStop.protocolVersion = 1;
  rampsData->shared.elsStop.calCommand   = 0;
  rampsData->shared.elsStop.calSeq       = 0;
  rampsData->shared.elsStop.calResult    = ELS_CAL_OK;
  rampsData->shared.elsStop.takeupResult = ELS_CAL_OK;
  rampsData->shared.elsStop.takeupSeq    = 0;
  rampsData->elsCal.phase                = ELS_CAL_IDLE;

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

/* Leadscrew backlash calibration driver.
 *
 * Measures the lash by driving the carriage against a flank, then reversing and
 * counting the servo steps consumed before the Z scale moves. That step count IS
 * the backlash — see els_backlash_cal.h for why this is the only construction
 * that can distinguish a healthy take-up from an uncoupled drivetrain, and why
 * "Z must move backlashSteps worth" is wrong in the unsafe direction.
 *
 * Three reversals are measured; the HOST judges whether the spread is acceptable
 * and writes the resulting backlashSteps. Measurement here, policy in the UI.
 *
 * The state machine itself is the pure elsCalTick(); this function is only the
 * actuation shell — request intake, servo commands, abort conditions, and
 * publishing results. That split is what makes the logic host-testable without
 * the HAL.
 *
 * SAFETY: refuses to run unless elsStop.enable == 0 (no live job, so sync is not
 * driving the leadscrew and the spindle cannot fight the measurement) and
 * servoMode == 1 (otherwise stepsToGo is never consumed and the run would hang).
 * Total travel is a few lash widths — well under a millimetre — but it IS
 * bidirectional carriage motion with the half-nut engaged, so the operator-facing
 * modal owns "tool clear of the work, spindle stopped". */
static inline void elsCalUpdate(rampsHandler_t *data) {
  rampsSharedData_t *shared = &data->shared;

  /* ---- Request intake. calCommand is consumed atomically here and cleared in
   * the same ISR pass, which is the whole point of the command/ack split: a
   * 32-bit host write racing this ISR could otherwise be seen half-applied.
   * calSeq is the ack — SW must edge-detect THAT, never poll calCommand. ---- */
  if (shared->elsStop.calCommand != 0u) {
    shared->elsStop.calCommand = 0u;
    if (data->elsCal.phase == ELS_CAL_IDLE) {
      uint16_t refuse = ELS_CAL_OK;
      if (shared->elsStop.enable != 0u) {
        refuse = ELS_CAL_ERR_ENABLED;
      } else if (shared->fastData.servoMode != 1u) {
        refuse = ELS_CAL_ERR_SERVOMODE;
      } else if (shared->elsStop.calCeilingSteps <= 0
                 || shared->elsStop.calMotionThreshCounts <= 0) {
        refuse = ELS_CAL_ERR_CONFIG;
      }

      if (refuse != ELS_CAL_OK) {
        shared->elsStop.calResult = refuse;
        shared->elsStop.calSeq++;            /* refusals are outcomes too */
      } else {
        /* Cutting direction, same derivation as the takeup and els_phase.h. When
         * no thread geometry is configured (a machine-level calibration on a
         * fresh setup) the product is 0, which is not < 0, so this degrades to
         * sign(syncRatioNum) — a sane default rather than a refusal. */
        int32_t cuttingDir = (shared->scales[0].syncRatioNum > 0) ? 1 : -1;
        if (shared->elsStop.threadPitchSteps * shared->elsStop.zCountsPerPitch < 0.0f) {
          cuttingDir = -cuttingDir;
        }
        shared->servo.stepsToGo    = 0;
        shared->servo.currentSpeed = 0;
        elsCalStart(&data->elsCal, cuttingDir,
                    (int32_t)shared->servo.currentSteps,
                    shared->scales[shared->elsStop.scaleIndex].position);
        shared->servo.stepsToGo = data->elsCal.driveSign * shared->elsStop.calCeilingSteps;
      }
    }
  }

  if (data->elsCal.phase == ELS_CAL_IDLE) return;

  /* ---- Abort. Either condition means the ground shifted under a run that is
   * physically moving the carriage, so stop commanding motion immediately rather
   * than finishing a measurement whose premises no longer hold. ---- */
  if (shared->elsStop.enable != 0u || shared->fastData.servoMode != 1u) {
    shared->servo.stepsToGo    = 0;
    shared->servo.currentSpeed = 0;
    data->elsCal.phase         = ELS_CAL_IDLE;
    shared->elsStop.calResult  = ELS_CAL_ERR_ABORTED;
    shared->elsStop.calSeq++;
    return;
  }

  elsCalAction_t act = elsCalTick(&data->elsCal,
                                  (int32_t)shared->servo.currentSteps,
                                  shared->scales[shared->elsStop.scaleIndex].position,
                                  shared->servo.stepsToGo,
                                  shared->elsStop.calMotionThreshCounts);

  if (act.startPhase) {
    /* Command the full ceiling; the leg ends early the moment Z moves. Draining
     * to zero without motion is exactly the uncoupled failure. */
    shared->servo.stepsToGo = act.driveSign * shared->elsStop.calCeilingSteps;
  }

  if (act.finished) {
    shared->servo.stepsToGo    = 0;
    shared->servo.currentSpeed = 0;
    /* Publish partial measurements on failure too — a run that measured two
     * reversals and then lost the carriage is diagnostically richer than a bare
     * error code, and the host already knows how many to trust from calResult. */
    for (int32_t i = 0; i < ELS_CAL_CYCLES; i++) {
      shared->elsStop.calMeasured[i] = data->elsCal.measured[i];
    }
    shared->elsStop.calResult = data->elsCal.result;
    shared->elsStop.calSeq++;
    data->elsCal.phase = ELS_CAL_IDLE;
  }
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
    /* Also abandon any in-flight takeup. This is the escape hatch that makes the
     * fail-closed Z confirmation gate below RECOVERABLE: a takeup withheld for
     * want of Z confirmation holds takeupPending = 1 indefinitely, which gates
     * sync off, and nothing else in the ISR clears it. Dropping enable ends the
     * job, so there is nothing left to confirm and no phase correction worth
     * applying. Without this, failing closed would be unrecoverable — a worse
     * defect than the one being fixed. */
    shared->elsStop.takeupPending = 0;
    data->elsStopSettleCount      = 0;
    data->elsStopTakeupTicks      = 0;
    data->elsStopTakeupLatched    = 0;
  }

  data->elsStopPreviousEnable = shared->elsStop.enable;

  // Detect completion of post-resume backlash takeup move, dwell for the servo
  // to settle, CONFIRM THE CARRIAGE ACTUALLY MOVED, then apply phase correction.
  // takeupPending stays set throughout so sync remains gated and the servo holds
  // at the takeup target.
  if (shared->elsStop.takeupPending) {
    data->elsStopTakeupTicks++;
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
        /* ---- Z CONFIRMATION GATE -------------------------------------------
         * `takeupReached` above is a PURE COMMANDED-STEP-COUNT CROSSING TEST. It
         * compares servo.currentSteps against a target this same firmware
         * computed and assigned at initiation, so it confirms exactly one thing:
         * that the firmware finished issuing the pulses it decided to issue.
         * Nothing in it observes the machine. If the half-nut is open, the servo
         * is disabled or faulted, or the coupling has slipped, currentSteps
         * crosses the target on schedule and the firmware reports a completed
         * takeup into thin air — and applyPhaseCorrection then snapshots a Z
         * from a drivetrain that was never coupled.
         *
         * ("Wait longer" is NOT the fix — see the ELS_SETTLE_TICKS note at the
         * top of this file. A 1 s dwell was tried on the real machine and
         * changed nothing. Dwelling cannot manufacture evidence.)
         *
         * The gate is decidable ONLY because the take-up is commanded past the
         * measured lash (elsCalTakeupCommand's measured + margin). A take-up
         * sized exactly at the lash legitimately produces zero carriage motion,
         * which is indistinguishable from an open half-nut — see the physics
         * note in els_backlash_cal.h. The margin is what creates the evidence.
         *
         * FAIL CLOSED, and note WHERE this runs: at the START of a pass. active
         * has just been cleared, the takeup has consumed stepsToGo, and
         * takeupPending gates the sync accumulator below. The machine is
         * standing still with the tool clear of the work, about to BEGIN a cut.
         * Refusing here declines to start a pass; it does not abandon a tool
         * buried in a groove. Between a machine that visibly refuses to start
         * and one that confidently starts in the wrong groove, the refusal is
         * far the cheaper failure.
         *
         * The gate RE-EVALUATES every tick rather than latching dead, so a
         * takeup whose Z motion is merely late still clears itself;
         * elsStopSettleCount is deliberately left at ELS_SETTLE_TICKS while
         * withheld so no second dwell is imposed once it does. */
        int32_t zNow = shared->scales[shared->elsStop.scaleIndex].position;
        shared->elsStop.lastTakeupZDelta =
            elsZMovedAlong(zNow, data->elsStopTakeupZStart, data->elsStopTakeupZSign, 1);

        /* Confirm against EXPECTED motion, not the bare detection floor. A
         * calibrated take-up is entitled to move the carriage by
         * (margin + detection distance); demanding a fraction of that rejects a
         * take-up that barely twitched when it should have moved several counts
         * — partial half-nut engagement, a slipping nut — which the floor alone
         * would wave through. Falls back to the floor with no calibration on
         * file or in turning mode, so an uncommissioned machine behaves exactly
         * as it did before. Published for the UI so a refusal can say what it
         * wanted, not just that it refused. */
        shared->elsStop.takeupThreshCounts = elsTakeupConfirmThreshold(
            (int32_t)shared->elsStop.backlashSteps,
            elsCalMeanValid(shared->elsStop.calMeasured, ELS_CAL_CYCLES),
            shared->elsStop.threadPitchSteps,
            shared->elsStop.zCountsPerPitch,
            shared->elsStop.calMotionThreshCounts);

        /* ATTRIBUTED motion only. lastTakeupZDelta above still reports the raw
         * endpoint delta — it is the wrong-way diagnostic and the number the UI
         * shows — but the endpoint comparison is NOT what decides any more.
         *
         * The endpoint delta cannot tell "the drivetrain moved the carriage"
         * from "something else moved the carriage during the same 250 ms", and
         * on 2026-08-08 a hand nudge with the half-nut open exploited exactly
         * that. elsSlipConfirmed() counts only the Z counts that arrived while
         * the servo was driving or settling from a pulse (els_slip.h), which is
         * evidence of coupling rather than merely evidence of motion.
         *
         * Same floor, same fail-closed convention, same `>=`: this is a
         * strictly SMALLER numerator against an unchanged threshold, so nothing
         * that was refused before is confirmed now. Read els_slip.h on why this
         * narrows the exposure without closing it — a nudge landing inside the
         * settle horizon of a real pulse is still indistinguishable, and no
         * amount of arithmetic here changes that. */
        if (elsSlipConfirmed(&data->elsSlip, shared->elsStop.takeupThreshCounts)) {
          if (shared->elsStop.takeupResult != ELS_CAL_OK) {
            shared->elsStop.takeupResult = ELS_CAL_OK;
          }
          shared->elsStop.takeupSeq++;
          data->elsStopSettleCount      = 0;
          data->elsStopTakeupTicks      = 0;
          shared->elsStop.takeupPending = 0;
          applyPhaseCorrection(shared);     // snapshot Z only once CONFIRMED
        } else if (shared->elsStop.takeupResult != ELS_TAKEUP_ERR_UNCONFIRMED) {
          /* Report once on the transition, not every tick, so takeupSeq stays a
           * count of OUTCOMES rather than a tick counter. */
          shared->elsStop.takeupResult = ELS_TAKEUP_ERR_UNCONFIRMED;
          shared->elsStop.takeupSeq++;
        }

        /* Close the window once late motion has had its chance, then ABORT THE
         * PASS back to the stopped state rather than holding the machine.
         *
         * Holding takeupPending forever was worse than useless: sync stayed
         * gated with no way out but the enable 1->0 escape hatch, and that hatch
         * clears referenceLatched on the next 0->1 edge — so recovering from a
         * FALSE alarm cost the operator their thread phase reference. The
         * remedy was more expensive than the fault.
         *
         * Aborting to active = 1 returns the machine to exactly where it was
         * before the operator pressed Cut: stopped at the shoulder, reference
         * intact, sync gated because we are stopped rather than because we are
         * stuck. The operator closes the half-nut and presses Cut again. Nothing
         * to reset, nothing lost.
         *
         * takeupResult stays UNCONFIRMED so the UI can say why, and is cleared
         * at the next take-up initiation, which makes the warning self-clearing
         * on retry. */
        if (!data->elsStopTakeupLatched) {
          if (data->elsStopSettleCount
              < (ELS_SETTLE_TICKS + ELS_TAKEUP_CONFIRM_WINDOW_TICKS)) {
            data->elsStopSettleCount++;
          } else {
            data->elsStopTakeupLatched    = 1;
            shared->elsStop.takeupPending = 0;   /* stop holding the machine */
            shared->elsStop.active        = 1;   /* back to stopped-at-shoulder */
            shared->servo.stepsToGo       = 0;
            shared->servo.currentSpeed    = 0;
            data->elsStopSettleCount      = 0;
            data->elsStopTakeupTicks      = 0;
            /* referenceLatched deliberately untouched: a retry must be free. */
          }
        }
      }
    } else if (data->elsStopTakeupTicks > ELS_TAKEUP_TIMEOUT_TICKS
               && shared->elsStop.takeupResult != ELS_TAKEUP_ERR_TIMEOUT) {
      /* Backstop for a takeup that never reaches its target at all — the
       * servoMode != 1 hazard being the known way in (stepsToGo is only consumed
       * by updateIndexingPosition, so in mode 0 or 2 the takeup never advances
       * and sync stays gated with nothing to diagnose it). This does NOT release
       * the gate: releasing would start a pass on a takeup we know did not
       * happen. It names the failure so the UI can say so; recovery is the
       * enable 1->0 escape hatch above. */
      shared->elsStop.takeupResult = ELS_TAKEUP_ERR_TIMEOUT;
      shared->elsStop.takeupSeq++;
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
        /* Hysteresis gate (elsStop.hysteresis, Ramps.h:102). Distance the axis
         * currently sits CLEAR of the threshold, on the retract side. The flag
         * is sticky-true until the next latch, so a resume issued with the axis
         * still at/past the threshold cannot re-latch in the same ISR pass and
         * swallow the 1->0 edge the resume path (Ramps.c:455) depends on.
         * hysteresis <= 0 sets the flag unconditionally every pass, which is
         * exactly the pre-gate behavior. */
        int32_t clearance = (shared->elsStop.stopDirection >= 0)
                            ? (shared->elsStop.stopPosition - refPos)
                            : (refPos - shared->elsStop.stopPosition);
        if (shared->elsStop.hysteresis <= 0 || clearance >= shared->elsStop.hysteresis) {
          data->elsStopHysteresisCleared = 1;
        }
        bool shouldStop = ((shared->elsStop.stopDirection >= 0)
                          ? (refPos >= shared->elsStop.stopPosition)
                          : (refPos <= shared->elsStop.stopPosition))
                          && data->elsStopHysteresisCleared;
        if (shouldStop) {
          shared->elsStop.active = 1;
          data->elsStopHysteresisCleared = 0;
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

      /* Sync paused while stopped, while a backlash takeup is in progress, or
       * while a backlash CALIBRATION is running.
       *
       * The calibration gate is not optional and elsStop.enable == 0 does not
       * imply it: scales[].syncEnable is INDEPENDENT of the ELS stop feature, so
       * a turning spindle drives the leadscrew through sync even with no
       * threading job armed. During a calibration that motion is superimposed on
       * the measurement legs — at the reference geometry it swamps them
       * outright, and the run either measures the spindle or fails to arm at all
       * (observed: measured 0,0,0 / NO_MOTION on a perfectly healthy simulated
       * drivetrain before this gate existed). A calibration leg is only
       * meaningful if the leadscrew is moved by the calibration and nothing
       * else. */
      if (!shared->elsStop.active && !shared->elsStop.takeupPending
          && data->elsCal.phase == ELS_CAL_IDLE) {
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
        /* Baseline for the Z confirmation gate. Captured at INITIATION, before
         * any pulses are issued, so the gate measures the whole takeup. */
        data->elsStopTakeupZStart      = shared->scales[shared->elsStop.scaleIndex].position;
        /* Same instant, same reason, for the attribution accumulator: it must
         * cover the whole take-up. It starts credited with nothing and stays
         * that way until the first pulse fires (els_slip.h), so the Z motion of
         * this initiation tick — which physically predates the take-up — cannot
         * be counted as evidence for it. */
        elsSlipReset(&data->elsSlip);
        data->elsStopTakeupTicks       = 0;
        data->elsStopTakeupLatched     = 0;
        shared->elsStop.takeupResult   = ELS_CAL_OK;
        /* Direction the Z scale should count in for this takeup. droSign is
         * els_phase.h's quantity — how a cutting-direction servo move changes the
         * DRO — and the takeup itself runs in cuttingDir, so this reduces
         * algebraically to stopDirection. Kept as the explicit product because
         * the reduction is a coincidence of this call site, not an invariant.
         * Only the MAGNITUDE gates completion (see els_backlash_cal.h on why
         * detection is polarity-free); this sign is what turns lastTakeupZDelta
         * into a wrong-way diagnostic rather than just a distance. */
        data->elsStopTakeupZSign       = data->elsStopTakeupSign
                                       * ((int32_t)shared->elsStop.stopDirection * cuttingDir);
      } else {
        applyPhaseCorrection(shared);
      }
    }
  }
  data->elsStopPreviousActive = shared->elsStop.active;

  /* Backlash calibration. Deliberately placed AFTER the scale-read loop (so it
   * sees this tick's Z, not last tick's) and BEFORE updateIndexingPosition (so a
   * drive burst it commands takes effect the same tick, and the stepsToGo it
   * reads is the post-drain value from the previous tick). */
  elsCalUpdate(data);

  if (shared->fastData.servoMode == 1) updateIndexingPosition(data);
  if (shared->fastData.servoMode == 2) updateJogPosition(data);

  /* Bracket the pulse generator to recover THIS TICK's signed commanded step
   * delta. The generator's own `direction` is a local that dies at the end of
   * the block, and it is set to 1 even on ticks that emit nothing (it doubles as
   * the DIR pin state), so it is not the quantity wanted here. Differencing the
   * counter the generator actually writes is: it is -1/0/+1 by construction, it
   * cannot disagree with the pulses emitted, and it needs no new state. Unsigned
   * wraparound is intentional and well-defined — the ±1 survives the round
   * trip. */
  uint32_t servoStepsBeforePulse = shared->servo.currentSteps;

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

  /* MOTION ATTRIBUTION — and the placement is the whole trick.
   *
   * This is the ONLY point in the ISR where this tick's Z delta and this tick's
   * commanded step delta are both fresh. The confirmation gate above runs at the
   * TOP of the pass, before the scale-refresh loop and long before this block,
   * so a per-tick correlation computed up there would pair LAST tick's Z against
   * THIS tick's pulse — a stale correlation that would look like it worked, and
   * would quietly mis-attribute exactly the motion it exists to judge.
   *
   * Cost of that ordering: the accumulator the gate reads is one tick behind,
   * the same 10 us staleness the gate already had against its own Z sample. It
   * is noise against a 250 ms window and against any settle horizon worth
   * setting.
   *
   * dZ is taken already signed by scaleDir, matching how scales[].position is
   * accumulated in the refresh loop, so attribution and the endpoint diagnostic
   * agree about which way the carriage went. */
  if (shared->elsStop.takeupPending) {
    uint16_t zIdx   = shared->elsStop.scaleIndex;
    int32_t  dZ     = data->scalesDeltaPos[zIdx].delta * shared->scales[zIdx].scaleDir;
    int32_t  dServo = (int32_t)(shared->servo.currentSteps - servoStepsBeforePulse);
    elsSlipTick(&data->elsSlip, dZ, dServo,
                elsSlipSettleTicks(ELS_SLIP_SETTLE_TICKS, (int32_t)servoCycles));
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