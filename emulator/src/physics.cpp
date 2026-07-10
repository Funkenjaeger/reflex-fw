/*
 * Lathe physics model implementation.
 */

#include "physics.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>

extern "C" {
#include "emulator_state.h"
#include "Ramps.h"
void emu_update_timer_counters(void);
}

LathePhysics::LathePhysics(const EmuConfig &cfg) {
    spindle_inertia = cfg.spindle_inertia;
    spindle_max_torque = cfg.spindle_max_torque;
    spindle_friction = cfg.spindle_friction;
    spindle_counts_per_rev = cfg.spindle_counts_per_rev;
    leadscrew_mm_per_step = cfg.leadscrew_mm_per_step;
    leadscrew_tpi = cfg.leadscrew_tpi;
    leadscrew_grid_spacing_mm = 25.4 / leadscrew_tpi;
    z_counts_per_mm = cfg.z_encoder_counts_per_mm;
    z_backlash_mm = cfg.z_backlash_mm;
    z_max_mm = cfg.z_max_mm;
    z_min_mm = cfg.z_min_mm;
    x_counts_per_mm = cfg.x_encoder_counts_per_mm;
    x_max_mm = cfg.x_max_mm;
    x_min_mm = cfg.x_min_mm;
    x_manual_step_mm = cfg.x_manual_step_mm;

    /* Physical wiring signs (see physics.h). Default +1 => no change. */
    spindle_scale_sign = (cfg.spindle_scale_dir < 0) ? -1 : 1;
    z_scale_sign       = (cfg.z_scale_dir < 0) ? -1 : 1;
    x_scale_sign       = (cfg.x_scale_dir < 0) ? -1 : 1;
    servo_sign         = (cfg.servo_dir < 0) ? -1 : 1;

    spindle_theta = 0.0;
    spindle_omega = cfg.spindle_initial_rpm * 2.0 * M_PI / 60.0;
    spindle_target_rpm = cfg.spindle_initial_rpm;

    leadscrew_position_mm = 0.0;
    leadscrew_total_steps = 0;

    carriage_mm = cfg.z_initial_mm;
    half_nut_state = cfg.z_half_nut_engaged ? ENGAGED : DISENGAGED;
    half_nut_request_pending = false;
    backlash_offset = z_backlash_mm;  /* assume nut starts on "+ wall" so first +Z motion drives carriage */
    last_manual_dir = 0.0;

    cross_slide_mm = cfg.x_initial_mm;

    z_jog_velocity = 0.0;
    z_jog_target_dir = 0.0;
    z_move_active = false;
    z_move_target = 0.0;
    x_jog_velocity = 0.0;
    x_jog_target_dir = 0.0;
    x_move_active = false;
    x_move_target = 0.0;
    jog_max_velocity = cfg.jog_max_velocity_mm_s;
    jog_acceleration = cfg.jog_acceleration_mm_s2;
    z_jog_idle_timer = JOG_KEY_TIMEOUT + 1.0;
    x_jog_idle_timer = JOG_KEY_TIMEOUT + 1.0;
}

void LathePhysics::tick(double dt, const void *shared_data) {
    const rampsSharedData_t *shared = (const rampsSharedData_t *)shared_data;
    /* --- Spindle dynamics --- */
    double target_omega = spindle_target_rpm * 2.0 * M_PI / 60.0;
    double error = target_omega - spindle_omega;

    /* Proportional + bang-bang torque controller.
     * Far from target: full torque (fast ramp).
     * Near target: proportional torque to settle smoothly.
     * At target: compensate friction exactly (no oscillation). */
    double torque_motor = 0.0;
    double omega_threshold = spindle_max_torque / spindle_inertia * 0.5; /* ~20 rad/s transition band */

    if (std::abs(error) < 0.001) {
        /* At steady state: exactly counteract friction to hold speed */
        if (std::abs(spindle_omega) > 0.001)
            torque_motor = (spindle_omega > 0) ? spindle_friction : -spindle_friction;
    } else if (std::abs(error) < omega_threshold) {
        /* Near target: proportional control + friction feedforward */
        double kp = spindle_max_torque / omega_threshold;
        torque_motor = kp * error;
        if (std::abs(target_omega) > 0.001)
            torque_motor += (target_omega > 0) ? spindle_friction : -spindle_friction;
    } else {
        /* Far from target: full torque */
        torque_motor = (error > 0) ? spindle_max_torque : -spindle_max_torque;
    }

    /* Friction opposes motion */
    double torque_friction = 0.0;
    if (std::abs(spindle_omega) > 0.001) {
        torque_friction = (spindle_omega > 0) ? -spindle_friction : spindle_friction;
    }

    double alpha = (torque_motor + torque_friction) / spindle_inertia;
    spindle_omega += alpha * dt;

    /* Clamp to target if we've effectively arrived */
    if (std::abs(target_omega - spindle_omega) < 0.001 && std::abs(target_omega) > 0.001) {
        spindle_omega = target_omega;
    }

    /* Stop fully if target is zero and speed is very low */
    if (std::abs(spindle_target_rpm) < 0.01 && std::abs(spindle_omega) < 0.1) {
        spindle_omega = 0.0;
    }

    spindle_theta += spindle_omega * dt;

    /* --- Half-nut engagement logic --- */
    if (half_nut_request_pending) {
        if (half_nut_state == DISENGAGED || half_nut_state == ENGAGING) {
            /* Request to engage.
             * The leadscrew moves only when the firmware's servo is producing
             * steps (sync enabled + spindle turning). Check actual servo speed. */
            bool leadscrew_moving = shared && std::abs(shared->servo.currentSpeed) > 0.1;

            if (!leadscrew_moving) {
                /* Leadscrew stationary: snap carriage to nearest grid point and engage.
                 * Set the lash wall from the last manual move direction: after the
                 * operator hand-moves the carriage (nut open) and re-closes, the nut
                 * rests on the wall opposite the next drive direction, so the first
                 * leadscrew move back the other way must traverse the full lash.
                 * (Moved +Z -> rest on +wall; moved -Z -> rest on -wall.) */
                snapCarriageToGrid();
                if (last_manual_dir > 0.0)      backlash_offset = z_backlash_mm;
                else if (last_manual_dir < 0.0) backlash_offset = 0.0;
                half_nut_state = ENGAGED;
                half_nut_request_pending = false;
                emu_log_event("half-nut ENGAGED (snap to %.3f mm, lash=%.3f)",
                              carriage_mm, backlash_offset);
            } else {
                /* Leadscrew turning: wait for phase alignment */
                if (half_nut_state != ENGAGING) {
                    half_nut_state = ENGAGING;
                    emu_log_event("half-nut ENGAGING...");
                }
                if (checkPhaseAlignment()) {
                    half_nut_state = ENGAGED;
                    half_nut_request_pending = false;
                    emu_log_event("half-nut ENGAGED (phase match)");
                }
            }
        } else {
            /* Currently ENGAGED: request to disengage */
            half_nut_state = DISENGAGED;
            half_nut_request_pending = false;
            emu_log_event("half-nut DISENGAGED");
        }
    }

    /* --- Manual move integration --- */

    /* Helper: compute jog direction for a move-to-position target.
     * Returns +1/-1 while moving, 0 when arrived (with deceleration). */
    auto moveToDir = [&](double current, double target, double velocity) -> double {
        double error = target - current;
        double stop_dist = (velocity * velocity) / (2.0 * jog_acceleration);
        if (std::abs(error) < 0.001 && std::abs(velocity) < 0.01)
            return 0.0;  /* arrived */
        if (std::abs(error) <= stop_dist + 0.001)
            return 0.0;  /* time to decelerate */
        return (error > 0) ? 1.0 : -1.0;
    };

    /* Z-axis (only when half-nut disengaged) */
    z_jog_idle_timer += dt;
    if (z_jog_idle_timer > JOG_KEY_TIMEOUT) z_jog_target_dir = 0.0;

    if (half_nut_state != ENGAGED) {
        /* Move-to-position overrides arrow key jog */
        double z_dir = z_jog_target_dir;
        if (z_move_active) {
            z_dir = moveToDir(carriage_mm, z_move_target, z_jog_velocity);
            if (z_dir == 0.0 && std::abs(z_jog_velocity) < 0.01) {
                z_move_active = false;
                carriage_mm = z_move_target;  /* snap to exact target */
                z_jog_velocity = 0.0;
                emu_log_event("Z move complete: %.3f mm", carriage_mm);
            }
        }

        double z_target_vel = z_dir * jog_max_velocity;
        double z_vel_error = z_target_vel - z_jog_velocity;
        if (std::abs(z_vel_error) > 0.001) {
            double accel = (z_vel_error > 0) ? jog_acceleration : -jog_acceleration;
            z_jog_velocity += accel * dt;
            double new_error = z_target_vel - z_jog_velocity;
            if (z_vel_error * new_error < 0) z_jog_velocity = z_target_vel;
        } else {
            z_jog_velocity = z_target_vel;
        }
        double z_step = z_jog_velocity * dt;
        if (std::abs(z_step) > 1e-9) last_manual_dir = (z_step > 0) ? 1.0 : -1.0;
        carriage_mm += z_step;
        carriage_mm = std::max(z_min_mm, std::min(z_max_mm, carriage_mm));
    } else {
        z_jog_velocity = 0.0;
        z_move_active = false;
    }

    /* X-axis (always manual) */
    x_jog_idle_timer += dt;
    if (x_jog_idle_timer > JOG_KEY_TIMEOUT) x_jog_target_dir = 0.0;

    {
        double x_dir = x_jog_target_dir;
        if (x_move_active) {
            x_dir = moveToDir(cross_slide_mm, x_move_target, x_jog_velocity);
            if (x_dir == 0.0 && std::abs(x_jog_velocity) < 0.01) {
                x_move_active = false;
                cross_slide_mm = x_move_target;
                x_jog_velocity = 0.0;
                emu_log_event("X move complete: %.3f mm", cross_slide_mm);
            }
        }

        double x_target_vel = x_dir * jog_max_velocity;
        double x_vel_error = x_target_vel - x_jog_velocity;
        if (std::abs(x_vel_error) > 0.001) {
            double accel = (x_vel_error > 0) ? jog_acceleration : -jog_acceleration;
            x_jog_velocity += accel * dt;
            double new_error = x_target_vel - x_jog_velocity;
            if (x_vel_error * new_error < 0) x_jog_velocity = x_target_vel;
        } else {
            x_jog_velocity = x_target_vel;
        }
        cross_slide_mm += x_jog_velocity * dt;
        cross_slide_mm = std::max(x_min_mm, std::min(x_max_mm, cross_slide_mm));
    }

    /* --- Update encoder counters for firmware --- */
    emu_hw.scale_counters[0] = (uint32_t)(int32_t)getSpindleEncoderCounts();
    emu_hw.scale_counters[1] = (uint32_t)(int32_t)getCarriageEncoderCounts();
    emu_hw.scale_counters[2] = (uint32_t)(int32_t)getCrossSlideEncoderCounts();
    emu_hw.scale_counters[3] = 0;

    /* Write into the TIM counter registers */
    emu_update_timer_counters();
}

void LathePhysics::onStepPulse(int direction) {
    /* direction: +1 or -1, from DIR pin (set by the firmware's servoDir register).
     * servo_sign models the PHYSICAL motor wiring: on a reverse-wired motor the
     * same DIR pin drives the leadscrew the opposite physical way. This is
     * independent of servoDir, which reflex-ui owns -- the operator's servo
     * reverse toggle must cancel servo_sign for a commanded +move to go +. */
    int phys_dir = direction * servo_sign;
    leadscrew_total_steps += direction;
    leadscrew_position_mm += phys_dir * leadscrew_mm_per_step;

    if (half_nut_state == ENGAGED) {
        /* Faithful (non-lossy) backlash model: track nut position within play window
         * [0, z_backlash_mm]. Carriage only moves when nut hits a wall and pushes.
         * Partial reversals only consume the actual traversal distance. */
        double move = phys_dir * leadscrew_mm_per_step;
        double new_offset = backlash_offset + move;

        if (new_offset > z_backlash_mm) {
            /* Hit the +wall: overshoot drives carriage forward */
            carriage_mm += new_offset - z_backlash_mm;
            new_offset = z_backlash_mm;
        } else if (new_offset < 0.0) {
            /* Hit the -wall: overshoot drives carriage backward */
            carriage_mm += new_offset;  /* new_offset is negative */
            new_offset = 0.0;
        }
        backlash_offset = new_offset;

        carriage_mm = std::max(z_min_mm, std::min(z_max_mm, carriage_mm));
    }
}

void LathePhysics::setTargetRPM(double rpm) {
    spindle_target_rpm = rpm;
}

void LathePhysics::toggleDirection() {
    spindle_target_rpm = -spindle_target_rpm;
}

void LathePhysics::emergencyStop() {
    spindle_target_rpm = 0.0;
    emu_log_event("E-STOP: spindle target -> 0");
}

int64_t LathePhysics::getSpindleEncoderCounts() const {
    /* Convert cumulative angle to encoder counts.
     * This wraps at 16-bit for TIM1 (which is how the real encoder works). */
    double counts = spindle_theta / (2.0 * M_PI) * spindle_counts_per_rev;
    return (int64_t)counts * spindle_scale_sign;
}

void LathePhysics::requestHalfNutToggle() {
    half_nut_request_pending = true;
}

void LathePhysics::jogCarriage(int direction) {
    if (half_nut_state == ENGAGED) return;
    z_move_active = false;  /* arrow key jog cancels move-to-position */
    z_jog_target_dir = (double)direction;
    z_jog_idle_timer = 0.0;
}

void LathePhysics::moveCarriageTo(double target_mm) {
    if (half_nut_state == ENGAGED) return;
    target_mm = std::max(z_min_mm, std::min(z_max_mm, target_mm));
    z_move_target = target_mm;
    z_move_active = true;
    z_jog_target_dir = 0.0;  /* cancel any arrow key jog */
}

int64_t LathePhysics::getCarriageEncoderCounts() const {
    return (int64_t)(carriage_mm * z_counts_per_mm) * z_scale_sign;
}

void LathePhysics::jogCrossSlide(int direction) {
    x_move_active = false;  /* arrow key jog cancels move-to-position */
    x_jog_target_dir = (double)direction;
    x_jog_idle_timer = 0.0;
}

void LathePhysics::moveCrossSlideTo(double target_mm) {
    target_mm = std::max(x_min_mm, std::min(x_max_mm, target_mm));
    x_move_target = target_mm;
    x_move_active = true;
    x_jog_target_dir = 0.0;
}

int64_t LathePhysics::getCrossSlideEncoderCounts() const {
    return (int64_t)(cross_slide_mm * x_counts_per_mm) * x_scale_sign;
}

/* --- Half-nut engagement helpers --- */

double LathePhysics::getLeadscrewPhase() const {
    /* Phase within one revolution of the leadscrew (0.0 to 1.0) */
    double revolutions = leadscrew_position_mm / leadscrew_grid_spacing_mm;
    double phase = fmod(revolutions, 1.0);
    if (phase < 0.0) phase += 1.0;
    return phase;
}

double LathePhysics::getCarriageGridPhase() const {
    /* Where is the carriage relative to the leadscrew thread grid? */
    double grid = leadscrew_grid_spacing_mm;
    double offset = fmod(leadscrew_position_mm, grid);
    if (offset < 0.0) offset += grid;
    double revolutions = (carriage_mm - offset) / grid;
    double phase = fmod(revolutions, 1.0);
    if (phase < 0.0) phase += 1.0;
    return phase;
}

void LathePhysics::snapCarriageToGrid() {
    double grid = leadscrew_grid_spacing_mm;
    double offset = fmod(leadscrew_position_mm, grid);
    if (offset < 0.0) offset += grid;
    double before = carriage_mm;
    carriage_mm = round((carriage_mm - offset) / grid) * grid + offset;
    carriage_mm = std::max(z_min_mm, std::min(z_max_mm, carriage_mm));
    emu_log_event("SNAP before=%.4f after=%.4f grid=%.4f offset=%.4f lsPos=%.4f dZ=%.4f",
                  before, carriage_mm, grid, offset, leadscrew_position_mm,
                  carriage_mm - before);
}

bool LathePhysics::checkPhaseAlignment() const {
    double ls_phase = getLeadscrewPhase();
    double carr_phase = getCarriageGridPhase();
    double delta = std::abs(ls_phase - carr_phase);
    if (delta > 0.5) delta = 1.0 - delta;  /* wrap around */

    /* Tolerance: within ~2% of a revolution */
    return delta < 0.02;
}
