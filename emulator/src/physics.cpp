/*
 * Lathe physics model implementation.
 */

#include "physics.h"
#include <cmath>
#include <algorithm>
#include <cstdlib>

extern "C" {
#include "emulator_state.h"
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

    time_since_step_s = 1e9;  /* no pulses yet: leadscrew reads stationary */
    last_phys_dir = 0;
    {
        unsigned seed = 0x5EED1A7E;  /* fixed default: reproducible runs */
        if (const char *s = getenv("EMU_SEED")) seed = (unsigned)strtoul(s, nullptr, 0);
        lash_rng.seed(seed);
    }

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
    (void)shared_data;  /* signature kept for callers; firmware state is no longer read here */
    time_since_step_s += dt;

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
             * "Moving" is judged from our own recent step-pulse activity, NOT
             * from shared->servo.currentSpeed: that is the indexing/jog ramp
             * variable, pinned to 0 during pure ELS sync (sync bypasses the
             * ramp and adds straight to desiredSteps), so it read the
             * leadscrew as stationary mid-cut and snap-teleported the
             * carriage instead of taking the phase-match path.
             * Caveats: (a) main.cpp edge-detects STEP once per tick post-ISR,
             * so back-to-back pulses at the ~10k steps/s ceiling can miss
             * edges — no workflow engages during a full-speed retract, so
             * accepted; (b) below ~10 Hz pulse rate (≈1 RPM sync) the gate
             * reads stationary and the snap path runs, matching an operator
             * hand-jogging the nut in; (c) if pulses pause >window while
             * ENGAGING (e.g. an ELS stop fires), the request completes via
             * the snap path below — intended fallback, see the WARN. */
            bool leadscrew_moving = time_since_step_s < STEP_ACTIVITY_WINDOW_S;

            if (!leadscrew_moving) {
                /* Leadscrew stationary: snap carriage to nearest grid point
                 * and engage. Where the nut lands within the lash window is
                 * operator-arbitrary (nothing loads it against a wall until
                 * the leadscrew turns), so draw a uniform drop-in position. */
                double dz = snapCarriageToGrid();
                if (std::abs(dz) > PHASE_TOL_FRAC * leadscrew_grid_spacing_mm) {
                    emu_log_event("WARN half-nut snap teleported carriage %.4f mm "
                                  "(> %.4f mm tol) — unphysical convenience engage",
                                  dz, PHASE_TOL_FRAC * leadscrew_grid_spacing_mm);
                }
                backlash_offset = std::uniform_real_distribution<double>(
                                      0.0, z_backlash_mm)(lash_rng);
                half_nut_state = ENGAGED;
                half_nut_request_pending = false;
                emu_log_event("half-nut ENGAGED (snap to %.3f mm, dz=%.4f, lash=%.3f)",
                              carriage_mm, dz, backlash_offset);
            } else {
                /* Leadscrew turning: wait for phase alignment */
                if (half_nut_state != ENGAGING) {
                    half_nut_state = ENGAGING;
                    emu_log_event("half-nut ENGAGING...");
                }
                if (checkPhaseAlignment()) {
                    /* Physical seating: the nut drops onto the thread, taking
                     * up the ≤PHASE_TOL_FRAC residual exactly, and rests on
                     * the wall OPPOSITE the drive direction — worst case, the
                     * full lash transient runs before the carriage moves. */
                    double dz = snapCarriageToGrid();
                    backlash_offset = (last_phys_dir > 0) ? 0.0 : z_backlash_mm;
                    half_nut_state = ENGAGED;
                    half_nut_request_pending = false;
                    emu_log_event("half-nut ENGAGED (phase match, seated dz=%.4f, "
                                  "lash wall %s)", dz,
                                  (last_phys_dir > 0) ? "-" : "+");
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
    time_since_step_s = 0.0;  /* engagement gate: leadscrew is actively stepping */
    last_phys_dir = phys_dir; /* sets the lash wall on a moving (phase-match) engage */

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
    /* A toggle while a moving engage is still waiting for phase alignment
     * cancels it (operator releases the lever before it drops in). Written
     * from the dashboard/serve thread while the ISR thread may transition
     * ENGAGING->ENGAGED — same benign-race pattern as the pending flag. */
    if (half_nut_state == ENGAGING) {
        half_nut_state = DISENGAGED;
        half_nut_request_pending = false;
        emu_log_event("half-nut engage CANCELLED");
        return;
    }
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

double LathePhysics::getCarriageGridPhase() const {
    /* Where is the carriage relative to the leadscrew thread grid?
     * Lattice-relative: 0 (mod 1) means the carriage sits exactly on a
     * thread-grid point for the current leadscrew position. */
    double grid = leadscrew_grid_spacing_mm;
    double offset = fmod(leadscrew_position_mm, grid);
    if (offset < 0.0) offset += grid;
    double revolutions = (carriage_mm - offset) / grid;
    double phase = fmod(revolutions, 1.0);
    if (phase < 0.0) phase += 1.0;
    return phase;
}

double LathePhysics::nearestGridPositionMM(double target_mm) const {
    double grid = leadscrew_grid_spacing_mm;
    double offset = fmod(leadscrew_position_mm, grid);
    if (offset < 0.0) offset += grid;
    return round((target_mm - offset) / grid) * grid + offset;
}

double LathePhysics::snapCarriageToGrid() {
    double before = carriage_mm;
    carriage_mm = nearestGridPositionMM(carriage_mm);
    carriage_mm = std::max(z_min_mm, std::min(z_max_mm, carriage_mm));
    double dz = carriage_mm - before;
    emu_log_event("SNAP before=%.4f after=%.4f grid=%.4f lsPos=%.4f dZ=%.4f",
                  before, carriage_mm, leadscrew_grid_spacing_mm,
                  leadscrew_position_mm, dz);
    return dz;
}

bool LathePhysics::checkPhaseAlignment() const {
    /* getCarriageGridPhase() is already lattice-relative, so aligned means
     * the phase is within tolerance of an INTEGER. (Comparing it against the
     * leadscrew phase again double-counted the offset: engagement fired at
     * carriage ≡ 2·offset — twice per rev, at off-lattice positions.) */
    double p = getCarriageGridPhase();
    double d = std::abs(p - std::round(p));
    return d < PHASE_TOL_FRAC;
}
