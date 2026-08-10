/*
 * Lathe physics model.
 *
 * Simulates spindle with inertia, leadscrew, carriage with half-nut, and cross-slide.
 */
#ifndef EMU_PHYSICS_H
#define EMU_PHYSICS_H

#include "config.h"
#include <cstdint>
#include <atomic>
#include <cmath>
#include <random>

class LathePhysics {
public:
    explicit LathePhysics(const EmuConfig &cfg);

    /* Called each ISR tick BEFORE calling into firmware.
     * Updates spindle and feeds encoder counter values into emu_hw. */
    /* shared must point to the firmware's rampsSharedData_t for reading servo state */
    void tick(double dt_seconds, const void *shared_data);

    /* Called from GPIO shim when a STEP rising edge is detected. */
    void onStepPulse(int direction);

    /* --- Spindle control (from dashboard keyboard) --- */
    void setTargetRPM(double rpm);
    void toggleDirection();
    void emergencyStop();

    double getSpindleRPM() const { return spindle_omega * 60.0 / (2.0 * M_PI); }
    double getTargetRPM() const { return spindle_target_rpm; }
    bool   getSpindleCW() const { return spindle_target_rpm >= 0; }
    int64_t getSpindleEncoderCounts() const;

    /* --- Half-nut --- */
    enum HalfNutState { DISENGAGED, ENGAGING, ENGAGED };

    void requestHalfNutToggle();

    /* Explicit-state request, for callers that know the state they want rather
     * than the transition they want (the stdin command channel; system tests).
     *
     * IDEMPOTENT, and that is the entire reason it exists rather than callers
     * poking requestHalfNutToggle() twice: a toggle issued while an engage is
     * still waiting for phase alignment CANCELS it (see requestHalfNutToggle),
     * so "close it" expressed as a toggle is state-dependent and would
     * sometimes mean "give up". A test that has to know the current state in
     * order to ask for the next one is a test that can silently ask for the
     * opposite. */
    void setHalfNutEngaged(bool engaged);

    HalfNutState getHalfNutState() const { return half_nut_state; }

    /* --- Carriage (Z-axis) manual move --- */
    double getCarriageMM() const { return carriage_mm; }
    void   jogCarriage(int direction);       /* Arrow key jog: direction +1/-1 */
    void   moveCarriageTo(double target_mm); /* Move to specific position */
    bool   isZMoveTargetActive() const { return z_move_active; }
    int64_t getCarriageEncoderCounts() const;

    /* --- Cross-slide (X-axis) manual move --- */
    double getCrossSlideMM() const { return cross_slide_mm; }
    void   jogCrossSlide(int direction);
    void   moveCrossSlideTo(double target_mm);
    bool   isXMoveTargetActive() const { return x_move_active; }
    int64_t getCrossSlideEncoderCounts() const;

    /* --- Jog status --- */
    bool isZJogging() const { return std::abs(z_jog_velocity) > 0.01; }
    bool isXJogging() const { return std::abs(x_jog_velocity) > 0.01; }

    /* --- Leadscrew --- */
    double getLeadscrewPositionMM() const { return leadscrew_position_mm; }
    double getLeadscrewGridSpacingMM() const { return leadscrew_grid_spacing_mm; }

    /* Nearest carriage position that sits exactly on the leadscrew thread
     * lattice for the CURRENT leadscrew position. Not clamped to travel
     * limits: a target within half a grid of z_min/z_max can end up clamped
     * off-lattice by moveCarriageTo — acceptable edge case. */
    double nearestGridPositionMM(double target_mm) const;

    /* Lash window position: 0 = on -wall, z_backlash_mm = on +wall. Used by the
     * ELS scenario to verify the takeup loads the correct (cutting-side) wall. */
    double getBacklashOffsetMM() const { return backlash_offset; }

private:
    /* Config */
    double spindle_inertia;
    double spindle_max_torque;
    double spindle_friction;
    int    spindle_counts_per_rev;
    double leadscrew_mm_per_step;
    double leadscrew_tpi;
    double leadscrew_grid_spacing_mm;  /* 25.4 / tpi */
    double z_counts_per_mm;
    double z_backlash_mm;
    double z_max_mm, z_min_mm;
    double x_counts_per_mm;
    double x_max_mm, x_min_mm;
    double x_manual_step_mm;

    /* PHYSICAL wiring signs (+1/-1), independent of the firmware canonicalization
     * registers (scaleDir/servoDir) that reflex-ui writes. These model how the
     * real machine happens to be wired -- encoder cable orientation / motor
     * wiring -- which the operator's UI reversing toggle must cancel. Applied in
     * the encoder-count getters and onStepPulse; NOT the same as the Modbus
     * scaleDir/servoDir registers (which the host overwrites on connect). */
    int    spindle_scale_sign;
    int    z_scale_sign;
    int    x_scale_sign;
    int    servo_sign;

    /* Spindle state */
    double spindle_theta;         /* cumulative angle in radians */
    double spindle_omega;         /* angular velocity in rad/s */
    double spindle_target_rpm;

    /* Leadscrew state (always tracks stepper steps) */
    double leadscrew_position_mm; /* cumulative from step pulses */
    int64_t leadscrew_total_steps;

    /* Carriage state */
    double carriage_mm;
    HalfNutState half_nut_state;
    bool   half_nut_request_pending;
    double backlash_offset;    /* nut position within play window: [0, z_backlash_mm] */

    /* Half-nut engagement gate: leadscrew activity is tracked from our own
     * step pulses (onStepPulse), NOT from firmware ramp internals —
     * servo.currentSpeed is the indexing/jog ramp variable and is pinned to 0
     * during pure ELS sync. */
    double time_since_step_s;  /* seconds since the last STEP pulse; large at boot */
    int    last_phys_dir;      /* physical direction of the last step pulse (+1/-1, 0 = none yet) */

    /* Drop-in position generator for stationary engagement: where within the
     * lash window the nut lands is operator-arbitrary. Fixed default seed for
     * reproducible runs; override with EMU_SEED. */
    std::minstd_rand lash_rng;

    /* Cross-slide state */
    double cross_slide_mm;

    /* Exposed (to the firmware's emulated timer-counter registers) Z/X
     * encoder counts. Ramp toward getCarriageEncoderCounts()/
     * getCrossSlideEncoderCounts() by at most MAX_EXPOSED_STEP_COUNTS per
     * tick instead of jumping straight there. Needed because
     * z_initial_mm/x_initial_mm pre-seed carriage_mm/cross_slide_mm to a
     * nonzero value before the very first tick, so the raw encoder count is
     * already large on tick 1 -- while the firmware's own DRO tracking
     * (Ramps.c's scalesDeltaPos oldPosition/position) starts at zero. A
     * single-tick exposure of that gap overflows Ramps.c:387-390's
     * `(int16_t)(position - oldPosition)` cast once the offset exceeds
     * 32767 counts (81.9175mm at 400 counts/mm) -- see els_boot_delta_test.
     * Real hardware never pre-seeds a timer counter (no
     * TIM_SetCounter/__HAL_TIM_SET_COUNTER in Core/Src), so this ramp is
     * emulator-only and invisible to any codepath hardware exercises: once
     * caught up (a handful of 100us ticks after boot), z_exposed_counts ==
     * the true count every tick, identical to the un-ramped code, and the
     * clamp (4000 counts/tick = 10mm/tick at 400 counts/mm, vs. a realistic
     * per-tick jog delta of well under 1 count at 10 mm/s / 10kHz) never
     * engages during normal motion. */
    int64_t z_exposed_counts;
    int64_t x_exposed_counts;
    static constexpr int64_t MAX_EXPOSED_STEP_COUNTS = 4000;

    /* Manual jog state (velocity-based with acceleration) */
    double z_jog_velocity;       /* current mm/s */
    double z_jog_target_dir;     /* -1, 0, or +1 (arrow key jog) */
    double x_jog_velocity;       /* current mm/s */
    double x_jog_target_dir;     /* -1, 0, or +1 (arrow key jog) */

    /* Move-to-position state */
    bool   z_move_active;
    double z_move_target;        /* target position in mm */
    bool   x_move_active;
    double x_move_target;
    double jog_max_velocity;     /* mm/s from config */
    double jog_acceleration;     /* mm/s^2 from config */
    double jog_timeout;          /* seconds since last key event */
    double z_jog_idle_timer;     /* time since last Z key */
    double x_jog_idle_timer;     /* time since last X key */
    static constexpr double JOG_KEY_TIMEOUT = 0.15; /* seconds of no key → stop */

    /* Half-nut engagement helpers */
    double getCarriageGridPhase() const;
    double snapCarriageToGrid();   /* returns signed carriage displacement (mm) */
    bool   checkPhaseAlignment() const;

    /* Engagement tolerance as a fraction of the thread grid. Distinct from
     * the (numerically similar) default z_backlash_mm and the serve-mode
     * arrival tolerance in main.cpp — three unrelated 0.02s. */
    static constexpr double PHASE_TOL_FRAC = 0.02;
    /* "Leadscrew moving" = a step pulse within this window. Pulse rate below
     * 10 Hz (≈1 RPM sync on the reference geometry) reads as stationary. */
    static constexpr double STEP_ACTIVITY_WINDOW_S = 0.1;
};

#endif /* EMU_PHYSICS_H */
