/*
 * Two-pane ANSI terminal dashboard with sparklines.
 */
#ifndef EMU_DASHBOARD_H
#define EMU_DASHBOARD_H

#include "config.h"
#include "physics.h"
#include "transport.h"
#include <vector>
#include <deque>
#include <atomic>

extern "C" {
#include "Ramps.h"
}

class Dashboard {
public:
    Dashboard(const EmuConfig &cfg, LathePhysics &physics, Transport &transport,
              rampsSharedData_t &shared);

    /* Run the dashboard loop (blocking, runs on main thread).
     * Returns when user presses Q. */
    void run();

    /* Signal the dashboard to stop. */
    void requestStop() { running.store(false); }

private:
    const EmuConfig &cfg;
    LathePhysics &physics;
    Transport &transport;
    rampsSharedData_t &shared;

    std::atomic<bool> running;
    bool manual_move;          /* true when manual move is active */
    double manual_move_timer;  /* seconds since last arrow input */
    bool manual_move_used;     /* true once user has made at least one move */

    /* Emulator-only per-pass spindle phase tracking. The firmware writes
     * emu_hw.els_last_stop_{spindle,z,seq} atomically at the trigger tick;
     * the dashboard edge-detects the sequence counter. Enable polling is
     * sampled — operator-driven, slow enough that 10 Hz suffices. */
    uint32_t prev_els_seq;
    uint16_t prev_els_enable;
    int32_t  last_stop_spindle;
    bool     last_stop_spindle_valid;
    int32_t  prev_stop_spindle;        /* spindle count from previous stop, for delta computation */
    bool     prev_stop_spindle_valid;
    int32_t  els_stop_pass_count;      /* stops since last enable rising edge */

    /* Geometry consistency check: catches misconfiguration where the
     * physics (`mm_per_step × encoder_counts_per_mm`) and the firmware's
     * model (`zCountsPerPitch / threadPitchSteps`) disagree on z-counts
     * per leadscrew step. Logs once on the first valid geometry and again
     * whenever the values change. See DEBUGGING.md post-mortem. */
    float    prev_thread_pitch_steps;
    float    prev_z_counts_per_pitch;
    bool     geom_first_check_done;

    /* Sparkline history ring buffers */
    struct SparklineBuffer {
        std::deque<double> samples;
        int max_samples;
        int width;  /* character width for rendering */

        SparklineBuffer(int seconds, int hz, int w)
            : max_samples(seconds * hz), width(w) {}

        void push(double val) {
            samples.push_back(val);
            while ((int)samples.size() > max_samples)
                samples.pop_front();
        }

        std::string render() const;
    };

    SparklineBuffer spark_rpm;
    SparklineBuffer spark_zpos;
    SparklineBuffer spark_zerr;

    /* Unit conversion */
    double toDisplay(double mm) const;
    const char* unitSuffix() const;
    int unitPrecision() const;

    /* Rendering */
    void draw();
    void drawStatePane(int startRow, int startCol, int width);
    void drawLogPane(int startRow, int startCol, int width, int height);
    void drawStatusBar(int row, int width);

    /* Input */
    void handleInput();
    void promptSpindleRPM();
    void promptZPosition();
    void promptXPosition();
    void promptLogMessage();
};

#endif /* EMU_DASHBOARD_H */
