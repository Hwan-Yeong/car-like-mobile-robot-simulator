#pragma once

#include <cstddef>
#include <vector>

namespace gui {

// Rolling time-series buffers for live ImPlot telemetry. record() is meant to be
// called once per executed simulation step (not once per render frame), so the
// time axis reflects simulated time rather than wall-clock frame rate.
class TelemetryPanel {
public:
    void record(double time, double beta, double yaw_rate, double cross_track_error, double steering,
                double speed);
    void draw();

private:
    static constexpr std::size_t kMaxSamples = 600; // ~10s of history at 60 steps/s

    std::vector<float> time_;
    std::vector<float> beta_;
    std::vector<float> yaw_rate_;
    std::vector<float> cross_track_error_;
    std::vector<float> steering_;
    std::vector<float> speed_;
};

} // namespace gui
