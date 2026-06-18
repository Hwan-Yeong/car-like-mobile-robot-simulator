#include "gui/TelemetryPanel.hpp"

#include <imgui.h>
#include <implot.h>

namespace gui {

namespace {

void pushSample(std::vector<float>& buffer, float value, std::size_t max_samples) {
    if (buffer.size() >= max_samples) {
        buffer.erase(buffer.begin());
    }
    buffer.push_back(value);
}

void drawPlot(const char* title, const char* y_label, const std::vector<float>& time,
              const std::vector<float>& values) {
    if (ImPlot::BeginPlot(title, ImVec2(-1, 110))) {
        ImPlot::SetupAxes("t (s)", y_label, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
        if (!time.empty()) {
            ImPlot::PlotLine(title, time.data(), values.data(), static_cast<int>(time.size()));
        }
        ImPlot::EndPlot();
    }
}

} // namespace

void TelemetryPanel::record(double time, double beta, double yaw_rate, double cross_track_error,
                             double steering, double speed) {
    pushSample(time_, static_cast<float>(time), kMaxSamples);
    pushSample(beta_, static_cast<float>(beta), kMaxSamples);
    pushSample(yaw_rate_, static_cast<float>(yaw_rate), kMaxSamples);
    pushSample(cross_track_error_, static_cast<float>(cross_track_error), kMaxSamples);
    pushSample(steering_, static_cast<float>(steering), kMaxSamples);
    pushSample(speed_, static_cast<float>(speed), kMaxSamples);
}

void TelemetryPanel::draw() {
    ImGui::SetNextWindowPos(ImVec2(860, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(410, 700), ImGuiCond_FirstUseEver);
    ImGui::Begin("Telemetry");

    drawPlot("Side-Slip beta", "beta (rad)", time_, beta_);
    drawPlot("Yaw Rate r", "r (rad/s)", time_, yaw_rate_);
    drawPlot("Cross-Track Error", "e (m)", time_, cross_track_error_);
    drawPlot("Steering delta", "delta (rad)", time_, steering_);
    drawPlot("Speed", "v (m/s)", time_, speed_);

    ImGui::End();
}

} // namespace gui
