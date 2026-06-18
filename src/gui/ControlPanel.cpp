#include "gui/ControlPanel.hpp"

#include <imgui.h>

namespace gui {

namespace {

void drawCombo(const char* label, int& selected_index, const std::vector<std::string>& names) {
    if (names.empty()) {
        return;
    }
    if (ImGui::BeginCombo(label, names[selected_index].c_str())) {
        for (int i = 0; i < static_cast<int>(names.size()); ++i) {
            const bool is_selected = (i == selected_index);
            if (ImGui::Selectable(names[i].c_str(), is_selected)) {
                selected_index = i;
            }
        }
        ImGui::EndCombo();
    }
}

} // namespace

void ControlPanel::draw(ControlPanelState& state, const std::vector<std::string>& path_names,
                         const std::vector<std::string>& model_names,
                         const std::vector<std::string>& controller_names) {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::Begin("Control");

    ImGui::Checkbox("Paused", &state.paused);
    state.reset_requested = ImGui::Button("Reset");

    static constexpr double kMinSpeed = 0.0, kMaxSpeed = 20.0;
    ImGui::SliderScalar("Target Speed (m/s)", ImGuiDataType_Double, &state.target_speed, &kMinSpeed,
                         &kMaxSpeed);

    drawCombo("Path", state.path_index, path_names);
    drawCombo("Vehicle Model", state.model_index, model_names);
    drawCombo("Controller", state.controller_index, controller_names);

    ImGui::End();
}

} // namespace gui
