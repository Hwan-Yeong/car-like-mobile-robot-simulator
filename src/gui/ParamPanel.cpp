#include "gui/ParamPanel.hpp"

#include <imgui.h>

namespace gui {

void ParamPanel::draw(core::VehicleParams& params) {
    ImGui::SetNextWindowPos(ImVec2(10, 240), ImGuiCond_FirstUseEver);
    ImGui::Begin("Vehicle Parameters");

    static constexpr double kWheelbaseMin = 1.0, kWheelbaseMax = 5.0;
    ImGui::SliderScalar("Wheelbase L (m)", ImGuiDataType_Double, &params.wheelbase, &kWheelbaseMin,
                         &kWheelbaseMax);

    static constexpr double kMaxSteerMin = 0.1, kMaxSteerMax = 1.0;
    ImGui::SliderScalar("Max Steer (rad)", ImGuiDataType_Double, &params.max_steer, &kMaxSteerMin,
                         &kMaxSteerMax);

    ImGui::End();
}

} // namespace gui
