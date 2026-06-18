#include "gui/ParamPanel.hpp"

#include <imgui.h>

namespace gui {

void ParamPanel::draw(core::VehicleParams& params) {
    ImGui::SetNextWindowPos(ImVec2(10, 240), ImGuiCond_FirstUseEver);
    ImGui::Begin("Vehicle Parameters");

    ImGui::SeparatorText("Kinematic Model");

    static constexpr double kWheelbaseMin = 1.0, kWheelbaseMax = 5.0;
    ImGui::SliderScalar("Wheelbase L (m)", ImGuiDataType_Double, &params.wheelbase, &kWheelbaseMin,
                         &kWheelbaseMax);

    static constexpr double kMaxSteerMin = 0.1, kMaxSteerMax = 1.0;
    ImGui::SliderScalar("Max Steer (rad)", ImGuiDataType_Double, &params.max_steer, &kMaxSteerMin,
                         &kMaxSteerMax);

    ImGui::SeparatorText("Dynamic Model");

    static constexpr double kCorneringStiffnessMin = 20000.0, kCorneringStiffnessMax = 150000.0;
    ImGui::SliderScalar("Cf (N/rad)", ImGuiDataType_Double, &params.cf, &kCorneringStiffnessMin,
                         &kCorneringStiffnessMax);
    ImGui::SliderScalar("Cr (N/rad)", ImGuiDataType_Double, &params.cr, &kCorneringStiffnessMin,
                         &kCorneringStiffnessMax);

    static constexpr double kMassMin = 800.0, kMassMax = 3000.0;
    ImGui::SliderScalar("Mass m (kg)", ImGuiDataType_Double, &params.m, &kMassMin, &kMassMax);

    static constexpr double kIzMin = 500.0, kIzMax = 5000.0;
    ImGui::SliderScalar("Yaw Inertia Iz (kg*m^2)", ImGuiDataType_Double, &params.iz, &kIzMin, &kIzMax);

    static constexpr double kCgOffsetMin = 0.5, kCgOffsetMax = 2.0;
    ImGui::SliderScalar("CG to Front a (m)", ImGuiDataType_Double, &params.a, &kCgOffsetMin, &kCgOffsetMax);
    ImGui::SliderScalar("CG to Rear b (m)", ImGuiDataType_Double, &params.b, &kCgOffsetMin, &kCgOffsetMax);

    ImGui::End();
}

} // namespace gui
