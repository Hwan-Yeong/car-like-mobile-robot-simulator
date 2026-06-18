#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <SFML/Graphics.hpp>
#include <imgui-SFML.h>
#include <imgui.h>
#include <implot.h>

#include "core/Config.hpp"
#include "core/VehicleParams.hpp"
#include "core/VehicleState.hpp"
#include "core/Waypoint.hpp"
#include "engine/ModelFactory.hpp"
#include "engine/SimulationEngine.hpp"
#include "gui/ControlPanel.hpp"
#include "gui/ParamPanel.hpp"
#include "gui/TelemetryPanel.hpp"
#include "renderer/SFMLRenderer.hpp"

namespace {

core::Path makePath(int path_index) {
    switch (path_index) {
        case 0:
            return core::makeOvalPath(/*radius_x=*/30.0, /*radius_y=*/15.0, /*num_points=*/64);
        case 1:
        default:
            return core::makeFigureEightPath(/*scale=*/25.0, /*num_points=*/64);
    }
}

core::VehicleState makeResetState(const core::Path& path, double target_speed) {
    core::VehicleState state;
    state.x = path.front().x;
    state.y = path.front().y;
    state.psi = core::initialHeading(path);
    state.vx = target_speed;
    return state;
}

// Index of `name` in `names`, or 0 (with a warning) if not found -- used to turn
// the config file's human-readable names into ControlPanelState's combo indices.
int indexOrDefault(const std::vector<std::string>& names, const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    if (it == names.end()) {
        std::cerr << "Config: unknown name '" << name << "', defaulting to '" << names.front() << "'\n";
        return 0;
    }
    return static_cast<int>(std::distance(names.begin(), it));
}

} // namespace

int main() {
    const core::SimulationConfig file_config = core::loadConfig("config/config.yaml");

    core::VehicleParams params = file_config.vehicle_params;
    const std::vector<std::string> path_names = {"oval", "figure_eight"};
    const std::vector<std::string> model_names = engine::ModelFactory::vehicleModelNames();
    const std::vector<std::string> controller_names = engine::ModelFactory::controllerNames();

    gui::ControlPanelState control_state;
    control_state.target_speed = file_config.target_speed;
    control_state.path_index = indexOrDefault(path_names, file_config.initial_path);
    control_state.model_index = indexOrDefault(model_names, file_config.initial_model);
    control_state.controller_index = indexOrDefault(controller_names, file_config.initial_controller);

    core::Path path = makePath(control_state.path_index);
    core::VehicleState initial_state = makeResetState(path, control_state.target_speed);

    engine::SimulationEngine sim(
        engine::ModelFactory::createVehicleModel(model_names[control_state.model_index], &params),
        engine::ModelFactory::createController(controller_names[control_state.controller_index], &params),
        path, initial_state);

    sf::RenderWindow window(sf::VideoMode(1280, 720), "Car-Like Mobile Robot Simulator");
    window.setFramerateLimit(60);
    if (!ImGui::SFML::Init(window)) {
        return 1;
    }
    ImPlot::CreateContext();

    renderer::SFMLRenderer car_renderer(window);
    car_renderer.setPath(path);
    sim.addObserver(&car_renderer);
    car_renderer.onStateUpdated(initial_state);

    gui::ParamPanel param_panel;
    gui::ControlPanel control_panel;
    gui::TelemetryPanel telemetry_panel;
    int prev_path_index = control_state.path_index;
    int prev_model_index = control_state.model_index;
    int prev_controller_index = control_state.controller_index;

    constexpr double dt = 1.0 / 60.0;
    double sim_time = 0.0;
    sf::Clock delta_clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            ImGui::SFML::ProcessEvent(window, event);
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        ImGui::SFML::Update(window, delta_clock.restart());
        param_panel.draw(params);
        control_panel.draw(control_state, path_names, model_names, controller_names);
        telemetry_panel.draw();

        if (control_state.path_index != prev_path_index) {
            core::Path new_path = makePath(control_state.path_index);
            sim.setPath(new_path);
            car_renderer.setPath(new_path);
            control_state.reset_requested = true;
            prev_path_index = control_state.path_index;
        }
        if (control_state.model_index != prev_model_index) {
            sim.setModel(engine::ModelFactory::createVehicleModel(model_names[control_state.model_index], &params));
            prev_model_index = control_state.model_index;
        }
        if (control_state.controller_index != prev_controller_index) {
            sim.setController(
                engine::ModelFactory::createController(controller_names[control_state.controller_index], &params));
            prev_controller_index = control_state.controller_index;
        }

        sim.setSpeed(control_state.target_speed);

        if (control_state.reset_requested) {
            sim.reset(makeResetState(sim.path(), control_state.target_speed));
            control_state.reset_requested = false;
        }

        if (!control_state.paused) {
            sim.step(dt);
            sim_time += dt;

            const core::VehicleState& s = sim.state();
            const double beta = std::atan2(s.vy, s.vx);
            const double cross_track_error = core::crossTrackError({s.x, s.y}, sim.path());
            telemetry_panel.record(sim_time, beta, s.r, cross_track_error, sim.lastSteering(), s.vx);
        }

        window.clear(sf::Color(30, 30, 30));
        car_renderer.draw();
        ImGui::SFML::Render(window);
        window.display();
    }

    ImPlot::DestroyContext();
    ImGui::SFML::Shutdown();
    return 0;
}
