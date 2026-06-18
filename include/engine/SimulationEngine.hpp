#pragma once

#include <memory>
#include <vector>

#include "controller/IController.hpp"
#include "core/IVehicleModel.hpp"
#include "core/VehicleState.hpp"
#include "core/Waypoint.hpp"
#include "engine/ISimulationObserver.hpp"

namespace engine {

// Owns the model/controller/path and drives the fixed-timestep simulation loop,
// notifying observers (renderer, GUI panels) after each step.
class SimulationEngine {
public:
    SimulationEngine(std::unique_ptr<core::IVehicleModel> model,
                      std::unique_ptr<controller::IController> controller, core::Path path,
                      core::VehicleState initial_state);

    void addObserver(ISimulationObserver* observer);

    // Advances the simulation by dt seconds and notifies observers.
    void step(double dt);

    void setModel(std::unique_ptr<core::IVehicleModel> model) { model_ = std::move(model); }
    void setController(std::unique_ptr<controller::IController> controller) {
        controller_ = std::move(controller);
    }
    void setPath(core::Path path) { path_ = std::move(path); }
    void setSpeed(double vx) { state_.vx = vx; }

    // Overwrites the state directly (e.g. repositioning onto a new path) and notifies observers.
    void reset(const core::VehicleState& state);

    const core::VehicleState& state() const { return state_; }
    const core::Path& path() const { return path_; }

private:
    void notifyObservers();

    std::unique_ptr<core::IVehicleModel> model_;
    std::unique_ptr<controller::IController> controller_;
    core::Path path_;
    core::VehicleState state_;
    std::vector<ISimulationObserver*> observers_;
};

} // namespace engine
