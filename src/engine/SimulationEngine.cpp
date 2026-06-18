#include "engine/SimulationEngine.hpp"

namespace engine {

SimulationEngine::SimulationEngine(std::unique_ptr<core::IVehicleModel> model,
                                    std::unique_ptr<controller::IController> controller,
                                    core::Path path, core::VehicleState initial_state)
    : model_(std::move(model)),
      controller_(std::move(controller)),
      path_(std::move(path)),
      state_(initial_state) {}

void SimulationEngine::addObserver(ISimulationObserver* observer) { observers_.push_back(observer); }

void SimulationEngine::step(double dt) {
    const double delta = controller_->computeSteering(state_, path_);
    state_ = model_->step(state_, delta, dt);
    notifyObservers();
}

void SimulationEngine::notifyObservers() {
    for (ISimulationObserver* observer : observers_) {
        observer->onStateUpdated(state_);
    }
}

} // namespace engine
