#pragma once

#include "core/VehicleState.hpp"

namespace engine {

// Observer interface notified after each simulation step (e.g. renderer, GUI panels).
class ISimulationObserver {
public:
    virtual ~ISimulationObserver() = default;

    virtual void onStateUpdated(const core::VehicleState& state) = 0;
};

} // namespace engine
