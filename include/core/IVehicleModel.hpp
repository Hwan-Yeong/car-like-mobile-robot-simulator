#pragma once

#include "core/VehicleState.hpp"

namespace core {

// Strategy interface for swappable vehicle dynamics models.
class IVehicleModel {
public:
    virtual ~IVehicleModel() = default;

    // Advances the state by dt seconds given a steering input delta [rad].
    virtual VehicleState step(const VehicleState& state, double delta, double dt) const = 0;
};

} // namespace core
