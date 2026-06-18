#pragma once

#include "core/VehicleState.hpp"
#include "core/Waypoint.hpp"

namespace controller {

// Strategy interface for swappable path-tracking controllers.
class IController {
public:
    virtual ~IController() = default;

    // Computes the steering command [rad] to track `path` from `state`.
    virtual double computeSteering(const core::VehicleState& state, const core::Path& path) const = 0;
};

} // namespace controller
