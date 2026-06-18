#pragma once

namespace core {

// Kinematic bicycle state, rear-axle reference: [X, Y, psi, vx]
struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = 0.0;
};

} // namespace core
