#pragma once

namespace core {

// Vehicle state, rear-axle/CG reference: [X, Y, psi, vx, vy, r]
// vy and r (lateral velocity, yaw rate) are only meaningful when a DynamicBicycleModel
// is active; KinematicBicycleModel leaves vy at 0 and writes r for telemetry only.
struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double r = 0.0;
};

} // namespace core
