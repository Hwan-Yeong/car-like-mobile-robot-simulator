#include "core/KinematicBicycleModel.hpp"

#include <algorithm>
#include <cmath>

namespace core {

KinematicBicycleModel::KinematicBicycleModel(const VehicleParams* params) : params_(params) {}

KinematicBicycleModel::Derivative KinematicBicycleModel::computeDerivative(
    const VehicleState& state, double delta) const {
    return {
        state.vx * std::cos(state.psi),
        state.vx * std::sin(state.psi),
        state.vx * std::tan(delta) / params_->wheelbase,
    };
}

VehicleState KinematicBicycleModel::step(const VehicleState& state, double delta, double dt) const {
    const double clamped_delta = std::clamp(delta, -params_->max_steer, params_->max_steer);

    auto advance = [&](const VehicleState& s, const Derivative& d, double h) {
        VehicleState next = s;
        next.x += h * d.xdot;
        next.y += h * d.ydot;
        next.psi += h * d.psidot;
        return next;
    };

    const Derivative k1 = computeDerivative(state, clamped_delta);
    const Derivative k2 = computeDerivative(advance(state, k1, dt / 2.0), clamped_delta);
    const Derivative k3 = computeDerivative(advance(state, k2, dt / 2.0), clamped_delta);
    const Derivative k4 = computeDerivative(advance(state, k3, dt), clamped_delta);

    VehicleState next = state;
    next.x += (dt / 6.0) * (k1.xdot + 2.0 * k2.xdot + 2.0 * k3.xdot + k4.xdot);
    next.y += (dt / 6.0) * (k1.ydot + 2.0 * k2.ydot + 2.0 * k3.ydot + k4.ydot);
    next.psi += (dt / 6.0) * (k1.psidot + 2.0 * k2.psidot + 2.0 * k3.psidot + k4.psidot);
    next.vy = 0.0;             // no side-slip in a pure kinematic model
    next.r = (next.psi - state.psi) / dt;  // realized yaw rate, for telemetry only
    return next;
}

} // namespace core
