#include "core/DynamicBicycleModel.hpp"

#include <algorithm>
#include <cmath>

namespace core {

namespace {
constexpr double kMinVx = 0.5; // guards the slip-angle division at near-zero speed
}

DynamicBicycleModel::DynamicBicycleModel(const VehicleParams* params) : params_(params) {}

DynamicBicycleModel::Derivative DynamicBicycleModel::computeDerivative(const VehicleState& state,
                                                                         double delta) const {
    const double vx_safe = std::max(state.vx, kMinVx);
    const double alpha_f = delta - (state.vy + params_->a * state.r) / vx_safe;
    const double alpha_r = (params_->b * state.r - state.vy) / vx_safe;
    const double fyf = params_->cf * alpha_f;
    const double fyr = params_->cr * alpha_r;

    return {
        state.vx * std::cos(state.psi) - state.vy * std::sin(state.psi),
        state.vx * std::sin(state.psi) + state.vy * std::cos(state.psi),
        state.r,
        -state.vx * state.r + (fyf + fyr) / params_->m,
        (params_->a * fyf - params_->b * fyr) / params_->iz,
    };
}

VehicleState DynamicBicycleModel::step(const VehicleState& state, double delta, double dt) const {
    const double clamped_delta = std::clamp(delta, -params_->max_steer, params_->max_steer);

    auto advance = [&](const VehicleState& s, const Derivative& d, double h) {
        VehicleState next = s;
        next.x += h * d.xdot;
        next.y += h * d.ydot;
        next.psi += h * d.psidot;
        next.vy += h * d.vydot;
        next.r += h * d.rdot;
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
    next.vy += (dt / 6.0) * (k1.vydot + 2.0 * k2.vydot + 2.0 * k3.vydot + k4.vydot);
    next.r += (dt / 6.0) * (k1.rdot + 2.0 * k2.rdot + 2.0 * k3.rdot + k4.rdot);
    return next;
}

} // namespace core
