#pragma once

#include "core/IVehicleModel.hpp"
#include "core/VehicleParams.hpp"

namespace core {

// Rear-axle reference kinematic bicycle model, state [X, Y, psi, vx]:
//   Xdot   = vx * cos(psi)
//   Ydot   = vx * sin(psi)
//   psidot = vx * tan(delta) / L
// Integrated with RK4. vx is held constant over the step (no throttle input yet).
class KinematicBicycleModel : public IVehicleModel {
public:
    explicit KinematicBicycleModel(VehicleParams params);

    VehicleState step(const VehicleState& state, double delta, double dt) const override;

private:
    struct Derivative {
        double xdot;
        double ydot;
        double psidot;
    };

    Derivative computeDerivative(const VehicleState& state, double delta) const;

    VehicleParams params_;
};

} // namespace core
