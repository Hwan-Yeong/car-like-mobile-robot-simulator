#pragma once

#include "core/IVehicleModel.hpp"
#include "core/VehicleParams.hpp"

namespace core {

// 2-DOF lateral dynamics (linear tire model), state [X, Y, psi, vy, r]:
//   alpha_f = delta - (vy + a*r) / vx
//   alpha_r = (b*r - vy) / vx
//   Fyf = Cf * alpha_f, Fyr = Cr * alpha_r
//   vy_dot   = -vx*r + (Fyf + Fyr) / m
//   r_dot    = (a*Fyf - b*Fyr) / Iz
//   Xdot     = vx*cos(psi) - vy*sin(psi)
//   Ydot     = vx*sin(psi) + vy*cos(psi)
//   psidot   = r
// Integrated with RK4. vx is held constant over the step (no longitudinal dynamics yet).
class DynamicBicycleModel : public IVehicleModel {
public:
    // params must outlive this model -- it's read fresh on every step so that
    // live GUI tuning of Cf/Cr/m/Iz/a/b/max_steer takes effect immediately.
    explicit DynamicBicycleModel(const VehicleParams* params);

    VehicleState step(const VehicleState& state, double delta, double dt) const override;

private:
    struct Derivative {
        double xdot;
        double ydot;
        double psidot;
        double vydot;
        double rdot;
    };

    Derivative computeDerivative(const VehicleState& state, double delta) const;

    const VehicleParams* params_;
};

} // namespace core
