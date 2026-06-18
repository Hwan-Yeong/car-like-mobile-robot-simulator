#pragma once

#include "controller/IController.hpp"
#include "core/VehicleParams.hpp"

namespace controller {

// Front-axle reference tracker combining heading error and cross-track error:
//   front axle = (x, y) + wheelbase * (cos(psi), sin(psi))
//   e          = signed cross-track error at the front axle, projected onto the
//                vehicle's heading (positive when the path is to the vehicle's right)
//   psi_e      = heading error: path tangent heading minus vehicle heading
//   delta      = psi_e + atan2(k * e, k_soft + vx)
class StanleyController : public IController {
public:
    // params must outlive this controller -- wheelbase is read fresh on every
    // call so that live GUI tuning takes effect immediately.
    StanleyController(const core::VehicleParams* params, double k, double k_soft);

    double computeSteering(const core::VehicleState& state, const core::Path& path) const override;

private:
    std::size_t findNearestIndex(double front_x, double front_y, const core::Path& path) const;

    const core::VehicleParams* params_;
    double k_;
    double k_soft_;
};

} // namespace controller
