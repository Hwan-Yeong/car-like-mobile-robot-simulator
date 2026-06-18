#pragma once

#include "controller/IController.hpp"
#include "core/VehicleParams.hpp"

namespace controller {

// Geometric path tracker with adaptive lookahead:
//   Ld    = clamp(k * vx, min, max)
//   alpha = angle from rear axle to the lookahead target, relative to heading
//   delta = atan2(2 * L * sin(alpha), Ld)
class PurePursuitController : public IController {
public:
    // params must outlive this controller -- wheelbase is read fresh on every
    // call so that live GUI tuning takes effect immediately.
    PurePursuitController(const core::VehicleParams* params, double k, double min_lookahead,
                           double max_lookahead);

    double computeSteering(const core::VehicleState& state, const core::Path& path) const override;

private:
    core::Waypoint findLookaheadTarget(const core::VehicleState& state, const core::Path& path,
                                        double lookahead) const;

    const core::VehicleParams* params_;
    double k_;
    double min_lookahead_;
    double max_lookahead_;
};

} // namespace controller
