#pragma once

#include "controller/IController.hpp"
#include "core/VehicleParams.hpp"

namespace controller {

// Linear time-varying MPC over a small lateral error model, re-linearized every
// control step around the current speed and the path's local curvature:
//   state  x = [e_y, e_psi]   (lateral error, heading error vs. the path tangent)
//   e_y_dot   = vx * e_psi                      (small-angle Frenet kinematics)
//   e_psi_dot = (vx / L) * delta - vx * kappa    (kappa = local path curvature)
// Discretized (dt_mpc) and stacked over a horizon into a condensed prediction
// (Sx, Su, Sd matrices), the cost  sum_k x_k^T Q x_k + r * delta_k^2  has no
// inequality constraints, so the QP reduces to a single dense linear solve
// (Eigen LDLT) rather than needing a dedicated QP solver. Only the first
// control of the solved sequence is applied (receding horizon), clamped to
// +-max_steer.
class MPCController : public IController {
public:
    // params must outlive this controller (live-tunable wheelbase/max_steer).
    MPCController(const core::VehicleParams* params, int horizon, double dt_mpc, double q_lateral,
                  double q_heading, double r_steering);

    double computeSteering(const core::VehicleState& state, const core::Path& path) const override;

private:
    std::size_t findNearestIndex(double x, double y, const core::Path& path) const;
    double estimateCurvature(std::size_t idx, const core::Path& path) const;

    const core::VehicleParams* params_;
    int horizon_;
    double dt_mpc_;
    double q_lateral_;
    double q_heading_;
    double r_steering_;
};

} // namespace controller
