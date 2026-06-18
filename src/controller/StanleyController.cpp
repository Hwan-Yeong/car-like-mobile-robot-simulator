#include "controller/StanleyController.hpp"

#include <cmath>
#include <limits>

namespace controller {

namespace {
double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
} // namespace

StanleyController::StanleyController(const core::VehicleParams* params, double k, double k_soft)
    : params_(params), k_(k), k_soft_(k_soft) {}

std::size_t StanleyController::findNearestIndex(double front_x, double front_y,
                                                  const core::Path& path) const {
    std::size_t nearest = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - front_x;
        const double dy = path[i].y - front_y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest = i;
        }
    }
    return nearest;
}

double StanleyController::computeSteering(const core::VehicleState& state, const core::Path& path) const {
    if (path.size() < 2) {
        return 0.0;
    }

    const double front_x = state.x + params_->wheelbase * std::cos(state.psi);
    const double front_y = state.y + params_->wheelbase * std::sin(state.psi);

    const std::size_t idx = findNearestIndex(front_x, front_y, path);
    const core::Waypoint& p0 = path[idx];
    const core::Waypoint& p1 = path[(idx + 1) % path.size()];
    const double path_heading = std::atan2(p1.y - p0.y, p1.x - p0.x);

    const double dx = front_x - p0.x;
    const double dy = front_y - p0.y;
    const double cross_track_error = dx * std::sin(state.psi) - dy * std::cos(state.psi);

    const double heading_error = normalizeAngle(path_heading - state.psi);
    return heading_error + std::atan2(k_ * cross_track_error, k_soft_ + state.vx);
}

} // namespace controller
