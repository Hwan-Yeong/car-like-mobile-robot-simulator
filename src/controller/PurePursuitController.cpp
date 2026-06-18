#include "controller/PurePursuitController.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace controller {

PurePursuitController::PurePursuitController(double wheelbase, double k, double min_lookahead,
                                               double max_lookahead)
    : wheelbase_(wheelbase), k_(k), min_lookahead_(min_lookahead), max_lookahead_(max_lookahead) {}

core::Waypoint PurePursuitController::findLookaheadTarget(const core::VehicleState& state,
                                                            const core::Path& path,
                                                            double lookahead) const {
    std::size_t closest_idx = 0;
    double closest_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - state.x;
        const double dy = path[i].y - state.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < closest_dist_sq) {
            closest_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    for (std::size_t offset = 0; offset < path.size(); ++offset) {
        const std::size_t i = (closest_idx + offset) % path.size();
        const double dx = path[i].x - state.x;
        const double dy = path[i].y - state.y;
        if (std::sqrt(dx * dx + dy * dy) >= lookahead) {
            return path[i];
        }
    }
    return path[closest_idx];
}

double PurePursuitController::computeSteering(const core::VehicleState& state,
                                               const core::Path& path) const {
    if (path.empty()) {
        return 0.0;
    }

    const double lookahead = std::clamp(k_ * state.vx, min_lookahead_, max_lookahead_);
    const core::Waypoint target = findLookaheadTarget(state, path, lookahead);

    const double dx = target.x - state.x;
    const double dy = target.y - state.y;
    const double target_bearing = std::atan2(dy, dx);
    const double alpha = target_bearing - state.psi;

    const double ld = std::max(std::hypot(dx, dy), 1e-6);
    return std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld);
}

} // namespace controller
