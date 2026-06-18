#include "controller/MPCController.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace controller {

namespace {
constexpr double kMinVx = 0.5; // guards the e_psi_dot/L division at near-zero speed
double normalizeAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
} // namespace

MPCController::MPCController(const core::VehicleParams* params, int horizon, double dt_mpc,
                              double q_lateral, double q_heading, double r_steering)
    : params_(params),
      horizon_(horizon),
      dt_mpc_(dt_mpc),
      q_lateral_(q_lateral),
      q_heading_(q_heading),
      r_steering_(r_steering) {}

std::size_t MPCController::findNearestIndex(double x, double y, const core::Path& path) const {
    std::size_t nearest = 0;
    double min_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const double dx = path[i].x - x;
        const double dy = path[i].y - y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest = i;
        }
    }
    return nearest;
}

double MPCController::estimateCurvature(std::size_t idx, const core::Path& path) const {
    const std::size_t n = path.size();
    const core::Waypoint& prev = path[(idx + n - 1) % n];
    const core::Waypoint& curr = path[idx];
    const core::Waypoint& next = path[(idx + 1) % n];

    const double heading_in = std::atan2(curr.y - prev.y, curr.x - prev.x);
    const double heading_out = std::atan2(next.y - curr.y, next.x - curr.x);
    const double segment_length = std::hypot(next.x - curr.x, next.y - curr.y);
    if (segment_length < 1e-6) {
        return 0.0;
    }
    return normalizeAngle(heading_out - heading_in) / segment_length;
}

double MPCController::computeSteering(const core::VehicleState& state, const core::Path& path) const {
    if (path.size() < 3) {
        return 0.0;
    }

    const std::size_t idx = findNearestIndex(state.x, state.y, path);
    const core::Waypoint& p0 = path[idx];
    const core::Waypoint& p1 = path[(idx + 1) % path.size()];
    const double path_heading = std::atan2(p1.y - p0.y, p1.x - p0.x);

    const double dx = state.x - p0.x;
    const double dy = state.y - p0.y;
    const double e_y = -dx * std::sin(path_heading) + dy * std::cos(path_heading);
    const double e_psi = normalizeAngle(state.psi - path_heading);
    const double kappa = estimateCurvature(idx, path);

    const double vx = std::max(state.vx, kMinVx);
    const double L = params_->wheelbase;

    Eigen::Matrix2d A;
    A << 1.0, dt_mpc_ * vx, 0.0, 1.0;
    Eigen::Vector2d B(0.0, dt_mpc_ * vx / L);
    Eigen::Vector2d Bd(0.0, -dt_mpc_ * vx);

    constexpr int nx = 2;
    const int n = horizon_;

    Eigen::MatrixXd Sx(n * nx, nx);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(n * nx, n);
    Eigen::MatrixXd Sd(n * nx, 1);

    Sx.block(0, 0, nx, nx) = A;
    Su.block(0, 0, nx, 1) = B;
    Sd.block(0, 0, nx, 1) = Bd;

    for (int k = 1; k < n; ++k) {
        Sx.block(k * nx, 0, nx, nx) = A * Sx.block((k - 1) * nx, 0, nx, nx);
        Su.block(k * nx, 0, nx, n) = A * Su.block((k - 1) * nx, 0, nx, n);
        Su.block(k * nx, k, nx, 1) = B;
        Sd.block(k * nx, 0, nx, 1) = A * Sd.block((k - 1) * nx, 0, nx, 1) + Bd;
    }

    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(n * nx, n * nx);
    for (int k = 0; k < n; ++k) {
        Qbar(k * nx, k * nx) = q_lateral_;
        Qbar(k * nx + 1, k * nx + 1) = q_heading_;
    }
    const Eigen::MatrixXd Rbar = Eigen::MatrixXd::Identity(n, n) * r_steering_;

    const Eigen::Vector2d x0(e_y, e_psi);
    const Eigen::MatrixXd offset = Sx * x0 + Sd * kappa;

    const Eigen::MatrixXd H = Su.transpose() * Qbar * Su + Rbar;
    const Eigen::VectorXd f = Su.transpose() * Qbar * offset;

    const Eigen::VectorXd u = -H.ldlt().solve(f);

    return std::clamp(u(0), -params_->max_steer, params_->max_steer);
}

} // namespace controller
