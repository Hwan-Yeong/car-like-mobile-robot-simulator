#include "core/Waypoint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace core {

namespace {

double distanceToSegment(const Waypoint& point, const Waypoint& a, const Waypoint& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 == 0.0) {
        return std::hypot(point.x - a.x, point.y - a.y);
    }
    const double t = std::clamp(((point.x - a.x) * dx + (point.y - a.y) * dy) / len2, 0.0, 1.0);
    return std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

// Uniform Catmull-Rom basis, evaluated at t in [0,1] between p1 and p2.
Waypoint catmullRomPoint(const Waypoint& p0, const Waypoint& p1, const Waypoint& p2, const Waypoint& p3,
                         double t) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    auto blend = [&](double a0, double a1, double a2, double a3) {
        return 0.5 * ((2.0 * a1) + (-a0 + a2) * t + (2.0 * a0 - 5.0 * a1 + 4.0 * a2 - a3) * t2 +
                       (-a0 + 3.0 * a1 - 3.0 * a2 + a3) * t3);
    };
    return {blend(p0.x, p1.x, p2.x, p3.x), blend(p0.y, p1.y, p2.y, p3.y)};
}

} // namespace

Path makeOvalPath(double radius_x, double radius_y, int num_points) {
    Path path;
    path.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
        path.push_back({radius_x * std::cos(theta), radius_y * std::sin(theta)});
    }
    return path;
}

Path makeFigureEightPath(double scale, int num_points) {
    Path path;
    path.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
        path.push_back({scale * std::cos(theta), scale * std::sin(theta) * std::cos(theta)});
    }
    return path;
}

double initialHeading(const Path& path) {
    if (path.size() < 2) {
        return 0.0;
    }
    return std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);
}

double crossTrackError(const Waypoint& point, const Path& path) {
    double min_dist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < path.size(); ++i) {
        const Waypoint& a = path[i];
        const Waypoint& b = path[(i + 1) % path.size()];
        min_dist = std::min(min_dist, distanceToSegment(point, a, b));
    }
    return min_dist;
}

Path smoothClosedPath(const std::vector<Waypoint>& control_points, int points_per_segment) {
    Path path;
    const std::size_t n = control_points.size();
    path.reserve(n * static_cast<std::size_t>(points_per_segment));
    for (std::size_t i = 0; i < n; ++i) {
        const Waypoint& p0 = control_points[(i + n - 1) % n];
        const Waypoint& p1 = control_points[i];
        const Waypoint& p2 = control_points[(i + 1) % n];
        const Waypoint& p3 = control_points[(i + 2) % n];
        for (int j = 0; j < points_per_segment; ++j) {
            const double t = static_cast<double>(j) / static_cast<double>(points_per_segment);
            path.push_back(catmullRomPoint(p0, p1, p2, p3, t));
        }
    }
    return path;
}

} // namespace core
