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

} // namespace core
