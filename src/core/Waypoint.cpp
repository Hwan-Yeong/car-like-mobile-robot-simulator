#include "core/Waypoint.hpp"

#include <cmath>

namespace core {

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

} // namespace core
