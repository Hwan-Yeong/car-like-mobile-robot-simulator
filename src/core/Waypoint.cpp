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

} // namespace core
