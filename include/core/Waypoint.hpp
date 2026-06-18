#pragma once

#include <vector>

namespace core {

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

using Path = std::vector<Waypoint>;

// Generates a closed oval path centered at the origin.
Path makeOvalPath(double radius_x, double radius_y, int num_points);

} // namespace core
