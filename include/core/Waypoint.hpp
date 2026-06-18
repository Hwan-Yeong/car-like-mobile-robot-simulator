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

// Generates a closed figure-eight (lemniscate of Gerono) path centered at the origin.
Path makeFigureEightPath(double scale, int num_points);

// Heading [rad] tangent to the path at its first point, from the first two waypoints.
double initialHeading(const Path& path);

// Minimum perpendicular distance from `point` to the closed path polyline.
double crossTrackError(const Waypoint& point, const Path& path);

// Smooths a closed loop of sparse anchor points into a dense path via uniform
// Catmull-Rom interpolation (points_per_segment samples between each pair of
// anchors). Used to turn a small hand-placed control-point outline into a
// drivable curved track.
Path smoothClosedPath(const std::vector<Waypoint>& control_points, int points_per_segment);

} // namespace core
