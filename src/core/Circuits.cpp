#include "core/Circuits.hpp"

#include <cmath>
#include <vector>

namespace core {

namespace {
constexpr int kPointsPerSegment = 12;

Path scaled(std::vector<Waypoint> anchors, double scale) {
    for (Waypoint& w : anchors) {
        w.x *= scale;
        w.y *= scale;
    }
    return smoothClosedPath(anchors, kPointsPerSegment);
}
} // namespace

// Suzuka's defining feature is its figure-8 crossover (the back section crosses
// over the front via a bridge), with the two loops noticeably different sizes.
// Modeled directly as a lemniscate with cosine-modulated amplitude rather than
// hand-placed anchors, since that's the natural closed form for a smooth,
// asymmetric figure-8.
Path makeSuzukaPath(double scale) {
    constexpr int num_points = 140;
    Path path;
    path.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(num_points);
        const double a = scale * (35.0 + 15.0 * std::cos(theta));
        const double b = scale * (22.0 + 8.0 * std::cos(theta));
        path.push_back({a * std::cos(theta), b * std::sin(theta) * std::cos(theta)});
    }
    return path;
}

// Monaco: tight, narrow harbor-front street circuit. The pinch at the
// Sainte-Devote/hairpin anchor is pulled inward to evoke the famously slow
// Grand Hotel Hairpin without producing a literal self-intersection.
Path makeMonacoPath(double scale) {
    const std::vector<Waypoint> anchors = {
        {0, 50},     {45, 45},   {60, 10},   {45, -30},  {15, -50},
        {-5, -20},  // pinch: Grand Hotel Hairpin
        {-45, -35}, {-60, 0},   {-45, 40},  {-15, 50},
    };
    return scaled(anchors, scale);
}

// Silverstone: fast, flowing former-airfield circuit -- a gently rounded loop
// with no sharp pinches, just varying straight lengths.
Path makeSilverstonePath(double scale) {
    const std::vector<Waypoint> anchors = {
        {0, 55},    {40, 50},  {65, 15},  {55, -25}, {20, -55},
        {-25, -50}, {-55, -20}, {-60, 20}, {-35, 50}, {-10, 58},
    };
    return scaled(anchors, scale);
}

// Spa-Francorchamps: long and narrow, with the La Source hairpin pinch at one
// end (start/finish) and the Eau Rouge/Raidillon kink partway down one side.
Path makeSpaPath(double scale) {
    const std::vector<Waypoint> anchors = {
        {0, 70},
        {-5, 45},  // pinch: La Source hairpin
        {15, 20},  // Eau Rouge / Raidillon kink
        {10, -10}, {5, -50}, {-20, -80}, {-45, -65},
        {-55, -30}, {-50, 10}, {-35, 45}, {-15, 65},
    };
    return scaled(anchors, scale);
}

// Monza: the "Temple of Speed" -- long straights joined by a few wide corners,
// a gently rounded triangle with no tight pinches.
Path makeMonzaPath(double scale) {
    const std::vector<Waypoint> anchors = {
        {0, 60},    {50, 55},   {70, 10},  {55, -35}, {35, -55},
        {0, -60},   {-35, -55}, {-65, -15}, {-55, 35}, {-20, 58},
    };
    return scaled(anchors, scale);
}

} // namespace core
