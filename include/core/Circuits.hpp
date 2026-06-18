#pragma once

#include "core/Waypoint.hpp"

namespace core {

// Stylized, scaled-down approximations of famous real-world circuits' recognizable
// shapes -- built from general knowledge of each track's corner sequence and
// silhouette, NOT from surveyed/GPS-accurate coordinates. `scale` multiplies the
// hand-placed anchor coordinates (meters) before smoothing into a drivable path.
Path makeSuzukaPath(double scale = 1.0);
Path makeMonacoPath(double scale = 1.0);
Path makeSilverstonePath(double scale = 1.0);
Path makeSpaPath(double scale = 1.0);
Path makeMonzaPath(double scale = 1.0);

} // namespace core
