#pragma once

namespace core {

struct VehicleParams {
    double wheelbase = 2.7;       // L: distance between front and rear axles [m]
    double max_steer = 0.6;       // max |delta| [rad]
    double max_accel = 3.0;       // [m/s^2], reserved for future controllers
};

} // namespace core
