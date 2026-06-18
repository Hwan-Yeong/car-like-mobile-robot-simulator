#pragma once

namespace core {

struct VehicleParams {
    double wheelbase = 2.7;       // L: distance between front and rear axles [m]
    double max_steer = 0.6;       // max |delta| [rad]
    double max_accel = 3.0;       // [m/s^2], reserved for future controllers

    // DynamicBicycleModel (2-DOF lateral dynamics) params. a + b should equal wheelbase.
    double cf = 80000.0;  // front cornering stiffness [N/rad]
    double cr = 80000.0;  // rear cornering stiffness [N/rad]
    double m = 1500.0;    // vehicle mass [kg]
    double iz = 2250.0;   // yaw moment of inertia [kg*m^2]
    double a = 1.2;       // distance from CG to front axle [m]
    double b = 1.5;       // distance from CG to rear axle [m]
};

} // namespace core
