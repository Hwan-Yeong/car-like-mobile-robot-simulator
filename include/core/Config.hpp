#pragma once

#include <string>

#include "core/VehicleParams.hpp"

namespace core {

// Startup configuration: vehicle physical params plus the initial simulation
// selections (path/model/controller names, target speed).
struct SimulationConfig {
    VehicleParams vehicle_params;
    std::string initial_path = "oval";
    std::string initial_model = "kinematic_bicycle";
    std::string initial_controller = "pure_pursuit";
    double target_speed = 5.0;
};

// Loads `path` (YAML) over the defaults above. Missing file, missing keys, or
// a malformed file all fall back to defaults field-by-field rather than
// throwing -- a bad config shouldn't prevent the simulator from starting.
SimulationConfig loadConfig(const std::string& path);

} // namespace core
