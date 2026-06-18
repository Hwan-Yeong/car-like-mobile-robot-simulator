#include "core/Config.hpp"

#include <iostream>

#include <yaml-cpp/yaml.h>

namespace core {

namespace {

template <typename T>
T getOr(const YAML::Node& node, const std::string& key, T default_value) {
    if (!node || !node[key]) {
        return default_value;
    }
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception&) {
        return default_value;
    }
}

} // namespace

SimulationConfig loadConfig(const std::string& path) {
    SimulationConfig config;

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Config: could not load '" << path << "' (" << e.what() << "), using defaults\n";
        return config;
    }

    const YAML::Node vp = root["vehicle_params"];
    config.vehicle_params.wheelbase = getOr(vp, "wheelbase", config.vehicle_params.wheelbase);
    config.vehicle_params.max_steer = getOr(vp, "max_steer", config.vehicle_params.max_steer);
    config.vehicle_params.max_accel = getOr(vp, "max_accel", config.vehicle_params.max_accel);
    config.vehicle_params.cf = getOr(vp, "cf", config.vehicle_params.cf);
    config.vehicle_params.cr = getOr(vp, "cr", config.vehicle_params.cr);
    config.vehicle_params.m = getOr(vp, "m", config.vehicle_params.m);
    config.vehicle_params.iz = getOr(vp, "iz", config.vehicle_params.iz);
    config.vehicle_params.a = getOr(vp, "a", config.vehicle_params.a);
    config.vehicle_params.b = getOr(vp, "b", config.vehicle_params.b);

    const YAML::Node sim = root["simulation"];
    config.initial_path = getOr(sim, "initial_path", config.initial_path);
    config.initial_model = getOr(sim, "initial_model", config.initial_model);
    config.initial_controller = getOr(sim, "initial_controller", config.initial_controller);
    config.target_speed = getOr(sim, "target_speed", config.target_speed);

    return config;
}

} // namespace core
