#include "engine/ModelFactory.hpp"

#include <stdexcept>

#include "controller/PurePursuitController.hpp"
#include "core/KinematicBicycleModel.hpp"

namespace engine {

std::unique_ptr<core::IVehicleModel> ModelFactory::createVehicleModel(
    const std::string& type_name, const core::VehicleParams* params) {
    if (type_name == "kinematic_bicycle") {
        return std::make_unique<core::KinematicBicycleModel>(params);
    }
    throw std::invalid_argument("Unknown vehicle model type: " + type_name);
}

std::unique_ptr<controller::IController> ModelFactory::createController(
    const std::string& type_name, const core::VehicleParams* params) {
    if (type_name == "pure_pursuit") {
        constexpr double k = 0.5;
        constexpr double min_lookahead = 2.0;
        constexpr double max_lookahead = 10.0;
        return std::make_unique<controller::PurePursuitController>(params, k, min_lookahead, max_lookahead);
    }
    throw std::invalid_argument("Unknown controller type: " + type_name);
}

std::vector<std::string> ModelFactory::vehicleModelNames() { return {"kinematic_bicycle"}; }

std::vector<std::string> ModelFactory::controllerNames() { return {"pure_pursuit"}; }

} // namespace engine
