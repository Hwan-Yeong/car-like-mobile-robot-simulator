#pragma once

#include <memory>
#include <string>

#include "controller/IController.hpp"
#include "core/IVehicleModel.hpp"
#include "core/VehicleParams.hpp"

namespace engine {

// Factory for constructing vehicle models and controllers by name, so the
// engine can swap implementations at runtime without depending on concrete types.
class ModelFactory {
public:
    static std::unique_ptr<core::IVehicleModel> createVehicleModel(const std::string& type_name,
                                                                     const core::VehicleParams& params);

    static std::unique_ptr<controller::IController> createController(const std::string& type_name,
                                                                       const core::VehicleParams& params);
};

} // namespace engine
