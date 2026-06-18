#pragma once

#include "core/VehicleParams.hpp"

namespace gui {

// Real-time sliders for the active vehicle model's physical parameters.
class ParamPanel {
public:
    void draw(core::VehicleParams& params);
};

} // namespace gui
