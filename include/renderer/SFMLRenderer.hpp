#pragma once

#include <SFML/Graphics.hpp>

#include "core/VehicleState.hpp"
#include "core/Waypoint.hpp"
#include "engine/ISimulationObserver.hpp"
#include "renderer/Camera.hpp"

namespace renderer {

// Draws the path and vehicle into an SFML window. Implements ISimulationObserver
// so it can be registered with the SimulationEngine and stay in sync each step.
class SFMLRenderer : public engine::ISimulationObserver {
public:
    explicit SFMLRenderer(sf::RenderWindow& window);

    void setPath(const core::Path& path) { path_ = path; }

    void onStateUpdated(const core::VehicleState& state) override { latest_state_ = state; }

    void draw();

private:
    sf::RenderWindow& window_;
    Camera camera_;
    core::Path path_;
    core::VehicleState latest_state_;
};

} // namespace renderer
