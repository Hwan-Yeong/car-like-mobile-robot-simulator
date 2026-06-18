#pragma once

#include <array>

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
    void drawRoad();
    void drawCar();

    sf::RenderWindow& window_;
    Camera camera_;
    core::Path path_;
    core::VehicleState latest_state_;

    // Car silhouette, defined once in car-local meters (origin = vehicle reference
    // point, +x = forward) and redrawn each frame via a translate+rotate+scale
    // sf::Transform -- see drawCar().
    sf::ConvexShape car_body_;
    sf::ConvexShape car_cabin_;
    sf::CircleShape headlight_left_;
    sf::CircleShape headlight_right_;
    std::array<sf::RectangleShape, 4> wheels_;
};

} // namespace renderer
