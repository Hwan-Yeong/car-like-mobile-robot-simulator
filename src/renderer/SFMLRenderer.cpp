#include "renderer/SFMLRenderer.hpp"

#include <cmath>

namespace renderer {

SFMLRenderer::SFMLRenderer(sf::RenderWindow& window)
    : window_(window), camera_(window.getSize().x, window.getSize().y) {}

void SFMLRenderer::draw() {
    camera_.follow(latest_state_.x, latest_state_.y);

    if (!path_.empty()) {
        sf::VertexArray path_line(sf::LineStrip, path_.size() + 1);
        for (std::size_t i = 0; i < path_.size(); ++i) {
            path_line[i].position = camera_.worldToScreen(path_[i].x, path_[i].y);
            path_line[i].color = sf::Color(100, 100, 100);
        }
        path_line[path_.size()].position = camera_.worldToScreen(path_[0].x, path_[0].y);
        path_line[path_.size()].color = sf::Color(100, 100, 100);
        window_.draw(path_line);
    }

    constexpr double car_length_m = 2.0;
    constexpr double car_width_m = 1.0;
    const float ppm = static_cast<float>(camera_.pixelsPerMeter());
    sf::RectangleShape car_shape(sf::Vector2f(car_length_m * ppm, car_width_m * ppm));
    car_shape.setOrigin(car_shape.getSize().x / 2.0f, car_shape.getSize().y / 2.0f);
    car_shape.setFillColor(sf::Color(220, 50, 50));

    const sf::Vector2f screen_pos = camera_.worldToScreen(latest_state_.x, latest_state_.y);
    car_shape.setPosition(screen_pos);
    car_shape.setRotation(static_cast<float>(-latest_state_.psi * 180.0 / M_PI));
    window_.draw(car_shape);
}

} // namespace renderer
