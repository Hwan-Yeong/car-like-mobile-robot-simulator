#include "renderer/Camera.hpp"

#include <algorithm>

namespace renderer {

Camera::Camera(unsigned int screen_width, unsigned int screen_height)
    : screen_width_(screen_width), screen_height_(screen_height) {}

void Camera::pan(double dx_world, double dy_world) {
    center_x_ += dx_world;
    center_y_ += dy_world;
}

void Camera::zoom(double factor) {
    pixels_per_meter_ = std::clamp(pixels_per_meter_ * factor, 1.0, 500.0);
}

sf::Vector2f Camera::worldToScreen(double world_x, double world_y) const {
    const double sx = (world_x - center_x_) * pixels_per_meter_ + screen_width_ / 2.0;
    const double sy = screen_height_ / 2.0 - (world_y - center_y_) * pixels_per_meter_;
    return sf::Vector2f(static_cast<float>(sx), static_cast<float>(sy));
}

} // namespace renderer
