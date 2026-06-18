#pragma once

#include <SFML/Graphics.hpp>

namespace renderer {

// World<->screen transform with pan/zoom and an optional follow target.
// World coordinates are meters with Y-up; screen coordinates are pixels with Y-down.
class Camera {
public:
    Camera(unsigned int screen_width, unsigned int screen_height);

    void setPixelsPerMeter(double pixels_per_meter) { pixels_per_meter_ = pixels_per_meter; }
    double pixelsPerMeter() const { return pixels_per_meter_; }
    void pan(double dx_world, double dy_world);
    void zoom(double factor);
    void follow(double world_x, double world_y) { center_x_ = world_x; center_y_ = world_y; }

    sf::Vector2f worldToScreen(double world_x, double world_y) const;

private:
    unsigned int screen_width_;
    unsigned int screen_height_;
    double pixels_per_meter_ = 10.0;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
};

} // namespace renderer
