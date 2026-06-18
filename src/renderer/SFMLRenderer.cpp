#include "renderer/SFMLRenderer.hpp"

#include <cmath>
#include <vector>

namespace renderer {

namespace {
// Car-local geometry, meters, origin = vehicle reference point, +x = forward.
constexpr float kRearOverhang = -0.3f;
constexpr float kNoseTip = 3.2f;
constexpr float kHalfWidth = 0.9f;

constexpr double kRoadHalfWidth = 4.0; // 8m wide road surface
constexpr double kCurbWidth = 1.0;
} // namespace

SFMLRenderer::SFMLRenderer(sf::RenderWindow& window)
    : window_(window), camera_(window.getSize().x, window.getSize().y) {
    car_body_.setPointCount(7);
    car_body_.setPoint(0, {kRearOverhang, kHalfWidth});
    car_body_.setPoint(1, {kRearOverhang, -kHalfWidth});
    car_body_.setPoint(2, {2.0f, -0.95f});
    car_body_.setPoint(3, {2.8f, -0.6f});
    car_body_.setPoint(4, {kNoseTip, 0.0f});
    car_body_.setPoint(5, {2.8f, 0.6f});
    car_body_.setPoint(6, {2.0f, 0.95f});
    car_body_.setFillColor(sf::Color(200, 30, 30));
    car_body_.setOutlineColor(sf::Color(110, 10, 10));
    car_body_.setOutlineThickness(0.05f);

    car_cabin_.setPointCount(4);
    car_cabin_.setPoint(0, {0.3f, 0.6f});
    car_cabin_.setPoint(1, {0.3f, -0.6f});
    car_cabin_.setPoint(2, {1.9f, -0.5f});
    car_cabin_.setPoint(3, {1.9f, 0.5f});
    car_cabin_.setFillColor(sf::Color(50, 60, 80));

    headlight_left_.setRadius(0.15f);
    headlight_left_.setOrigin(0.15f, 0.15f);
    headlight_left_.setFillColor(sf::Color(255, 250, 200));
    headlight_left_.setPosition(3.0f, 0.5f);

    headlight_right_.setRadius(0.15f);
    headlight_right_.setOrigin(0.15f, 0.15f);
    headlight_right_.setFillColor(sf::Color(255, 250, 200));
    headlight_right_.setPosition(3.0f, -0.5f);

    constexpr float wheel_length = 0.6f;
    constexpr float wheel_width = 0.25f;
    const std::array<sf::Vector2f, 4> wheel_centers = {
        sf::Vector2f(2.7f, kHalfWidth + 0.05f),  sf::Vector2f(2.7f, -kHalfWidth - 0.05f),
        sf::Vector2f(0.0f, kHalfWidth + 0.05f),  sf::Vector2f(0.0f, -kHalfWidth - 0.05f),
    };
    for (std::size_t i = 0; i < wheels_.size(); ++i) {
        wheels_[i].setSize({wheel_length, wheel_width});
        wheels_[i].setOrigin(wheel_length / 2.0f, wheel_width / 2.0f);
        wheels_[i].setPosition(wheel_centers[i]);
        wheels_[i].setFillColor(sf::Color(25, 25, 25));
    }
}

void SFMLRenderer::draw() {
    camera_.follow(latest_state_.x, latest_state_.y);

    drawRoad();
    drawCar();
}

void SFMLRenderer::drawRoad() {
    const std::size_t n = path_.size();
    if (n < 2) {
        return;
    }

    // Offset points (in screen space) at the road edge and at the outer curb
    // edge, on both sides of the centerline -- built from the averaged
    // incoming/outgoing segment tangent at each waypoint so corners don't gap.
    std::vector<sf::Vector2f> left(n), right(n), left_outer(n), right_outer(n);
    for (std::size_t i = 0; i < n; ++i) {
        const core::Waypoint& prev = path_[(i + n - 1) % n];
        const core::Waypoint& curr = path_[i];
        const core::Waypoint& next = path_[(i + 1) % n];

        double in_x = curr.x - prev.x;
        double in_y = curr.y - prev.y;
        double out_x = next.x - curr.x;
        double out_y = next.y - curr.y;
        const double in_len = std::hypot(in_x, in_y);
        const double out_len = std::hypot(out_x, out_y);
        if (in_len > 1e-9) {
            in_x /= in_len;
            in_y /= in_len;
        }
        if (out_len > 1e-9) {
            out_x /= out_len;
            out_y /= out_len;
        }

        double tx = in_x + out_x;
        double ty = in_y + out_y;
        double tlen = std::hypot(tx, ty);
        if (tlen < 1e-9) {
            tx = out_x;
            ty = out_y;
            tlen = 1.0;
        }
        tx /= tlen;
        ty /= tlen;

        const double nx = -ty; // left-hand normal of the tangent
        const double ny = tx;

        left[i] = camera_.worldToScreen(curr.x + nx * kRoadHalfWidth, curr.y + ny * kRoadHalfWidth);
        right[i] = camera_.worldToScreen(curr.x - nx * kRoadHalfWidth, curr.y - ny * kRoadHalfWidth);
        left_outer[i] = camera_.worldToScreen(curr.x + nx * (kRoadHalfWidth + kCurbWidth),
                                               curr.y + ny * (kRoadHalfWidth + kCurbWidth));
        right_outer[i] = camera_.worldToScreen(curr.x - nx * (kRoadHalfWidth + kCurbWidth),
                                                curr.y - ny * (kRoadHalfWidth + kCurbWidth));
    }

    sf::VertexArray surface(sf::TriangleStrip, 2 * (n + 1));
    for (std::size_t i = 0; i <= n; ++i) {
        const std::size_t idx = i % n;
        surface[2 * i].position = left[idx];
        surface[2 * i].color = sf::Color(60, 60, 65);
        surface[2 * i + 1].position = right[idx];
        surface[2 * i + 1].color = sf::Color(60, 60, 65);
    }
    window_.draw(surface);

    sf::VertexArray curbs(sf::Quads, 8 * n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        const sf::Color color = (i % 2 == 0) ? sf::Color(200, 30, 30) : sf::Color(235, 235, 235);
        const std::size_t base = i * 8;

        curbs[base + 0].position = left[i];
        curbs[base + 1].position = left[j];
        curbs[base + 2].position = left_outer[j];
        curbs[base + 3].position = left_outer[i];
        curbs[base + 4].position = right[i];
        curbs[base + 5].position = right[j];
        curbs[base + 6].position = right_outer[j];
        curbs[base + 7].position = right_outer[i];
        for (std::size_t k = 0; k < 8; ++k) {
            curbs[base + k].color = color;
        }
    }
    window_.draw(curbs);
}

void SFMLRenderer::drawCar() {
    const float ppm = static_cast<float>(camera_.pixelsPerMeter());
    const sf::Vector2f screen_pos = camera_.worldToScreen(latest_state_.x, latest_state_.y);

    sf::Transform transform;
    transform.translate(screen_pos);
    transform.rotate(static_cast<float>(-latest_state_.psi * 180.0 / M_PI));
    transform.scale(ppm, ppm);

    const sf::RenderStates states(transform);
    for (const sf::RectangleShape& wheel : wheels_) {
        window_.draw(wheel, states);
    }
    window_.draw(car_body_, states);
    window_.draw(car_cabin_, states);
    window_.draw(headlight_left_, states);
    window_.draw(headlight_right_, states);
}

} // namespace renderer
