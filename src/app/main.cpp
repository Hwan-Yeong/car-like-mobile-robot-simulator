#include <cmath>

#include <SFML/Graphics.hpp>

#include "core/VehicleParams.hpp"
#include "core/VehicleState.hpp"
#include "core/Waypoint.hpp"
#include "engine/ModelFactory.hpp"
#include "engine/SimulationEngine.hpp"
#include "renderer/SFMLRenderer.hpp"

int main() {
    core::VehicleParams params;
    core::Path oval_path = core::makeOvalPath(/*radius_x=*/30.0, /*radius_y=*/15.0, /*num_points=*/64);

    core::VehicleState initial_state;
    initial_state.x = oval_path.front().x;
    initial_state.y = oval_path.front().y;
    initial_state.psi = M_PI / 2.0; // path is traversed counter-clockwise starting at (radius_x, 0)
    initial_state.vx = 5.0;

    engine::SimulationEngine sim(engine::ModelFactory::createVehicleModel("kinematic_bicycle", params),
                                  engine::ModelFactory::createController("pure_pursuit", params),
                                  oval_path, initial_state);

    sf::RenderWindow window(sf::VideoMode(1280, 720), "Car-Like Mobile Robot Simulator");
    window.setFramerateLimit(60);

    renderer::SFMLRenderer car_renderer(window);
    car_renderer.setPath(oval_path);
    sim.addObserver(&car_renderer);
    car_renderer.onStateUpdated(initial_state);

    constexpr double dt = 1.0 / 60.0;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        sim.step(dt);

        window.clear(sf::Color(30, 30, 30));
        car_renderer.draw();
        window.display();
    }

    return 0;
}
