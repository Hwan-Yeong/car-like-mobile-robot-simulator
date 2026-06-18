#pragma once

#include <string>
#include <vector>

namespace gui {

// Plain state owned by the caller (main.cpp) and mutated by ControlPanel::draw.
// Keeping gui/ ignorant of engine/ types keeps the GUI layer decoupled --
// main.cpp is responsible for acting on changes here.
struct ControlPanelState {
    bool paused = false;
    bool reset_requested = false;
    double target_speed = 5.0;
    int path_index = 0;
    int model_index = 0;
    int controller_index = 0;
};

class ControlPanel {
public:
    void draw(ControlPanelState& state, const std::vector<std::string>& path_names,
              const std::vector<std::string>& model_names,
              const std::vector<std::string>& controller_names);
};

} // namespace gui
