#pragma once

#include <cstdint>
#include <iosfwd>

#include "ui/ui_controller.hpp"

namespace gripper::ui {

// Small localhost-only HTTP UI used for bench bring-up.
//
// It intentionally avoids GUI toolkit dependencies. The browser talks to the
// existing UiController, so hardware behavior stays on the same control path as
// the console UI.
class WebServer {
 public:
  WebServer(UiController* controller, std::uint16_t preferred_port);

  int run(std::ostream& output);

 private:
  UiController* controller_{nullptr};
  std::uint16_t preferred_port_{8765};
};

}  // namespace gripper::ui
