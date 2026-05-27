#pragma once

#include <iosfwd>

#include "ui/ui_controller.hpp"

namespace gripper::ui {

// Text UI used for early desktop verification before a Qt/ImGui frontend is
// selected. It exercises the same UiController API expected by a GUI.
class MainWindow {
 public:
  explicit MainWindow(UiController* controller);

  int run(std::istream& input, std::ostream& output);

 private:
  void printStatus(std::ostream& output) const;
  void printLog(std::ostream& output) const;

  UiController* controller_{nullptr};
};

}  // namespace gripper::ui
