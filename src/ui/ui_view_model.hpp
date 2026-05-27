#pragma once

#include <string>

#include "controller/gripper_types.hpp"

namespace gripper::ui {

struct UiViewModel {
  controller::GripperStateSnapshot state{};
  std::string status_text{};
  bool can_connect{true};
  bool can_enable{false};
  bool can_run_self_check{false};
  bool can_home{false};
  bool can_learn_limits{false};
  bool can_health_check{false};
  bool can_clamp{false};
  bool can_release{false};
  bool can_move_nut_stroke{false};
  bool can_motor_bringup_move{false};
  bool can_recover_active_stop{false};
};

}  // namespace gripper::ui
