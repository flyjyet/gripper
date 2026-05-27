#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class HomingPhase : std::uint8_t {
  Idle,
  Prepare,
  MoveTowardOpenStop,
  ConfirmStall,
  SetZero,
  Backoff,
  Completed,
  Failed,
};

struct HomingStateSnapshot {
  HomingPhase phase{HomingPhase::Idle};
  bool zero_configured{false};
  bool conservative_profile_used{false};
};

}  // namespace gripper::controller::state_machine
