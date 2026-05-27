#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class ReleasePhase : std::uint8_t {
  Idle,
  Prepare,
  EnableIfNeeded,
  MoveTowardOpen,
  ConfirmReleased,
  DisableAfterRelease,
  Completed,
  Failed,
};

struct ReleaseStateSnapshot {
  ReleasePhase phase{ReleasePhase::Idle};
  bool released{false};
  bool motor_disabled{false};
};

}  // namespace gripper::controller::state_machine
