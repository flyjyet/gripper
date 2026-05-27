#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class ClampPhase : std::uint8_t {
  Idle,
  Prepare,
  ApproachAtConstantSpeed,
  ContactDetected,
  BuildTargetForce,
  VerifyForce,
  UnloadBeforeDisable,
  Completed,
  Failed,
};

struct ClampStateSnapshot {
  ClampPhase phase{ClampPhase::Idle};
  bool contact_detected{false};
  bool target_force_reached{false};
};

}  // namespace gripper::controller::state_machine
