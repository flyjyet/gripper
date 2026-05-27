#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class TravelLearningPhase : std::uint8_t {
  Idle,
  Prepare,
  MoveTowardCloseLimit,
  ConfirmLimit,
  BackoffFromLimit,
  SetSoftLimits,
  Completed,
  Failed,
};

struct TravelLearningSnapshot {
  TravelLearningPhase phase{TravelLearningPhase::Idle};
  bool close_limit_found{false};
  bool soft_limits_configured{false};
};

}  // namespace gripper::controller::state_machine
