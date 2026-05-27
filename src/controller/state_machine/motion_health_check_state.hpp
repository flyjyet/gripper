#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class MotionHealthCheckPhase : std::uint8_t {
  Idle,
  Prepare,
  SelectSpeed,
  MoveOpenDirection,
  MoveCloseDirection,
  EvaluateSample,
  Completed,
  Failed,
};

struct MotionHealthCheckSnapshot {
  MotionHealthCheckPhase phase{MotionHealthCheckPhase::Idle};
  std::uint32_t tested_speed_count{0};
  std::uint32_t accepted_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

}  // namespace gripper::controller::state_machine
