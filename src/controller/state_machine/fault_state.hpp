#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class FaultSeverity : std::uint8_t {
  None,
  Warning,
  Recoverable,
  RequiresRehome,
  RequiresManualCheck,
};

enum class FaultSource : std::uint8_t {
  None,
  Hardware,
  Communication,
  Safety,
  SelfCheck,
  Homing,
  TravelLearning,
  MotionHealthCheck,
  Clamp,
  Release,
};

struct FaultSnapshot {
  FaultSeverity severity{FaultSeverity::None};
  FaultSource source{FaultSource::None};
  std::uint32_t code{0};
  bool cleared{false};
};

}  // namespace gripper::controller::state_machine
