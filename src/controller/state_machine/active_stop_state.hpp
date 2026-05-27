#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

enum class ActiveStopReason : std::uint8_t {
  None,
  UserStop,
  CurrentLimit,
  SpeedLimit,
  StrokeLimit,
  ContactOrJam,
  CommunicationTimeout,
  HardwareFault,
};

struct ActiveStopSnapshot {
  ActiveStopReason reason{ActiveStopReason::None};
  bool motor_command_zeroed{false};
  bool motor_disabled{false};
  bool recoverable{false};
};

}  // namespace gripper::controller::state_machine
