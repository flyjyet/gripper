#pragma once

#include <string>
#include <vector>

#include "common/result.hpp"
#include "hardware_interface/motor_types.hpp"

namespace gripper::hardware_interface {

// Abstract motor boundary used by upper layers. Implementations may be
// simulated or backed by real hardware, but must not expose SDK types.
class MotorInterface {
 public:
  virtual ~MotorInterface() = default;

  // Opens the motor-side connection. Fails if the transport or device is not
  // available. On failure the motor remains disconnected.
  [[nodiscard]] virtual common::Result connect() = 0;

  // Closes the motor-side connection. Implementations should leave the motor
  // disabled after success. Fails if shutdown cannot be completed cleanly.
  [[nodiscard]] virtual common::Result disconnect() = 0;

  // Enables command output. Fails when the motor is disconnected or faulted.
  [[nodiscard]] virtual common::Result enable() = 0;

  // Disables command output. Fails when the motor is disconnected.
  [[nodiscard]] virtual common::Result disable() = 0;

  // Sends one motor command using motor-side units. Fails when disconnected,
  // disabled, faulted, or when the command is unsupported by the implementation.
  [[nodiscard]] virtual common::Result sendCommand(
      const MotorCommand& command) = 0;

  // Reads the latest motor feedback into feedback. Fails when disconnected or
  // when feedback is null.
  [[nodiscard]] virtual common::Result readFeedback(
      MotorFeedback* feedback) = 0;

  // Optional low-energy communication probe for hardware bring-up. The default
  // implementation keeps upper layers independent from device-specific CAN
  // frames while still allowing real implementations to return raw diagnostic
  // lines from the transport boundary.
  [[nodiscard]] virtual common::Result runCommunicationProbe(
      std::vector<std::string>* lines) {
    if (lines == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "communication probe output is null");
    }
    lines->clear();
    lines->push_back("communication probe is not implemented by this motor");
    return common::Result::error(common::ErrorCode::Unsupported,
                                 "communication probe is not implemented");
  }

  [[nodiscard]] virtual bool isConnected() const noexcept = 0;
  [[nodiscard]] virtual bool isEnabled() const noexcept = 0;
};

}  // namespace gripper::hardware_interface
