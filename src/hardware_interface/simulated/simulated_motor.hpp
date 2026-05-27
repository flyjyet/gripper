#pragma once

#include <cstdint>

#include "hardware_interface/motor_interface.hpp"

namespace gripper::hardware_interface {

// Minimal simulated motor for upper-layer workflow tests. It does not model
// inertia, friction, limits, or thermal dynamics; commands are reflected into
// feedback immediately after validation.
class SimulatedMotor final : public MotorInterface {
 public:
  SimulatedMotor() = default;
  explicit SimulatedMotor(MotorId motor_id);

  [[nodiscard]] common::Result connect() override;
  [[nodiscard]] common::Result disconnect() override;
  [[nodiscard]] common::Result enable() override;
  [[nodiscard]] common::Result disable() override;
  [[nodiscard]] common::Result sendCommand(const MotorCommand& command) override;
  [[nodiscard]] common::Result readFeedback(MotorFeedback* feedback) override;

  [[nodiscard]] bool isConnected() const noexcept override;
  [[nodiscard]] bool isEnabled() const noexcept override;

  [[nodiscard]] MotorId motorId() const noexcept;

  // Test hook for fault-state coverage. A fault rejects enable and command
  // output until clearSimulatedFault() is called.
  void setSimulatedFault() noexcept;
  void clearSimulatedFault() noexcept;

 private:
  [[nodiscard]] common::Timestamp nextTimestamp() noexcept;

  MotorId motor_id_{};
  bool connected_{false};
  bool enabled_{false};
  bool fault_{false};
  MotorCommand last_command_{};
  MotorFeedback feedback_{};
  std::int64_t timestamp_ns_{0};
};

}  // namespace gripper::hardware_interface
