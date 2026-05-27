#pragma once

#include "common/units.hpp"

namespace gripper::controller::calibration {

// Force mapping coefficients are supplied by config or sample calibration.
// torque_per_force is Nm/N and current_per_torque is A/Nm.
struct ForceMapperConfig {
  common::Ratio torque_per_force{};
  common::Ratio current_per_torque{};
  common::Nm torque_offset{};
  common::A current_offset{};
  common::N min_target_force{};
  common::N max_target_force{};
  common::Nm max_motor_torque{};
  common::A max_motor_current{};
  bool target_force_limits_enabled{false};
};

struct ForceMapResult {
  common::N target_force{};
  common::Nm motor_torque{};
  common::A motor_current{};
  bool target_force_limited{false};
  bool torque_limited{false};
  bool current_limited{false};
};

class ForceMapper {
 public:
  ForceMapper() = default;
  explicit ForceMapper(ForceMapperConfig config);

  void setConfig(const ForceMapperConfig& config);
  [[nodiscard]] const ForceMapperConfig& config() const noexcept;

  // Maps desired single-gripper force in N to motor torque in Nm and motor
  // current in A using configured first-order coefficients.
  [[nodiscard]] ForceMapResult mapTargetForce(common::N target_force) const;

  // Converts motor torque in Nm to motor current in A using the configured
  // current-per-torque coefficient and offset.
  [[nodiscard]] common::A torqueToCurrent(common::Nm motor_torque) const;

 private:
  ForceMapperConfig config_{};
};

}  // namespace gripper::controller::calibration
