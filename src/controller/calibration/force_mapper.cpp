#include "controller/calibration/force_mapper.hpp"

#include <algorithm>
#include <cmath>

namespace gripper::controller::calibration {
namespace {

[[nodiscard]] double clampSymmetric(double value, double max_abs) {
  const double limit = std::abs(max_abs);
  return std::clamp(value, -limit, limit);
}

}  // namespace

ForceMapper::ForceMapper(ForceMapperConfig config) : config_(config) {}

void ForceMapper::setConfig(const ForceMapperConfig& config) {
  config_ = config;
}

const ForceMapperConfig& ForceMapper::config() const noexcept {
  return config_;
}

ForceMapResult ForceMapper::mapTargetForce(common::N target_force) const {
  ForceMapResult result{};
  result.target_force = target_force;

  if (config_.target_force_limits_enabled) {
    const double min_force =
        std::min(config_.min_target_force.value, config_.max_target_force.value);
    const double max_force =
        std::max(config_.min_target_force.value, config_.max_target_force.value);
    const double limited_force =
        std::clamp(target_force.value, min_force, max_force);
    result.target_force = common::N{limited_force};
    result.target_force_limited = limited_force != target_force.value;
  }

  const double raw_torque = result.target_force.value *
                                config_.torque_per_force.value +
                            config_.torque_offset.value;
  const double limited_torque =
      clampSymmetric(raw_torque, config_.max_motor_torque.value);
  result.motor_torque = common::Nm{limited_torque};
  result.torque_limited = limited_torque != raw_torque;

  result.motor_current = torqueToCurrent(result.motor_torque);
  const double limited_current =
      clampSymmetric(result.motor_current.value, config_.max_motor_current.value);
  result.current_limited = limited_current != result.motor_current.value;
  result.motor_current = common::A{limited_current};

  return result;
}

common::A ForceMapper::torqueToCurrent(common::Nm motor_torque) const {
  return common::A{motor_torque.value * config_.current_per_torque.value +
                   config_.current_offset.value};
}

}  // namespace gripper::controller::calibration
