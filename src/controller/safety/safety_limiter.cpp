#include "controller/safety/safety_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace gripper::controller::safety {
namespace {

[[nodiscard]] double clampSymmetric(double value, double max_abs) {
  const double limit = std::abs(max_abs);
  return std::clamp(value, -limit, limit);
}

[[nodiscard]] bool changed(double lhs, double rhs) {
  return lhs != rhs;
}

}  // namespace

SafetyLimiter::SafetyLimiter(SafetyLimitConfig config) : config_(config) {}

void SafetyLimiter::setConfig(const SafetyLimitConfig& config) {
  config_ = config;
}

const SafetyLimitConfig& SafetyLimiter::config() const noexcept {
  return config_;
}

SafetyLimitResult SafetyLimiter::applyLimits(
    const SafetyLimitCommand& command) const {
  SafetyLimitResult result{};

  const double limited_current =
      clampSymmetric(command.motor_current.value, config_.max_motor_current.value);
  double limited_speed =
      clampSymmetric(command.nut_speed.value, config_.max_nut_speed.value);

  result.motor_current = common::A{limited_current};
  result.target_nut_stroke = command.target_nut_stroke;
  result.current_limited = changed(limited_current, command.motor_current.value);
  result.speed_limited = changed(limited_speed, command.nut_speed.value);

  if (config_.acceleration_limits_enabled &&
      command.control_period.value > 0.0 &&
      config_.max_nut_acceleration.value > 0.0) {
    const double max_delta =
        std::abs(config_.max_nut_acceleration.value) *
        command.control_period.value;
    const double low = command.previous_nut_speed.value - max_delta;
    const double high = command.previous_nut_speed.value + max_delta;
    const double acceleration_limited_speed =
        std::clamp(limited_speed, low, high);
    result.acceleration_limited =
        changed(acceleration_limited_speed, limited_speed);
    limited_speed = acceleration_limited_speed;
  }

  result.nut_speed = common::MmPerS{limited_speed};

  if (config_.stroke_limits_enabled) {
    const double min_stroke = std::min(config_.min_nut_stroke.value,
                                      config_.max_nut_stroke.value);
    const double max_stroke = std::max(config_.min_nut_stroke.value,
                                      config_.max_nut_stroke.value);
    const double limited_target =
        std::clamp(command.target_nut_stroke.value, min_stroke, max_stroke);

    result.target_nut_stroke = common::Mm{limited_target};
    result.stroke_limited = changed(limited_target,
                                    command.target_nut_stroke.value) ||
                            command.current_nut_stroke.value < min_stroke ||
                            command.current_nut_stroke.value > max_stroke;

    if (result.stroke_limited && config_.active_stop_on_stroke_limit) {
      result.active_stop_required = true;
      result.active_stop_reason = state_machine::ActiveStopReason::StrokeLimit;
      return result;
    }
  }

  if (result.current_limited && config_.active_stop_on_current_limit) {
    result.active_stop_required = true;
    result.active_stop_reason = state_machine::ActiveStopReason::CurrentLimit;
    return result;
  }

  if (result.speed_limited && config_.active_stop_on_speed_limit) {
    result.active_stop_required = true;
    result.active_stop_reason = state_machine::ActiveStopReason::SpeedLimit;
    return result;
  }

  if (result.acceleration_limited &&
      config_.active_stop_on_acceleration_limit) {
    result.active_stop_required = true;
    result.active_stop_reason = state_machine::ActiveStopReason::SpeedLimit;
  }

  return result;
}

}  // namespace gripper::controller::safety
