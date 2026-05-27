#include "gripper_control/safety_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace gripper_control {

SafetyLimiter::SafetyLimiter(GripperLimits limits) : limits_(limits) {}

const GripperLimits& SafetyLimiter::limits() const { return limits_; }

void SafetyLimiter::setLimits(const GripperLimits& limits) { limits_ = limits; }

double SafetyLimiter::clampStroke(double stroke_mm) const {
  return std::clamp(stroke_mm, limits_.stroke_min_mm, limits_.stroke_max_mm);
}

double SafetyLimiter::clampCurrent(double requested_A, double state_limit_A) const {
  const double abs_limit = std::max(0.0, std::min(limits_.current_abs_max_A, state_limit_A));
  return std::clamp(requested_A, -abs_limit, abs_limit);
}

double SafetyLimiter::clampVelocity(double requested_mm_s, double state_limit_mm_s) const {
  const double abs_limit = std::max(0.0, state_limit_mm_s);
  return std::clamp(requested_mm_s, -abs_limit, abs_limit);
}

double SafetyLimiter::clampTargetForce(double target_force_N, const ForceMapParams& params) const {
  return std::clamp(target_force_N, 0.0, params.max_target_force_N);
}

bool SafetyLimiter::isStrokeInsideSoftLimit(double stroke_mm) const {
  return stroke_mm >= limits_.stroke_min_mm && stroke_mm <= limits_.stroke_max_mm;
}

bool SafetyLimiter::isTemperatureFault(double temperature_C) const {
  return temperature_C >= limits_.over_temperature_C;
}

bool SafetyLimiter::shouldDerateForTemperature(double temperature_C) const {
  return temperature_C >= limits_.derate_temperature_C;
}

MotorCommand SafetyLimiter::velocityCommand(double requested_velocity_mm_s,
                                            double state_velocity_limit_mm_s,
                                            double requested_current_limit_A,
                                            double state_current_limit_A) const {
  MotorCommand command;
  command.mode = MotorMode::Velocity;
  command.target_velocity_mm_s = clampVelocity(requested_velocity_mm_s, state_velocity_limit_mm_s);
  command.current_limit_A =
      std::abs(clampCurrent(std::abs(requested_current_limit_A), state_current_limit_A));
  return command;
}

MotorCommand SafetyLimiter::currentCommand(double requested_current_A,
                                           double state_current_limit_A) const {
  MotorCommand command;
  command.mode = MotorMode::Current;
  command.target_current_A = clampCurrent(requested_current_A, state_current_limit_A);
  command.current_limit_A = std::abs(command.target_current_A);
  return command;
}

}  // namespace gripper_control
