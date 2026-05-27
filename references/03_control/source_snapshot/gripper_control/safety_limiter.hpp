#pragma once

#include "gripper_control/types.hpp"

namespace gripper_control {

class SafetyLimiter {
 public:
  explicit SafetyLimiter(GripperLimits limits = {});

  const GripperLimits& limits() const;
  void setLimits(const GripperLimits& limits);

  double clampStroke(double stroke_mm) const;
  double clampCurrent(double requested_A, double state_limit_A) const;
  double clampVelocity(double requested_mm_s, double state_limit_mm_s) const;
  double clampTargetForce(double target_force_N, const ForceMapParams& params) const;
  bool isStrokeInsideSoftLimit(double stroke_mm) const;
  bool isTemperatureFault(double temperature_C) const;
  bool shouldDerateForTemperature(double temperature_C) const;

  MotorCommand velocityCommand(double requested_velocity_mm_s,
                               double state_velocity_limit_mm_s,
                               double requested_current_limit_A,
                               double state_current_limit_A) const;

  MotorCommand currentCommand(double requested_current_A,
                              double state_current_limit_A) const;

 private:
  GripperLimits limits_;
};

}  // namespace gripper_control
