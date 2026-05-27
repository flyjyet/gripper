#include "controller/mechanism/gripper_kinematics.hpp"

namespace gripper::controller::mechanism {

GripperKinematics::GripperKinematics(GripperKinematicsConfig config)
    : config_(config) {}

void GripperKinematics::setConfig(const GripperKinematicsConfig& config) {
  config_ = config;
}

const GripperKinematicsConfig& GripperKinematics::config() const noexcept {
  return config_;
}

common::Rad GripperKinematics::strokeToAngle(common::Mm nut_stroke) const {
  return common::Rad{config_.zero_stroke_angle.value +
                     (nut_stroke.value - config_.zero_angle_stroke.value) *
                         config_.angle_per_stroke.value};
}

common::Mm GripperKinematics::angleToStroke(common::Rad gripper_angle) const {
  if (config_.angle_per_stroke.value == 0.0) {
    return config_.zero_angle_stroke;
  }

  return common::Mm{config_.zero_angle_stroke.value +
                    (gripper_angle.value - config_.zero_stroke_angle.value) /
                        config_.angle_per_stroke.value};
}

common::RadPerS GripperKinematics::nutSpeedToAngularSpeed(
    common::MmPerS nut_speed) const {
  return common::RadPerS{nut_speed.value *
                         config_.angular_speed_per_nut_speed.value};
}

common::Ratio GripperKinematics::angularDerivativePerStroke(
    common::Mm nut_stroke) const {
  (void)nut_stroke;
  return config_.angular_speed_per_nut_speed;
}

common::MmPerS GripperKinematics::angularSpeedToNutSpeed(
    common::RadPerS target_gripper_angular_speed,
    common::Mm current_nut_stroke) const {
  const auto derivative = angularDerivativePerStroke(current_nut_stroke);
  if (derivative.value == 0.0) {
    return {};
  }
  return common::MmPerS{target_gripper_angular_speed.value / derivative.value};
}

}  // namespace gripper::controller::mechanism
