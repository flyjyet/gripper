#pragma once

#include "common/units.hpp"

namespace gripper::controller::mechanism {

// Kinematic parameters supplied by config or StructureProfile.
//
// The current runtime implementation still uses a first-order derivative until
// the full closed-chain model or a calibrated lookup table is moved from
// references/02_mechanics into src. The public functions already expose the
// nonlinear-control semantics required by the controller:
//   omega = J(s) * v_nut
//   v_nut = omega / J(s)
struct GripperKinematicsConfig {
  common::Mm zero_angle_stroke{};
  common::Rad zero_stroke_angle{};
  common::Ratio angle_per_stroke{};
  common::Ratio angular_speed_per_nut_speed{};
};

class GripperKinematics {
 public:
  GripperKinematics() = default;
  explicit GripperKinematics(GripperKinematicsConfig config);

  void setConfig(const GripperKinematicsConfig& config);
  [[nodiscard]] const GripperKinematicsConfig& config() const noexcept;

  // Converts nut stroke in mm to gripper angle in rad using the configured
  // first-order mechanism approximation.
  [[nodiscard]] common::Rad strokeToAngle(common::Mm nut_stroke) const;

  // Converts gripper angle in rad back to nut stroke in mm. If the configured
  // slope is zero, the configured zero-angle stroke is returned.
  [[nodiscard]] common::Mm angleToStroke(common::Rad gripper_angle) const;

  // Converts nut linear speed in mm/s to gripper angular speed in rad/s.
  [[nodiscard]] common::RadPerS nutSpeedToAngularSpeed(
      common::MmPerS nut_speed) const;

  // Returns J(s) = d(gripper_angle)/d(nut_stroke), in rad/mm.
  [[nodiscard]] common::Ratio angularDerivativePerStroke(
      common::Mm nut_stroke) const;

  // Converts a target gripper angular speed to nut speed at the current stroke.
  // A complete nonlinear model should make J(s) stroke-dependent; the first-pass
  // fallback uses the configured constant derivative.
  [[nodiscard]] common::MmPerS angularSpeedToNutSpeed(
      common::RadPerS target_gripper_angular_speed,
      common::Mm current_nut_stroke) const;

 private:
  GripperKinematicsConfig config_{};
};

}  // namespace gripper::controller::mechanism
