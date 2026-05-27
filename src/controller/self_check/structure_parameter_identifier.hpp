#pragma once

#include <cstdint>
#include <vector>

#include "common/result.hpp"
#include "common/units.hpp"
#include "controller/self_check/structure_profile.hpp"

namespace gripper::controller::self_check {

struct MotionIdentificationSample {
  MotionDirection commanded_direction{MotionDirection::Unknown};
  common::Mm commanded_distance{};
  common::Mm measured_distance{};
  common::MmPerS target_nut_speed{};
  common::MmPerS average_nut_speed{};
  common::MmPerS max_nut_speed{};
  common::A average_motor_current{};
  common::A max_motor_current{};
  common::Nm average_motor_torque{};
  common::Nm max_motor_torque{};
  common::DegC motor_temperature{};
  bool direction_matches{false};
  bool position_monotonic{false};
  bool velocity_stable{false};
  bool current_stable{false};
  bool limit_triggered{false};
  bool jam_detected{false};
  bool active_stop_triggered{false};
  // True only when the controller has stopped output and observed a stable
  // post-motion feedback sample. Unsettled samples are diagnostic only.
  bool settled{false};
};

struct FeedbackNoiseSample {
  common::Rad motor_position_delta{};
  common::RadPerS motor_velocity{};
  common::A motor_current{};
  common::Nm motor_torque{};
  common::Mm nut_stroke_delta{};
};

struct StructureParameterIdentifierConfig {
  std::uint32_t min_valid_motion_samples{1};
  std::uint32_t min_noise_samples{1};
  common::Mm min_measured_distance{};
  common::Mm max_distance_error{};
  common::MmPerS fallback_opening_minimum_stable_nut_speed{};
  common::MmPerS fallback_closing_minimum_stable_nut_speed{};
  common::MmPerS fallback_low_speed_unstable_upper_bound{};
  common::Rad fallback_motor_position_noise{};
  common::RadPerS fallback_motor_velocity_noise{};
  common::A fallback_motor_current_noise{};
  common::Nm fallback_motor_torque_noise{};
  common::Mm fallback_nut_stroke_noise{};
};

struct StructureParameterIdentificationResult {
  common::Result result{};
  DirectionalStructureProfile opening{};
  DirectionalStructureProfile closing{};
  FeedbackNoiseFloor noise_floor{};
  common::DegC average_temperature{};
  std::uint32_t total_sample_count{0};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

class StructureParameterIdentifier {
 public:
  explicit StructureParameterIdentifier(
      StructureParameterIdentifierConfig config = {});

  [[nodiscard]] StructureParameterIdentificationResult identify(
      const std::vector<MotionIdentificationSample>& motion_samples,
      const std::vector<FeedbackNoiseSample>& noise_samples) const;

 private:
  [[nodiscard]] bool isValidMotionSample(
      const MotionIdentificationSample& sample) const;

  StructureParameterIdentifierConfig config_{};
};

}  // namespace gripper::controller::self_check
