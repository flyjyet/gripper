#include "controller/self_check/structure_parameter_identifier.hpp"

#include <algorithm>
#include <cmath>

#include "common/error_code.hpp"

namespace gripper::controller::self_check {

namespace {

[[nodiscard]] double absValue(double value) { return std::abs(value); }

[[nodiscard]] bool hasPositiveValue(double value) { return value > 0.0; }

[[nodiscard]] common::DegC averageTemperature(
    const std::vector<MotionIdentificationSample>& samples) {
  double sum_temperature = 0.0;
  std::uint32_t count = 0;
  for (const auto& sample : samples) {
    sum_temperature += sample.motor_temperature.value;
    ++count;
  }
  if (count == 0) {
    return {};
  }
  return common::DegC{sum_temperature / static_cast<double>(count)};
}

}  // namespace

StructureParameterIdentifier::StructureParameterIdentifier(
    StructureParameterIdentifierConfig config)
    : config_(config) {}

StructureParameterIdentificationResult StructureParameterIdentifier::identify(
    const std::vector<MotionIdentificationSample>& motion_samples,
    const std::vector<FeedbackNoiseSample>& noise_samples) const {
  StructureParameterIdentificationResult output{};
  output.total_sample_count = static_cast<std::uint32_t>(motion_samples.size() +
                                                        noise_samples.size());
  output.average_temperature = averageTemperature(motion_samples);

  DirectionalStructureProfile* profiles[] = {&output.opening, &output.closing};
  for (DirectionalStructureProfile* profile : profiles) {
    profile->minimum_stable_nut_speed =
        config_.fallback_opening_minimum_stable_nut_speed;
    profile->low_speed_unstable_upper_bound =
        config_.fallback_low_speed_unstable_upper_bound;
    profile->quality = IdentificationQuality::ConservativeDefault;
  }
  output.closing.minimum_stable_nut_speed =
      config_.fallback_closing_minimum_stable_nut_speed;

  for (const auto& sample : motion_samples) {
    if (!isValidMotionSample(sample)) {
      ++output.rejected_sample_count;
      continue;
    }

    DirectionalStructureProfile* profile = nullptr;
    if (sample.commanded_direction == MotionDirection::Opening) {
      profile = &output.opening;
    } else if (sample.commanded_direction == MotionDirection::Closing) {
      profile = &output.closing;
    }
    if (profile == nullptr) {
      ++output.rejected_sample_count;
      continue;
    }

    ++output.valid_sample_count;
    ++profile->stable_speed_sample_count;
    if (profile->stable_speed_sample_count == 1 ||
        sample.target_nut_speed.value <
            profile->minimum_stable_nut_speed.value) {
      profile->minimum_stable_nut_speed = sample.target_nut_speed;
    }
    if (sample.target_nut_speed.value >
        profile->low_speed_unstable_upper_bound.value) {
      profile->low_speed_unstable_upper_bound = sample.target_nut_speed;
    }
    profile->quality = IdentificationQuality::LowConfidence;
  }

  auto applyQuality = [this](DirectionalStructureProfile& profile) {
    if (profile.stable_speed_sample_count >= config_.min_valid_motion_samples) {
      profile.quality = IdentificationQuality::Verified;
    }
  };
  applyQuality(output.opening);
  applyQuality(output.closing);

  if (noise_samples.size() >= config_.min_noise_samples) {
    for (const auto& sample : noise_samples) {
      output.noise_floor.motor_position_noise.value =
          std::max(output.noise_floor.motor_position_noise.value,
                   absValue(sample.motor_position_delta.value));
      output.noise_floor.motor_velocity_noise.value =
          std::max(output.noise_floor.motor_velocity_noise.value,
                   absValue(sample.motor_velocity.value));
      output.noise_floor.motor_current_noise.value =
          std::max(output.noise_floor.motor_current_noise.value,
                   absValue(sample.motor_current.value));
      output.noise_floor.motor_torque_noise.value =
          std::max(output.noise_floor.motor_torque_noise.value,
                   absValue(sample.motor_torque.value));
      output.noise_floor.nut_stroke_noise.value =
          std::max(output.noise_floor.nut_stroke_noise.value,
                   absValue(sample.nut_stroke_delta.value));
    }
    output.noise_floor.sample_count =
        static_cast<std::uint32_t>(noise_samples.size());
    output.noise_floor.quality = IdentificationQuality::Verified;
  } else {
    output.noise_floor.motor_position_noise =
        config_.fallback_motor_position_noise;
    output.noise_floor.motor_velocity_noise =
        config_.fallback_motor_velocity_noise;
    output.noise_floor.motor_current_noise = config_.fallback_motor_current_noise;
    output.noise_floor.motor_torque_noise = config_.fallback_motor_torque_noise;
    output.noise_floor.nut_stroke_noise = config_.fallback_nut_stroke_noise;
    output.noise_floor.sample_count =
        static_cast<std::uint32_t>(noise_samples.size());
    output.noise_floor.quality = IdentificationQuality::ConservativeDefault;
  }

  if (output.opening.quality == IdentificationQuality::Verified &&
      output.closing.quality == IdentificationQuality::Verified &&
      output.noise_floor.quality == IdentificationQuality::Verified) {
    output.result = common::Result::ok();
  } else {
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "Insufficient stable motion or noise samples; conservative defaults used");
  }
  return output;
}

bool StructureParameterIdentifier::isValidMotionSample(
    const MotionIdentificationSample& sample) const {
  const double distance_error =
      absValue(sample.measured_distance.value - sample.commanded_distance.value);

  return sample.commanded_direction != MotionDirection::Unknown &&
         sample.settled &&
         sample.direction_matches && sample.position_monotonic &&
         !sample.limit_triggered && !sample.jam_detected &&
         !sample.active_stop_triggered &&
         sample.measured_distance.value >= config_.min_measured_distance.value &&
         distance_error <= config_.max_distance_error.value &&
         hasPositiveValue(sample.target_nut_speed.value);
}

}  // namespace gripper::controller::self_check
