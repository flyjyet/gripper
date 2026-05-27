#include "controller/self_check/friction_identifier.hpp"

#include <algorithm>

#include "common/error_code.hpp"

namespace gripper::controller::self_check {

namespace {

struct DirectionAccumulator {
  common::A static_current_max{};
  common::Nm static_torque_max{};
  common::A dynamic_current_sum{};
  common::A dynamic_current_max{};
  common::Nm dynamic_torque_sum{};
  common::Nm dynamic_torque_max{};
  std::uint32_t static_count{0};
  std::uint32_t dynamic_count{0};
};

void applyFallback(DirectionalStructureProfile& profile) {
  profile.static_friction_current = {};
  profile.static_friction_torque = {};
  profile.dynamic_friction_current_average = {};
  profile.dynamic_friction_current_max = {};
  profile.dynamic_friction_torque_average = {};
  profile.dynamic_friction_torque_max = {};
  profile.quality = IdentificationQuality::ConservativeDefault;
}

void applyAccumulator(DirectionalStructureProfile& profile,
                      const DirectionAccumulator& accumulator,
                      const FrictionIdentifierConfig& config) {
  profile.static_friction_sample_count = accumulator.static_count;
  profile.dynamic_friction_sample_count = accumulator.dynamic_count;

  if (accumulator.static_count >= config.min_static_samples_per_direction) {
    profile.static_friction_current = accumulator.static_current_max;
    profile.static_friction_torque = accumulator.static_torque_max;
  } else {
    profile.static_friction_current = {};
    profile.static_friction_torque = {};
  }

  if (accumulator.dynamic_count >= config.min_dynamic_samples_per_direction) {
    const double dynamic_count =
        static_cast<double>(accumulator.dynamic_count);
    profile.dynamic_friction_current_average =
        common::A{accumulator.dynamic_current_sum.value / dynamic_count};
    profile.dynamic_friction_current_max = accumulator.dynamic_current_max;
    profile.dynamic_friction_torque_average =
        common::Nm{accumulator.dynamic_torque_sum.value / dynamic_count};
    profile.dynamic_friction_torque_max = accumulator.dynamic_torque_max;
  } else {
    profile.dynamic_friction_current_average = {};
    profile.dynamic_friction_current_max = {};
    profile.dynamic_friction_torque_average = {};
    profile.dynamic_friction_torque_max = {};
  }

  if (accumulator.static_count >= config.min_static_samples_per_direction &&
      accumulator.dynamic_count >= config.min_dynamic_samples_per_direction) {
    profile.quality = IdentificationQuality::Verified;
  } else if (accumulator.static_count > 0 || accumulator.dynamic_count > 0) {
    profile.quality = IdentificationQuality::LowConfidence;
  } else {
    profile.quality = IdentificationQuality::ConservativeDefault;
  }
}

}  // namespace

FrictionIdentifier::FrictionIdentifier(FrictionIdentifierConfig config)
    : config_(config) {}

FrictionIdentificationResult FrictionIdentifier::identify(
    const std::vector<StaticFrictionSample>& static_samples,
    const std::vector<DynamicFrictionSample>& dynamic_samples,
    const DirectionalStructureProfile& opening_seed,
    const DirectionalStructureProfile& closing_seed) const {
  FrictionIdentificationResult output{};
  output.opening = opening_seed;
  output.closing = closing_seed;
  output.total_sample_count = static_cast<std::uint32_t>(static_samples.size() +
                                                        dynamic_samples.size());

  DirectionAccumulator opening_accumulator{};
  DirectionAccumulator closing_accumulator{};

  for (const auto& sample : static_samples) {
    if (!isValidStaticSample(sample)) {
      ++output.rejected_sample_count;
      continue;
    }
    DirectionAccumulator* accumulator =
        sample.direction == MotionDirection::Opening ? &opening_accumulator
                                                     : &closing_accumulator;
    ++output.valid_sample_count;
    ++accumulator->static_count;
    accumulator->static_current_max.value =
        std::max(accumulator->static_current_max.value,
                 sample.breakaway_current.value);
    accumulator->static_torque_max.value =
        std::max(accumulator->static_torque_max.value,
                 sample.breakaway_torque.value);
  }

  for (const auto& sample : dynamic_samples) {
    if (!isValidDynamicSample(sample)) {
      ++output.rejected_sample_count;
      continue;
    }
    DirectionAccumulator* accumulator =
        sample.direction == MotionDirection::Opening ? &opening_accumulator
                                                     : &closing_accumulator;
    ++output.valid_sample_count;
    ++accumulator->dynamic_count;
    accumulator->dynamic_current_sum.value += sample.average_motor_current.value;
    accumulator->dynamic_current_max.value =
        std::max(accumulator->dynamic_current_max.value,
                 sample.max_motor_current.value);
    accumulator->dynamic_torque_sum.value += sample.average_motor_torque.value;
    accumulator->dynamic_torque_max.value =
        std::max(accumulator->dynamic_torque_max.value,
                 sample.max_motor_torque.value);
  }

  applyFallback(output.opening);
  applyFallback(output.closing);
  output.opening.minimum_stable_nut_speed =
      opening_seed.minimum_stable_nut_speed;
  output.opening.low_speed_unstable_upper_bound =
      opening_seed.low_speed_unstable_upper_bound;
  output.closing.minimum_stable_nut_speed =
      closing_seed.minimum_stable_nut_speed;
  output.closing.low_speed_unstable_upper_bound =
      closing_seed.low_speed_unstable_upper_bound;

  applyAccumulator(output.opening, opening_accumulator, config_);
  applyAccumulator(output.closing, closing_accumulator, config_);

  if (output.opening.quality == IdentificationQuality::Verified &&
      output.closing.quality == IdentificationQuality::Verified) {
    output.result = common::Result::ok();
  } else {
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "Insufficient friction samples; conservative defaults used");
  }
  return output;
}

bool FrictionIdentifier::isValidStaticSample(
    const StaticFrictionSample& sample) const {
  return sample.direction != MotionDirection::Unknown && sample.motion_started &&
         sample.within_probe_window && !sample.limit_triggered &&
         sample.breakaway_current.value >= 0.0 &&
         sample.breakaway_torque.value >= 0.0;
}

bool FrictionIdentifier::isValidDynamicSample(
    const DynamicFrictionSample& sample) const {
  const double required_speed =
      sample.minimum_stable_nut_speed.value + config_.stable_speed_margin.value;
  return sample.direction != MotionDirection::Unknown &&
         sample.constant_velocity_segment &&
         !sample.limit_triggered && !sample.jam_detected &&
         sample.measured_nut_speed.value >= required_speed &&
         sample.average_motor_current.value >= 0.0 &&
         sample.average_motor_torque.value >= 0.0;
}

}  // namespace gripper::controller::self_check
