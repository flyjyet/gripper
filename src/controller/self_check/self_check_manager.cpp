#include "controller/self_check/self_check_manager.hpp"

#include "common/error_code.hpp"

namespace gripper::controller::self_check {

namespace {

[[nodiscard]] IdentificationQuality mergeQuality(
    IdentificationQuality lhs, IdentificationQuality rhs) {
  if (lhs == IdentificationQuality::Invalid ||
      rhs == IdentificationQuality::Invalid) {
    return IdentificationQuality::Invalid;
  }
  if (lhs == IdentificationQuality::ConservativeDefault ||
      rhs == IdentificationQuality::ConservativeDefault) {
    return IdentificationQuality::ConservativeDefault;
  }
  if (lhs == IdentificationQuality::LowConfidence ||
      rhs == IdentificationQuality::LowConfidence) {
    return IdentificationQuality::LowConfidence;
  }
  return IdentificationQuality::Verified;
}

[[nodiscard]] common::Result firstError(const common::Result& current,
                                        const common::Result& next) {
  if (current.isError()) {
    return current;
  }
  return next;
}

}  // namespace

SelfCheckManager::SelfCheckManager(SelfCheckManagerConfig config)
    : config_(config) {}

SelfCheckOutput SelfCheckManager::run(const SelfCheckInput& input) const {
  SelfCheckOutput output{};

  const StructureParameterIdentifier structure_identifier{
      config_.structure_parameter_config};
  const auto structure_result =
      structure_identifier.identify(input.motion_samples, input.noise_samples);

  const FrictionIdentifier friction_identifier{config_.friction_config};
  const auto friction_result = friction_identifier.identify(
      input.static_friction_samples, input.dynamic_friction_samples,
      structure_result.opening, structure_result.closing);

  const TravelLimitIdentifier travel_limit_identifier{
      config_.travel_limit_config};
  const auto travel_result = travel_limit_identifier.identify(input.limit_samples);

  MotionHealthCheckerConfig health_config = config_.motion_health_config;
  if (health_config.software_open_limit.value == 0.0 &&
      health_config.software_closed_limit.value == 0.0) {
    health_config.software_open_limit =
        travel_result.profile.software_open_limit;
    health_config.software_closed_limit =
        travel_result.profile.software_closed_limit;
  }
  const MotionHealthChecker health_checker{health_config};
  const auto health_result = health_checker.check(input.health_samples);

  output.profile.opening = friction_result.opening;
  output.profile.closing = friction_result.closing;
  output.profile.noise_floor = structure_result.noise_floor;
  output.profile.travel_limits = travel_result.profile;
  output.profile.motion_health = health_result.profile;
  output.profile.identification_temperature =
      structure_result.average_temperature;

  output.profile.total_sample_count =
      structure_result.total_sample_count + friction_result.total_sample_count +
      travel_result.total_sample_count + health_result.total_sample_count;
  output.profile.valid_sample_count =
      structure_result.valid_sample_count + friction_result.valid_sample_count +
      travel_result.valid_sample_count +
      health_result.profile.valid_sample_count;
  output.profile.rejected_sample_count =
      structure_result.rejected_sample_count +
      friction_result.rejected_sample_count + travel_result.rejected_sample_count +
      health_result.profile.rejected_sample_count;

  output.profile.quality = mergeQuality(
      mergeQuality(output.profile.opening.quality,
                   output.profile.closing.quality),
      mergeQuality(output.profile.noise_floor.quality,
                   mergeQuality(output.profile.travel_limits.quality,
                                output.profile.motion_health.quality)));

  if (output.profile.motion_health.status == MotionHealthStatus::Healthy &&
      output.profile.travel_limits.quality == IdentificationQuality::Verified &&
      output.profile.opening.quality == IdentificationQuality::Verified &&
      output.profile.closing.quality == IdentificationQuality::Verified &&
      output.profile.noise_floor.quality == IdentificationQuality::Verified) {
    output.profile.validity = StructureProfileValidity::MotionHealthChecked;
  } else if (output.profile.travel_limits.quality ==
             IdentificationQuality::Verified) {
    output.profile.validity = StructureProfileValidity::TravelLimitsLearned;
  } else if (output.profile.opening.quality != IdentificationQuality::Invalid &&
             output.profile.closing.quality != IdentificationQuality::Invalid) {
    output.profile.validity = StructureProfileValidity::PreSelfCheckCompleted;
  } else {
    output.profile.validity = StructureProfileValidity::ConservativeDefaults;
  }

  output.result = firstError(common::Result::ok(), structure_result.result);
  output.result = firstError(output.result, friction_result.result);
  output.result = firstError(output.result, travel_result.result);
  output.result = firstError(output.result, health_result.result);
  if (output.result.isOk() &&
      output.profile.quality != IdentificationQuality::Verified) {
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "Self check completed without fully verified profile");
  }

  return output;
}

}  // namespace gripper::controller::self_check
