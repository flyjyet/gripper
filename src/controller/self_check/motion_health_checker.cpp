#include "controller/self_check/motion_health_checker.hpp"

#include <algorithm>

#include "common/error_code.hpp"

namespace gripper::controller::self_check {

MotionHealthChecker::MotionHealthChecker(MotionHealthCheckerConfig config)
    : config_(config) {}

MotionHealthCheckResult MotionHealthChecker::check(
    const std::vector<MotionHealthSample>& samples) const {
  MotionHealthCheckResult output{};
  output.total_sample_count = static_cast<std::uint32_t>(samples.size());

  bool unsafe_sample_seen = false;
  for (const auto& sample : samples) {
    if (!isValidSample(sample)) {
      ++output.profile.rejected_sample_count;
      if (!isSafeSample(sample)) {
        unsafe_sample_seen = true;
      }
      continue;
    }

    ++output.profile.valid_sample_count;
    output.profile.max_velocity_tracking_error.value =
        std::max(output.profile.max_velocity_tracking_error.value,
                 sample.max_velocity_tracking_error.value);
    output.profile.max_current_ripple.value =
        std::max(output.profile.max_current_ripple.value,
                 sample.current_ripple.value);
    output.profile.max_torque_ripple.value =
        std::max(output.profile.max_torque_ripple.value,
                 sample.torque_ripple.value);
    output.profile.max_temperature.value =
        std::max(output.profile.max_temperature.value,
                 sample.max_temperature.value);
  }

  const bool enough_samples =
      output.profile.valid_sample_count >= config_.min_valid_samples;
  const bool degraded = output.profile.max_velocity_tracking_error.value >
                            config_.max_velocity_tracking_error.value ||
                        output.profile.max_current_ripple.value >
                            config_.max_current_ripple.value ||
                        output.profile.max_torque_ripple.value >
                            config_.max_torque_ripple.value ||
                        output.profile.max_temperature.value >
                            config_.max_temperature.value;

  if (!enough_samples) {
    output.profile.status =
        unsafe_sample_seen ? MotionHealthStatus::Unsafe : MotionHealthStatus::Unknown;
    output.profile.quality = IdentificationQuality::ConservativeDefault;
    output.profile.score = common::Ratio{0.0};
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "Insufficient motion health samples");
  } else if (unsafe_sample_seen) {
    output.profile.status = MotionHealthStatus::Unsafe;
    output.profile.quality = IdentificationQuality::Verified;
    output.profile.score = common::Ratio{0.0};
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckUnsafeMotion,
        "Unsafe motion health sample detected");
  } else if (degraded) {
    output.profile.status = MotionHealthStatus::Degraded;
    output.profile.quality = IdentificationQuality::Verified;
    output.profile.score = common::Ratio{0.5};
    output.result = common::Result::error(
        common::ErrorCode::ControlUnstable,
        "Motion health thresholds exceeded");
  } else {
    output.profile.status = MotionHealthStatus::Healthy;
    output.profile.quality = IdentificationQuality::Verified;
    output.profile.score = common::Ratio{1.0};
    output.result = common::Result::ok();
  }

  return output;
}

bool MotionHealthChecker::isValidSample(
    const MotionHealthSample& sample) const {
  return sample.direction != MotionDirection::Unknown && isSafeSample(sample) &&
         sample.position_monotonic && sample.repeatable &&
         !sample.current_spike_detected &&
         sample.target_nut_speed.value > 0.0;
}

bool MotionHealthChecker::isSafeSample(
    const MotionHealthSample& sample) const {
  const bool start_inside =
      sample.start_nut_stroke.value >= config_.software_open_limit.value &&
      sample.start_nut_stroke.value <= config_.software_closed_limit.value;
  const bool end_inside =
      sample.end_nut_stroke.value >= config_.software_open_limit.value &&
      sample.end_nut_stroke.value <= config_.software_closed_limit.value;

  return sample.within_software_limits && start_inside && end_inside &&
         !sample.jam_detected;
}

}  // namespace gripper::controller::self_check
