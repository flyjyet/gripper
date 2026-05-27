#include "controller/self_check/travel_limit_identifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "common/error_code.hpp"

namespace gripper::controller::self_check {

TravelLimitIdentifier::TravelLimitIdentifier(TravelLimitIdentifierConfig config)
    : config_(config) {}

TravelLimitIdentificationResult TravelLimitIdentifier::identify(
    const std::vector<LimitObservationSample>& samples) const {
  TravelLimitIdentificationResult output{};
  output.total_sample_count = static_cast<std::uint32_t>(samples.size());

  double open_limit = std::numeric_limits<double>::max();
  double closed_limit = std::numeric_limits<double>::lowest();
  std::uint32_t open_count = 0;
  std::uint32_t closed_count = 0;

  for (const auto& sample : samples) {
    if (!isValidSample(sample)) {
      ++output.rejected_sample_count;
      continue;
    }
    ++output.valid_sample_count;
    if (sample.limit_direction == MotionDirection::Opening) {
      open_limit = std::min(open_limit, sample.observed_nut_stroke.value);
      ++open_count;
    } else if (sample.limit_direction == MotionDirection::Closing) {
      closed_limit = std::max(closed_limit, sample.observed_nut_stroke.value);
      ++closed_count;
    }
  }

  const bool enough_open = open_count >= config_.min_samples_per_limit;
  const bool enough_closed = closed_count >= config_.min_samples_per_limit;
  if (!enough_open) {
    open_limit = config_.fallback_open_limit.value;
  }
  if (!enough_closed) {
    closed_limit = config_.fallback_closed_limit.value;
  }

  output.profile.preliminary_open_limit = common::Mm{open_limit};
  output.profile.preliminary_closed_limit = common::Mm{closed_limit};
  output.profile.learned_travel = common::Mm{closed_limit - open_limit};
  output.profile.theoretical_travel_error =
      common::Mm{std::abs(output.profile.learned_travel.value -
                          config_.theoretical_travel.value)};
  output.profile.valid_limit_sample_count = output.valid_sample_count;

  output.profile.safe_zone_open_limit =
      common::Mm{open_limit + config_.safe_zone_margin.value};
  output.profile.safe_zone_closed_limit =
      common::Mm{closed_limit - config_.safe_zone_margin.value};
  output.profile.software_open_limit =
      common::Mm{open_limit + config_.software_limit_margin.value};
  output.profile.software_closed_limit =
      common::Mm{closed_limit - config_.software_limit_margin.value};

  const bool ordered =
      output.profile.software_open_limit.value <
      output.profile.software_closed_limit.value;
  const bool theory_match =
      output.profile.theoretical_travel_error.value <=
      config_.max_theoretical_travel_error.value;

  if (enough_open && enough_closed && ordered && theory_match) {
    output.profile.quality = IdentificationQuality::Verified;
    output.result = common::Result::ok();
  } else if (ordered) {
    output.profile.quality = IdentificationQuality::ConservativeDefault;
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "Travel limits used conservative defaults or mismatched theory");
  } else {
    output.profile.quality = IdentificationQuality::Invalid;
    output.result = common::Result::error(
        common::ErrorCode::SelfCheckUnsafeMotion,
        "Travel limits do not leave a valid software motion range");
  }
  return output;
}

bool TravelLimitIdentifier::isValidSample(
    const LimitObservationSample& sample) const {
  return sample.limit_direction != MotionDirection::Unknown &&
         sample.limit_detected && !sample.jam_detected &&
         !sample.active_stop_triggered;
}

}  // namespace gripper::controller::self_check
