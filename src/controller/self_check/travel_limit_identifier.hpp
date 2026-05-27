#pragma once

#include <cstdint>
#include <vector>

#include "common/result.hpp"
#include "common/units.hpp"
#include "controller/self_check/structure_profile.hpp"

namespace gripper::controller::self_check {

struct LimitObservationSample {
  MotionDirection limit_direction{MotionDirection::Unknown};
  common::Mm observed_nut_stroke{};
  bool limit_detected{false};
  bool jam_detected{false};
  bool active_stop_triggered{false};
};

struct TravelLimitIdentifierConfig {
  std::uint32_t min_samples_per_limit{1};
  common::Mm theoretical_travel{};
  common::Mm max_theoretical_travel_error{};
  common::Mm safe_zone_margin{};
  common::Mm software_limit_margin{};
  common::Mm fallback_open_limit{};
  common::Mm fallback_closed_limit{};
};

struct TravelLimitIdentificationResult {
  common::Result result{};
  TravelLimitProfile profile{};
  std::uint32_t total_sample_count{0};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

class TravelLimitIdentifier {
 public:
  explicit TravelLimitIdentifier(TravelLimitIdentifierConfig config = {});

  [[nodiscard]] TravelLimitIdentificationResult identify(
      const std::vector<LimitObservationSample>& samples) const;

 private:
  [[nodiscard]] bool isValidSample(const LimitObservationSample& sample) const;

  TravelLimitIdentifierConfig config_{};
};

}  // namespace gripper::controller::self_check
