#pragma once

#include <cstdint>
#include <vector>

#include "common/result.hpp"
#include "common/units.hpp"
#include "controller/self_check/structure_profile.hpp"

namespace gripper::controller::self_check {

struct MotionHealthSample {
  MotionDirection direction{MotionDirection::Unknown};
  common::Mm start_nut_stroke{};
  common::Mm end_nut_stroke{};
  common::MmPerS target_nut_speed{};
  common::MmPerS max_velocity_tracking_error{};
  common::A current_ripple{};
  common::Nm torque_ripple{};
  common::DegC max_temperature{};
  bool within_software_limits{false};
  bool position_monotonic{false};
  bool repeatable{false};
  bool jam_detected{false};
  bool current_spike_detected{false};
};

struct MotionHealthCheckerConfig {
  std::uint32_t min_valid_samples{1};
  common::Mm software_open_limit{};
  common::Mm software_closed_limit{};
  common::MmPerS max_velocity_tracking_error{};
  common::A max_current_ripple{};
  common::Nm max_torque_ripple{};
  common::DegC max_temperature{};
};

struct MotionHealthCheckResult {
  common::Result result{};
  MotionHealthProfile profile{};
  std::uint32_t total_sample_count{0};
};

class MotionHealthChecker {
 public:
  explicit MotionHealthChecker(MotionHealthCheckerConfig config = {});

  [[nodiscard]] MotionHealthCheckResult check(
      const std::vector<MotionHealthSample>& samples) const;

 private:
  [[nodiscard]] bool isValidSample(const MotionHealthSample& sample) const;
  [[nodiscard]] bool isSafeSample(const MotionHealthSample& sample) const;

  MotionHealthCheckerConfig config_{};
};

}  // namespace gripper::controller::self_check
