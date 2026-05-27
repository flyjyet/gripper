#pragma once

#include <cstdint>
#include <vector>

#include "common/result.hpp"
#include "common/units.hpp"
#include "controller/self_check/structure_profile.hpp"

namespace gripper::controller::self_check {

struct StaticFrictionSample {
  MotionDirection direction{MotionDirection::Unknown};
  common::A breakaway_current{};
  common::Nm breakaway_torque{};
  bool motion_started{false};
  bool within_probe_window{false};
  bool limit_triggered{false};
};

struct DynamicFrictionSample {
  MotionDirection direction{MotionDirection::Unknown};
  common::MmPerS measured_nut_speed{};
  common::MmPerS minimum_stable_nut_speed{};
  common::A average_motor_current{};
  common::A max_motor_current{};
  common::Nm average_motor_torque{};
  common::Nm max_motor_torque{};
  bool constant_velocity_segment{false};
  bool velocity_stable{false};
  bool current_stable{false};
  bool limit_triggered{false};
  bool jam_detected{false};
};

struct FrictionIdentifierConfig {
  std::uint32_t min_static_samples_per_direction{1};
  std::uint32_t min_dynamic_samples_per_direction{1};
  common::MmPerS stable_speed_margin{};
};

struct FrictionIdentificationResult {
  common::Result result{};
  DirectionalStructureProfile opening{};
  DirectionalStructureProfile closing{};
  std::uint32_t total_sample_count{0};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

class FrictionIdentifier {
 public:
  explicit FrictionIdentifier(FrictionIdentifierConfig config = {});

  [[nodiscard]] FrictionIdentificationResult identify(
      const std::vector<StaticFrictionSample>& static_samples,
      const std::vector<DynamicFrictionSample>& dynamic_samples,
      const DirectionalStructureProfile& opening_seed,
      const DirectionalStructureProfile& closing_seed) const;

 private:
  [[nodiscard]] bool isValidStaticSample(
      const StaticFrictionSample& sample) const;
  [[nodiscard]] bool isValidDynamicSample(
      const DynamicFrictionSample& sample) const;

  FrictionIdentifierConfig config_{};
};

}  // namespace gripper::controller::self_check
