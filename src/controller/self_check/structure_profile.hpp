#pragma once

#include <cstdint>

#include "common/units.hpp"
#include "controller/gripper_types.hpp"

namespace gripper::controller::self_check {

enum class MotionDirection : std::uint8_t {
  Unknown,
  Opening,
  Closing,
};

enum class IdentificationQuality : std::uint8_t {
  Invalid,
  ConservativeDefault,
  LowConfidence,
  Verified,
};

enum class MotionHealthStatus : std::uint8_t {
  Unknown,
  Healthy,
  Degraded,
  Unsafe,
};

struct DirectionalStructureProfile {
  common::A bootstrap_breakaway_current{};
  common::Nm bootstrap_breakaway_torque{};

  common::MmPerS minimum_stable_nut_speed{};
  common::MmPerS low_speed_unstable_upper_bound{};

  common::A static_friction_current{};
  common::A dynamic_friction_current_average{};
  common::A dynamic_friction_current_max{};

  common::Nm static_friction_torque{};
  common::Nm dynamic_friction_torque_average{};
  common::Nm dynamic_friction_torque_max{};

  std::uint32_t bootstrap_breakaway_sample_count{0};
  std::uint32_t stable_speed_sample_count{0};
  std::uint32_t static_friction_sample_count{0};
  std::uint32_t dynamic_friction_sample_count{0};
  IdentificationQuality quality{IdentificationQuality::Invalid};
};

struct FeedbackNoiseFloor {
  common::Rad motor_position_noise{};
  common::RadPerS motor_velocity_noise{};
  common::A motor_current_noise{};
  common::Nm motor_torque_noise{};
  common::Mm nut_stroke_noise{};
  std::uint32_t sample_count{0};
  IdentificationQuality quality{IdentificationQuality::Invalid};
};

struct TravelLimitProfile {
  common::Mm preliminary_open_limit{};
  common::Mm preliminary_closed_limit{};
  common::Mm safe_zone_open_limit{};
  common::Mm safe_zone_closed_limit{};
  common::Mm software_open_limit{};
  common::Mm software_closed_limit{};
  common::Mm learned_travel{};
  common::Mm theoretical_travel_error{};
  std::uint32_t valid_limit_sample_count{0};
  IdentificationQuality quality{IdentificationQuality::Invalid};
};

struct MotionHealthProfile {
  MotionHealthStatus status{MotionHealthStatus::Unknown};
  common::Ratio score{};
  common::MmPerS max_velocity_tracking_error{};
  common::A max_current_ripple{};
  common::Nm max_torque_ripple{};
  common::DegC max_temperature{};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
  IdentificationQuality quality{IdentificationQuality::Invalid};
};

struct StructureProfile {
  StructureProfileValidity validity{StructureProfileValidity::Unknown};
  IdentificationQuality quality{IdentificationQuality::Invalid};

  DirectionalStructureProfile opening{};
  DirectionalStructureProfile closing{};
  FeedbackNoiseFloor noise_floor{};
  TravelLimitProfile travel_limits{};
  MotionHealthProfile motion_health{};

  common::DegC identification_temperature{};
  std::uint32_t total_sample_count{0};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

}  // namespace gripper::controller::self_check
