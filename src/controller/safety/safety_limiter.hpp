#pragma once

#include "common/units.hpp"
#include "controller/state_machine/active_stop_state.hpp"

namespace gripper::controller::safety {

// Static safety limits supplied by config or a learned StructureProfile.
//
// Units:
// - current: A
// - nut speed: mm/s
// - nut acceleration: mm/s^2
// - stroke: mm
//
// The limiter is intentionally pure and hardware-free. It clips command values
// and reports whether the caller should trigger ActiveStop.
struct SafetyLimitConfig {
  common::A max_motor_current{};
  common::MmPerS max_nut_speed{};
  common::MmPerS2 max_nut_acceleration{};
  common::Mm min_nut_stroke{};
  common::Mm max_nut_stroke{};
  bool stroke_limits_enabled{false};
  bool acceleration_limits_enabled{false};
  bool active_stop_on_current_limit{true};
  bool active_stop_on_speed_limit{true};
  bool active_stop_on_acceleration_limit{true};
  bool active_stop_on_stroke_limit{true};
};

// Command before the final safety clamp. Positive nut speed/current moves
// toward increasing stroke, which is the closing direction. The previous speed
// and control period are used to limit acceleration.
struct SafetyLimitCommand {
  common::A motor_current{};
  common::MmPerS nut_speed{};
  common::Mm target_nut_stroke{};
  common::Mm current_nut_stroke{};
  common::MmPerS previous_nut_speed{};
  common::S control_period{};
};

struct SafetyLimitResult {
  common::A motor_current{};
  common::MmPerS nut_speed{};
  common::Mm target_nut_stroke{};
  bool current_limited{false};
  bool speed_limited{false};
  bool acceleration_limited{false};
  bool stroke_limited{false};
  bool active_stop_required{false};
  state_machine::ActiveStopReason active_stop_reason{
      state_machine::ActiveStopReason::None};
};

class SafetyLimiter {
 public:
  SafetyLimiter() = default;
  explicit SafetyLimiter(SafetyLimitConfig config);

  void setConfig(const SafetyLimitConfig& config);
  [[nodiscard]] const SafetyLimitConfig& config() const noexcept;

  // Clips current, speed, acceleration, and target stroke to configured limits.
  // It does not access hardware. If a configured hard limit is exceeded, the
  // result requests ActiveStop so the controller can stop command generation.
  [[nodiscard]] SafetyLimitResult applyLimits(
      const SafetyLimitCommand& command) const;

 private:
  SafetyLimitConfig config_{};
};

}  // namespace gripper::controller::safety
