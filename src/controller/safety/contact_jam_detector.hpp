#pragma once

#include "common/units.hpp"
#include "controller/state_machine/active_stop_state.hpp"

namespace gripper::controller::safety {

// Contact/jam detector for a sensor-limited gripper.
//
// The mechanism has no external force sensor and no jaw-side position sensor.
// Detection therefore uses motor-side current rise, measured nut-speed drop,
// and stroke context as a proxy. Thresholds come from config or learned
// StructureProfile values.
//
// Units:
// - current: A
// - speed: mm/s
// - stroke: mm
// - time: s
struct ContactJamDetectorConfig {
  common::A contact_current_rise{};
  common::A jam_current_rise{};
  common::MmPerS velocity_drop_threshold{};
  common::Mm stroke_limit_margin{};
  common::S minimum_detection_time{};
  bool contact_detection_enabled{false};
  bool jam_detection_enabled{false};
};

struct ContactJamContext {
  common::A motor_current{};
  common::A baseline_motion_current{};
  common::MmPerS measured_nut_speed{};
  common::MmPerS commanded_nut_speed{};
  common::Mm current_nut_stroke{};
  common::Mm min_nut_stroke{};
  common::Mm max_nut_stroke{};
  common::S elapsed_in_motion{};
};

enum class ContactJamState {
  Clear,
  ContactDetected,
  JamDetected,
  StrokeLimitSuspected,
};

struct ContactJamResult {
  ContactJamState state{ContactJamState::Clear};
  bool contact_detected{false};
  bool jam_detected{false};
  bool stroke_limit_suspected{false};
  bool active_stop_required{false};
  state_machine::ActiveStopReason active_stop_reason{
      state_machine::ActiveStopReason::None};
};

class ContactJamDetector {
 public:
  ContactJamDetector() = default;
  explicit ContactJamDetector(ContactJamDetectorConfig config);

  void setConfig(const ContactJamDetectorConfig& config);
  [[nodiscard]] const ContactJamDetectorConfig& config() const noexcept;

  // Detects simplified contact and jam states from current rise, measured
  // velocity drop, and stroke context. It evaluates one provided sample and
  // keeps no internal time history; the caller is responsible for sampling and
  // elapsed-time gating.
  [[nodiscard]] ContactJamResult detect(const ContactJamContext& context) const;

 private:
  ContactJamDetectorConfig config_{};
};

}  // namespace gripper::controller::safety
