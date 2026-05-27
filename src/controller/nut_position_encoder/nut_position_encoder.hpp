#pragma once

// Module role:
// - Converts motor-side continuous position feedback into virtual nut position.
// - Owns the motor-position-to-lead-screw mapping used by V2 controller stages.
// - Does not access CAN, Damiao SDK objects, UI, or state-machine transitions.

#include "common/timestamp.hpp"
#include "common/units.hpp"
#include "hardware_interface/motor_types.hpp"

namespace gripper::controller::nut_position_encoder {

// V2 controller-facing position feedback. This is the single movement-position
// source for self-check, homing, travel learning, health check, and clamp logic.
struct NutPositionFeedback {
  common::Mm nut_position{};
  common::MmPerS nut_velocity{};
  common::Rad zero_motor_position{};
  common::Mm zero_nut_position{};
  common::Rad motor_position{};
  common::Rad motor_delta{};
  common::Ratio motor_delta_revolutions{};
  common::Ratio millimeters_per_revolution_estimate{};
  hardware_interface::MotorFeedback last_motor_feedback{};
  bool fresh{false};
  common::Timestamp last_update_timestamp{};
};

// Configuration for the V2 virtual nut-position encoder.
struct NutPositionEncoderConfig {
  common::Mm lead_screw_pitch{2.0};
  int direction_sign{1};
  common::S feedback_stale_timeout{0.2};
  common::Mm startup_nut_position{0.0};
};

// V2 virtual nut-position encoder interface.
//
// Implementations use MotorFeedback.position as the already validated
// motor-side continuous position. They must not parse vendor frames or read
// hardware directly; those responsibilities stay in hardware_interface.
class NutPositionEncoder {
 public:
  virtual ~NutPositionEncoder() = default;

  virtual void configure(const NutPositionEncoderConfig& config) = 0;
  virtual void resetZero(common::Rad motor_position,
                         common::Mm nut_position) = 0;
  virtual void update(
      const hardware_interface::MotorFeedback& motor_feedback) = 0;
  [[nodiscard]] virtual common::Mm nutPosition() const = 0;
  [[nodiscard]] virtual hardware_interface::MotorFeedback lastMotorFeedback()
      const = 0;
  [[nodiscard]] virtual NutPositionFeedback feedback() const = 0;
  [[nodiscard]] virtual bool isFresh(common::Timestamp now) const = 0;
};

// Lead-screw implementation for the current gripper mechanism.
//
// The class is intentionally pure controller logic. It maps:
//   motor_delta_rad / (2*pi) * lead_screw_pitch_mm_per_rev
// into nut travel and keeps the latest raw MotorFeedback for UI/log diagnosis.
class LeadScrewNutPositionEncoder final : public NutPositionEncoder {
 public:
  LeadScrewNutPositionEncoder() = default;
  explicit LeadScrewNutPositionEncoder(NutPositionEncoderConfig config);

  void configure(const NutPositionEncoderConfig& config) override;
  void resetZero(common::Rad motor_position,
                 common::Mm nut_position) override;
  void update(
      const hardware_interface::MotorFeedback& motor_feedback) override;
  [[nodiscard]] common::Mm nutPosition() const override;
  [[nodiscard]] hardware_interface::MotorFeedback lastMotorFeedback()
      const override;
  [[nodiscard]] NutPositionFeedback feedback() const override;
  [[nodiscard]] bool isFresh(common::Timestamp now) const override;

 private:
  [[nodiscard]] int normalizedDirectionSign() const noexcept;
  [[nodiscard]] common::Ratio motorDeltaRevolutions(
      common::Rad motor_delta) const noexcept;
  [[nodiscard]] common::Mm nutDeltaFromMotorDelta(
      common::Rad motor_delta) const noexcept;
  [[nodiscard]] common::MmPerS nutVelocityFromMotorVelocity(
      common::RadPerS motor_velocity) const noexcept;

  NutPositionEncoderConfig config_{};
  bool zero_initialized_{false};
  NutPositionFeedback feedback_{};
};

}  // namespace gripper::controller::nut_position_encoder
