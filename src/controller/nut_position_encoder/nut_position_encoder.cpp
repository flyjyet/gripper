#include "controller/nut_position_encoder/nut_position_encoder.hpp"

#include <cmath>

namespace gripper::controller::nut_position_encoder {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

}  // namespace

LeadScrewNutPositionEncoder::LeadScrewNutPositionEncoder(
    NutPositionEncoderConfig config)
    : config_(config) {}

void LeadScrewNutPositionEncoder::configure(
    const NutPositionEncoderConfig& config) {
  config_ = config;
  zero_initialized_ = false;
  feedback_ = {};
}

void LeadScrewNutPositionEncoder::resetZero(common::Rad motor_position,
                                            common::Mm nut_position) {
  zero_initialized_ = true;
  feedback_.zero_motor_position = motor_position;
  feedback_.zero_nut_position = nut_position;
  feedback_.motor_position = motor_position;
  feedback_.motor_delta = {};
  feedback_.motor_delta_revolutions = {};
  feedback_.nut_position = nut_position;
  feedback_.nut_velocity = {};
  feedback_.millimeters_per_revolution_estimate =
      common::Ratio{config_.lead_screw_pitch.value};
  feedback_.fresh = false;
  feedback_.last_update_timestamp = {};
}

void LeadScrewNutPositionEncoder::update(
    const hardware_interface::MotorFeedback& motor_feedback) {
  if (!zero_initialized_) {
    resetZero(motor_feedback.position, config_.startup_nut_position);
  }

  const common::Rad motor_delta{
      motor_feedback.position.value - feedback_.zero_motor_position.value};

  feedback_.last_motor_feedback = motor_feedback;
  feedback_.motor_position = motor_feedback.position;
  feedback_.motor_delta = motor_delta;
  feedback_.motor_delta_revolutions = motorDeltaRevolutions(motor_delta);
  feedback_.nut_position =
      common::Mm{feedback_.zero_nut_position.value +
                 nutDeltaFromMotorDelta(motor_delta).value};
  feedback_.nut_velocity = nutVelocityFromMotorVelocity(motor_feedback.velocity);
  feedback_.millimeters_per_revolution_estimate =
      common::Ratio{config_.lead_screw_pitch.value};
  feedback_.fresh = true;
  feedback_.last_update_timestamp = motor_feedback.timestamp;
}

common::Mm LeadScrewNutPositionEncoder::nutPosition() const {
  return feedback_.nut_position;
}

hardware_interface::MotorFeedback LeadScrewNutPositionEncoder::lastMotorFeedback()
    const {
  return feedback_.last_motor_feedback;
}

NutPositionFeedback LeadScrewNutPositionEncoder::feedback() const {
  return feedback_;
}

bool LeadScrewNutPositionEncoder::isFresh(common::Timestamp now) const {
  if (!feedback_.fresh) {
    return false;
  }
  if (config_.feedback_stale_timeout.value <= 0.0) {
    return true;
  }
  const common::Duration age = now - feedback_.last_update_timestamp;
  return age.ns >= 0 &&
         age.seconds() <= config_.feedback_stale_timeout.value;
}

int LeadScrewNutPositionEncoder::normalizedDirectionSign() const noexcept {
  return config_.direction_sign < 0 ? -1 : 1;
}

common::Ratio LeadScrewNutPositionEncoder::motorDeltaRevolutions(
    common::Rad motor_delta) const noexcept {
  return common::Ratio{motor_delta.value / kTwoPi};
}

common::Mm LeadScrewNutPositionEncoder::nutDeltaFromMotorDelta(
    common::Rad motor_delta) const noexcept {
  return common::Mm{static_cast<double>(normalizedDirectionSign()) *
                    motorDeltaRevolutions(motor_delta).value *
                    config_.lead_screw_pitch.value};
}

common::MmPerS LeadScrewNutPositionEncoder::nutVelocityFromMotorVelocity(
    common::RadPerS motor_velocity) const noexcept {
  return common::MmPerS{static_cast<double>(normalizedDirectionSign()) *
                        (motor_velocity.value / kTwoPi) *
                        config_.lead_screw_pitch.value};
}

}  // namespace gripper::controller::nut_position_encoder
