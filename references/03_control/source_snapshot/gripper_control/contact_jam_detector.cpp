#include "gripper_control/contact_jam_detector.hpp"

#include <cmath>

namespace gripper_control {

ContactJamDetector::ContactJamDetector(JamDetectionParams params) : params_(params) {}

void ContactJamDetector::reset() {
  have_last_position_ = false;
  last_position_mm_ = 0.0;
  last_window_start_s_ = 0.0;
  window_start_position_mm_ = 0.0;
  contact_started_s_ = -1.0;
  stall_started_s_ = -1.0;
}

void ContactJamDetector::setParams(const JamDetectionParams& params) { params_ = params; }

const JamDetectionParams& ContactJamDetector::params() const { return params_; }

DetectionResult ContactJamDetector::update(double now_s,
                                           const MotorFeedback& feedback,
                                           double commanded_velocity_mm_s,
                                           const FrictionParams& friction,
                                           bool closing,
                                           bool contact_can_be_normal) {
  DetectionResult result;

  if (!have_last_position_) {
    have_last_position_ = true;
    last_position_mm_ = feedback.stroke_mm;
    last_window_start_s_ = now_s;
    window_start_position_mm_ = feedback.stroke_mm;
    return result;
  }

  if (now_s - last_window_start_s_ >= params_.position_window_s) {
    last_window_start_s_ = now_s;
    window_start_position_mm_ = last_position_mm_;
  }

  const double position_delta = std::abs(feedback.stroke_mm - window_start_position_mm_);
  const double current_over_friction =
      std::abs(feedback.current_A) - directionFriction(friction, closing);
  const bool command_active = std::abs(commanded_velocity_mm_s) > params_.speed_cmd_min_mm_s;
  const bool speed_low = std::abs(feedback.velocity_mm_s) < params_.speed_stall_threshold_mm_s;
  const bool position_not_moving = position_delta < params_.position_delta_min_mm;

  const bool contact_signal =
      closing && command_active && speed_low &&
      current_over_friction > params_.current_contact_delta_A;

  if (contact_signal) {
    if (contact_started_s_ < 0.0) {
      contact_started_s_ = now_s;
    }
    if (now_s - contact_started_s_ >= params_.contact_debounce_s) {
      result.contact = true;
    }
  } else {
    contact_started_s_ = -1.0;
  }

  const bool stall_signal =
      command_active && speed_low && position_not_moving &&
      current_over_friction > params_.current_stall_delta_A;

  if (stall_signal) {
    if (stall_started_s_ < 0.0) {
      stall_started_s_ = now_s;
    }
    if (now_s - stall_started_s_ >= params_.stall_debounce_s) {
      result.stalled = true;
      result.stall_kind = contact_can_be_normal ? StallKind::Contact : StallKind::AbnormalJam;
    }
  } else {
    stall_started_s_ = -1.0;
  }

  last_position_mm_ = feedback.stroke_mm;
  return result;
}

double ContactJamDetector::directionFriction(const FrictionParams& friction, bool closing) const {
  if (!friction.valid) {
    return closing ? 0.10 : 0.08;
  }
  return closing ? friction.current_close_A : friction.current_open_A;
}

}  // namespace gripper_control
