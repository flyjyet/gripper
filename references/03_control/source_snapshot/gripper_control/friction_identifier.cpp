#include "gripper_control/friction_identifier.hpp"

#include <algorithm>
#include <cmath>

namespace gripper_control {

void FrictionIdentifier::reset(double now_s) {
  started_s_ = now_s;
  done_ = false;
  close_current_sum_ = 0.0;
  open_current_sum_ = 0.0;
  close_samples_ = 0;
  open_samples_ = 0;
  result_ = {};
}

bool FrictionIdentifier::update(double now_s, const MotorFeedback& feedback) {
  const double elapsed = now_s - started_s_;
  if (elapsed > 0.2 && elapsed < 1.0) {
    close_current_sum_ += std::abs(feedback.current_A);
    ++close_samples_;
  } else if (elapsed > 1.2 && elapsed < 2.0) {
    open_current_sum_ += std::abs(feedback.current_A);
    ++open_samples_;
  } else if (elapsed >= 2.1 && !done_) {
    result_.current_close_A =
        close_samples_ > 0 ? close_current_sum_ / close_samples_ : 0.10;
    result_.current_open_A =
        open_samples_ > 0 ? open_current_sum_ / open_samples_ : 0.08;
    result_.torque_close_Nm = result_.current_close_A * 0.55;
    result_.torque_open_Nm = result_.current_open_A * 0.55;
    result_.valid = true;
    done_ = true;
  }
  return done_;
}

bool FrictionIdentifier::done() const { return done_; }

FrictionParams FrictionIdentifier::result() const { return result_; }

double FrictionIdentifier::commandVelocity(double homing_velocity_mm_s) const {
  // First move slightly close, then open, both at low speed/current.
  return (started_s_ == 0.0) ? 0.0 : homing_velocity_mm_s;
}

}  // namespace gripper_control
