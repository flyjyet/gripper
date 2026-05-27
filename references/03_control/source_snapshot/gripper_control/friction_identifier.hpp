#pragma once

#include "gripper_control/types.hpp"

namespace gripper_control {

class FrictionIdentifier {
 public:
  void reset(double now_s);
  bool update(double now_s, const MotorFeedback& feedback);
  bool done() const;
  FrictionParams result() const;
  double commandVelocity(double homing_velocity_mm_s) const;

 private:
  double started_s_ = 0.0;
  bool done_ = false;
  double close_current_sum_ = 0.0;
  double open_current_sum_ = 0.0;
  int close_samples_ = 0;
  int open_samples_ = 0;
  FrictionParams result_;
};

}  // namespace gripper_control
