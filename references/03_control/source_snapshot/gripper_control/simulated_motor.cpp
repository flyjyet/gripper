#include "gripper_control/simulated_motor.hpp"

#include <algorithm>
#include <cmath>

namespace gripper_control {

bool SimulatedMotor::enable() {
  feedback_.drive_enabled = true;
  return true;
}

bool SimulatedMotor::disable() {
  feedback_.drive_enabled = false;
  feedback_.velocity_mm_s = 0.0;
  feedback_.current_A = 0.0;
  command_ = {};
  return true;
}

bool SimulatedMotor::resetFault() {
  feedback_.fault_bits = 0;
  feedback_.encoder_ok = true;
  return true;
}

bool SimulatedMotor::sendCommand(const MotorCommand& command) {
  command_ = command;
  return true;
}

bool SimulatedMotor::readFeedback(MotorFeedback& feedback) {
  feedback = feedback_;
  return true;
}

bool SimulatedMotor::setStrokeZero() {
  feedback_.stroke_mm = 0.0;
  return true;
}

void SimulatedMotor::update(double now_s) {
  if (last_update_s_ == 0.0) {
    last_update_s_ = now_s;
    feedback_.timestamp_s = now_s;
    return;
  }
  const double dt = std::clamp(now_s - last_update_s_, 0.0, 0.1);
  last_update_s_ = now_s;
  feedback_.timestamp_s = now_s;

  if (!feedback_.drive_enabled || command_.mode == MotorMode::Disabled) {
    feedback_.velocity_mm_s = 0.0;
    feedback_.current_A = 0.0;
    feedback_.torque_Nm = 0.0;
    return;
  }

  const double contact_depth = std::max(0.0, feedback_.stroke_mm - object_.contact_stroke_mm);
  const double contact_current = contact_depth * object_.stiffness_A_per_mm;
  double velocity = 0.0;
  double current = 0.0;

  if (command_.mode == MotorMode::Velocity) {
    const bool closing = command_.target_velocity_mm_s >= 0.0;
    const double friction = closing ? friction_close_A_ : friction_open_A_;
    const double demanded_current = friction + (closing ? contact_current : 0.0);
    current = std::min(std::abs(command_.current_limit_A), demanded_current);
    velocity = demanded_current > std::abs(command_.current_limit_A)
                   ? 0.0
                   : command_.target_velocity_mm_s;
  } else if (command_.mode == MotorMode::Current || command_.mode == MotorMode::Torque) {
    const double sign = command_.target_current_A >= 0.0 ? 1.0 : -1.0;
    current = std::abs(command_.target_current_A);
    const double friction = sign >= 0.0 ? friction_close_A_ : friction_open_A_;
    const double margin = current - friction - (sign >= 0.0 ? contact_current : 0.0);
    velocity = margin > 0.0 ? sign * 0.08 * margin : 0.0;
  }

  feedback_.stroke_mm = std::clamp(feedback_.stroke_mm + velocity * dt, 0.0, 16.0);
  if (feedback_.stroke_mm <= 0.0 || feedback_.stroke_mm >= 16.0) {
    velocity = 0.0;
  }
  feedback_.velocity_mm_s = velocity;
  feedback_.current_A = current;
  feedback_.torque_Nm = current * 0.55;
  feedback_.temperature_C += std::max(0.0, current * current * 0.002 * dt - 0.01 * dt);
}

void SimulatedMotor::setObject(const SimulatedObject& object) { object_ = object; }

void SimulatedMotor::setInitialStroke(double stroke_mm) {
  feedback_.stroke_mm = std::clamp(stroke_mm, 0.0, 16.0);
}

}  // namespace gripper_control
