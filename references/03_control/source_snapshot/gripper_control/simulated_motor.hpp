#pragma once

#include "gripper_control/motor_interface.hpp"

namespace gripper_control {

struct SimulatedObject {
  double contact_stroke_mm = 8.0;
  double stiffness_A_per_mm = 0.8;
};

class SimulatedMotor : public MotorInterface {
 public:
  bool enable() override;
  bool disable() override;
  bool resetFault() override;
  bool sendCommand(const MotorCommand& command) override;
  bool readFeedback(MotorFeedback& feedback) override;
  bool setStrokeZero() override;

  void update(double now_s);
  void setObject(const SimulatedObject& object);
  void setInitialStroke(double stroke_mm);

 private:
  MotorFeedback feedback_;
  MotorCommand command_;
  SimulatedObject object_;
  double last_update_s_ = 0.0;
  double friction_close_A_ = 0.10;
  double friction_open_A_ = 0.08;
};

}  // namespace gripper_control
