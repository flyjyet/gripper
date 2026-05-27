#pragma once

#include "gripper_control/types.hpp"

namespace gripper_control {

class MotorInterface {
 public:
  virtual ~MotorInterface() = default;

  virtual bool enable() = 0;
  virtual bool disable() = 0;
  virtual bool resetFault() = 0;
  virtual bool sendCommand(const MotorCommand& command) = 0;
  virtual bool readFeedback(MotorFeedback& feedback) = 0;

  // Set current encoder position as gripper stroke zero after low-current homing.
  virtual bool setStrokeZero() = 0;
};

}  // namespace gripper_control
