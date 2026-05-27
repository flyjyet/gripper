#include "hardware_interface/simulated/simulated_motor.hpp"

#include "common/error_code.hpp"

namespace gripper::hardware_interface {

namespace {

common::Result connectionNotOpen() {
  return common::Error(common::ErrorCode::ConnectionNotOpen,
                       "simulated motor is not connected");
}

}  // namespace

SimulatedMotor::SimulatedMotor(MotorId motor_id) : motor_id_(motor_id) {}

common::Result SimulatedMotor::connect() {
  connected_ = true;
  enabled_ = false;
  feedback_.enabled = false;
  feedback_.fault = fault_;
  feedback_.runtime_position_limit = {};
  feedback_.runtime_velocity_limit = {};
  feedback_.runtime_torque_limit = {};
  feedback_.runtime_limits_valid = false;
  feedback_.timestamp = nextTimestamp();
  return common::Ok();
}

common::Result SimulatedMotor::disconnect() {
  if (!connected_) {
    return connectionNotOpen();
  }

  enabled_ = false;
  connected_ = false;
  feedback_.enabled = false;
  feedback_.timestamp = nextTimestamp();
  return common::Ok();
}

common::Result SimulatedMotor::enable() {
  if (!connected_) {
    return connectionNotOpen();
  }
  if (fault_) {
    return common::Error(common::ErrorCode::HardwareFault,
                         "simulated motor is faulted");
  }

  enabled_ = true;
  feedback_.enabled = true;
  feedback_.timestamp = nextTimestamp();
  return common::Ok();
}

common::Result SimulatedMotor::disable() {
  if (!connected_) {
    return connectionNotOpen();
  }

  enabled_ = false;
  feedback_.enabled = false;
  feedback_.velocity = {};
  feedback_.current = {};
  feedback_.torque = {};
  feedback_.timestamp = nextTimestamp();
  return common::Ok();
}

common::Result SimulatedMotor::sendCommand(const MotorCommand& command) {
  if (!connected_) {
    return connectionNotOpen();
  }
  if (!enabled_) {
    return common::Error(common::ErrorCode::ControlNotReady,
                         "simulated motor is not enabled");
  }
  if (fault_) {
    return common::Error(common::ErrorCode::HardwareFault,
                         "simulated motor is faulted");
  }

  last_command_ = command;

  if (command.clear_fault) {
    fault_ = false;
  }
  if (!command.enable ||
      command.control_mode == MotorControlMode::Disabled) {
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
  } else {
    if (command.control_mode == MotorControlMode::Velocity) {
      feedback_.position =
          common::Rad{feedback_.position.value + command.target_velocity.value};
      feedback_.velocity = command.target_velocity;
    } else {
      // Position-mode velocity is a command envelope, not the settled feedback
      // velocity in this ideal simulator.
      feedback_.position = command.target_position;
      feedback_.velocity = {};
    }
    feedback_.current = command.target_current;
    feedback_.torque =
        command.target_torque.value != 0.0
            ? command.target_torque
            : common::Nm{command.target_current.value};
    feedback_.enabled = true;
  }

  feedback_.fault = fault_;
  feedback_.timestamp = nextTimestamp();
  return common::Ok();
}

common::Result SimulatedMotor::readFeedback(MotorFeedback* feedback) {
  if (feedback == nullptr) {
    return common::Error(common::ErrorCode::InvalidArgument,
                         "feedback output pointer is null");
  }
  if (!connected_) {
    return connectionNotOpen();
  }

  feedback_.fault = fault_;
  feedback_.enabled = enabled_;
  feedback_.timestamp = nextTimestamp();
  *feedback = feedback_;
  return common::Ok();
}

bool SimulatedMotor::isConnected() const noexcept { return connected_; }

bool SimulatedMotor::isEnabled() const noexcept { return enabled_; }

MotorId SimulatedMotor::motorId() const noexcept { return motor_id_; }

void SimulatedMotor::setSimulatedFault() noexcept {
  fault_ = true;
  enabled_ = false;
  feedback_.fault = true;
  feedback_.enabled = false;
}

void SimulatedMotor::clearSimulatedFault() noexcept {
  fault_ = false;
  feedback_.fault = false;
}

common::Timestamp SimulatedMotor::nextTimestamp() noexcept {
  timestamp_ns_ += 1000000;
  return common::Timestamp::fromNanoseconds(timestamp_ns_);
}

}  // namespace gripper::hardware_interface
