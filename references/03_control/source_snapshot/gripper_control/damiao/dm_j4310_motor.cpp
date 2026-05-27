#include "gripper_control/damiao/dm_j4310_motor.hpp"

#include <algorithm>
#include <cmath>

namespace gripper_control::damiao {

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool isFault(DmErrorCode error) {
  return error != DmErrorCode::Disabled && error != DmErrorCode::Enabled;
}

}  // namespace

DmJ4310Motor::DmJ4310Motor(std::shared_ptr<CanBus> bus, DmJ4310Config config)
    : bus_(std::move(bus)), config_(config), start_time_(std::chrono::steady_clock::now()) {}

bool DmJ4310Motor::enable() {
  if (!bus_) {
    return false;
  }
  if (!bus_->send(packEnable(config_.motor_id))) {
    return false;
  }
  enabled_commanded_ = true;
  return true;
}

bool DmJ4310Motor::disable() {
  if (!bus_) {
    return false;
  }
  sendStopForCurrentMode();
  const bool ok = bus_->send(packDisable(config_.motor_id));
  enabled_commanded_ = false;
  last_feedback_.drive_enabled = false;
  return ok;
}

bool DmJ4310Motor::resetFault() {
  if (!bus_) {
    return false;
  }
  return bus_->send(packClearError(config_.motor_id));
}

bool DmJ4310Motor::sendCommand(const MotorCommand& command) {
  if (!bus_) {
    return false;
  }
  if (command.mode == MotorMode::Disabled) {
    return sendStopForCurrentMode();
  }

  if (command.mode == MotorMode::Velocity) {
    const double velocity_rad_s = velocityMmSToOutputRadS(command.target_velocity_mm_s);
    if (config_.use_pvt_for_current_limit && command.current_limit_A > 0.0) {
      if (config_.switch_mode_on_command && current_mode_ != DmControlMode::PositionForce &&
          !switchMode(DmControlMode::PositionForce)) {
        return false;
      }
      const double target_rad =
          strokeToOutputRad(command.target_velocity_mm_s >= 0.0 ? 16.0 : 0.0);
      return bus_->send(packPositionForceCommand(
          config_.motor_id, target_rad, std::abs(velocity_rad_s),
          currentLimitNormalized(command.current_limit_A)));
    }
    if (config_.switch_mode_on_command && current_mode_ != DmControlMode::Velocity &&
        !switchMode(DmControlMode::Velocity)) {
      return false;
    }
    return bus_->send(packVelocityCommand(config_.motor_id, velocity_rad_s));
  }

  if (command.mode == MotorMode::Position) {
    if (config_.switch_mode_on_command && current_mode_ != DmControlMode::PositionVelocity &&
        !switchMode(DmControlMode::PositionVelocity)) {
      return false;
    }
    return bus_->send(packPositionVelocityCommand(
        config_.motor_id, strokeToOutputRad(command.target_position_mm),
        std::abs(velocityMmSToOutputRadS(command.target_velocity_mm_s))));
  }

  if (command.mode == MotorMode::Current || command.mode == MotorMode::Torque) {
    if (config_.switch_mode_on_command && current_mode_ != DmControlMode::Mit &&
        !switchMode(DmControlMode::Mit)) {
      return false;
    }
    const double torque = command.mode == MotorMode::Torque
                              ? command.target_torque_Nm
                              : currentToOutputTorque(command.target_current_A);
    return bus_->send(packMitCommand(config_.motor_id, 0.0, 0.0, 0.0, 0.0, torque,
                                     config_.mapping));
  }

  return false;
}

bool DmJ4310Motor::readFeedback(MotorFeedback& feedback) {
  DmFeedback dm_feedback;
  const int timeout_ms =
      std::max(1, static_cast<int>(std::round(config_.feedback_timeout_s * 1000.0)));
  if (!waitForFeedback(dm_feedback, timeout_ms)) {
    return false;
  }
  applyParsedFeedback(dm_feedback, feedback);
  last_feedback_ = feedback;
  return true;
}

bool DmJ4310Motor::setStrokeZero() {
  zero_output_rad_ = last_output_rad_;
  last_feedback_.stroke_mm = 0.0;
  have_zero_ = true;
  return true;
}

bool DmJ4310Motor::switchMode(DmControlMode mode) {
  if (!writeRegisterUint32(DmRegister::ControlMode, static_cast<std::uint32_t>(mode))) {
    return false;
  }
  current_mode_ = mode;
  return true;
}

bool DmJ4310Motor::readRegisterFloat(DmRegister reg, float& value) {
  if (!bus_ || !bus_->send(packRegisterRead(config_.motor_id, reg))) {
    return false;
  }
  CanFrame frame;
  const int timeout_ms =
      std::max(1, static_cast<int>(std::round(config_.feedback_timeout_s * 1000.0)));
  while (bus_->receive(frame, timeout_ms)) {
    if (parseRegisterReply(frame, config_.motor_id, 0x33, reg, value)) {
      return true;
    }
  }
  return false;
}

bool DmJ4310Motor::readRegisterUint32(DmRegister reg, std::uint32_t& value) {
  if (!bus_ || !bus_->send(packRegisterRead(config_.motor_id, reg))) {
    return false;
  }
  CanFrame frame;
  const int timeout_ms =
      std::max(1, static_cast<int>(std::round(config_.feedback_timeout_s * 1000.0)));
  while (bus_->receive(frame, timeout_ms)) {
    if (parseRegisterReply(frame, config_.motor_id, 0x33, reg, value)) {
      return true;
    }
  }
  return false;
}

bool DmJ4310Motor::writeRegisterFloat(DmRegister reg, float value) {
  if (!bus_ || !bus_->send(packRegisterWriteFloat(config_.motor_id, reg, value))) {
    return false;
  }
  float echoed = 0.0F;
  CanFrame frame;
  const int timeout_ms =
      std::max(1, static_cast<int>(std::round(config_.feedback_timeout_s * 1000.0)));
  while (bus_->receive(frame, timeout_ms)) {
    if (parseRegisterReply(frame, config_.motor_id, 0x55, reg, echoed)) {
      return true;
    }
  }
  return false;
}

bool DmJ4310Motor::writeRegisterUint32(DmRegister reg, std::uint32_t value) {
  if (!bus_ || !bus_->send(packRegisterWriteUint32(config_.motor_id, reg, value))) {
    return false;
  }
  std::uint32_t echoed = 0;
  CanFrame frame;
  const int timeout_ms =
      std::max(1, static_cast<int>(std::round(config_.feedback_timeout_s * 1000.0)));
  while (bus_->receive(frame, timeout_ms)) {
    if (parseRegisterReply(frame, config_.motor_id, 0x55, reg, echoed)) {
      return true;
    }
  }
  return false;
}

const DmJ4310Config& DmJ4310Motor::config() const { return config_; }

double DmJ4310Motor::nowSeconds() const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
}

std::uint32_t DmJ4310Motor::specialCommandId() const {
  switch (current_mode_) {
    case DmControlMode::PositionVelocity:
      return 0x100 + config_.motor_id;
    case DmControlMode::Velocity:
      return 0x200 + config_.motor_id;
    case DmControlMode::PositionForce:
      return 0x300 + config_.motor_id;
    case DmControlMode::Mit:
    default:
      return config_.motor_id;
  }
}

double DmJ4310Motor::strokeToOutputRad(double stroke_mm) const {
  return zero_output_rad_ + config_.stroke_sign * stroke_mm * kTwoPi / config_.screw_lead_mm;
}

double DmJ4310Motor::outputRadToStroke(double output_rad) const {
  return config_.stroke_sign * (output_rad - zero_output_rad_) * config_.screw_lead_mm / kTwoPi;
}

double DmJ4310Motor::velocityMmSToOutputRadS(double velocity_mm_s) const {
  return config_.stroke_sign * velocity_mm_s * kTwoPi / config_.screw_lead_mm;
}

double DmJ4310Motor::outputRadSToVelocityMmS(double velocity_rad_s) const {
  return config_.stroke_sign * velocity_rad_s * config_.screw_lead_mm / kTwoPi;
}

double DmJ4310Motor::currentToOutputTorque(double current_A) const {
  return current_A * config_.torque_per_amp_Nm;
}

double DmJ4310Motor::currentLimitNormalized(double current_A) const {
  if (config_.max_phase_current_A <= 0.0) {
    return 0.0;
  }
  return std::clamp(std::abs(current_A) / config_.max_phase_current_A, 0.0, 1.0);
}

bool DmJ4310Motor::sendStopForCurrentMode() {
  switch (current_mode_) {
    case DmControlMode::PositionForce:
      return bus_->send(packPositionForceCommand(
          config_.motor_id, strokeToOutputRad(last_feedback_.stroke_mm), 0.0, 0.0));
    case DmControlMode::PositionVelocity:
      return bus_->send(packPositionVelocityCommand(config_.motor_id,
                                                    strokeToOutputRad(last_feedback_.stroke_mm),
                                                    0.0));
    case DmControlMode::Mit:
      return bus_->send(
          packMitCommand(config_.motor_id, 0.0, 0.0, 0.0, 0.0, 0.0, config_.mapping));
    case DmControlMode::Velocity:
    default:
      return bus_->send(packVelocityCommand(config_.motor_id, 0.0));
  }
}

bool DmJ4310Motor::waitForFeedback(DmFeedback& feedback, int timeout_ms) {
  CanFrame frame;
  while (bus_->receive(frame, timeout_ms)) {
    if (parseFeedback(frame, config_.master_id, config_.mapping, feedback) &&
        feedback.motor_id == (config_.motor_id & 0x0F)) {
      return true;
    }
  }
  return false;
}

void DmJ4310Motor::applyParsedFeedback(const DmFeedback& dm_feedback, MotorFeedback& feedback) {
  last_output_rad_ = dm_feedback.position_rad;
  if (!have_zero_) {
    zero_output_rad_ = dm_feedback.position_rad;
    have_zero_ = true;
  }
  feedback.timestamp_s = nowSeconds();
  feedback.stroke_mm = outputRadToStroke(dm_feedback.position_rad);
  feedback.velocity_mm_s = outputRadSToVelocityMmS(dm_feedback.velocity_rad_s);
  feedback.torque_Nm = dm_feedback.torque_Nm;
  feedback.current_A =
      config_.torque_per_amp_Nm > 0.0 ? dm_feedback.torque_Nm / config_.torque_per_amp_Nm : 0.0;
  feedback.temperature_C = std::max(dm_feedback.mos_temperature_C, dm_feedback.rotor_temperature_C);
  feedback.fault_bits = mapDriveFault(dm_feedback.error);
  feedback.encoder_ok = true;
  feedback.drive_enabled = dm_feedback.error == DmErrorCode::Enabled;
  if (!isFault(dm_feedback.error) && enabled_commanded_) {
    feedback.drive_enabled = true;
  }
}

std::uint32_t DmJ4310Motor::mapDriveFault(DmErrorCode error) const {
  return isFault(error) ? (1U << (static_cast<std::uint8_t>(error) & 0x0F)) : 0U;
}

}  // namespace gripper_control::damiao
