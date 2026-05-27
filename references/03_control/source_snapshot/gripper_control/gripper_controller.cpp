#include "gripper_control/gripper_controller.hpp"

#include <algorithm>
#include <cmath>

namespace gripper_control {

namespace {
constexpr double kDefaultUnloadMm = 0.05;
constexpr double kForceRampTimeS = 0.8;
constexpr double kUnloadTimeoutS = 0.6;
constexpr double kUnlockProbeTimeS = 0.35;
constexpr double kAntiJamAttemptTimeS = 0.8;
}  // namespace

GripperController::GripperController(std::shared_ptr<MotorInterface> motor)
    : motor_(std::move(motor)), limiter_(), detector_(), force_mapper_() {
  status_.state = GripperState::Disabled;
  status_.fault = GripperFault::None;
}

void GripperController::setLimits(const GripperLimits& limits) {
  limiter_.setLimits(limits);
}

void GripperController::setJamDetectionParams(const JamDetectionParams& params) {
  detector_.setParams(params);
}

void GripperController::setForceMapParams(const ForceMapParams& params) {
  force_mapper_.setParams(params);
}

bool GripperController::initialize(double now_s) {
  status_.fault = GripperFault::None;
  status_.fault_text.clear();
  if (!motor_->enable()) {
    enterFault(GripperFault::DriveFault, "Motor enable failed", now_s);
    return false;
  }
  enterState(GripperState::PowerOnSelfCheck, now_s);
  return true;
}

bool GripperController::clampForce(const ClampCommand& command, double now_s) {
  if (status_.state != GripperState::Ready && status_.state != GripperState::ClampDoneDisabled) {
    return false;
  }
  active_clamp_ = command;
  if (!motor_->enable()) {
    enterFault(GripperFault::DriveFault, "Motor enable failed", now_s);
    return false;
  }
  contact_stroke_mm_ = feedback_.stroke_mm;
  target_current_A_ = 0.0;
  detector_.reset();
  enterState(GripperState::GuardedClosing, now_s);
  return true;
}

bool GripperController::open(const OpenCommand& command, double now_s) {
  active_open_ = command;
  if (!motor_->enable()) {
    enterFault(GripperFault::DriveFault, "Motor enable failed", now_s);
    return false;
  }
  detector_.reset();
  enterState(GripperState::UnlockBeforeOpen, now_s);
  return true;
}

bool GripperController::stop() {
  MotorCommand command;
  command.mode = MotorMode::Disabled;
  const bool sent = motor_->sendCommand(command);
  motor_->disable();
  status_.state = GripperState::Disabled;
  return sent;
}

bool GripperController::disable() {
  const bool ok = motor_->disable();
  status_.state = GripperState::Disabled;
  return ok;
}

bool GripperController::resetFault() {
  status_.fault = GripperFault::None;
  status_.fault_text.clear();
  return motor_->resetFault();
}

void GripperController::update(double now_s) {
  if (!readFeedback()) {
    enterFault(GripperFault::FeedbackTimeout, "Failed to read motor feedback", now_s);
    return;
  }

  if (!feedback_.encoder_ok) {
    enterFault(GripperFault::EncoderFault, "Encoder feedback invalid", now_s);
    return;
  }
  if (feedback_.fault_bits != 0) {
    enterFault(GripperFault::DriveFault, "Drive reports fault bits", now_s);
    return;
  }
  if (limiter_.isTemperatureFault(feedback_.temperature_C)) {
    enterFault(GripperFault::OverTemperature, "Motor temperature over limit", now_s);
    return;
  }
  if (!limiter_.isStrokeInsideSoftLimit(feedback_.stroke_mm)) {
    enterFault(GripperFault::PositionLimit, "Stroke outside soft limit", now_s);
    return;
  }

  switch (status_.state) {
    case GripperState::PowerOnSelfCheck:
      updatePowerOnSelfCheck(now_s);
      break;
    case GripperState::HomingOpenStop:
      updateHoming(now_s);
      break;
    case GripperState::FrictionIdentify:
      updateFrictionIdentify(now_s);
      break;
    case GripperState::GuardedClosing:
      updateGuardedClosing(now_s);
      break;
    case GripperState::ForceBuild:
      updateForceBuild(now_s);
      break;
    case GripperState::UnloadBeforeDisable:
      updateUnloadBeforeDisable(now_s);
      break;
    case GripperState::UnlockBeforeOpen:
      updateUnlockBeforeOpen(now_s);
      break;
    case GripperState::Opening:
      updateOpening(now_s);
      break;
    case GripperState::AntiJamRelease:
      updateAntiJamRelease(now_s);
      break;
    case GripperState::Disabled:
    case GripperState::Ready:
    case GripperState::ContactDetected:
    case GripperState::ClampDoneDisabled:
    case GripperState::FaultStop:
      break;
  }

  status_.state = status_.state;
  status_.stroke_mm = feedback_.stroke_mm;
  status_.velocity_mm_s = feedback_.velocity_mm_s;
  status_.current_A = feedback_.current_A;
  status_.torque_Nm = feedback_.torque_Nm;
  status_.temperature_C = feedback_.temperature_C;
  status_.motor_enabled = feedback_.drive_enabled;
  status_.estimated_force_per_side_N =
      force_mapper_.estimateForceFromCurrent(feedback_.current_A, contact_stroke_mm_, friction_);
}

GripperStatus GripperController::getStatus() const { return status_; }

void GripperController::enterState(GripperState state, double now_s) {
  status_.state = state;
  state_started_s_ = now_s;
  last_commanded_velocity_mm_s_ = 0.0;
}

void GripperController::enterFault(GripperFault fault, const std::string& text, double now_s) {
  (void)now_s;
  status_.fault = fault;
  status_.fault_text = text;
  status_.state = GripperState::FaultStop;
  MotorCommand command;
  command.mode = MotorMode::Disabled;
  motor_->sendCommand(command);
  motor_->disable();
}

bool GripperController::readFeedback() {
  if (!motor_) {
    return false;
  }
  const bool ok = motor_->readFeedback(feedback_);
  if (ok) {
    status_.stroke_mm = feedback_.stroke_mm;
    status_.velocity_mm_s = feedback_.velocity_mm_s;
    status_.current_A = feedback_.current_A;
    status_.torque_Nm = feedback_.torque_Nm;
    status_.temperature_C = feedback_.temperature_C;
    status_.motor_enabled = feedback_.drive_enabled;
  }
  return ok;
}

bool GripperController::sendVelocity(double velocity_mm_s,
                                     double velocity_limit_mm_s,
                                     double current_limit_A) {
  const auto command =
      limiter_.velocityCommand(velocity_mm_s, velocity_limit_mm_s, current_limit_A, current_limit_A);
  last_commanded_velocity_mm_s_ = command.target_velocity_mm_s;
  return motor_->sendCommand(command);
}

bool GripperController::sendCurrent(double current_A, double current_limit_A) {
  const auto command = limiter_.currentCommand(current_A, current_limit_A);
  return motor_->sendCommand(command);
}

double GripperController::elapsed(double now_s) const { return now_s - state_started_s_; }

void GripperController::updatePowerOnSelfCheck(double now_s) {
  if (elapsed(now_s) >= 0.05) {
    enterState(GripperState::HomingOpenStop, now_s);
  }
}

void GripperController::updateHoming(double now_s) {
  const auto& limits = limiter_.limits();
  sendVelocity(-limits.v_homing_max_mm_s, limits.v_homing_max_mm_s, limits.current_homing_max_A);

  const bool at_open_stop = feedback_.stroke_mm <= limits.stroke_min_mm + 0.001 &&
                            elapsed(now_s) > 0.2;
  if (at_open_stop) {
    motor_->setStrokeZero();
    status_.homed = true;
    enterState(GripperState::FrictionIdentify, now_s);
    friction_identifier_.reset(now_s);
  }
  if (elapsed(now_s) > 5.0) {
    enterFault(GripperFault::HomingFailed, "Low-current homing timeout", now_s);
  }
}

void GripperController::updateFrictionIdentify(double now_s) {
  const auto& limits = limiter_.limits();
  const double t = elapsed(now_s);
  const double velocity = (t < 1.1) ? limits.v_homing_max_mm_s : -limits.v_homing_max_mm_s;
  sendVelocity(velocity, limits.v_homing_max_mm_s, limits.current_selfcheck_max_A);

  if (friction_identifier_.update(now_s, feedback_)) {
    friction_ = friction_identifier_.result();
    status_.friction_identified = friction_.valid;
    motor_->sendCommand(MotorCommand{});
    enterState(GripperState::Ready, now_s);
  }
}

void GripperController::updateGuardedClosing(double now_s) {
  const auto& limits = limiter_.limits();
  const double state_velocity_limit = active_clamp_.known_cable_mode
                                          ? limits.v_close_known_max_mm_s
                                          : limits.v_close_unknown_max_mm_s;
  const double requested_velocity =
      std::min(active_clamp_.close_speed_mm_s, state_velocity_limit);
  const double current_limit = std::min(active_clamp_.max_current_A, limits.current_close_guard_max_A);
  sendVelocity(requested_velocity, state_velocity_limit, current_limit);

  const auto result = detector_.update(now_s, feedback_, last_commanded_velocity_mm_s_, friction_,
                                       true, true);
  if (result.contact || result.stall_kind == StallKind::Contact) {
    contact_stroke_mm_ = feedback_.stroke_mm;
    motor_->sendCommand(MotorCommand{});
    enterState(GripperState::ForceBuild, now_s);
    return;
  }

  if (feedback_.stroke_mm >= limits.stroke_max_mm - 0.001) {
    enterFault(GripperFault::ContactNotFound, "Close soft limit reached before contact", now_s);
    return;
  }
  if (elapsed(now_s) > active_clamp_.timeout_s) {
    enterFault(GripperFault::ContactNotFound, "Clamp contact search timeout", now_s);
  }
}

void GripperController::updateForceBuild(double now_s) {
  const auto& limits = limiter_.limits();
  const double target_force =
      limiter_.clampTargetForce(active_clamp_.target_force_per_side_N, force_mapper_.params());
  const double target_current =
      force_mapper_.targetCurrentForForce(target_force, contact_stroke_mm_, friction_);
  target_current_A_ = std::min(target_current, active_clamp_.max_current_A);
  const double ramp = std::clamp(elapsed(now_s) / kForceRampTimeS, 0.0, 1.0);
  sendCurrent(target_current_A_ * ramp, limits.current_close_guard_max_A);

  if (elapsed(now_s) >= kForceRampTimeS + 0.2) {
    unload_target_stroke_mm_ =
        std::max(limits.stroke_min_mm, feedback_.stroke_mm - kDefaultUnloadMm);
    enterState(GripperState::UnloadBeforeDisable, now_s);
  }
}

void GripperController::updateUnloadBeforeDisable(double now_s) {
  const auto& limits = limiter_.limits();
  sendVelocity(-limits.v_unlock_max_mm_s, limits.v_unlock_max_mm_s, limits.current_unlock_max_A);
  if (feedback_.stroke_mm <= unload_target_stroke_mm_ || elapsed(now_s) > kUnloadTimeoutS) {
    motor_->disable();
    enterState(GripperState::ClampDoneDisabled, now_s);
  }
}

void GripperController::updateUnlockBeforeOpen(double now_s) {
  const auto& limits = limiter_.limits();
  sendVelocity(-limits.v_unlock_max_mm_s, limits.v_unlock_max_mm_s, limits.current_unlock_max_A);
  const bool moving = std::abs(feedback_.velocity_mm_s) > 0.005;
  if (moving || elapsed(now_s) > kUnlockProbeTimeS) {
    detector_.reset();
    enterState(GripperState::Opening, now_s);
  }
}

void GripperController::updateOpening(double now_s) {
  const auto& limits = limiter_.limits();
  const double speed = std::min(active_open_.open_speed_mm_s, limits.v_open_max_mm_s);
  sendVelocity(-speed, limits.v_open_max_mm_s, active_open_.max_current_A);

  const auto result = detector_.update(now_s, feedback_, last_commanded_velocity_mm_s_, friction_,
                                       false, false);
  if (result.stall_kind == StallKind::AbnormalJam) {
    anti_jam_attempt_ = 0;
    enterState(GripperState::AntiJamRelease, now_s);
    return;
  }
  if (feedback_.stroke_mm <= limits.stroke_min_mm + 0.001) {
    motor_->setStrokeZero();
    motor_->sendCommand(MotorCommand{});
    enterState(GripperState::Ready, now_s);
  }
  if (elapsed(now_s) > active_open_.timeout_s) {
    enterFault(GripperFault::AbnormalJam, "Opening timeout", now_s);
  }
}

void GripperController::updateAntiJamRelease(double now_s) {
  const auto& limits = limiter_.limits();
  sendVelocity(-limits.v_unlock_max_mm_s, limits.v_unlock_max_mm_s, limits.current_anti_jam_max_A);
  if (std::abs(feedback_.velocity_mm_s) > 0.005) {
    enterState(GripperState::Opening, now_s);
    return;
  }
  if (elapsed(now_s) > kAntiJamAttemptTimeS) {
    ++anti_jam_attempt_;
    if (anti_jam_attempt_ >= 3) {
      enterFault(GripperFault::AntiJamFailed, "Anti-jam release attempts exhausted", now_s);
    } else {
      enterState(GripperState::AntiJamRelease, now_s);
    }
  }
}

}  // namespace gripper_control
