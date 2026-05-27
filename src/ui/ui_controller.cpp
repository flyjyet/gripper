#include "ui/ui_controller.hpp"

#include <sstream>
#include <utility>

#include "common/error_code.hpp"
#include "config/config_loader.hpp"

namespace gripper::ui {
namespace {

[[nodiscard]] const char* encoderUnwrapSourceName(
    config::EncoderUnwrapSource source) {
  switch (source) {
    case config::EncoderUnwrapSource::ProtocolPosition:
      return "protocol_position";
    case config::EncoderUnwrapSource::RawPositionCounts:
      return "raw_position_counts";
  }
  return "unknown";
}

[[nodiscard]] std::string topStateName(
    controller::state_machine::GripperTopState state) {
  using controller::state_machine::GripperTopState;
  switch (state) {
    case GripperTopState::Disconnected:
      return "Disconnected";
    case GripperTopState::Connected:
      return "Connected";
    case GripperTopState::HardwareSanityCheck:
      return "HardwareSanityCheck";
    case GripperTopState::ModeSwitching:
      return "ModeSwitching";
    case GripperTopState::Enabled:
      return "Enabled";
    case GripperTopState::PreSelfCheck:
      return "PreSelfCheck";
    case GripperTopState::HomingOpenStop:
      return "HomingOpenStop";
    case GripperTopState::TravelLearning:
      return "TravelLearning";
    case GripperTopState::MotionHealthCheck:
      return "MotionHealthCheck";
    case GripperTopState::Ready:
      return "Ready";
    case GripperTopState::Clamping:
      return "Clamping";
    case GripperTopState::Unloading:
      return "Unloading";
    case GripperTopState::Releasing:
      return "Releasing";
    case GripperTopState::ManualPositioning:
      return "ManualPositioning";
    case GripperTopState::Disabled:
      return "Disabled";
    case GripperTopState::ActiveStop:
      return "ActiveStop";
    case GripperTopState::Fault:
      return "Fault";
  }
  return "Unknown";
}

}  // namespace

UiController::UiController(
    std::unique_ptr<controller::GripperController> controller)
    : controller_(std::move(controller)) {
  if (controller_) {
    controller_->setProgressCallback(
        [this](std::string message) { log_model_.append(std::move(message)); });
  }
}

common::Result UiController::configure(const config::GripperConfig& config) {
  const auto result = logResult("configure", controller_->configure(config));
  if (result.isOk()) {
    config_ = config;
    config_valid_ = true;
    std::ostringstream stream;
    stream << "config_summary"
           << " | encoder_unwrap_source="
           << encoderUnwrapSourceName(config.motor.encoder_unwrap_source)
           << " | encoder_wrap_range_rad="
           << config.motor.encoder_wrap_range.value
           << " | lead_screw_pitch_mm_per_rev="
           << config.mechanism.lead_screw_pitch.value
           << " | feedback_poll_period_s="
           << config.motor.feedback_poll_period.value
           << " | position_command_sign="
           << config.motor.position_command_sign
           << " | direction_sign=" << config.motor.direction_sign;
    log_model_.append(stream.str());
  }
  return result;
}

common::Result UiController::configureDefault() {
  return configure(config::defaultConfig());
}

common::Result UiController::connect() {
  return logResult("connect", controller_->connect());
}

common::Result UiController::disconnect() {
  return logResult("disconnect", controller_->disconnect());
}

common::Result UiController::enable() {
  return logResult("enable", controller_->enable());
}

common::Result UiController::disable() {
  return logResult("disable", controller_->disable());
}

common::Result UiController::enterMotorBringupMode(bool unloaded_confirmed) {
  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = unloaded_confirmed;
  request.source = controller::CommandSource::Ui;
  return logResult("bringup_enter",
                   controller_->enterMotorBringupMode(request));
}

common::Result UiController::exitMotorBringupMode() {
  return logResult("bringup_exit", controller_->exitMotorBringupMode());
}

common::Result UiController::refreshMotorBringupFeedback() {
  return logResult("bringup_feedback",
                   controller_->refreshMotorBringupFeedback());
}

common::Result UiController::runMotorBringupCommunicationProbe() {
  std::vector<std::string> lines{};
  const auto result = controller_->runMotorBringupCommunicationProbe(&lines);
  for (const auto& line : lines) {
    log_model_.append(std::string{"bringup_can_probe | "} + line);
  }
  return logResult("bringup_can_probe", result);
}

common::Result UiController::enableMotorBringupOutput() {
  return logResult("bringup_enable",
                   controller_->enableMotorBringupOutput());
}

common::Result UiController::disableMotorBringupOutput() {
  return logResult("bringup_disable",
                   controller_->disableMotorBringupOutput());
}

common::Result UiController::jogMotorBringup(
    common::Rad relative_motor_position, common::RadPerS max_motor_velocity,
    common::A max_motor_current, common::S pulse_duration) {
  controller::MotorBringupJogCommand command{};
  command.relative_motor_position = relative_motor_position;
  command.motor_direction_sign = relative_motor_position.value < 0.0 ? -1 : 1;
  command.max_motor_velocity = max_motor_velocity;
  command.max_motor_current = max_motor_current;
  command.pulse_duration = pulse_duration;
  command.source = controller::CommandSource::Ui;
  std::ostringstream action;
  action << "bringup_jog"
         << " direction_window_rad=" << relative_motor_position.value
         << " vel_rad_s=" << max_motor_velocity.value
         << " feedback_current_stop_a=" << max_motor_current.value
         << " pulse_s=" << pulse_duration.value;
  return logResult(action.str(), controller_->jogMotorBringup(command));
}

common::Result UiController::moveMotorBringupRelativeTurns(
    common::Ratio relative_motor_revolutions,
    common::RadPerS max_motor_velocity,
    common::A feedback_current_limit, common::S timeout) {
  controller::MotorBringupPositionMoveCommand command{};
  command.relative_motor_revolutions = relative_motor_revolutions;
  command.max_motor_velocity = max_motor_velocity;
  command.feedback_current_limit = feedback_current_limit;
  command.timeout = timeout;
  command.source = controller::CommandSource::Ui;
  std::ostringstream action;
  action << "bringup_turns"
         << " relative_rev=" << relative_motor_revolutions.value
         << " max_vel_rad_s=" << max_motor_velocity.value
         << " feedback_current_stop_a=" << feedback_current_limit.value
         << " timeout_s=" << timeout.value;
  return logResult(action.str(),
                   controller_->moveMotorBringupRelativeTurns(command));
}

common::Result UiController::runPreSelfCheck() {
  return logResult("pre_self_check", controller_->runPreSelfCheck());
}

common::Result UiController::home() {
  return logResult("home", controller_->homeOpenStop());
}

common::Result UiController::learnTravelLimits() {
  return logResult("learn_limits", controller_->learnTravelLimits());
}

common::Result UiController::runMotionHealthCheck() {
  return logResult("health_check", controller_->runMotionHealthCheck());
}

common::Result UiController::clampForce(common::N target_force,
                                        common::MmPerS speed) {
  controller::ClampForceCommand command{};
  command.target_force = target_force;
  command.speed_mode = controller::ClampSpeedMode::NutLinearSpeed;
  command.max_nut_speed = speed;
  command.source = controller::CommandSource::Ui;
  return logResult("clamp_force", controller_->clampByForce(command));
}

common::Result UiController::release() {
  return logResult("release", controller_->release());
}

common::Result UiController::moveNutStroke(common::Mm target_nut_stroke,
                                           common::MmPerS speed,
                                           bool unloaded_confirmed) {
  controller::MoveNutStrokeCommand command{};
  command.target_nut_stroke = target_nut_stroke;
  command.max_nut_speed = speed;
  command.unloaded_or_structure_removed_confirmed = unloaded_confirmed;
  command.source = controller::CommandSource::Ui;
  std::ostringstream action;
  action << "move_nut_stroke target_mm=" << target_nut_stroke.value
         << " speed_mm_s=" << speed.value
         << " unloaded_confirmed=" << (unloaded_confirmed ? 1 : 0);
  return logResult(action.str(), controller_->moveToNutStroke(command));
}

common::Result UiController::stop() {
  return logResult("stop", controller_->stop());
}

common::Result UiController::clearFault() {
  return logResult("clear_fault", controller_->clearFault());
}

common::Result UiController::update() {
  return controller_->update();
}

void UiController::appendLog(std::string message) {
  log_model_.append(std::move(message));
}

UiViewModel UiController::viewModel() const {
  UiViewModel view{};
  view.state = controller_->state();
  view.status_text = topStateName(view.state.top_state);
  view.can_connect = !view.state.connected;
  const bool normal_connected =
      view.state.connected && !view.state.motor_bringup_active;
  const bool terminal =
      view.state.top_state == controller::state_machine::GripperTopState::Fault ||
      view.state.top_state ==
          controller::state_machine::GripperTopState::ActiveStop;
  view.can_enable = normal_connected && !view.state.enabled && !terminal;
  view.can_run_self_check =
      normal_connected && !terminal &&
      !view.state.pre_self_check_completed;
  view.can_home =
      normal_connected && !terminal &&
      view.state.pre_self_check_completed && !view.state.homed;
  view.can_learn_limits =
      normal_connected && !terminal && view.state.homed &&
      !view.state.travel_limits_learned;
  view.can_health_check =
      normal_connected && !terminal &&
      view.state.travel_limits_learned &&
      !view.state.motion_health_checked;
  view.can_clamp =
      normal_connected && !terminal && view.state.motion_health_checked;
  view.can_release = normal_connected && !terminal;
  view.can_move_nut_stroke =
      normal_connected && !terminal && view.state.pre_self_check_completed;
  view.can_motor_bringup_move =
      view.state.connected && view.state.motor_bringup_active && !terminal;
  view.can_recover_active_stop =
      view.state.top_state == controller::state_machine::GripperTopState::ActiveStop;
  return view;
}

const RuntimeLogModel& UiController::logModel() const noexcept {
  return log_model_;
}

bool UiController::hasConfig() const noexcept {
  return config_valid_;
}

const config::GripperConfig& UiController::config() const noexcept {
  return config_;
}

common::Result UiController::logResult(const std::string& action,
                                       common::Result result) {
  std::ostringstream stream;
  stream << action << ": " << common::toString(result.code());
  if (result.hasMessage()) {
    stream << " | " << result.message();
  }
  const auto state = controller_->state();
  stream << " | controller_state=" << topStateName(state.top_state)
         << " | stroke_mm=" << state.nut_stroke.value
         << " | motor_virtual_pos_rad=" << state.motor.position.value;
  stream << " | motor_wrapped_pos_rad=";
  if (state.motor.wrapped_position_valid) {
    stream << state.motor.wrapped_position.value;
  } else {
    stream << "-";
  }
  stream << " | motor_raw_pos_counts=";
  if (state.motor.raw_position_counts_valid) {
    stream << state.motor.raw_position_counts;
  } else {
    stream << "-";
  }
  stream
         << " | motor_vel_rad_s=" << state.motor.velocity.value
         << " | motor_current_a=" << state.motor.current.value
         << " | motor_torque_nm=" << state.motor.torque.value
         << " | force_n=" << state.estimated_clamp_force.value
         << " | motor_enabled=" << (state.enabled ? "true" : "false");
  log_model_.append(stream.str());
  return result;
}

}  // namespace gripper::ui
