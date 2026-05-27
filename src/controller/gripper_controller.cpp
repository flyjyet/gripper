#include "controller/gripper_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/error_code.hpp"
#include "config/gripper_config.hpp"
#include "controller/calibration/force_mapper.hpp"
#include "controller/mechanism/gripper_kinematics.hpp"
#include "controller/nut_position_encoder/nut_position_encoder.hpp"
#include "controller/safety/contact_jam_detector.hpp"
#include "controller/safety/safety_limiter.hpp"
#include "controller/self_check/friction_anomaly_map.hpp"
#include "controller/self_check/self_check_manager.hpp"
#include "controller/state_machine/gripper_state_machine.hpp"
#include "controller/state_machine/self_check_state.hpp"
#include "hardware_interface/motor_interface.hpp"

// Business-level gripper controller implementation.
//
// This file coordinates the control workflow and safety gates. It deliberately
// depends only on MotorInterface, not on Damiao SDK, CAN frames, UI, commander,
// or the simulated motor. The app composition layer chooses the concrete motor.
//
// The mechanism has no jaw-side position or force sensor. Motor position,
// velocity, current, torque, and temperature are therefore the only runtime
// observations available for self-check, contact detection, and force proxies.
namespace gripper::controller {
namespace {

using hardware_interface::MotorCommand;
using hardware_interface::MotorControlMode;
using hardware_interface::MotorFeedback;
using nut_position_encoder::LeadScrewNutPositionEncoder;
using nut_position_encoder::NutPositionEncoderConfig;
using state_machine::GripperEvent;
using state_machine::GripperTopState;
using state_machine::PreSelfCheckEvent;

constexpr double kTwoPi = 6.28318530717958647692;
constexpr std::size_t kPreBCurrentTraceMaxPoints = 1000U;

[[nodiscard]] common::Result rejectedTransition(const char* event_name) {
  return common::Result::error(common::ErrorCode::StateMachineRejected,
                               std::string{"state machine rejected event: "} +
                                   event_name);
}

[[nodiscard]] double absValue(double value) {
  return std::abs(value);
}

[[nodiscard]] common::Rad strokeToMotorPosition(common::Mm stroke,
                                                common::Mm pitch,
                                                int direction_sign) {
  if (pitch.value == 0.0) {
    return {};
  }
  return common::Rad{static_cast<double>(direction_sign) * stroke.value *
                     kTwoPi / pitch.value};
}

[[nodiscard]] common::Rad motorPositionDelta(common::Rad position,
                                             common::Rad reference) {
  return common::Rad{position.value - reference.value};
}

// First-order screw mapping from motor angle to nut stroke. This does not model
// backlash, compliance, or jaw-side deflection because those cannot be observed
// directly with the current sensor set.
[[nodiscard]] common::Mm motorPositionToStroke(common::Rad position,
                                               common::Mm pitch,
                                               int direction_sign) {
  if (direction_sign == 0) {
    direction_sign = 1;
  }
  return common::Mm{position.value * pitch.value /
                    (kTwoPi * static_cast<double>(direction_sign))};
}

[[nodiscard]] NutPositionEncoderConfig makeNutPositionEncoderConfig(
    const config::GripperConfig& config, common::Mm startup_nut_position) {
  NutPositionEncoderConfig output{};
  output.lead_screw_pitch = config.mechanism.lead_screw_pitch;
  output.direction_sign =
      config.motor.direction_sign * config.nut_position_encoder.direction_sign;
  output.feedback_stale_timeout = config.motor.feedback_stale_timeout;
  output.startup_nut_position = startup_nut_position;
  return output;
}

// Converts mechanism-side nut speed to motor-side angular velocity before
// crossing the hardware abstraction boundary.
[[nodiscard]] common::RadPerS nutSpeedToMotorVelocity(common::MmPerS speed,
                                                       common::Mm pitch,
                                                       int direction_sign) {
  if (pitch.value == 0.0) {
    return {};
  }
  return common::RadPerS{static_cast<double>(direction_sign) * speed.value *
                         kTwoPi / pitch.value};
}

// Converts measured motor velocity back into nut speed for safety checks. This
// must use feedback, not the commanded speed, so contact/jam detection can see
// real velocity drop.
[[nodiscard]] common::MmPerS motorVelocityToNutSpeed(
    common::RadPerS velocity, common::Mm pitch, int direction_sign) {
  if (direction_sign == 0) {
    direction_sign = 1;
  }
  return common::MmPerS{velocity.value * pitch.value /
                        (kTwoPi * static_cast<double>(direction_sign))};
}

[[nodiscard]] bool isActiveMotionState(GripperTopState state) {
  return state == GripperTopState::PreSelfCheck ||
         state == GripperTopState::HomingOpenStop ||
         state == GripperTopState::TravelLearning ||
         state == GripperTopState::MotionHealthCheck ||
         state == GripperTopState::Clamping ||
         state == GripperTopState::Unloading ||
         state == GripperTopState::Releasing ||
         state == GripperTopState::ManualPositioning;
}

[[nodiscard]] double positiveOrZero(double value) {
  return value > 0.0 ? value : 0.0;
}

void capFirstPassProjectionQuality(self_check::StructureProfile* profile) {
  if (profile == nullptr) {
    return;
  }

  // The current controller creates identification samples from one command plus
  // one feedback snapshot. That is useful for wiring the flow, but it is not the
  // multi-speed, multi-region characterization required to call the structure
  // model verified on real hardware.
  auto cap = [](self_check::IdentificationQuality* quality) {
    if (*quality == self_check::IdentificationQuality::Verified) {
      *quality = self_check::IdentificationQuality::LowConfidence;
    }
  };
  cap(&profile->quality);
  cap(&profile->opening.quality);
  cap(&profile->closing.quality);
  cap(&profile->noise_floor.quality);
  cap(&profile->travel_limits.quality);
  cap(&profile->motion_health.quality);
}

[[nodiscard]] config::GripperConfig makeSafeConfig(
    const config::GripperConfig& config) {
  auto safe = config;
  if (safe.motor.direction_sign == 0) {
    safe.motor.direction_sign = 1;
  }
  if (safe.nut_position_encoder.direction_sign == 0) {
    safe.nut_position_encoder.direction_sign = 1;
  }
  return safe;
}

[[nodiscard]] self_check::SelfCheckManagerConfig makeSelfCheckConfig(
    const config::GripperConfig& config) {
  // The self-check manager owns pure identification logic. This adapter keeps
  // thresholds in config instead of hard-coding mechanism parameters in the
  // controller.
  self_check::SelfCheckManagerConfig output{};

  output.structure_parameter_config.min_measured_distance =
      config.self_check.min_measured_distance;
  output.structure_parameter_config.max_distance_error =
      config.self_check.max_distance_error;
  output.structure_parameter_config
      .fallback_opening_minimum_stable_nut_speed =
      config.self_check.min_speed_scan_start;
  output.structure_parameter_config
      .fallback_closing_minimum_stable_nut_speed =
      config.self_check.min_speed_scan_start;
  output.structure_parameter_config.fallback_low_speed_unstable_upper_bound =
      config.self_check.min_speed_scan_stop;
  output.structure_parameter_config.fallback_motor_position_noise =
      config.self_check.fallback_motor_position_noise;
  output.structure_parameter_config.fallback_motor_velocity_noise =
      config.self_check.fallback_motor_velocity_noise;
  output.structure_parameter_config.fallback_motor_current_noise =
      config.self_check.fallback_motor_current_noise;
  output.structure_parameter_config.fallback_motor_torque_noise =
      config.self_check.fallback_motor_torque_noise;
  output.structure_parameter_config.fallback_nut_stroke_noise =
      config.self_check.fallback_nut_stroke_noise;

  output.friction_config.stable_speed_margin =
      config.self_check.stable_speed_margin;

  output.travel_limit_config.theoretical_travel =
      common::Mm{config.mechanism.theoretical_close_limit.value -
                 config.mechanism.theoretical_open_limit.value};
  output.travel_limit_config.max_theoretical_travel_error =
      config.self_check.max_theoretical_travel_error;
  output.travel_limit_config.safe_zone_margin = config.self_check.safe_zone_margin;
  output.travel_limit_config.software_limit_margin =
      config.self_check.software_limit_margin;
  output.travel_limit_config.fallback_open_limit =
      config.mechanism.theoretical_open_limit;
  output.travel_limit_config.fallback_closed_limit =
      config.mechanism.theoretical_close_limit;

  output.motion_health_config.software_open_limit =
      common::Mm{config.mechanism.theoretical_open_limit.value +
                 config.self_check.software_limit_margin.value};
  output.motion_health_config.software_closed_limit =
      common::Mm{config.mechanism.theoretical_close_limit.value -
                 config.self_check.software_limit_margin.value};
  output.motion_health_config.max_velocity_tracking_error =
      config.self_check.max_velocity_tracking_error;
  output.motion_health_config.max_current_ripple =
      config.self_check.max_current_ripple;
  output.motion_health_config.max_torque_ripple =
      config.self_check.max_torque_ripple;
  output.motion_health_config.max_temperature =
      config.self_check.max_motor_temperature;

  return output;
}

[[nodiscard]] self_check::FrictionAnomalyDetectorConfig
makeFrictionAnomalyDetectorConfig(const config::GripperConfig& config) {
  self_check::FrictionAnomalyDetectorConfig output{};
  output.enabled = config.self_check.friction_anomaly_enabled;
  output.current_ratio_threshold =
      config.self_check.friction_anomaly_current_ratio_threshold;
  output.sliding_window_distance =
      config.self_check.friction_anomaly_sliding_window_distance;
  output.min_width = config.self_check.friction_anomaly_min_width;
  output.min_confirmations =
      config.self_check.friction_anomaly_min_confirmations;
  output.minor_ratio = config.self_check.friction_anomaly_minor_ratio;
  output.moderate_ratio = config.self_check.friction_anomaly_moderate_ratio;
  output.max_records = config.self_check.friction_anomaly_max_records;
  output.min_baseline_current =
      config.self_check.friction_anomaly_min_baseline_current;
  return output;
}

[[nodiscard]] safety::SafetyLimitConfig makeSafetyLimitConfig(
    const config::GripperConfig& config,
    const self_check::StructureProfile& profile,
    common::A current_limit,
    bool use_software_limits) {
  // Config provides global safety bounds; the command can further reduce the
  // current limit for a specific operation. Learned software limits are used
  // only after the profile has a valid motion range.
  safety::SafetyLimitConfig output{};
  output.max_motor_current = common::A{
      std::min(config.safety.max_motor_current.value, current_limit.value)};
  output.max_nut_speed = config.safety.max_nut_speed;
  output.max_nut_acceleration = config.safety.max_nut_acceleration;
  output.acceleration_limits_enabled =
      config.safety.max_nut_acceleration.value > 0.0;
  output.active_stop_on_speed_limit = false;
  output.active_stop_on_acceleration_limit = false;

  if (use_software_limits) {
    output.min_nut_stroke = profile.travel_limits.software_open_limit;
    output.max_nut_stroke = profile.travel_limits.software_closed_limit;
    output.stroke_limits_enabled =
        output.max_nut_stroke.value > output.min_nut_stroke.value;
  } else {
    output.min_nut_stroke = config.mechanism.theoretical_open_limit;
    output.max_nut_stroke = config.mechanism.theoretical_close_limit;
    output.stroke_limits_enabled =
        output.max_nut_stroke.value > output.min_nut_stroke.value;
  }
  return output;
}

[[nodiscard]] safety::ContactJamDetectorConfig makeContactConfig(
    const config::GripperConfig& config) {
  // Contact and jam thresholds are conservative proxies based on motor feedback
  // because no external force or jaw-position sensor exists.
  safety::ContactJamDetectorConfig output{};
  output.contact_current_rise = config.safety.contact_current_rise_threshold;
  output.jam_current_rise = config.safety.jam_current_threshold;
  output.velocity_drop_threshold = config.safety.jam_speed_threshold;
  output.stroke_limit_margin = config.safety.stroke_limit_margin;
  output.minimum_detection_time = config.safety.contact_detection_time;
  output.contact_detection_enabled = true;
  output.jam_detection_enabled = true;
  return output;
}

[[nodiscard]] calibration::ForceMapperConfig makeForceMapperConfig(
    const config::GripperConfig& config,
    const ClampForceCommand& command) {
  calibration::ForceMapperConfig output{};
  output.torque_per_force = config.clamp.torque_per_force;
  output.current_per_torque = config.clamp.current_per_torque;
  output.torque_offset = config.clamp.torque_offset;
  output.current_offset = config.clamp.current_offset;
  output.min_target_force = config.clamp.min_target_force;
  output.max_target_force = config.clamp.max_target_force;
  output.max_motor_torque =
      command.max_motor_torque.value > 0.0 ? command.max_motor_torque
                                           : config.clamp.max_motor_torque;
  output.max_motor_current =
      command.max_motor_current.value > 0.0 ? command.max_motor_current
                                            : config.safety.clamp_current_limit;
  output.target_force_limits_enabled = true;
  return output;
}

[[nodiscard]] mechanism::GripperKinematicsConfig makeKinematicsConfig(
    const config::GripperConfig& config) {
  mechanism::GripperKinematicsConfig output{};
  output.zero_angle_stroke = config.mechanism.zero_angle_stroke;
  output.zero_stroke_angle = config.mechanism.zero_stroke_gripper_angle;
  output.angle_per_stroke = config.mechanism.gripper_angle_per_nut_stroke;
  output.angular_speed_per_nut_speed =
      config.mechanism.gripper_angular_speed_per_nut_speed;
  return output;
}

}  // namespace

class DefaultGripperController final : public GripperController {
 public:
  explicit DefaultGripperController(
      std::unique_ptr<hardware_interface::MotorInterface> motor)
      : motor_(std::move(motor)) {}

  common::Result configure(const config::GripperConfig& config) override {
    if (isActiveMotionState(snapshot_.top_state)) {
      return fail(common::ErrorCode::InvalidOperation,
                  "cannot configure while motion or self-check is active");
    }

    config_ = makeSafeConfig(config);
    configured_ = true;
    state_machine_.resetToDisconnected();
    pre_self_check_state_machine_.reset();
    kinematics_.setConfig(makeKinematicsConfig(config_));
    force_mapper_.setConfig(makeForceMapperConfig(config_, ClampForceCommand{}));
    nut_position_encoder_.configure(makeNutPositionEncoderConfig(
        config_, provisionalReferenceStroke()));
    profile_ = makeConservativeProfile();
    motor_bringup_active_ = false;
    motor_bringup_unloaded_confirmed_ = false;
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result enterMotorBringupMode(
      const MotorBringupSessionRequest& request) override {
    const auto ready = requireConfiguredConnected();
    if (ready.isError()) {
      return ready;
    }
    if (!request.unloaded_or_structure_removed_confirmed) {
      return fail(common::ErrorCode::InvalidArgument,
                  "motor bring-up requires unloaded/structure-safe confirmation");
    }
    if (isActiveMotionState(state_machine_.state())) {
      return fail(common::ErrorCode::InvalidOperation,
                  "cannot enter motor bring-up while motion is active");
    }

    if (motor_->isEnabled()) {
      const auto disable_result = motor_->disable();
      if (disable_result.isError()) {
        return fail(disable_result);
      }
    }
    motor_bringup_active_ = true;
    motor_bringup_unloaded_confirmed_ =
        request.unloaded_or_structure_removed_confirmed;
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result exitMotorBringupMode() override {
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      const auto disable_result = motor_->disable();
      if (disable_result.isError()) {
        return fail(disable_result);
      }
    }
    motor_bringup_active_ = false;
    motor_bringup_unloaded_confirmed_ = false;
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result refreshMotorBringupFeedback() override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    const auto result = updateFeedback();
    if (result.isError()) {
      return fail(result);
    }
    return finish(common::Ok());
  }

  common::Result runMotorBringupCommunicationProbe(
      std::vector<std::string>* lines) override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    if (lines == nullptr) {
      return fail(common::ErrorCode::InvalidArgument,
                  "communication probe output is null");
    }
    const auto result = motor_->runCommunicationProbe(lines);
    if (result.isError()) {
      return fail(result);
    }
    return finish(common::Ok());
  }

  common::Result enableMotorBringupOutput() override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    const auto result = motor_->enable();
    if (result.isError()) {
      return fail(result);
    }
    (void)updateFeedback();
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result disableMotorBringupOutput() override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      const auto result = motor_->disable();
      if (result.isError()) {
        return fail(result);
      }
    }
    (void)updateFeedback();
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result jogMotorBringup(
      const MotorBringupJogCommand& command) override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    const auto validated = validateMotorBringupJog(command);
    if (validated.isError()) {
      return fail(validated);
    }

    const bool was_enabled = motor_->isEnabled();
    if (!was_enabled) {
      const auto enable_result = motor_->enable();
      if (enable_result.isError()) {
        return fail(enable_result);
      }
    }

    const auto feedback_before = updateFeedback();
    if (feedback_before.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return fail(feedback_before);
    }

    const MotorBringupJogCommand limited = limitedMotorBringupJog(command);
    const common::Rad start_position = last_feedback_.position;
    MotorCommand motor_command{};
    motor_command.control_mode = MotorControlMode::Velocity;
    const double direction =
        limited.relative_motor_position.value >= 0.0 ? 1.0 : -1.0;
    motor_command.target_velocity =
        common::RadPerS{direction * limited.max_motor_velocity.value};
    motor_command.enable = true;

    const auto send_result = motor_->sendCommand(motor_command);
    if (send_result.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return fail(send_result);
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::milliseconds>(
                              std::chrono::duration<double>{
                                  positiveOrZero(limited.pulse_duration.value)});
    common::Result feedback_during = common::Ok();
    bool current_limited = false;
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      feedback_during = updateFeedback();
      if (feedback_during.isError()) {
        break;
      }
      if (std::abs(last_feedback_.current.value) >
          limited.max_motor_current.value) {
        current_limited = true;
        break;
      }
    }
    if (feedback_during.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return fail(feedback_during);
    }

    const auto feedback_after = updateFeedback();
    const common::Rad end_position = last_feedback_.position;
    const auto disable_result = motor_->disable();
    updateSnapshotFlags();
    if (feedback_after.isError()) {
      return fail(feedback_after);
    }
    if (disable_result.isError()) {
      return fail(disable_result);
    }
    const common::Rad measured_delta{
        end_position.value - start_position.value};
    if (current_limited) {
      std::ostringstream message;
      message << "bring-up velocity jog stopped by feedback current limit"
              << " start_rad=" << start_position.value
              << " end_rad=" << end_position.value
              << " delta_rad=" << measured_delta.value
              << " vel_rad_s=" << motor_command.target_velocity.value
              << " current_limit_a=" << limited.max_motor_current.value
              << " feedback_current_a=" << last_feedback_.current.value;
      return finish(common::Result::error(common::ErrorCode::SafetyActiveStop,
                                          message.str()));
    }
    const double min_expected_motion = std::min(
        limited.max_motor_velocity.value * limited.pulse_duration.value * 0.05,
        0.005);
    if (std::abs(measured_delta.value) < min_expected_motion) {
      std::ostringstream message;
      message << "bring-up jog motion is below diagnostic threshold"
              << " start_rad=" << start_position.value
              << " end_rad=" << end_position.value
              << " delta_rad=" << measured_delta.value
              << " vel_rad_s=" << motor_command.target_velocity.value
              << " duration_s=" << limited.pulse_duration.value
              << " current_limit_a=" << limited.max_motor_current.value;
      return finish(common::Result::error(common::ErrorCode::ControlSaturation,
                                          message.str()));
    }
    std::ostringstream message;
    message << "bring-up velocity jog completed"
            << " requested_direction_window_rad="
            << command.relative_motor_position.value
            << " effective_direction_window_rad="
            << limited.relative_motor_position.value
            << " requested_vel_rad_s=" << command.max_motor_velocity.value
            << " effective_vel_rad_s=" << motor_command.target_velocity.value
            << " requested_pulse_s=" << command.pulse_duration.value
            << " effective_pulse_s=" << limited.pulse_duration.value
            << " measured_delta_rad=" << measured_delta.value;
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result moveMotorBringupRelativeTurns(
      const MotorBringupPositionMoveCommand& command) override {
    const auto ready = requireMotorBringupActive();
    if (ready.isError()) {
      return ready;
    }
    const auto validated = validateMotorBringupPositionMove(command);
    if (validated.isError()) {
      return fail(validated);
    }

    if (!motor_->isEnabled()) {
      const auto enable_result = motor_->enable();
      if (enable_result.isError()) {
        return fail(enable_result);
      }
    }

    const auto feedback_before = updateFeedback();
    if (feedback_before.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return fail(feedback_before);
    }

    const MotorBringupPositionMoveCommand limited =
        limitedMotorBringupPositionMove(command);
    const common::Rad start_position = last_feedback_.position;
    const common::Mm start_stroke = current_nut_stroke_;
    const common::Rad target_position{
        start_position.value +
        limited.relative_motor_revolutions.value * kTwoPi};
    const auto target_range = validateMotorBringupTargetPosition(
        start_position, target_position, limited.relative_motor_revolutions);
    if (target_range.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return finishFailureWithoutFault(target_range);
    }

    MotorCommand motor_command{};
    motor_command.control_mode = MotorControlMode::Position;
    motor_command.target_position = target_position;
    motor_command.target_velocity = limited.max_motor_velocity;
    motor_command.enable = true;

    const auto send_result = motor_->sendCommand(motor_command);
    if (send_result.isError()) {
      (void)motor_->disable();
      updateSnapshotFlags();
      return fail(send_result);
    }

    const auto timeout = motorBringupPositionMoveTimeout(limited);
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>{timeout.value});
    common::Result feedback_result = common::Ok();
    common::A peak_current{};
    bool sustained_current_limited = false;
    bool hard_current_limited = false;
    bool current_limit_timer_running = false;
    std::chrono::steady_clock::time_point current_limit_started_at{};
    bool target_reached = false;
    while (std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        break;
      }
      peak_current = common::A{std::max(
          std::abs(peak_current.value), std::abs(last_feedback_.current.value))};
      const auto now = std::chrono::steady_clock::now();
      if (feedbackCurrentExceedsHardLimit()) {
        hard_current_limited = true;
        break;
      }
      if (std::abs(last_feedback_.current.value) >
          std::abs(limited.feedback_current_limit.value)) {
        if (!current_limit_timer_running) {
          current_limit_started_at = now;
          current_limit_timer_running = true;
        }
      } else {
        current_limit_timer_running = false;
      }
      if (current_limit_timer_running &&
          now - current_limit_started_at >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{
                      config_.safety.contact_detection_time.value})) {
        sustained_current_limited = true;
        break;
      }
      if (motorPositionTargetReached(target_position,
                                     limited.max_motor_velocity)) {
        target_reached = true;
        break;
      }
    }

    const auto settle_result = stopAndWaitForSettledFeedback();
    updateSnapshotFlags();
    if (feedback_result.isError()) {
      return fail(feedback_result);
    }
    if (settle_result.result.isError()) {
      return fail(settle_result.result);
    }

    const bool final_target_reached =
        target_reached ||
        motorPositionTargetReached(target_position, limited.max_motor_velocity);
    const common::Rad end_position = settle_result.settled_motor_position;
    const common::Mm end_stroke = settle_result.settled_stroke;
    const common::Rad motor_delta{
        end_position.value - start_position.value};
    const common::Mm nut_delta{end_stroke.value - start_stroke.value};
    const double measured_revolutions = motor_delta.value / kTwoPi;
    const double measured_mm_per_rev =
        std::abs(measured_revolutions) > 1e-9
            ? nut_delta.value / measured_revolutions
            : 0.0;

    std::ostringstream message;
    message << "bring-up position move"
            << " requested_rev=" << command.relative_motor_revolutions.value
            << " effective_rev=" << limited.relative_motor_revolutions.value
            << " start_motor_pos_rad=" << start_position.value
            << " target_motor_pos_rad=" << target_position.value
            << " end_motor_pos_rad=" << end_position.value
            << " motor_delta_rad=" << motor_delta.value
            << " measured_rev=" << measured_revolutions
            << " start_nut_mm=" << start_stroke.value
            << " end_nut_mm=" << end_stroke.value
            << " nut_delta_mm=" << nut_delta.value
            << " mm_per_rev_estimate=" << measured_mm_per_rev
            << " max_vel_rad_s=" << limited.max_motor_velocity.value
            << " feedback_current_limit_a="
            << limited.feedback_current_limit.value
            << " peak_current_a=" << peak_current.value
            << " timeout_s=" << timeout.value
            << " current_limit_confirm_time_s="
            << config_.safety.contact_detection_time.value
            << " target_reached="
            << (final_target_reached ? "true" : "false")
            << " motor_disabled=" << (motor_->isEnabled() ? 0 : 1);
    if (sustained_current_limited || hard_current_limited) {
      return finish(common::Result::error(
          common::ErrorCode::SafetyActiveStop,
          message.str() +
              (hard_current_limited ? " reason=hard_feedback_current_limit"
                                    : " reason=sustained_feedback_current_limit")));
    }
    if (!final_target_reached) {
      return finish(common::Result::error(
          common::ErrorCode::OperationTimedOut,
          message.str() + " reason=target_not_reached_before_timeout"));
    }
    return finish(common::Result{common::ErrorCode::Ok,
                                 message.str() + " reason=completed"});
  }

  common::Result connect() override {
    if (!configured_) {
      return fail(common::ErrorCode::ConfigMissing,
                  "controller must be configured before connect");
    }
    if (!motor_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "controller has no motor implementation");
    }
    const auto result = motor_->connect();
    if (result.isError()) {
      state_machine_.dispatch(GripperEvent::ConnectFailed);
      return fail(result);
    }
    const auto transition = state_machine_.dispatch(GripperEvent::ConnectSucceeded);
    if (transition != state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("ConnectSucceeded"));
    }
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result disconnect() override {
    requestOperationCancel("disconnect requested");
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      (void)motor_->disable();
    }
    const auto result =
        motor_ && motor_->isConnected() ? motor_->disconnect() : common::Ok();
    state_machine_.dispatch(GripperEvent::DisconnectRequested);
    resetOperationFlagsForDisconnect();
    return finish(result);
  }

  common::Result enable() override {
    const auto ready = requireConfiguredConnected();
    if (ready.isError()) {
      return ready;
    }
    if (motor_bringup_active_) {
      return fail(common::ErrorCode::InvalidOperation,
                  "exit motor bring-up mode before normal enable");
    }
    if (operationCancelRequested()) {
      if (state_machine_.state() == GripperTopState::ActiveStop ||
          state_machine_.state() == GripperTopState::Fault) {
        return fail(common::ErrorCode::ControlNotReady,
                    "clear ActiveStop/Fault after the running operation has stopped before enabling");
      }
      clearOperationCancel();
    }

    if (state_machine_.state() == GripperTopState::Connected) {
      state_machine_.dispatch(GripperEvent::HardwareCheckRequested);
      state_machine_.dispatch(GripperEvent::HardwareCheckPassed);
    }
    if (state_machine_.state() == GripperTopState::ModeSwitching) {
      state_machine_.dispatch(GripperEvent::ModeSwitchSucceeded);
    }

    const auto result = motor_->enable();
    if (result.isError()) {
      state_machine_.dispatch(GripperEvent::EnableFailed);
      return fail(result);
    }

    const auto transition = state_machine_.dispatch(GripperEvent::EnableSucceeded);
    if (transition != state_machine::TransitionDecision::Accepted &&
        state_machine_.state() != GripperTopState::Enabled) {
      return fail(rejectedTransition("EnableSucceeded"));
    }
    (void)updateFeedback();
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result disable() override {
    requestOperationCancel("disable requested");
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      const auto result = motor_->disable();
      if (result.isError()) {
        return fail(result);
      }
    }
    if (state_machine_.state() == GripperTopState::Ready ||
        state_machine_.state() == GripperTopState::Unloading) {
      state_machine_.dispatch(GripperEvent::DisableRequested);
      state_machine_.dispatch(GripperEvent::DisableCompleted);
    } else if (state_machine_.state() != GripperTopState::Disconnected) {
      state_machine_.dispatch(GripperEvent::DisableRequested);
    }
    (void)updateFeedback();
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result runPreSelfCheck() override {
    clearOperationCancel();
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    if (state_machine_.dispatch(GripperEvent::PreSelfCheckRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("PreSelfCheckRequested"));
    }

    pre_self_check_state_machine_.reset();
    pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::Start);
    pre_self_check_opening_boundary_suspected_ = false;
    pre_self_check_closing_boundary_suspected_ = false;
    pre_b_mechanism_anomaly_ = false;
    pre_b_current_trace_points_.clear();
    pre_b_trace_decimation_counter_ = 0U;
    pre_b_current_trace_segment_id_ = 0U;
    friction_anomaly_records_.clear();
    self_check::FrictionAnomalyDetector friction_anomaly_detector{
        makeFrictionAnomalyDetectorConfig(config_)};
    loadSelfCheckSeedIfAvailable();
    const self_check::StructureProfile seed_profile_before_identification =
        profile_;
    reportProgress("PreSelfCheck | phase=LimitedProbe | start");

    self_check::SelfCheckInput input{};
    collectFeedbackNoiseSamples(&input);
    if (const auto cancel = checkOperationCancelled("PreSelfCheck feedback noise sampling");
        cancel.isError()) {
      return abortPreSelfCheck(cancel);
    }

    const auto bidirectional_probe = runProgressiveBidirectionalProbe(&input);
    if (bidirectional_probe.isError()) {
      reportProgress(std::string{"PreSelfCheck | phase=LimitedProbe | failed | "} +
                     bidirectional_probe.message());
      (void)disableAfterMotion();
      pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::ProbeFailed);
      state_machine_.dispatch(GripperEvent::PreSelfCheckFailed);
      return fail(bidirectional_probe);
    }
    pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::ProbePassed);
    pre_self_check_state_machine_.dispatch(
        PreSelfCheckEvent::BidirectionalMoveEnabled);

    if (const auto cancel = checkOperationCancelled("PreSelfCheck limited probe");
        cancel.isError()) {
      return abortPreSelfCheck(cancel);
    }

    reportProgress(
        "PreSelfCheck | phase=StableShortStrokeMotion | start");
    const auto stable_short_stroke =
        runStableShortStrokeSamples(&input, &friction_anomaly_detector);
    if (stable_short_stroke.isError()) {
      reportProgress(
          std::string{
              "PreSelfCheck | phase=StableShortStrokeMotion | degraded | "} +
          stable_short_stroke.message());
    }
    pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::StableStrokePassed);

    if (const auto cancel = checkOperationCancelled("PreSelfCheck stable short-stroke");
        cancel.isError()) {
      return abortPreSelfCheck(cancel);
    }

    PreSelfCheckExpansionBounds expansion_bounds{};
    reportProgress(
        "PreSelfCheck | phase=PreliminaryLimitSearch | start"
        " | mode=full_guard_continuous_pre_b_scan");
    const auto preliminary_limits = runPreliminaryLimitSearch(
        &input, &friction_anomaly_detector, &expansion_bounds);
    if (preliminary_limits.isError()) {
      if (!canDegradePreBSegment(preliminary_limits)) {
        reportProgress(
            std::string{
                "PreSelfCheck | phase=PreliminaryLimitSearch | failed | "} +
            preliminary_limits.message());
        (void)disableAfterMotion();
        pre_self_check_state_machine_.dispatch(
            PreSelfCheckEvent::PreliminaryLimitFailed);
        state_machine_.dispatch(GripperEvent::PreSelfCheckFailed);
        return fail(preliminary_limits);
      }
      reportProgress(
          std::string{
              "PreSelfCheck | phase=PreliminaryLimitSearch | degraded | "} +
          preliminary_limits.message() +
          " | note=continuing_with_partial_or_pre_a_bounds");
      pre_self_check_state_machine_.dispatch(
          PreSelfCheckEvent::PreliminaryLimitFound);
      if (expansion_bounds.boundary_release_failed ||
          !preSelfCheckSafeZoneFromBounds(expansion_bounds).valid) {
        const bool boundary_release_failed =
            expansion_bounds.boundary_release_failed;
        const auto unreleased_boundary_direction =
            expansion_bounds.unreleased_boundary_direction;
        expansion_bounds = fallbackPreSelfCheckExpansionBounds();
        expansion_bounds.boundary_release_failed = boundary_release_failed;
        expansion_bounds.unreleased_boundary_direction =
            unreleased_boundary_direction;
      }
    } else {
      pre_self_check_state_machine_.dispatch(
          PreSelfCheckEvent::PreliminaryLimitFound);
    }
    reportProgress(
        "PreSelfCheck | phase=TheoryTravelCheck | low-confidence bounds are inside"
        " theoretical guard; final software limits remain unverified");
    pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::TheoryTravelMatched);
    reportProgress(
        "PreSelfCheck | phase=SafeZoneBuild | building low-confidence safe zone");
    pre_self_check_state_machine_.dispatch(PreSelfCheckEvent::SafeZoneBuilt);
    if (expansion_bounds.boundary_release_failed) {
      reportProgress(
          "PreSelfCheck | phase=BoundaryRelease | skipped"
          " reason=pre_b_boundary_release_failed"
          " note=using_pre_a_conservative_window");
    } else {
      const auto boundary_release = releasePreBMechanicalBoundary(
          expansion_bounds, &input, nullptr);
      if (boundary_release.isError()) {
        reportProgress(
            std::string{"PreSelfCheck | phase=BoundaryRelease | degraded | "} +
            boundary_release.message());
        if (!canDegradePreBSegment(boundary_release)) {
          (void)disableAfterMotion();
          pre_self_check_state_machine_.dispatch(
              PreSelfCheckEvent::RegionSampleRejected);
          state_machine_.dispatch(GripperEvent::PreSelfCheckFailed);
          return fail(boundary_release);
        }
      }
    }
    common::Result safe_zone_samples = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "pre-self-check multi-region learning skipped after boundary release failure");
    if (expansion_bounds.boundary_release_failed) {
      reportProgress(
          "PreSelfCheck | phase=MultiRegionRoundTripLearning | skipped"
          " reason=boundary_release_failed"
          " action=rollback_to_pre_a_conservative_window");
      pre_self_check_state_machine_.dispatch(
          PreSelfCheckEvent::RegionSampleRejected);
    } else {
      reportProgress(
          "PreSelfCheck | phase=MultiRegionRoundTripLearning | start"
          " | mode=multi_region_safe_zone_round_trip");
      safe_zone_samples = runSafeZoneRoundTripSamples(
          &input, &friction_anomaly_detector, expansion_bounds);
      if (safe_zone_samples.isError()) {
      reportProgress(
          std::string{
              "PreSelfCheck | phase=MultiRegionRoundTripLearning | degraded | "} +
          safe_zone_samples.message());
      pre_self_check_state_machine_.dispatch(
          PreSelfCheckEvent::RegionSampleRejected);
      } else {
      pre_self_check_state_machine_.dispatch(
          PreSelfCheckEvent::RegionSampleAccepted);
      }
    }
    friction_anomaly_detector.finish();
    friction_anomaly_records_ = friction_anomaly_detector.records();
    reportFrictionAnomalySummary();
    pre_self_check_state_machine_.dispatch(
        PreSelfCheckEvent::RegionLearningCompleted);

    if (const auto cancel = checkOperationCancelled("PreSelfCheck profile update");
        cancel.isError()) {
      return abortPreSelfCheck(cancel);
    }

    // Low-confidence output is allowed to continue as a conservative profile,
    // but it is not equivalent to a fully verified real-mechanism self-check.
    const self_check::StructureProfile runtime_profile_before_identification =
        profile_;
    const self_check::SelfCheckManager manager{makeSelfCheckConfig(config_)};
    const auto output = manager.run(input);
    profile_ = output.profile;
    if (profile_.travel_limits.software_closed_limit.value <=
        profile_.travel_limits.software_open_limit.value) {
      profile_ = makeConservativeProfile();
    }
    applyPreSelfCheckExpansionBoundsToProfile(expansion_bounds, &profile_);
    mergeLoadedSelfCheckSeed(seed_profile_before_identification, &profile_);
    mergeLoadedSelfCheckSeed(runtime_profile_before_identification, &profile_);
    if (expansion_bounds.boundary_release_failed) {
      applyPreSelfCheckExpansionBoundsToProfile(expansion_bounds, &profile_);
    }
    profile_.validity = StructureProfileValidity::PreSelfCheckCompleted;
    capFirstPassProjectionQuality(&profile_);
    saveSelfCheckSeed();

    std::ostringstream profile_message;
    profile_message
        << "PreSelfCheck | phase=StructureProfileUpdate | profile updated"
        << " validity=PreSelfCheckCompleted"
        << " confidence=low"
        << " safe_zone_open_mm="
        << profile_.travel_limits.safe_zone_open_limit.value
        << " safe_zone_closed_mm="
        << profile_.travel_limits.safe_zone_closed_limit.value
        << " anomaly_candidates=" << friction_anomaly_records_.size()
        << " note="
        << (expansion_bounds.boundary_release_failed
                ? "pre_b_boundary_release_failed_using_pre_a_window"
                : "pre_b_bounds_low_confidence_not_final_software_limits");
    reportProgress(profile_message.str());
    pre_self_check_state_machine_.dispatch(
        PreSelfCheckEvent::StructureProfileUpdated);

    pre_self_check_completed_ = true;
    state_machine_.dispatch(GripperEvent::PreSelfCheckSucceeded);
    const auto settle_result = stopAndWaitForSettledFeedback();
    if (settle_result.result.isError()) {
      state_machine_.dispatch(GripperEvent::ActiveStopTriggered);
      updateSnapshotFlags();
      return fail(settle_result.result);
    }
    state_machine_.dispatch(GripperEvent::DisableRequested);
    updateSnapshotFlags();
    if (output.result.isError()) {
      return finish(common::Result{
          common::ErrorCode::Ok,
          "pre-self-check completed with conservative feedback-derived profile"});
    }
    return finish(common::Ok());
  }

  common::Result homeOpenStop() override {
    clearOperationCancel();
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    if (!pre_self_check_completed_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "pre-self-check must complete before homing");
    }
    if (state_machine_.dispatch(GripperEvent::HomingRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("HomingRequested"));
    }

    const double known_low_confidence_width = std::max(
        0.0, profile_.travel_limits.safe_zone_closed_limit.value -
                 profile_.travel_limits.safe_zone_open_limit.value);
    const double theoretical_travel = std::max(
        0.0, config_.mechanism.theoretical_close_limit.value -
                 config_.mechanism.theoretical_open_limit.value);
    const double home_search_distance =
        std::max({known_low_confidence_width,
                  theoretical_travel,
                  std::abs(config_.self_check.pre_b_max_expansion_distance.value),
                  config_.self_check.min_measured_distance.value});
    const common::Mm home_target{
        current_nut_stroke_.value - home_search_distance};
    const common::Mm window_margin{std::max(
        config_.self_check.max_distance_error.value,
        config_.self_check.fallback_nut_stroke_noise.value * 3.0)};
    const OperationCurrentLimit homing_current = homingCurrentLimitFromProfile();
    std::ostringstream start_message;
    start_message << "HomingOpenStop | relative opening search"
                  << " current_mm=" << current_nut_stroke_.value
                  << " target_mm=" << home_target.value
                  << " search_distance_mm=" << home_search_distance
                  << " current_limit_a=" << homing_current.limit.value
                  << " current_source=" << homing_current.source
                  << " note=pre_homing_coordinate_not_absolute_zero";
    reportProgress(start_message.str());
    const MotionWaitOptions options{
        home_target,
        common::MmPerS{-absValue(config_.homing.homing_speed.value)},
        homing_current.limit,
        {},
        false,
        true,
        true,
        {},
        true,
        common::Mm{home_target.value - window_margin.value},
        common::Mm{current_nut_stroke_.value + window_margin.value},
        true,
        true,
        config_.homing.jam_confirm_time};
    const auto motion = runMotionUntil(options);
    const bool opening_stop_detected =
        motion.contact_detected || motion.jam_detected ||
        motion.force_current_reached ||
        motion.result.code() == common::ErrorCode::SafetyJamDetected;
    if (motion.result.isError() &&
        motion.result.code() != common::ErrorCode::SafetyJamDetected &&
        motion.result.code() != common::ErrorCode::OperationTimedOut) {
      (void)disableAfterMotion();
      state_machine_.dispatch(GripperEvent::HomingFailed);
      return fail(motion.result);
    }
    if (!opening_stop_detected) {
      (void)disableAfterMotion();
      state_machine_.dispatch(GripperEvent::HomingFailed);
      return fail(common::ErrorCode::OperationTimedOut,
                  "homing relative opening search ended before detecting an open stop");
    }

    setOpenZeroReference(last_feedback_.position);
    rebaseFrictionAnomalyRecordsToCurrentReference("homing_open_zero");
    const double backoff_distance = std::max(
        0.0, config_.homing.backoff_distance.value);
    common::Mm backoff_target = config_.mechanism.theoretical_open_limit;
    common::Result backoff_result = common::Ok();
    common::Mm backoff_measured_distance{};
    if (backoff_distance > 0.0) {
      backoff_target = common::Mm{std::min(
          config_.mechanism.theoretical_open_limit.value + backoff_distance,
          config_.mechanism.theoretical_close_limit.value)};
      const common::A backoff_current = homing_current.limit;
      std::ostringstream backoff_start;
      backoff_start << "HomingOpenStop | endpoint release"
                    << " target_mm=" << backoff_target.value
                    << " distance_mm=" << backoff_distance
                    << " speed_mm_s="
                    << std::abs(config_.homing.homing_speed.value)
                    << " current_a=" << backoff_current.value
                    << " current_source=" << homing_current.source;
      reportProgress(backoff_start.str());
      const auto backoff_motion = runMotionUntil(MotionWaitOptions{
          backoff_target,
          common::MmPerS{absValue(config_.homing.homing_speed.value)},
          backoff_current,
          {},
          false,
          false,
          false,
          {}});
      backoff_result = backoff_motion.result;
      backoff_measured_distance = backoff_motion.measured_distance;
      if (backoff_result.isError()) {
        (void)disableAfterMotion();
        state_machine_.dispatch(GripperEvent::HomingFailed);
        std::ostringstream failure;
        failure << "homing open zero found but endpoint release failed"
                << " open_zero_motor_rad=" << motor_zero_reference_.value
                << " backoff_target_mm=" << backoff_target.value
                << " measured_backoff_mm="
                << backoff_measured_distance.value
                << " reason=" << backoff_result.message();
        return fail(backoff_result.code(), failure.str());
      }
    } else {
      (void)disableAfterMotion();
    }
    homed_ = true;
    profile_.validity = StructureProfileValidity::Homed;
    state_machine_.dispatch(GripperEvent::HomingSucceeded);
    updateSnapshotFlags();
    std::ostringstream message;
    message << "homing completed open_zero_motor_rad="
            << motor_zero_reference_.value
            << " measured_distance_mm=" << motion.measured_distance.value
            << " backoff_target_mm=" << backoff_target.value
            << " measured_backoff_mm=" << backoff_measured_distance.value
            << " motor_disabled=" << (motor_->isEnabled() ? 0 : 1);
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result learnTravelLimits() override {
    clearOperationCancel();
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    if (!homed_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "homing must complete before travel limit learning");
    }
    if (state_machine_.dispatch(GripperEvent::TravelLearningRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("TravelLearningRequested"));
    }

    const double known_low_confidence_width = std::max(
        0.0, profile_.travel_limits.safe_zone_closed_limit.value -
                 profile_.travel_limits.safe_zone_open_limit.value);
    const double theoretical_travel = std::max(
        0.0, config_.mechanism.theoretical_close_limit.value -
                 config_.mechanism.theoretical_open_limit.value);
    const double configured_search =
        std::abs(config_.self_check.travel_learning_search_distance.value);
    const common::Mm requested_search_distance{std::max(
        {configured_search > 0.0 ? configured_search : theoretical_travel,
         known_low_confidence_width,
         theoretical_travel,
         config_.self_check.min_measured_distance.value})};
    std::string motor_window_detail;
    const common::Mm search_distance =
        travelLearningSearchDistanceInsideMotorWindow(requested_search_distance,
                                                      &motor_window_detail);
    if (search_distance.value <= config_.self_check.max_distance_error.value) {
      (void)disableAfterMotion();
      state_machine_.dispatch(GripperEvent::TravelLearningFailed);
      std::ostringstream message;
      message << "travel learning has no safe closing search room"
              << " requested_search_distance_mm="
              << requested_search_distance.value << motor_window_detail;
      return fail(common::ErrorCode::OutOfRange, message.str());
    }
    const common::Mm close_target{
        config_.mechanism.theoretical_open_limit.value + search_distance.value};
    const common::Mm window_margin{std::max(
        config_.self_check.max_distance_error.value,
        config_.self_check.fallback_nut_stroke_noise.value * 3.0)};
    const common::Mm custom_open_limit{
        config_.mechanism.theoretical_open_limit.value - window_margin.value};
    const common::Mm custom_closed_limit{close_target.value +
                                         window_margin.value};
    const OperationCurrentLimit close_current =
        travelLearningCurrentLimitFromProfile(self_check::MotionDirection::Closing);
    const OperationCurrentLimit open_current =
        travelLearningCurrentLimitFromProfile(self_check::MotionDirection::Opening);
    {
      std::ostringstream message;
      message << "TravelLearning | closing search start"
              << " current_mm=" << current_nut_stroke_.value
              << " target_mm=" << close_target.value
              << " search_distance_mm=" << search_distance.value
              << " requested_search_distance_mm="
              << requested_search_distance.value
              << " configured_search_distance_mm=" << configured_search
              << " low_confidence_width_mm=" << known_low_confidence_width
              << " reference_theoretical_close_mm="
              << config_.mechanism.theoretical_close_limit.value
              << " reference_theoretical_travel_mm=" << theoretical_travel
              << " speed_mm_s="
              << absValue(config_.self_check.travel_learning_speed.value)
              << " current_limit_a="
              << close_current.limit.value
              << " current_source=" << close_current.source
              << " no_progress_confirm_s="
              << config_.safety.contact_detection_time.value
              << " note=theoretical_0_16_is_reference_only"
              << motor_window_detail;
      reportProgress(message.str());
    }
    self_check::FrictionAnomalyDetector friction_anomaly_detector{
        makeFrictionAnomalyDetectorConfig(config_)};
    const MotionWaitOptions options{
        close_target,
        common::MmPerS{absValue(config_.self_check.travel_learning_speed.value)},
        close_current.limit,
        {},
        false,
        true,
        false,
        {},
        true,
        custom_open_limit,
        custom_closed_limit,
        true,
        true,
        config_.safety.contact_detection_time,
        &friction_anomaly_detector};
    const auto motion = runMotionUntil(options);
    friction_anomaly_detector.finish();
    mergeFrictionAnomalyRecords(friction_anomaly_detector.records());
    {
      std::ostringstream message;
      message << "TravelLearning | closing search result"
              << " code=" << common::toString(motion.result.code())
              << " start_mm=" << motion.start_stroke.value
              << " end_mm=" << motion.end_stroke.value
              << " measured_mm=" << motion.measured_distance.value
              << " measured_speed_mm_s=" << motion.measured_nut_speed.value
              << " target_mm=" << close_target.value
              << " search_distance_mm=" << search_distance.value
              << " requested_search_distance_mm="
              << requested_search_distance.value
              << " reference_theoretical_close_mm="
              << config_.mechanism.theoretical_close_limit.value
              << " reference_theoretical_travel_mm=" << theoretical_travel
              << " target_reached="
              << (motion.target_reached ? "true" : "false")
              << " contact_detected="
              << (motion.contact_detected ? "true" : "false")
              << " jam_detected=" << (motion.jam_detected ? "true" : "false")
              << " force_current_reached="
              << (motion.force_current_reached ? "true" : "false")
              << " current_limit_a="
              << close_current.limit.value
              << " current_source=" << close_current.source
              << " anomaly_candidates="
              << friction_anomaly_records_.size();
      if (motion.result.hasMessage()) {
        message << " message=" << motion.result.message();
      }
      reportProgress(message.str());
    }
    if (motion.result.isError()) {
      (void)disableAfterMotion();
      state_machine_.dispatch(GripperEvent::TravelLearningFailed);
      return fail(motion.result);
    }

    const common::Mm learned_closed_limit = current_nut_stroke_;
    const common::Mm learned_travel{
        learned_closed_limit.value - config_.mechanism.theoretical_open_limit.value};
    if (learned_travel.value <= config_.self_check.min_measured_distance.value) {
      (void)disableAfterMotion();
      state_machine_.dispatch(GripperEvent::TravelLearningFailed);
      return fail(common::ErrorCode::SelfCheckInconsistentFeedback,
                  "travel learning measured too little usable travel");
    }

    std::vector<self_check::LimitObservationSample> samples{
        self_check::LimitObservationSample{
            self_check::MotionDirection::Opening,
            config_.mechanism.theoretical_open_limit,
            true,
            false,
            false},
        self_check::LimitObservationSample{
            self_check::MotionDirection::Closing,
            learned_closed_limit,
            true,
            false,
            false}};
    const self_check::TravelLimitIdentifier identifier{
        makeSelfCheckConfig(config_).travel_limit_config};
    const auto limits = identifier.identify(samples);
    if (limits.profile.software_closed_limit.value <=
        limits.profile.software_open_limit.value) {
      state_machine_.dispatch(GripperEvent::TravelLearningFailed);
      return fail(limits.result);
    }

    profile_.travel_limits = limits.profile;
    profile_.travel_limits.learned_travel = learned_travel;
    if (limits.result.isError() ||
        profile_.travel_limits.quality ==
            self_check::IdentificationQuality::Verified) {
      profile_.travel_limits.quality =
          self_check::IdentificationQuality::LowConfidence;
    }
    profile_.validity = StructureProfileValidity::TravelLimitsLearned;

    const auto backoff = runMotionUntil(MotionWaitOptions{
        profile_.travel_limits.software_open_limit,
        common::MmPerS{-absValue(config_.self_check.travel_learning_speed.value)},
        open_current.limit,
        {},
        false,
        false,
        false,
        {},
        true,
        common::Mm{std::min(profile_.travel_limits.software_open_limit.value,
                            custom_open_limit.value)},
        common::Mm{std::max(profile_.travel_limits.software_closed_limit.value,
                            custom_closed_limit.value)}});
    (void)disableAfterMotion();
    if (backoff.result.isError()) {
      state_machine_.dispatch(GripperEvent::TravelLearningFailed);
      return fail(backoff.result);
    }
    travel_limits_learned_ = true;
    state_machine_.dispatch(GripperEvent::TravelLearningSucceeded);
    saveSelfCheckSeed();
    updateSnapshotFlags();
    std::ostringstream message;
    message << "travel learning completed learned_closed_mm="
            << learned_closed_limit.value
            << " learned_travel_mm=" << learned_travel.value
            << " search_distance_mm=" << search_distance.value
            << " requested_search_distance_mm="
            << requested_search_distance.value
            << " target_reached=" << (motion.target_reached ? "true" : "false")
            << " reference_theoretical_close_mm="
            << config_.mechanism.theoretical_close_limit.value
            << " software_open_mm="
            << profile_.travel_limits.software_open_limit.value
            << " software_closed_mm="
            << profile_.travel_limits.software_closed_limit.value
            << " closing_current_limit_a=" << close_current.limit.value
            << " closing_current_source=" << close_current.source
            << " opening_backoff_current_limit_a=" << open_current.limit.value
            << " opening_backoff_current_source=" << open_current.source;
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result runMotionHealthCheck() override {
    clearOperationCancel();
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    if (!travel_limits_learned_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "travel limits must be learned before motion health check");
    }
    if (state_machine_.dispatch(GripperEvent::MotionHealthCheckRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("MotionHealthCheckRequested"));
    }
    (void)disableAfterMotion();

    std::vector<self_check::MotionHealthSample> samples{};
    self_check::FrictionAnomalyDetector friction_anomaly_detector{
        makeFrictionAnomalyDetectorConfig(config_)};
    const auto health_config = motionHealthConfigFromCurrentProfile();
    const common::Mm health_move_distance{std::max(
        config_.self_check.min_measured_distance.value,
        std::min(config_.clamp.release_distance.value,
                 std::max(0.0,
                          (profile_.travel_limits.software_closed_limit.value -
                           profile_.travel_limits.software_open_limit.value) *
                              0.25)))};
    const OperationCurrentLimit closing_health_current =
        motionHealthCurrentLimitFromProfile(self_check::MotionDirection::Closing);
    const OperationCurrentLimit opening_health_current =
        motionHealthCurrentLimitFromProfile(self_check::MotionDirection::Opening);
    {
      std::ostringstream message;
      message << "MotionHealthCheck | start"
              << " health_move_distance_mm=" << health_move_distance.value
              << " closing_current_limit_a="
              << closing_health_current.limit.value
              << " closing_current_source=" << closing_health_current.source
              << " opening_current_limit_a="
              << opening_health_current.limit.value
              << " opening_current_source=" << opening_health_current.source;
      reportProgress(message.str());
    }
    for (const auto speed : config_.self_check.motion_health_check_speeds) {
      const double abs_speed = std::abs(speed.value);
      if (abs_speed <= 0.0) {
        continue;
      }
      const common::Mm close_target{std::min(
          current_nut_stroke_.value + health_move_distance.value,
          profile_.travel_limits.software_closed_limit.value)};
      const auto close_motion = runMotionUntil(MotionWaitOptions{
          close_target,
          common::MmPerS{abs_speed},
          closing_health_current.limit,
          {},
          true,
          false,
          false,
          {},
          true,
          profile_.travel_limits.software_open_limit,
          profile_.travel_limits.software_closed_limit,
          false,
          false,
          {},
          &friction_anomaly_detector});
      if (close_motion.result.isError()) {
        (void)disableAfterMotion();
        state_machine_.dispatch(GripperEvent::MotionHealthCheckFailed);
        return fail(close_motion.result);
      }
      samples.push_back(makeMotionHealthSample(close_motion,
                                               common::MmPerS{abs_speed}));

      const common::Mm open_target{std::max(
          current_nut_stroke_.value - health_move_distance.value,
          profile_.travel_limits.software_open_limit.value)};
      const auto open_motion = runMotionUntil(MotionWaitOptions{
          open_target,
          common::MmPerS{-abs_speed},
          opening_health_current.limit,
          {},
          true,
          false,
          false,
          {},
          true,
          profile_.travel_limits.software_open_limit,
          profile_.travel_limits.software_closed_limit,
          false,
          false,
          {},
          &friction_anomaly_detector});
      if (open_motion.result.isError()) {
        (void)disableAfterMotion();
        state_machine_.dispatch(GripperEvent::MotionHealthCheckFailed);
        return fail(open_motion.result);
      }
      samples.push_back(makeMotionHealthSample(open_motion,
                                               common::MmPerS{-abs_speed}));
    }

    (void)disableAfterMotion();
    friction_anomaly_detector.finish();
    mergeFrictionAnomalyRecords(friction_anomaly_detector.records());
    const self_check::MotionHealthChecker checker{health_config};
    const auto health = checker.check(samples);
    profile_.motion_health = health.profile;
    if (health.result.isError()) {
      state_machine_.dispatch(GripperEvent::MotionHealthCheckFailed);
      return fail(health.result);
    }
    capFirstPassProjectionQuality(&profile_);
    profile_.validity = StructureProfileValidity::MotionHealthChecked;
    motion_health_checked_ = true;
    state_machine_.dispatch(GripperEvent::MotionHealthCheckSucceeded);
    saveSelfCheckSeed();
    updateSnapshotFlags();
    std::ostringstream message;
    message << "MotionHealthCheck | completed"
            << " sample_count=" << samples.size()
            << " closing_current_limit_a=" << closing_health_current.limit.value
            << " closing_current_source=" << closing_health_current.source
            << " opening_current_limit_a=" << opening_health_current.limit.value
            << " opening_current_source=" << opening_health_current.source
            << " anomaly_candidates=" << friction_anomaly_records_.size();
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  [[nodiscard]] self_check::MotionHealthCheckerConfig
  motionHealthConfigFromCurrentProfile() const {
    auto output = makeSelfCheckConfig(config_).motion_health_config;
    if (profile_.travel_limits.software_closed_limit.value >
        profile_.travel_limits.software_open_limit.value) {
      output.software_open_limit = profile_.travel_limits.software_open_limit;
      output.software_closed_limit = profile_.travel_limits.software_closed_limit;
    }
    return output;
  }

  common::Result clampByForce(const ClampForceCommand& command) override {
    clearOperationCancel();
    const auto ready = requireReadyForWork();
    if (ready.isError()) {
      return ready;
    }
    if (state_machine_.dispatch(GripperEvent::ClampRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("ClampRequested"));
    }

    const common::N target_force = command.target_force.value > 0.0
                                       ? command.target_force
                                       : config_.clamp.target_force;
    force_mapper_.setConfig(makeForceMapperConfig(config_, command));
    const auto force = force_mapper_.mapTargetForce(target_force);

    const common::MmPerS nut_speed = resolveClampForceNutSpeed(command);
    const common::Mm target_stroke =
        profile_.travel_limits.software_closed_limit;
    const auto motion = runMotionUntil(MotionWaitOptions{
        target_stroke,
        nut_speed,
        force.motor_current,
        command.timeout,
        true,
        true,
        true,
        force.motor_current,
        false,
        {},
        {},
        false,
        true,
        config_.safety.contact_detection_time});
    if (motion.result.isError()) {
      state_machine_.dispatch(GripperEvent::ClampFailed);
      return fail(motion.result);
    }

    if (!motion.contact_detected && !motion.force_current_reached) {
      state_machine_.dispatch(GripperEvent::ClampFailed);
      return fail(common::ErrorCode::SafetyContactDetected,
                  "clamp reached travel target without contact or force proxy");
    }

    estimated_clamp_force_ =
        motion.force_current_reached ? force.target_force : common::N{};
    contact_detected_ =
        motion.contact_detected || motion.force_current_reached;
    state_machine_.dispatch(GripperEvent::ClampCompleted);
    state_machine_.dispatch(GripperEvent::DisableCompleted);
    updateSnapshotFlags();
    std::ostringstream message;
    message << "clamp force completed target_force_n=" << force.target_force.value
            << " motor_current_target_a=" << force.motor_current.value
            << " contact=" << (motion.contact_detected ? 1 : 0)
            << " force_proxy=" << (motion.force_current_reached ? 1 : 0)
            << " target_reached=" << (motion.target_reached ? 1 : 0);
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result clampBySpeed(const ClampSpeedCommand& command) override {
    const auto ready = requireReadyForWork();
    if (ready.isError()) {
      return ready;
    }
    if (state_machine_.dispatch(GripperEvent::ClampRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("ClampRequested"));
    }

    const common::MmPerS nut_speed =
        command.target_nut_speed.value > 0.0 ? command.target_nut_speed
                                             : config_.clamp.target_nut_speed;
    const common::A current_limit =
        command.max_motor_current.value > 0.0
            ? command.max_motor_current
            : config_.safety.clamp_current_limit;
    const common::Mm target_stroke =
        profile_.travel_limits.software_closed_limit;

    const auto motion = runMotionUntil(MotionWaitOptions{
        target_stroke,
        nut_speed,
        current_limit,
        command.timeout,
        true,
        true,
        true,
        {},
        false,
        {},
        {},
        false,
        true,
        config_.safety.contact_detection_time});
    if (motion.result.isError()) {
      state_machine_.dispatch(GripperEvent::ClampFailed);
      return fail(motion.result);
    }

    contact_detected_ = motion.contact_detected;
    if (!contact_detected_ && !motion.target_reached) {
      state_machine_.dispatch(GripperEvent::ClampFailed);
      return fail(common::ErrorCode::SafetyContactDetected,
                  "constant-speed clamp ended without contact feedback");
    }
    state_machine_.dispatch(GripperEvent::ClampCompleted);
    state_machine_.dispatch(GripperEvent::DisableCompleted);
    updateSnapshotFlags();
    std::ostringstream message;
    message << "constant-speed clamp completed contact="
            << (motion.contact_detected ? 1 : 0)
            << " target_reached=" << (motion.target_reached ? 1 : 0);
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result release() override {
    ReleaseCommand command{};
    command.release_nut_speed = config_.clamp.target_nut_speed;
    command.release_distance = config_.clamp.release_distance;
    command.timeout = config_.safety.command_timeout;
    command.source = CommandSource::Programmatic;
    return release(command);
  }

  common::Result release(const ReleaseCommand& command) override {
    clearOperationCancel();
    if (!configured_ || !motor_ || !motor_->isConnected()) {
      return fail(common::ErrorCode::ControlNotReady,
                  "release requires configured and connected controller");
    }
    if (motor_bringup_active_) {
      return fail(common::ErrorCode::InvalidOperation,
                  "exit motor bring-up mode before release");
    }
    if (!motor_->isEnabled()) {
      const auto enable_result = enable();
      if (enable_result.isError()) {
        return enable_result;
      }
    }
    if (state_machine_.state() == GripperTopState::Enabled) {
      state_machine_.dispatch(GripperEvent::MotionHealthCheckRequested);
      state_machine_.dispatch(GripperEvent::MotionHealthCheckSucceeded);
    }
    if (state_machine_.dispatch(GripperEvent::ReleaseRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("ReleaseRequested"));
    }

    const common::Mm release_target =
        common::Mm{std::max(current_nut_stroke_.value -
                                command.release_distance.value,
                            profile_.travel_limits.software_open_limit.value)};
    const common::MmPerS release_speed{
        -std::abs(command.release_nut_speed.value)};
    const auto motion = runMotionUntil(MotionWaitOptions{
        release_target,
        release_speed,
        config_.safety.homing_current_limit,
        command.timeout,
        true,
        false,
        false,
        {}});
    if (motion.result.isError()) {
      state_machine_.dispatch(GripperEvent::ReleaseFailed);
      return fail(motion.result);
    }

    estimated_clamp_force_ = {};
    contact_detected_ = false;
    state_machine_.dispatch(GripperEvent::ReleaseCompleted);
    state_machine_.dispatch(GripperEvent::DisableRequested);
    state_machine_.dispatch(GripperEvent::DisableCompleted);
    updateSnapshotFlags();
    std::ostringstream message;
    message << "release completed target_mm=" << release_target.value
            << " measured_delta_mm=" << motion.measured_distance.value;
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  common::Result moveToNutStroke(const MoveNutStrokeCommand& command) override {
    clearOperationCancel();
    const auto precheck = requireManualPositioningPreconditions();
    if (precheck.isError()) {
      return precheck;
    }

    const auto allowed_range = currentManualPositioningRange(
        command.unloaded_or_structure_removed_confirmed);
    if (!allowed_range.valid) {
      return fail(common::ErrorCode::ControlNotReady,
                  "no valid nut-stroke range is available for manual positioning");
    }
    if (command.target_nut_stroke.value < allowed_range.open_limit.value ||
        command.target_nut_stroke.value > allowed_range.closed_limit.value) {
      std::ostringstream message;
      message << "target nut stroke is outside allowed range"
              << " target_mm=" << command.target_nut_stroke.value
              << " min_mm=" << allowed_range.open_limit.value
              << " max_mm=" << allowed_range.closed_limit.value
              << " confidence=" << allowed_range.confidence;
      return fail(common::ErrorCode::OutOfRange, message.str());
    }

    const common::MmPerS nut_speed =
        resolveManualPositioningSpeed(command);
    if (nut_speed.value <= 0.0) {
      return fail(common::ErrorCode::ControlTargetInvalid,
                  "manual positioning speed resolved to zero");
    }
    const common::A current_limit =
        resolveManualPositioningCurrent(command);
    if (current_limit.value <= 0.0) {
      return fail(common::ErrorCode::ControlTargetInvalid,
                  "manual positioning current limit resolved to zero");
    }

    const auto ready = prepareManualPositioningOutput();
    if (ready.isError()) {
      return ready;
    }

    if (state_machine_.dispatch(GripperEvent::MoveNutStrokeRequested) !=
        state_machine::TransitionDecision::Accepted) {
      return fail(rejectedTransition("MoveNutStrokeRequested"));
    }

    const double delta =
        command.target_nut_stroke.value - current_nut_stroke_.value;
    const common::MmPerS signed_speed{
        delta >= 0.0 ? nut_speed.value : -nut_speed.value};
    const auto motion = runManualPositioningUntil(command.target_nut_stroke,
                                                  signed_speed, current_limit,
                                                  command.timeout,
                                                  allowed_range);
    if (motion.result.isError()) {
      if (isSafetyStopResult(motion.result)) {
        state_machine_.dispatch(GripperEvent::MoveNutStrokeFailed);
      } else {
        state_machine_.dispatch(GripperEvent::MoveNutStrokeCompleted);
        state_machine_.dispatch(GripperEvent::DisableCompleted);
      }
      updateSnapshotFlags();
      return isSafetyStopResult(motion.result)
                 ? fail(motion.result)
                 : finishFailureWithoutFault(motion.result);
    }

    state_machine_.dispatch(GripperEvent::MoveNutStrokeCompleted);
    state_machine_.dispatch(GripperEvent::DisableCompleted);
    updateSnapshotFlags();
    std::ostringstream message;
    message << "manual nut-stroke move completed"
            << " target_mm=" << command.target_nut_stroke.value
            << " start_mm=" << motion.start_stroke.value
            << " end_mm=" << motion.end_stroke.value
            << " measured_delta_mm=" << motion.measured_distance.value
            << " target_motor_pos_rad="
            << motion.target_motor_position.value
            << " start_motor_pos_rad=" << motion.start_motor_position.value
            << " end_motor_pos_rad=" << motion.end_motor_position.value
            << " motor_delta_rad="
            << (motion.end_motor_position.value -
                motion.start_motor_position.value)
            << " motor_delta_rev="
            << ((motion.end_motor_position.value -
                 motion.start_motor_position.value) / kTwoPi)
            << " command_steps=" << motion.command_steps
            << " max_command_step_mm=" << motion.max_command_step.value
            << " mm_per_rev_estimate=";
    const double motor_delta_rev =
        (motion.end_motor_position.value - motion.start_motor_position.value) /
        kTwoPi;
    if (std::abs(motor_delta_rev) > 1e-9) {
      message << (motion.end_stroke.value - motion.start_stroke.value) /
                     motor_delta_rev;
    } else {
      message << "nan";
    }
    message
            << " peak_current_a=" << motion.peak_motor_current.value
            << " confidence=" << allowed_range.confidence
            << " motor_disabled=1";
    return finish(common::Result{common::ErrorCode::Ok, message.str()});
  }

  [[nodiscard]] bool isSafetyStopResult(const common::Result& result) const {
    return result.code() == common::ErrorCode::SafetyActiveStop ||
           result.code() == common::ErrorCode::SafetyJamDetected ||
           result.code() == common::ErrorCode::SafetyContactDetected ||
           result.code() == common::ErrorCode::SafetyLimitExceeded;
  }

  common::Result stop() override {
    requestOperationCancel("active stop requested");
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      MotorCommand command{};
      command.control_mode = MotorControlMode::Disabled;
      command.enable = false;
      (void)motor_->sendCommand(command);
      (void)motor_->disable();
    }
    state_machine_.dispatch(GripperEvent::StopRequested);
    updateSnapshotFlags();
    return finish(common::Result::error(common::ErrorCode::SafetyActiveStop,
                                        "active stop requested"));
  }

  common::Result clearFault() override {
    if (motor_ && motor_->isConnected()) {
      MotorCommand command{};
      command.control_mode = MotorControlMode::Disabled;
      command.enable = false;
      command.clear_fault = true;
      (void)motor_->sendCommand(command);
    }
    if (state_machine_.state() == GripperTopState::Fault ||
        state_machine_.state() == GripperTopState::ActiveStop) {
      state_machine_.dispatch(GripperEvent::FaultCleared);
    }
    snapshot_.fault = {};
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  common::Result update() override {
    const auto result = updateFeedback();
    if (result.isError()) {
      return fail(result);
    }
    updateSnapshotFlags();
    return finish(common::Ok());
  }

  void setProgressCallback(ProgressCallback callback) override {
    progress_callback_ = std::move(callback);
  }

  [[nodiscard]] GripperStateSnapshot state() const override {
    std::lock_guard<std::mutex> lock{snapshot_mutex_};
    return snapshot_;
  }

 private:
  struct MotionWaitOptions {
    common::Mm target_stroke{};
    common::MmPerS nut_speed{};
    common::A current_limit{};
    common::S timeout{};
    bool use_software_limits{true};
    bool stop_on_contact{false};
    bool allow_contact_success{false};
    common::A force_current_target{};
    bool use_custom_stroke_limits{false};
    common::Mm custom_open_limit{};
    common::Mm custom_closed_limit{};
    bool allow_jam_success{false};
    bool require_no_progress_for_contact_stop{false};
    common::S no_progress_confirm_time{};
    self_check::FrictionAnomalyDetector* friction_anomaly_detector{nullptr};
  };

  struct MotionWaitResult {
    common::Result result{common::Ok()};
    common::Mm start_stroke{};
    common::Mm end_stroke{};
    common::Mm measured_distance{};
    common::MmPerS measured_nut_speed{};
    common::MmPerS max_nut_speed{};
    common::MmPerS max_velocity_tracking_error{};
    common::A average_motor_current{};
    common::A max_motor_current{};
    common::A min_motor_current{};
    common::A max_current_ripple{};
    common::Nm average_motor_torque{};
    common::Nm max_motor_torque{};
    common::Nm min_motor_torque{};
    common::Nm max_torque_ripple{};
    common::DegC max_motor_temperature{};
    std::uint32_t sample_count{0};
    bool target_reached{false};
    bool contact_detected{false};
    bool jam_detected{false};
    bool force_current_reached{false};
  };

  struct ManualPositioningResult {
    common::Result result{common::Ok()};
    common::Mm start_stroke{};
    common::Mm end_stroke{};
    common::Mm measured_distance{};
    common::MmPerS measured_nut_speed{};
    common::A peak_motor_current{};
    common::Mm max_command_step{};
    common::Rad target_motor_position{};
    common::Rad start_motor_position{};
    common::Rad end_motor_position{};
    common::Rad trigger_motor_position{};
    common::A trigger_motor_current{};
    common::RadPerS trigger_motor_velocity{};
    std::uint32_t command_steps{0};
    bool target_reached{false};
    bool current_limit_exceeded{false};
    bool jam_detected{false};
    bool stroke_limit_suspected{false};
  };

  struct MotionSettleResult {
    common::Result result{common::Ok()};
    common::Mm settled_stroke{};
    common::Rad settled_motor_position{};
    std::uint32_t settled_raw_position_counts{0};
    bool settled_raw_position_counts_valid{false};
    common::MmPerS settled_nut_speed{};
    common::MmPerS settled_position_delta_speed{};
    bool settled{false};
  };

  struct SelfCheckMotionProbeResult {
    common::Result result{common::Ok()};
    self_check::MotionIdentificationSample sample{};
    common::Mm start_stroke{};
    common::Mm end_stroke{};
    common::Rad start_motor_position{};
    common::Rad end_motor_position{};
    std::uint32_t start_raw_position_counts{0};
    std::uint32_t end_raw_position_counts{0};
    bool start_raw_position_counts_valid{false};
    bool end_raw_position_counts_valid{false};
    common::Mm measured_distance{};
    common::MmPerS average_nut_speed{};
    common::MmPerS max_nut_speed{};
    common::A average_motor_current{};
    common::A max_motor_current{};
    common::A last_motor_current{};
    common::Nm average_motor_torque{};
    common::Nm max_motor_torque{};
    common::Nm last_motor_torque{};
    common::Rad target_motor_position{};
    common::DegC max_motor_temperature{};
    common::A feedback_hard_current_limit{};
    common::A feedback_emergency_current_limit{};
    common::S hard_current_confirm_time{};
    common::S hard_current_over_limit_duration{};
    common::Mm hard_current_progress_distance{};
    bool limit_suspected{false};
    bool jam_suspected{false};
    bool target_reached{false};
    bool settled{false};
    bool hard_current_transient_observed{false};
    bool hard_current_confirmed{false};
    bool hard_current_no_progress_confirmed{false};
    bool hard_current_progress_observed{false};
    bool emergency_current_confirmed{false};
    bool velocity_collapsed{false};
    bool sustained_no_progress_confirmed{false};
    common::MmPerS settled_nut_speed{};
    common::MmPerS settled_position_delta_speed{};
    common::Result settle_result{common::Ok()};
  };

  struct SelfCheckBreakawayCandidate {
    bool found{false};
    self_check::MotionIdentificationSample sample{};
    common::A current_limit{};
    common::MmPerS target_nut_speed{};
    bool low_confidence_micro_motion{false};
  };

  struct SelfCheckProbeOptions {
    bool stop_after_motion_start{false};
    self_check::FrictionAnomalyDetector* friction_anomaly_detector{nullptr};
    bool suppress_soft_jam_stop{false};
    common::S early_progress_timeout{};
    common::S hard_current_confirm_time{};
    common::S sustained_no_progress_timeout{};
    bool use_custom_stroke_limits{false};
    common::Mm custom_open_limit{};
    common::Mm custom_closed_limit{};
    bool record_pre_b_current_trace{false};
    common::A command_current_cap{};
  };

  struct SelfCheckProbeFeedbackRisk {
    bool stop_required{false};
    bool limit_suspected{false};
    bool jam_suspected{false};
    bool velocity_collapsed{false};
    const char* message{"pre-self-check probe stopped by jam or limit feedback"};
  };

  struct ManualPositioningRange {
    common::Mm open_limit{};
    common::Mm closed_limit{};
    bool valid{false};
    bool use_software_limits{false};
    bool low_confidence_window{false};
    const char* confidence{"none"};
  };

  struct PreSelfCheckExpansionBounds {
    common::Mm open_limit{};
    common::Mm closed_limit{};
    bool open_boundary_suspected{false};
    bool closed_boundary_suspected{false};
    bool open_release_recommended{false};
    bool closed_release_recommended{false};
    bool boundary_release_failed{false};
    self_check::MotionDirection unreleased_boundary_direction{
        self_check::MotionDirection::Unknown};
    std::uint32_t open_sample_count{0};
    std::uint32_t closed_sample_count{0};
  };

  struct PreBLearningAnchorCandidate {
    common::Mm anchor{};
    std::uint32_t region_bucket{0};
  };

  struct OperationCurrentLimit {
    common::A limit{};
    std::string source{};
  };

  struct PreSelfCheckDirectionalSearchResult {
    common::Result result{common::Ok()};
    common::Mm observed_limit{};
  };

  struct PreBSoftJamRetryResult {
    common::Result result{common::Ok()};
    bool passed_through{false};
    common::Mm confirmation_distance{};
  };

  [[nodiscard]] common::Result requireConfiguredConnected() {
    if (!configured_) {
      return fail(common::ErrorCode::ConfigMissing,
                  "controller is not configured");
    }
    if (!motor_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "controller has no motor implementation");
    }
    if (!motor_->isConnected()) {
      return fail(common::ErrorCode::ConnectionNotOpen,
                  "motor is not connected");
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result ensureMotorOutputEnabled() {
    if (const auto cancel = checkOperationCancelled("ensure motor output enabled");
        cancel.isError()) {
      return cancel;
    }
    if (!motor_ || !motor_->isConnected()) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "motor is not connected");
    }
    if (motor_->isEnabled()) {
      return common::Ok();
    }
    const auto enable_result = motor_->enable();
    if (enable_result.isError()) {
      return enable_result;
    }
    (void)updateFeedback();
    updateSnapshotFlags();
    return common::Ok();
  }

  [[nodiscard]] common::Result ensureEnabled() {
    if (const auto cancel = checkOperationCancelled("ensure enabled");
        cancel.isError()) {
      return cancel;
    }
    const auto ready = requireConfiguredConnected();
    if (ready.isError()) {
      return ready;
    }
    if (motor_bringup_active_) {
      return fail(common::ErrorCode::InvalidOperation,
                  "exit motor bring-up mode before normal gripper workflow");
    }
    if (!motor_->isEnabled()) {
      if (state_machine_.state() == GripperTopState::Ready) {
        const auto result = motor_->enable();
        if (result.isError()) {
          return fail(result);
        }
        (void)updateFeedback();
        updateSnapshotFlags();
        return common::Ok();
      }
      return enable();
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result requireReadyForWork() {
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    if (!travel_limits_learned_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "travel limits must be learned before work motion");
    }
    if (state_machine_.state() == GripperTopState::Disabled &&
        motion_health_checked_) {
      const auto transition =
          state_machine_.dispatch(GripperEvent::EnableSucceeded);
      if (transition != state_machine::TransitionDecision::Accepted) {
        return fail(rejectedTransition("EnableSucceeded"));
      }
    }
    if (state_machine_.state() == GripperTopState::Enabled &&
        motion_health_checked_) {
      state_machine_.dispatch(GripperEvent::MotionHealthCheckRequested);
      state_machine_.dispatch(GripperEvent::MotionHealthCheckSucceeded);
    }
    if (state_machine_.state() != GripperTopState::Ready) {
      return fail(common::ErrorCode::StateMachineNotReady,
                  "controller is not in Ready state");
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result requireManualPositioningPreconditions() {
    const auto ready = requireConfiguredConnected();
    if (ready.isError()) {
      return ready;
    }
    if (motor_bringup_active_) {
      return fail(common::ErrorCode::InvalidOperation,
                  "exit motor bring-up mode before manual nut-stroke positioning");
    }
    if (!pre_self_check_completed_) {
      return fail(common::ErrorCode::ControlNotReady,
                  "pre-self-check must complete before manual nut-stroke positioning");
    }
    const auto state = state_machine_.state();
    if (state != GripperTopState::Disabled &&
        state != GripperTopState::Enabled &&
        state != GripperTopState::Ready) {
      return fail(common::ErrorCode::StateMachineNotReady,
                  "controller is not ready for manual nut-stroke positioning");
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result prepareManualPositioningOutput() {
    const auto ready = ensureEnabled();
    if (ready.isError()) {
      return ready;
    }
    const auto state = state_machine_.state();
    if (state == GripperTopState::Disabled) {
      const auto transition =
          state_machine_.dispatch(GripperEvent::EnableSucceeded);
      if (transition != state_machine::TransitionDecision::Accepted) {
        return fail(rejectedTransition("EnableSucceeded"));
      }
    } else if (state == GripperTopState::Ready) {
      return common::Ok();
    } else if (state != GripperTopState::Enabled) {
      return fail(common::ErrorCode::StateMachineNotReady,
                  "controller is not ready for manual nut-stroke positioning");
    }
    return common::Ok();
  }

  [[nodiscard]] ManualPositioningRange currentManualPositioningRange(
      bool unloaded_or_structure_removed_confirmed = false) const {
    ManualPositioningRange range{};
    if (motion_health_checked_) {
      range.open_limit = profile_.travel_limits.software_open_limit;
      range.closed_limit = profile_.travel_limits.software_closed_limit;
      range.use_software_limits = true;
      range.confidence = "motion_health_checked";
    } else if (unloaded_or_structure_removed_confirmed) {
      range.open_limit = profile_.travel_limits.safe_zone_open_limit;
      range.closed_limit = profile_.travel_limits.safe_zone_closed_limit;
      range.use_software_limits = false;
      range.low_confidence_window = false;
      range.confidence = "pre_self_check_unloaded_safe_zone";
    } else {
      range.open_limit = profile_.travel_limits.safe_zone_open_limit;
      range.closed_limit = profile_.travel_limits.safe_zone_closed_limit;
      range.use_software_limits = false;
      range.low_confidence_window = true;
      range.confidence = "pre_self_check_safe_zone";
    }
    if (range.closed_limit.value < range.open_limit.value) {
      std::swap(range.open_limit, range.closed_limit);
    }
    range.valid =
        pre_self_check_completed_ &&
        range.closed_limit.value > range.open_limit.value;
    return range;
  }

  [[nodiscard]] common::MmPerS resolveManualPositioningSpeed(
      const MoveNutStrokeCommand& command) const {
    const double requested =
        command.max_nut_speed.value > 0.0
            ? command.max_nut_speed.value
            : config_.self_check.travel_learning_speed.value;
    const double opening_min_speed =
        profile_.opening.stable_speed_sample_count > 0U
            ? std::abs(profile_.opening.minimum_stable_nut_speed.value)
            : 0.0;
    const double closing_min_speed =
        profile_.closing.stable_speed_sample_count > 0U
            ? std::abs(profile_.closing.minimum_stable_nut_speed.value)
            : 0.0;
    const double learned_min_speed = std::max(
        {std::abs(config_.self_check.min_speed_scan_start.value),
         opening_min_speed, closing_min_speed});
    const double lower_bounded =
        std::max(std::abs(requested), learned_min_speed);
    const double confidence_cap =
        motion_health_checked_
            ? std::abs(config_.safety.max_nut_speed.value)
            : std::min(std::abs(config_.self_check.min_speed_scan_stop.value),
                       std::abs(config_.safety.max_nut_speed.value));
    return common::MmPerS{std::min(lower_bounded, confidence_cap)};
  }

  [[nodiscard]] common::A resolveManualPositioningCurrent(
      const MoveNutStrokeCommand& command) const {
    const double default_current =
        config_.safety.manual_positioning_current_limit.value;
    const double requested = command.max_motor_current.value > 0.0
                                 ? command.max_motor_current.value
                                 : default_current;
    const double confidence_cap =
        motion_health_checked_
            ? config_.safety.manual_positioning_current_limit.value
            : config_.safety.manual_positioning_current_limit.value;
    return common::A{std::min({std::abs(requested),
                               std::abs(confidence_cap),
                               std::abs(config_.safety.max_motor_current.value)})};
  }

  [[nodiscard]] common::Result requireMotorBringupActive() {
    const auto ready = requireConfiguredConnected();
    if (ready.isError()) {
      return ready;
    }
    if (!motor_bringup_active_ || !motor_bringup_unloaded_confirmed_) {
      return fail(common::ErrorCode::InvalidOperation,
                  "motor bring-up mode is not active");
    }
    if (isActiveMotionState(state_machine_.state())) {
      return fail(common::ErrorCode::InvalidOperation,
                  "motor bring-up command rejected while motion is active");
    }
    return common::Ok();
  }

  void clearOperationCancel() {
    operation_cancel_requested_.store(false, std::memory_order_release);
  }

  void requestOperationCancel(const char* reason) {
    operation_cancel_requested_.store(true, std::memory_order_release);
    if (reason != nullptr) {
      reportProgress(std::string{"controller cancel requested | "} + reason);
    }
  }

  [[nodiscard]] bool operationCancelRequested() const {
    return operation_cancel_requested_.load(std::memory_order_acquire) ||
           state_machine_.state() == GripperTopState::ActiveStop ||
           state_machine_.state() == GripperTopState::Fault;
  }

  [[nodiscard]] common::Result checkOperationCancelled(
      const char* context) const {
    if (!operationCancelRequested()) {
      return common::Ok();
    }
    std::ostringstream message;
    message << "operation cancelled";
    if (context != nullptr) {
      message << " context=" << context;
    }
    return common::Result::error(common::ErrorCode::SafetyActiveStop,
                                 message.str());
  }

  [[nodiscard]] common::Result abortPreSelfCheck(
      const common::Result& reason) {
    reportProgress(std::string{"PreSelfCheck | cancelled | "} +
                   reason.message());
    (void)disableAfterMotion();
    state_machine_.dispatch(GripperEvent::StopRequested);
    updateSnapshotFlags();
    return fail(reason);
  }

  [[nodiscard]] MotorBringupJogCommand limitedMotorBringupJog(
      const MotorBringupJogCommand& command) const {
    MotorBringupJogCommand output = command;

    const double default_position =
        config_.motor_bringup.default_relative_motor_position.value;
    const double max_position = std::abs(
        config_.motor_bringup.max_relative_motor_position.value);
    const double requested_position =
        command.relative_motor_position.value == 0.0
            ? static_cast<double>(command.motor_direction_sign >= 0 ? 1 : -1) *
                  default_position
            : command.relative_motor_position.value;
    output.relative_motor_position = common::Rad{
        std::clamp(requested_position, -max_position, max_position)};

    const double requested_velocity =
        command.max_motor_velocity.value > 0.0
            ? command.max_motor_velocity.value
            : config_.motor_bringup.default_motor_velocity.value;
    output.max_motor_velocity = common::RadPerS{std::min(
        std::abs(requested_velocity),
        std::abs(config_.motor_bringup.max_motor_velocity.value))};

    const double requested_current =
        command.max_motor_current.value > 0.0
            ? command.max_motor_current.value
            : config_.motor_bringup.default_motor_current.value;
    output.max_motor_current = common::A{std::min(
        std::abs(requested_current),
        std::abs(config_.motor_bringup.max_motor_current.value))};

    const double requested_duration =
        command.pulse_duration.value > 0.0
            ? command.pulse_duration.value
            : config_.motor_bringup.default_pulse_duration.value;
    output.pulse_duration = common::S{std::min(
        std::abs(requested_duration),
        std::abs(config_.motor_bringup.max_pulse_duration.value))};
    return output;
  }

  [[nodiscard]] common::Result validateMotorBringupJog(
      const MotorBringupJogCommand& command) const {
    (void)command;
    if (config_.motor_bringup.max_relative_motor_position.value <= 0.0 ||
        config_.motor_bringup.max_motor_velocity.value <= 0.0 ||
        config_.motor_bringup.max_motor_current.value <= 0.0 ||
        config_.motor_bringup.max_pulse_duration.value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up max direction-window/velocity/current/duration must be positive");
    }
    if (config_.motor_bringup.max_motor_current.value >
        config_.safety.max_motor_current.value) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up current must not exceed global motor current limit");
    }
    if (config_.motor_bringup.max_motor_velocity.value >
        config_.motor.max_velocity.value) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up velocity must not exceed motor velocity limit");
    }
    const auto limited = limitedMotorBringupJog(command);
    if (std::abs(limited.relative_motor_position.value) <= 0.0 ||
        limited.max_motor_velocity.value <= 0.0 ||
        limited.max_motor_current.value <= 0.0 ||
        limited.pulse_duration.value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ControlTargetInvalid,
          "motor bring-up velocity jog resolved to zero direction, speed, current, or duration");
    }
    return common::Ok();
  }

  [[nodiscard]] MotorBringupPositionMoveCommand limitedMotorBringupPositionMove(
      const MotorBringupPositionMoveCommand& command) const {
    MotorBringupPositionMoveCommand output = command;

    const double default_revolutions =
        config_.motor_bringup.default_relative_motor_revolutions.value;
    const double max_revolutions =
        std::abs(config_.motor_bringup.max_relative_motor_revolutions.value);
    const double requested_revolutions =
        command.relative_motor_revolutions.value == 0.0
            ? default_revolutions
            : command.relative_motor_revolutions.value;
    output.relative_motor_revolutions = common::Ratio{std::clamp(
        requested_revolutions, -max_revolutions, max_revolutions)};

    const double requested_velocity =
        command.max_motor_velocity.value > 0.0
            ? command.max_motor_velocity.value
            : config_.motor_bringup.default_motor_velocity.value;
    output.max_motor_velocity = common::RadPerS{std::min(
        std::abs(requested_velocity),
        std::abs(config_.motor_bringup.max_motor_velocity.value))};

    const double requested_current =
        command.feedback_current_limit.value > 0.0
            ? command.feedback_current_limit.value
            : config_.motor_bringup.default_motor_current.value;
    output.feedback_current_limit = common::A{std::min(
        std::abs(requested_current),
        std::abs(config_.motor_bringup.max_motor_current.value))};

    const double requested_timeout =
        command.timeout.value > 0.0
            ? command.timeout.value
            : config_.motor_bringup.default_position_move_timeout.value;
    output.timeout = common::S{requested_timeout > 0.0
                                   ? std::min(std::abs(requested_timeout),
                                              std::abs(config_.motor_bringup
                                                           .max_position_move_timeout
                                                           .value))
                                   : 0.0};
    return output;
  }

  [[nodiscard]] common::Result validateMotorBringupPositionMove(
      const MotorBringupPositionMoveCommand& command) const {
    (void)command;
    if (config_.motor_bringup.max_relative_motor_revolutions.value <= 0.0 ||
        config_.motor_bringup.max_motor_velocity.value <= 0.0 ||
        config_.motor_bringup.max_motor_current.value <= 0.0 ||
        config_.motor_bringup.max_position_move_timeout.value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up max revolutions/velocity/current/timeout must be positive");
    }
    if (config_.motor_bringup.max_motor_current.value >
        config_.safety.max_motor_current.value) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up feedback current must not exceed global motor current limit");
    }
    if (config_.motor_bringup.max_motor_velocity.value >
        config_.motor.max_velocity.value) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "motor bring-up velocity must not exceed motor velocity limit");
    }
    const auto limited = limitedMotorBringupPositionMove(command);
    if (std::abs(limited.relative_motor_revolutions.value) <= 0.0 ||
        limited.max_motor_velocity.value <= 0.0 ||
        limited.feedback_current_limit.value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ControlTargetInvalid,
          "motor bring-up position move resolved to zero revolutions, speed, or current");
    }
    if (motorBringupPositionMoveTimeout(limited).value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ControlTargetInvalid,
          "motor bring-up position move timeout resolved to zero");
    }
    return common::Ok();
  }

  [[nodiscard]] common::S motorBringupPositionMoveTimeout(
      const MotorBringupPositionMoveCommand& limited) const {
    if (limited.timeout.value > 0.0) {
      return limited.timeout;
    }
    const double motion_time_s =
        std::abs(limited.relative_motor_revolutions.value * kTwoPi) /
        std::max(limited.max_motor_velocity.value, 1e-9);
    const double estimated_timeout_s =
        motion_time_s + std::max(1.0, motion_time_s * 0.25);
    return common::S{std::min(
        estimated_timeout_s,
        std::abs(config_.motor_bringup.max_position_move_timeout.value))};
  }

  [[nodiscard]] common::Result validateMotorBringupTargetPosition(
      common::Rad start_position, common::Rad target_position,
      common::Ratio relative_motor_revolutions) const {
    const double limit = last_feedback_.runtime_limits_valid
                             ? std::abs(last_feedback_.runtime_position_limit.value)
                             : std::abs(config_.motor.max_position.value);
    if (limit <= 0.0) {
      return common::Ok();
    }
    if (target_position.value >= -limit && target_position.value <= limit) {
      return common::Ok();
    }
    std::ostringstream message;
    message << "bring-up position target exceeds runtime P_MAX window"
            << " start_motor_pos_rad=" << start_position.value
            << " requested_rev=" << relative_motor_revolutions.value
            << " target_motor_pos_rad=" << target_position.value
            << " pmax_rad=" << limit
            << " allowed_min_rad=" << -limit
            << " allowed_max_rad=" << limit
            << " action=move_opposite_direction_or_increase_motor_pmax";
    return common::Result::error(common::ErrorCode::OutOfRange,
                                 message.str());
  }

  [[nodiscard]] bool motorPositionTargetReached(
      common::Rad target_position, common::RadPerS command_velocity) const {
    const double position_tolerance =
        std::max({config_.self_check.fallback_motor_position_noise.value * 5.0,
                  std::abs(command_velocity.value) *
                      config_.motor.feedback_poll_period.value * 0.5,
                  0.005});
    const double velocity_tolerance =
        std::max(config_.self_check.fallback_motor_velocity_noise.value * 5.0,
                 0.02);
    return std::abs(last_feedback_.position.value - target_position.value) <=
               position_tolerance &&
           std::abs(last_feedback_.velocity.value) <= velocity_tolerance;
  }

  [[nodiscard]] common::MmPerS resolveClampForceNutSpeed(
      const ClampForceCommand& command) const {
    const common::MmPerS nut_speed_limit =
        command.max_nut_speed.value > 0.0 ? command.max_nut_speed
                                          : config_.clamp.target_nut_speed;
    if (command.speed_mode == ClampSpeedMode::NutLinearSpeed) {
      return nut_speed_limit;
    }

    const common::RadPerS target_angular_speed =
        command.target_gripper_angular_speed.value > 0.0
            ? command.target_gripper_angular_speed
            : config_.clamp.target_gripper_angular_speed;
    const common::MmPerS mapped_nut_speed =
        kinematics_.angularSpeedToNutSpeed(target_angular_speed,
                                           current_nut_stroke_);
    if (mapped_nut_speed.value <= 0.0) {
      return nut_speed_limit;
    }
    return common::MmPerS{
        std::min(mapped_nut_speed.value, nut_speed_limit.value)};
  }

  [[nodiscard]] common::S motionTimeoutFor(common::Mm start_stroke,
                                           common::Mm target_stroke,
                                           common::MmPerS nut_speed,
                                           common::S requested_timeout) const {
    if (requested_timeout.value > 0.0) {
      return requested_timeout;
    }
    const double speed = std::max(std::abs(nut_speed.value), 0.05);
    const double distance =
        std::abs(target_stroke.value - start_stroke.value);
    const double settle_budget =
        std::max(0.5, config_.self_check.motion_settle_timeout.value);
    const double proportional_margin =
        std::max(0.5, distance / speed * 0.5);
    return common::S{std::max(0.5, distance / speed + settle_budget +
                                       proportional_margin)};
  }

  [[nodiscard]] common::MmPerS measuredNutSpeed() const {
    return motorVelocityToNutSpeed(last_feedback_.velocity,
                                   config_.mechanism.lead_screw_pitch,
                                   effectiveNutPositionDirectionSign());
  }

  [[nodiscard]] bool isTargetReached(common::Mm target_stroke,
                                     common::MmPerS nut_speed) const {
    const double tolerance = std::max(
        config_.self_check.max_distance_error.value,
        profile_.noise_floor.nut_stroke_noise.value);
    if (nut_speed.value >= 0.0) {
      return current_nut_stroke_.value >= target_stroke.value - tolerance;
    }
    return current_nut_stroke_.value <= target_stroke.value + tolerance;
  }

  [[nodiscard]] bool isSelfCheckProbeTargetReached(
      common::Mm start_stroke, common::Mm target_stroke,
      common::MmPerS nut_speed) const {
    const double commanded_distance =
        std::abs(target_stroke.value - start_stroke.value);
    const double tolerance = std::min(
        config_.self_check.max_distance_error.value,
        std::max(profile_.noise_floor.nut_stroke_noise.value * 3.0,
                 commanded_distance * 0.25));
    if (nut_speed.value >= 0.0) {
      return current_nut_stroke_.value >= target_stroke.value - tolerance;
    }
    return current_nut_stroke_.value <= target_stroke.value + tolerance;
  }

  [[nodiscard]] bool isManualPositioningTargetReached(
      common::Mm target_stroke, common::MmPerS nut_speed) const {
    const double tolerance = std::min(
        config_.self_check.max_distance_error.value,
        std::max({profile_.noise_floor.nut_stroke_noise.value * 3.0,
                  config_.self_check.fallback_nut_stroke_noise.value * 3.0,
                  0.01}));
    if (nut_speed.value >= 0.0) {
      return current_nut_stroke_.value >= target_stroke.value - tolerance;
    }
    return current_nut_stroke_.value <= target_stroke.value + tolerance;
  }

  [[nodiscard]] common::S manualPositioningControlPeriod() const {
    const double configured = config_.motor.feedback_poll_period.value;
    return common::S{std::clamp(configured > 0.0 ? configured : 0.02,
                                0.005, 0.05)};
  }

  [[nodiscard]] common::Result disableAfterMotion() {
    previous_commanded_nut_speed_ = {};
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      const auto disable_result = motor_->disable();
      if (disable_result.isError()) {
        return disable_result;
      }
    }
    (void)updateFeedback();
    return common::Ok();
  }

  [[nodiscard]] MotionSettleResult disableAndSampleEndpointFeedback() {
    MotionSettleResult output{};
    output.settled_stroke = current_nut_stroke_;
    output.settled_motor_position = last_feedback_.position;
    output.settled_raw_position_counts = last_feedback_.raw_position_counts;
    output.settled_raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    output.settled_nut_speed = measuredNutSpeed();

    previous_commanded_nut_speed_ = {};
    if (motor_ && motor_->isConnected() && motor_->isEnabled()) {
      const auto disable_result = motor_->disable();
      if (disable_result.isError()) {
        output.result = disable_result;
        return output;
      }
    }

    const auto feedback_result = updateFeedback();
    if (feedback_result.isError()) {
      output.result = feedback_result;
      return output;
    }
    output.settled_stroke = current_nut_stroke_;
    output.settled_motor_position = last_feedback_.position;
    output.settled_raw_position_counts = last_feedback_.raw_position_counts;
    output.settled_raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    output.settled_nut_speed = measuredNutSpeed();
    output.settled = true;
    output.result = common::Ok();
    updateSnapshotFlags();
    return output;
  }

  [[nodiscard]] common::Mm endpointProgressThreshold() const {
    return common::Mm{std::max(
        {config_.self_check.motion_start_distance.value,
         config_.self_check.fallback_nut_stroke_noise.value * 3.0,
         0.02})};
  }

  [[nodiscard]] common::Mm selfCheckNoProgressThreshold() const {
    return common::Mm{std::max(
        {config_.self_check.low_confidence_motion_distance.value,
         config_.self_check.fallback_nut_stroke_noise.value * 3.0,
         0.02})};
  }

  [[nodiscard]] bool madeProgressInCommandDirection(
      common::Mm reference_stroke, common::MmPerS nut_speed,
      common::Mm threshold) const {
    if (nut_speed.value < 0.0) {
      return current_nut_stroke_.value <=
             reference_stroke.value - threshold.value;
    }
    return current_nut_stroke_.value >=
           reference_stroke.value + threshold.value;
  }

  [[nodiscard]] bool homingEndpointStallFeedback(
      const MotionWaitOptions& options) const {
    const double current_margin =
        std::max(config_.self_check.max_current_ripple.value,
                 config_.self_check.fallback_motor_current_noise.value * 3.0);
    const double current_threshold = std::max(
        0.05, std::abs(options.current_limit.value) - current_margin);
    const double speed_threshold = std::max(
        std::abs(config_.safety.jam_speed_threshold.value),
        std::abs(config_.self_check.motion_settle_speed_threshold.value));
    return std::abs(last_feedback_.current.value) >= current_threshold &&
           std::abs(measuredNutSpeed().value) <= speed_threshold;
  }

  [[nodiscard]] const self_check::DirectionalStructureProfile&
  directionalProfile(self_check::MotionDirection direction) const {
    return direction == self_check::MotionDirection::Opening ? profile_.opening
                                                             : profile_.closing;
  }

  [[nodiscard]] std::optional<common::A> learnedMotionCurrent(
      self_check::MotionDirection direction) const {
    const auto& profile = directionalProfile(direction);
    double current = 0.0;
    if (profile.dynamic_friction_sample_count > 0U) {
      current = std::max(current,
                         std::abs(profile.dynamic_friction_current_max.value));
    }
    if (profile.static_friction_sample_count > 0U) {
      current =
          std::max(current, std::abs(profile.static_friction_current.value));
    }
    if (current <= 0.0) {
      return std::nullopt;
    }
    return common::A{current};
  }

  [[nodiscard]] common::A contactBaselineFor(
      common::MmPerS commanded_nut_speed) const {
    const auto direction = directionFromSignedNutSpeed(commanded_nut_speed);
    if (const auto learned = learnedMotionCurrent(direction);
        learned.has_value()) {
      return *learned;
    }
    return directionalProfile(direction).dynamic_friction_current_max;
  }

  [[nodiscard]] double learnedCurrentMargin() const {
    return std::max({config_.self_check.max_current_ripple.value,
                     config_.self_check.fallback_motor_current_noise.value * 3.0,
                     0.05});
  }

  [[nodiscard]] OperationCurrentLimit homingCurrentLimitFromProfile() const {
    const double cap =
        std::min(std::abs(config_.safety.homing_current_limit.value),
                 std::abs(config_.safety.max_motor_current.value));
    if (const auto learned =
            learnedMotionCurrent(self_check::MotionDirection::Opening);
        learned.has_value() && cap > 0.0) {
      const double requested = std::max(
          std::abs(config_.homing.homing_current.value),
          learned->value + learnedCurrentMargin());
      return OperationCurrentLimit{
          common::A{std::min(requested, cap)},
          "learned_structure_profile_capped_by_homing_current_limit"};
    }
    const double fallback = std::abs(config_.homing.homing_current.value);
    return OperationCurrentLimit{
        common::A{cap > 0.0 ? std::min(fallback, cap) : fallback},
        "homing_config"};
  }

  [[nodiscard]] OperationCurrentLimit travelLearningCurrentLimitFromProfile(
      self_check::MotionDirection direction) const {
    const double cap =
        std::min(std::abs(config_.safety.travel_learning_current_limit.value),
                 std::abs(config_.safety.max_motor_current.value));
    if (const auto learned = learnedMotionCurrent(direction);
        learned.has_value() && cap > 0.0) {
      const double requested = std::max(
          std::abs(config_.homing.homing_current.value),
          learned->value + learnedCurrentMargin() +
              std::max(std::abs(config_.safety.jam_current_threshold.value),
                       std::abs(config_.safety.contact_current_rise_threshold.value)));
      return OperationCurrentLimit{
          common::A{std::min(requested, cap)},
          "learned_structure_profile_capped_by_travel_learning_current_limit"};
    }
    return OperationCurrentLimit{common::A{cap},
                                 "travel_learning_config"};
  }

  [[nodiscard]] OperationCurrentLimit motionHealthCurrentLimitFromProfile(
      self_check::MotionDirection direction) const {
    const double cap =
        std::min(std::abs(config_.safety.travel_learning_current_limit.value),
                 std::abs(config_.safety.max_motor_current.value));
    if (const auto learned = learnedMotionCurrent(direction);
        learned.has_value() && cap > 0.0) {
      const double requested = std::max(
          0.2, learned->value + learnedCurrentMargin() +
                   std::abs(config_.safety.contact_current_rise_threshold.value));
      return OperationCurrentLimit{
          common::A{std::min(requested, cap)},
          "learned_structure_profile_capped_by_health_current_limit"};
    }
    return OperationCurrentLimit{common::A{cap},
                                 "travel_learning_config"};
  }

  [[nodiscard]] MotionWaitResult completeEndpointMotion(
      MotionWaitResult output, const char* reason, double confirm_duration_s,
      common::A trigger_current, common::MmPerS trigger_speed,
      common::Mm trigger_stroke) {
    const auto endpoint_sample = disableAndSampleEndpointFeedback();
    if (endpoint_sample.result.isError()) {
      output.result = endpoint_sample.result;
      return output;
    }
    output.end_stroke = endpoint_sample.settled_stroke;
    output.measured_nut_speed = endpoint_sample.settled_nut_speed;
    output.measured_distance = common::Mm{
        std::abs(output.end_stroke.value - output.start_stroke.value)};
    output.jam_detected = true;
    output.result = common::Ok();

    std::ostringstream message;
    message << "HomingOpenStop | opening endpoint detected"
            << " reason=" << (reason != nullptr ? reason : "unknown")
            << " confirm_s=" << confirm_duration_s
            << " trigger_stroke_mm=" << trigger_stroke.value
            << " trigger_current_a=" << trigger_current.value
            << " trigger_speed_mm_s=" << trigger_speed.value
            << " sampled_stroke_mm=" << output.end_stroke.value
            << " sampled_motor_rad="
            << endpoint_sample.settled_motor_position.value
            << " motor_disabled=1";
    reportProgress(message.str());
    return output;
  }

  void observeMotionWaitFeedback(MotionWaitResult* output,
                                 common::MmPerS commanded_nut_speed) const {
    if (output == nullptr) {
      return;
    }
    const double measured_speed = std::abs(measuredNutSpeed().value);
    const double current = std::abs(last_feedback_.current.value);
    const double torque = std::abs(last_feedback_.torque.value);
    if (output->sample_count == 0U) {
      output->min_motor_current = common::A{current};
      output->max_motor_current = common::A{current};
      output->min_motor_torque = common::Nm{torque};
      output->max_motor_torque = common::Nm{torque};
      output->max_motor_temperature = last_feedback_.temperature;
    } else {
      output->min_motor_current.value =
          std::min(output->min_motor_current.value, current);
      output->max_motor_current.value =
          std::max(output->max_motor_current.value, current);
      output->min_motor_torque.value =
          std::min(output->min_motor_torque.value, torque);
      output->max_motor_torque.value =
          std::max(output->max_motor_torque.value, torque);
      output->max_motor_temperature.value =
          std::max(output->max_motor_temperature.value,
                   last_feedback_.temperature.value);
    }

    ++output->sample_count;
    const double count = static_cast<double>(output->sample_count);
    output->average_motor_current.value +=
        (current - output->average_motor_current.value) / count;
    output->average_motor_torque.value +=
        (torque - output->average_motor_torque.value) / count;
    output->max_nut_speed.value =
        std::max(output->max_nut_speed.value, measured_speed);
    if (measured_speed > config_.self_check.motion_settle_speed_threshold.value) {
      output->max_velocity_tracking_error.value = std::max(
          output->max_velocity_tracking_error.value,
          std::abs(measured_speed - std::abs(commanded_nut_speed.value)));
    }
    output->max_current_ripple.value =
        std::max(0.0, output->max_motor_current.value -
                          output->min_motor_current.value);
    output->max_torque_ripple.value =
        std::max(0.0, output->max_motor_torque.value -
                          output->min_motor_torque.value);
  }

  [[nodiscard]] MotionSettleResult stopAndWaitForSettledFeedback() {
    // Motion results are sampled after output is disabled and residual velocity
    // has stayed below the configured still threshold. This keeps callers from
    // using in-flight encoder frames as final positions.
    MotionSettleResult output{};
    const common::Timestamp feedback_timestamp_before_stop =
        last_feedback_.timestamp;
    output.settled_stroke = current_nut_stroke_;
    output.settled_motor_position = last_feedback_.position;
    output.settled_raw_position_counts = last_feedback_.raw_position_counts;
    output.settled_raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    output.settled_nut_speed = measuredNutSpeed();
    const auto disable_result = disableAfterMotion();
    if (disable_result.isError()) {
      output.result = disable_result;
      return output;
    }

    const double timeout_s = std::max(
        0.0, config_.self_check.motion_settle_timeout.value);
    const double stable_time_s = std::max(
        0.0, config_.self_check.motion_settle_stable_time.value);
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>{timeout_s});
    std::chrono::steady_clock::time_point still_since{};
    bool still_timer_running = false;
    const common::Mm reference_stroke = current_nut_stroke_;
    common::Mm previous_feedback_stroke = current_nut_stroke_;
    common::Timestamp previous_feedback_timestamp = last_feedback_.timestamp;
    bool previous_feedback_sample_valid = false;

    while (true) {
      if (const auto cancel = checkOperationCancelled("stop and wait for settled feedback");
          cancel.isError()) {
        output.result = cancel;
        updateSnapshotFlags();
        return output;
      }
      const auto feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        output.result = feedback_result;
        return output;
      }
      const auto now = std::chrono::steady_clock::now();
      if (last_feedback_.timestamp <= feedback_timestamp_before_stop) {
        if (now >= deadline) {
          output.result = common::Result::error(
              common::ErrorCode::FeedbackTimedOut,
              "fresh feedback after stop command was not received before settle timeout");
          updateSnapshotFlags();
          return output;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
        continue;
      }

      output.settled_stroke = current_nut_stroke_;
      output.settled_motor_position = last_feedback_.position;
      output.settled_raw_position_counts = last_feedback_.raw_position_counts;
      output.settled_raw_position_counts_valid =
          last_feedback_.raw_position_counts_valid;
      output.settled_nut_speed = measuredNutSpeed();
      output.settled_position_delta_speed = {};
      bool position_delta_speed_valid = false;
      if (previous_feedback_sample_valid &&
          last_feedback_.timestamp > previous_feedback_timestamp) {
        const double dt_s =
            (last_feedback_.timestamp - previous_feedback_timestamp).seconds();
        if (dt_s > 0.0) {
          output.settled_position_delta_speed = common::MmPerS{
              (output.settled_stroke.value - previous_feedback_stroke.value) /
              dt_s};
          position_delta_speed_valid = true;
        }
      }
      previous_feedback_stroke = output.settled_stroke;
      previous_feedback_timestamp = last_feedback_.timestamp;
      previous_feedback_sample_valid = true;

      const double stroke_delta =
          output.settled_stroke.value - reference_stroke.value;
      const double position_still_threshold = std::max(
          config_.self_check.motion_start_distance.value,
          config_.self_check.fallback_nut_stroke_noise.value * 3.0);
      const bool simulated_instant_move =
          std::abs(output.settled_nut_speed.value) <=
              config_.self_check.fallback_nut_stroke_noise.value &&
          std::abs(stroke_delta) >= position_still_threshold;
      if (simulated_instant_move) {
        output.settled = true;
        output.result = common::Ok();
        updateSnapshotFlags();
        return output;
      }

      const double speed_threshold = std::max(
          config_.self_check.motion_settle_speed_threshold.value,
          config_.self_check.fallback_nut_stroke_noise.value);
      const bool feedback_speed_is_still =
          std::abs(output.settled_nut_speed.value) <= speed_threshold;
      const bool position_delta_speed_is_still =
          position_delta_speed_valid &&
          std::abs(output.settled_position_delta_speed.value) <=
              speed_threshold;
      const bool speed_is_still =
          feedback_speed_is_still || position_delta_speed_is_still;
      if (speed_is_still) {
        if (!still_timer_running) {
          still_timer_running = true;
          still_since = now;
        }
        const double still_duration_s =
            std::chrono::duration<double>{now - still_since}.count();
        if (still_duration_s >= stable_time_s) {
          output.settled = true;
          output.result = common::Ok();
          updateSnapshotFlags();
          return output;
        }
      } else {
        still_timer_running = false;
      }

      if (now >= deadline) {
        std::ostringstream message;
        message << "motion did not settle before final feedback sampling"
                << " timeout_s=" << timeout_s
                << " speed_mm_s=" << output.settled_nut_speed.value
                << " position_delta_speed_mm_s="
                << output.settled_position_delta_speed.value
                << " still_speed_threshold_mm_s=" << speed_threshold;
        output.result = common::Result::error(
            common::ErrorCode::OperationTimedOut, message.str());
        updateSnapshotFlags();
        return output;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
  }

  [[nodiscard]] common::Result sendSelfCheckHoldCommand(
      common::Mm hold_stroke, common::A current_limit) {
    const auto ready = ensureMotorOutputEnabled();
    if (ready.isError()) {
      return ready;
    }
    const common::A hold_current{
        std::min(std::max(std::abs(current_limit.value),
                          std::abs(config_.self_check.motion_hold_current.value)),
                 std::abs(config_.safety.self_check_current_limit.value))};
    const common::A hard_limit = selfCheckFeedbackHardCurrentLimit();
    const common::A bounded_hold_current{
        hard_limit.value > 0.0
            ? std::min(hold_current.value, std::abs(hard_limit.value))
            : hold_current.value};
    const common::MmPerS hold_speed{
        std::max(0.0, config_.self_check.motion_hold_speed.value)};
    return sendNutMotionCommand(hold_stroke, hold_speed, bounded_hold_current);
  }

  [[nodiscard]] common::A selfCheckCommandCurrentLimit(
      common::A requested_current,
      common::A command_current_cap = {}) const {
    const double requested = std::abs(requested_current.value);
    const double configured =
        command_current_cap.value > 0.0
            ? std::abs(command_current_cap.value)
            : std::abs(config_.safety.self_check_current_limit.value);
    const double global_cap = std::abs(config_.safety.max_motor_current.value);
    const double cap =
        global_cap > 0.0 ? std::min(configured, global_cap) : configured;
    if (cap <= 0.0) {
      return common::A{requested};
    }
    if (requested <= 0.0) {
      return common::A{cap};
    }
    return common::A{std::min(requested, cap)};
  }

  [[nodiscard]] MotionSettleResult holdAndWaitForSelfCheckSettledFeedback(
      common::A current_limit) {
    // PreSelfCheck must not release the mechanism between tiny probes. Hold the
    // current motor position with the same low-current envelope, then use fresh
    // hardware feedback to decide whether the mechanism is actually still.
    MotionSettleResult output{};
    const auto feedback_before = updateFeedback();
    if (feedback_before.isError()) {
      output.result = feedback_before;
      return output;
    }
    const common::Mm hold_stroke = current_nut_stroke_;
    const common::Timestamp feedback_timestamp_before_hold =
        last_feedback_.timestamp;
    output.settled_stroke = current_nut_stroke_;
    output.settled_motor_position = last_feedback_.position;
    output.settled_raw_position_counts = last_feedback_.raw_position_counts;
    output.settled_raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    output.settled_nut_speed = measuredNutSpeed();
    const auto hold_result = sendSelfCheckHoldCommand(hold_stroke, current_limit);
    if (hold_result.isError()) {
      output.result = hold_result;
      return output;
    }

    const double timeout_s = std::max(
        0.0, config_.self_check.motion_settle_timeout.value);
    const double stable_time_s = std::max(
        0.0, config_.self_check.motion_settle_stable_time.value);
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>{timeout_s});
    std::chrono::steady_clock::time_point still_since{};
    std::chrono::steady_clock::time_point last_hold_refresh =
        std::chrono::steady_clock::now();
    bool still_timer_running = false;
    common::Mm previous_feedback_stroke = current_nut_stroke_;
    common::Timestamp previous_feedback_timestamp = last_feedback_.timestamp;
    bool previous_feedback_sample_valid = false;

    while (true) {
      if (const auto cancel = checkOperationCancelled("self-check hold settle wait");
          cancel.isError()) {
        output.result = cancel;
        updateSnapshotFlags();
        return output;
      }
      const auto feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        output.result = feedback_result;
        return output;
      }
      const auto now = std::chrono::steady_clock::now();
      if (last_feedback_.timestamp <= feedback_timestamp_before_hold) {
        if (now >= deadline) {
          output.result = common::Result::error(
              common::ErrorCode::FeedbackTimedOut,
              "fresh feedback after self-check hold command was not received before settle timeout");
          updateSnapshotFlags();
          return output;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
        continue;
      }
      const double hold_refresh_period_s = std::max(
          0.02, config_.motor.feedback_poll_period.value);
      const double hold_refresh_elapsed_s =
          std::chrono::duration<double>{now - last_hold_refresh}.count();
      if (hold_refresh_elapsed_s >= hold_refresh_period_s) {
        const auto refresh_result =
            sendSelfCheckHoldCommand(hold_stroke, current_limit);
        if (refresh_result.isError()) {
          output.result = refresh_result;
          return output;
        }
        last_hold_refresh = now;
      }

      output.settled_stroke = current_nut_stroke_;
      output.settled_motor_position = last_feedback_.position;
      output.settled_raw_position_counts = last_feedback_.raw_position_counts;
      output.settled_raw_position_counts_valid =
          last_feedback_.raw_position_counts_valid;
      output.settled_nut_speed = measuredNutSpeed();
      output.settled_position_delta_speed = {};
      bool position_delta_speed_valid = false;
      if (previous_feedback_sample_valid &&
          last_feedback_.timestamp > previous_feedback_timestamp) {
        const double dt_s =
            (last_feedback_.timestamp - previous_feedback_timestamp).seconds();
        if (dt_s > 0.0) {
          output.settled_position_delta_speed = common::MmPerS{
              (output.settled_stroke.value - previous_feedback_stroke.value) /
              dt_s};
          position_delta_speed_valid = true;
        }
      }
      previous_feedback_stroke = output.settled_stroke;
      previous_feedback_timestamp = last_feedback_.timestamp;
      previous_feedback_sample_valid = true;

      const double speed_threshold = std::max(
          config_.self_check.motion_settle_speed_threshold.value,
          config_.self_check.fallback_nut_stroke_noise.value);
      const bool feedback_speed_is_still =
          std::abs(output.settled_nut_speed.value) <= speed_threshold;
      const bool position_delta_speed_is_still =
          position_delta_speed_valid &&
          std::abs(output.settled_position_delta_speed.value) <=
              speed_threshold;
      const bool speed_is_still =
          feedback_speed_is_still || position_delta_speed_is_still;
      if (speed_is_still) {
        if (!still_timer_running) {
          still_timer_running = true;
          still_since = now;
        }
        const double still_duration_s =
            std::chrono::duration<double>{now - still_since}.count();
        if (still_duration_s >= stable_time_s) {
          output.settled = true;
          output.result = common::Ok();
          updateSnapshotFlags();
          return output;
        }
      } else {
        still_timer_running = false;
      }

      if (now >= deadline) {
        std::ostringstream message;
        message << "self-check hold did not settle before final feedback sampling"
                << " timeout_s=" << timeout_s
                << " speed_mm_s=" << output.settled_nut_speed.value
                << " position_delta_speed_mm_s="
                << output.settled_position_delta_speed.value
                << " still_speed_threshold_mm_s=" << speed_threshold
                << " hold_stroke_mm=" << hold_stroke.value
                << " hold_speed_mm_s="
                << config_.self_check.motion_hold_speed.value
                << " hold_current_a="
                << std::min(std::max(std::abs(current_limit.value),
                                      std::abs(config_.self_check.motion_hold_current.value)),
                            std::abs(config_.safety.self_check_current_limit.value));
        output.result = common::Result::error(
            common::ErrorCode::OperationTimedOut, message.str());
        updateSnapshotFlags();
        return output;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
  }

  [[nodiscard]] MotionSettleResult disableAndWaitForSelfCheckSettledFeedback() {
    // Fallback for a PositionForce hold that keeps pulling at low current. The
    // sample is still accepted only after fresh post-disable feedback is still.
    return stopAndWaitForSettledFeedback();
  }

  [[nodiscard]] common::Result stopAndReenableForSelfCheck() {
    const auto settle_result = holdAndWaitForSelfCheckSettledFeedback(
        config_.safety.self_check_current_limit);
    if (settle_result.result.isError()) {
      return settle_result.result;
    }
    return reenableAfterSelfCheckProbe();
  }

  [[nodiscard]] common::Result reenableAfterSelfCheckProbe() {
    return ensureMotorOutputEnabled();
  }

  [[nodiscard]] common::Mm provisionalReferenceStroke() const {
    return common::Mm{(config_.mechanism.theoretical_open_limit.value +
                       config_.mechanism.theoretical_close_limit.value) *
                      0.5};
  }

  [[nodiscard]] int effectiveNutPositionDirectionSign() const noexcept {
    const int motor_direction = config_.motor.direction_sign < 0 ? -1 : 1;
    const int encoder_direction =
        config_.nut_position_encoder.direction_sign < 0 ? -1 : 1;
    return motor_direction * encoder_direction;
  }

  void initializeStrokeReferenceIfNeeded(common::Rad motor_position) {
    if (stroke_reference_valid_) {
      return;
    }
    stroke_reference_motor_position_ = motor_position;
    stroke_reference_nut_stroke_ = provisionalReferenceStroke();
    stroke_reference_valid_ = true;
    nut_position_encoder_.resetZero(stroke_reference_motor_position_,
                                    stroke_reference_nut_stroke_);
  }

  void setOpenZeroReference(common::Rad motor_position) {
    stroke_reference_motor_position_ = motor_position;
    stroke_reference_nut_stroke_ = config_.mechanism.theoretical_open_limit;
    stroke_reference_valid_ = true;
    nut_position_encoder_.resetZero(stroke_reference_motor_position_,
                                    stroke_reference_nut_stroke_);
    motor_zero_reference_valid_ = true;
    motor_zero_reference_ = motor_position;
    current_nut_stroke_ = config_.mechanism.theoretical_open_limit;
  }

  [[nodiscard]] common::Mm referenceSearchClosedLimit() const {
    const double theoretical_open = config_.mechanism.theoretical_open_limit.value;
    const double theoretical_close =
        config_.mechanism.theoretical_close_limit.value;
    const double configured_travel_search =
        std::abs(config_.self_check.travel_learning_search_distance.value);
    const double configured_pre_b_search =
        std::abs(config_.self_check.pre_b_max_expansion_distance.value);
    const double configured_closed =
        theoretical_open +
        std::max(configured_travel_search, configured_pre_b_search);
    return common::Mm{std::max(
        {theoretical_close,
         configured_closed,
         profile_.travel_limits.preliminary_closed_limit.value,
         profile_.travel_limits.safe_zone_closed_limit.value,
         profile_.travel_limits.software_closed_limit.value,
         current_nut_stroke_.value})};
  }

  [[nodiscard]] common::Mm clampReferenceSearchStroke(common::Mm stroke) const {
    const double open_limit = config_.mechanism.theoretical_open_limit.value;
    const double closed_limit = referenceSearchClosedLimit().value;
    if (closed_limit <= open_limit) {
      return stroke;
    }
    return common::Mm{std::clamp(stroke.value, open_limit, closed_limit)};
  }

  [[nodiscard]] common::Rad strokeToReferencedMotorPosition(
      common::Mm target_stroke) const {
    const common::Mm relative_stroke{
        target_stroke.value - stroke_reference_nut_stroke_.value};
    const common::Rad relative_position = strokeToMotorPosition(
        relative_stroke, config_.mechanism.lead_screw_pitch,
        effectiveNutPositionDirectionSign());
    return common::Rad{stroke_reference_motor_position_.value +
                       relative_position.value};
  }

  [[nodiscard]] common::Mm referencedMotorPositionToStroke(
      common::Rad motor_position) const {
    const common::Rad relative_position =
        motorPositionDelta(motor_position, stroke_reference_motor_position_);
    const common::Mm relative_stroke = motorPositionToStroke(
        relative_position, config_.mechanism.lead_screw_pitch,
        effectiveNutPositionDirectionSign());
    return common::Mm{stroke_reference_nut_stroke_.value +
                      relative_stroke.value};
  }

  [[nodiscard]] common::Mm updateNutPositionFromMotorFeedback(
      const MotorFeedback& feedback) {
    initializeStrokeReferenceIfNeeded(feedback.position);
    nut_position_encoder_.update(feedback);
    return nut_position_encoder_.nutPosition();
  }

  [[nodiscard]] common::Result sendNutMotionCommand(common::Mm target_stroke,
                                                     common::MmPerS nut_speed,
                                                     common::A current) {
    if (const auto cancel = checkOperationCancelled("send nut motion command");
        cancel.isError()) {
      return cancel;
    }
    // Final mechanism-to-motor conversion point. All public controller APIs use
    // mechanism-side units; hardware implementations receive motor-side units.
    MotorCommand command{};
    command.control_mode = MotorControlMode::PositionVelocityTorque;
    initializeStrokeReferenceIfNeeded(last_feedback_.position);
    command.target_position = strokeToReferencedMotorPosition(target_stroke);
    command.target_velocity = nutSpeedToMotorVelocity(
        nut_speed, config_.mechanism.lead_screw_pitch,
        effectiveNutPositionDirectionSign());
    command.target_current = current;
    command.target_torque = common::Nm{current.value};
    command.enable = true;
    return motor_->sendCommand(command);
  }

  [[nodiscard]] common::Result sendLimitedPositionCommand(
      common::Mm target_stroke, common::MmPerS nut_speed,
      common::A current_limit,
      const ManualPositioningRange& allowed_range) {
    if (const auto cancel = checkOperationCancelled("send limited position command");
        cancel.isError()) {
      return cancel;
    }
    const auto ready = ensureMotorOutputEnabled();
    if (ready.isError()) {
      return ready;
    }
    safety_limiter_.setConfig(makeSafetyLimitConfig(
        config_, profile_, config_.safety.max_motor_current,
        allowed_range.use_software_limits));
    auto limit_config = safety_limiter_.config();
    limit_config.min_nut_stroke = allowed_range.open_limit;
    limit_config.max_nut_stroke = allowed_range.closed_limit;
    limit_config.stroke_limits_enabled = allowed_range.valid;
    limit_config.active_stop_on_current_limit = false;
    safety_limiter_.setConfig(limit_config);

    const auto limited = safety_limiter_.applyLimits(
        safety::SafetyLimitCommand{current_limit, nut_speed, target_stroke,
                                   current_nut_stroke_,
                                   previous_commanded_nut_speed_,
                                   config_.safety.command_timeout});
    if (limited.active_stop_required) {
      state_machine_.forceActiveStop();
      return common::Result::error(common::ErrorCode::SafetyActiveStop,
                                   "manual positioning safety limiter requested active stop");
    }
    const auto send_result = sendNutMotionCommand(
        limited.target_nut_stroke, limited.nut_speed, limited.motor_current);
    if (send_result.isOk()) {
      previous_commanded_nut_speed_ = limited.nut_speed;
    }
    return send_result;
  }

  [[nodiscard]] common::Result sendLimitedCommand(common::Mm target_stroke,
                                                   common::MmPerS nut_speed,
                                                   common::A current,
                                                   bool use_software_limits,
                                                   bool use_custom_stroke_limits = false,
                                                   common::Mm custom_open_limit = {},
                                                   common::Mm custom_closed_limit = {},
                                                   common::A hard_current_limit = {}) {
    if (const auto cancel = checkOperationCancelled("send limited command");
        cancel.isError()) {
      return cancel;
    }
    const auto ready = ensureMotorOutputEnabled();
    if (ready.isError()) {
      return ready;
    }
    // Final safety gate before motor output. Current, speed, acceleration, and
    // stroke are checked here. Hard-limit violations request ActiveStop rather
    // than sending an unsafe command.
    const common::A limiter_current =
        hard_current_limit.value > 0.0 ? hard_current_limit : current;
    safety_limiter_.setConfig(makeSafetyLimitConfig(
        config_, profile_, limiter_current, use_software_limits));
    if (use_custom_stroke_limits) {
      auto limit_config = safety_limiter_.config();
      limit_config.min_nut_stroke = custom_open_limit;
      limit_config.max_nut_stroke = custom_closed_limit;
      limit_config.stroke_limits_enabled =
          limit_config.max_nut_stroke.value > limit_config.min_nut_stroke.value;
      safety_limiter_.setConfig(limit_config);
    }
    const auto limited = safety_limiter_.applyLimits(
        safety::SafetyLimitCommand{current, nut_speed, target_stroke,
                                   current_nut_stroke_,
                                   previous_commanded_nut_speed_,
                                   config_.safety.command_timeout});
    if (limited.active_stop_required) {
      state_machine_.forceActiveStop();
      return common::Result::error(common::ErrorCode::SafetyActiveStop,
                                   "safety limiter requested active stop");
    }

    const auto send_result = sendNutMotionCommand(
        limited.target_nut_stroke, limited.nut_speed, limited.motor_current);
    if (send_result.isOk()) {
      previous_commanded_nut_speed_ = limited.nut_speed;
    }
    return send_result;
  }

  [[nodiscard]] MotionWaitResult runMotionUntil(
      const MotionWaitOptions& options) {
    MotionWaitResult output{};
    output.start_stroke = current_nut_stroke_;

    const auto timeout = motionTimeoutFor(output.start_stroke,
                                         options.target_stroke,
                                         options.nut_speed,
                                         options.timeout);
    const auto command_result =
        sendLimitedCommand(options.target_stroke, options.nut_speed,
                           options.current_limit, options.use_software_limits,
                           options.use_custom_stroke_limits,
                           options.custom_open_limit,
                           options.custom_closed_limit);
    if (command_result.isError()) {
      output.result = command_result;
      return output;
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::duration<double>{timeout.value});
    common::Result feedback_result = common::Ok();
    const common::Mm endpoint_progress_threshold = endpointProgressThreshold();
    common::Mm endpoint_progress_reference = output.start_stroke;
    std::chrono::steady_clock::time_point endpoint_stall_since{};
    bool endpoint_stall_timer_running = false;
    common::Mm contact_progress_reference = output.start_stroke;
    std::chrono::steady_clock::time_point contact_no_progress_since{};
    bool contact_no_progress_timer_running = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto cancel = checkOperationCancelled("motion wait");
          cancel.isError()) {
        output.result = cancel;
        output.end_stroke = current_nut_stroke_;
        output.measured_nut_speed = measuredNutSpeed();
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        return output;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        output.result = feedback_result;
        output.end_stroke = current_nut_stroke_;
        output.measured_nut_speed = measuredNutSpeed();
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        return output;
      }
      const auto now = std::chrono::steady_clock::now();

      output.end_stroke = current_nut_stroke_;
      output.measured_nut_speed = measuredNutSpeed();
      feedPreBFeedbackRecorders(options.friction_anomaly_detector,
                                directionFromSignedNutSpeed(options.nut_speed),
                                output.measured_nut_speed, false);
      output.measured_distance = common::Mm{
          std::abs(output.end_stroke.value - output.start_stroke.value)};
      observeMotionWaitFeedback(&output, options.nut_speed);
      output.target_reached =
          isTargetReached(options.target_stroke, options.nut_speed);
      output.force_current_reached =
          options.force_current_target.value > 0.0 &&
          std::abs(last_feedback_.current.value) >=
              options.force_current_target.value -
                  config_.self_check.max_current_ripple.value;
      if (options.require_no_progress_for_contact_stop &&
          madeProgressInCommandDirection(contact_progress_reference,
                                         options.nut_speed,
                                         endpoint_progress_threshold)) {
        contact_progress_reference = current_nut_stroke_;
        contact_no_progress_timer_running = false;
      }
      if (options.allow_jam_success) {
        if (madeProgressInCommandDirection(endpoint_progress_reference,
                                           options.nut_speed,
                                           endpoint_progress_threshold)) {
          endpoint_progress_reference = current_nut_stroke_;
          endpoint_stall_timer_running = false;
        }
        if (homingEndpointStallFeedback(options)) {
          if (!endpoint_stall_timer_running) {
            endpoint_stall_since = now;
            endpoint_stall_timer_running = true;
          }
          const double confirm_duration_s =
              std::chrono::duration<double>{now - endpoint_stall_since}.count();
          if (confirm_duration_s >=
              std::max(0.0, config_.homing.jam_confirm_time.value)) {
            return completeEndpointMotion(
                output, "low_current_no_progress", confirm_duration_s,
                last_feedback_.current, output.measured_nut_speed,
                current_nut_stroke_);
          }
        } else {
          endpoint_stall_timer_running = false;
        }
      }

      const auto contact =
          options.use_custom_stroke_limits
              ? evaluateContactInRange(options.nut_speed,
                                       options.custom_open_limit,
                                       options.custom_closed_limit)
              : evaluateContact(options.nut_speed);
      output.contact_detected = contact.contact_detected;
      output.jam_detected = contact.jam_detected ||
                            contact.stroke_limit_suspected;
      const bool contact_or_jam_detected =
          output.jam_detected ||
          (options.stop_on_contact && output.contact_detected) ||
          output.force_current_reached;
      bool contact_stop_confirmed = contact_or_jam_detected;
      if (options.require_no_progress_for_contact_stop &&
          contact_or_jam_detected) {
        if (!contact_no_progress_timer_running) {
          contact_no_progress_since = now;
          contact_no_progress_timer_running = true;
          contact_stop_confirmed = false;
        } else {
          const auto no_progress_duration = now - contact_no_progress_since;
          contact_stop_confirmed =
              no_progress_duration >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{std::max(
                      0.0, options.no_progress_confirm_time.value)});
        }
      } else if (!contact_or_jam_detected) {
        contact_no_progress_timer_running = false;
      }
      if (options.require_no_progress_for_contact_stop &&
          contact_or_jam_detected && !contact_stop_confirmed) {
        output.contact_detected = false;
        output.jam_detected = false;
        if (!output.target_reached) {
          output.force_current_reached = false;
        }
      }
      if (output.jam_detected && contact_stop_confirmed) {
        if (options.allow_jam_success) {
          return completeEndpointMotion(
              output,
              contact.stroke_limit_suspected ? "stroke_limit_feedback"
                                             : "jam_feedback",
              0.0, last_feedback_.current, output.measured_nut_speed,
              current_nut_stroke_);
        }
        state_machine_.forceActiveStop();
        output.result = common::Result::error(
            common::ErrorCode::SafetyJamDetected,
            "motion stopped by jam or stroke-limit feedback");
        return output;
      }
      if (options.stop_on_contact && output.contact_detected &&
          contact_stop_confirmed) {
        if (options.allow_jam_success) {
          return completeEndpointMotion(
              output, "contact_feedback", 0.0, last_feedback_.current,
              output.measured_nut_speed, current_nut_stroke_);
        }
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isError()) {
          output.result = settle_result.result;
          return output;
        }
        output.end_stroke = settle_result.settled_stroke;
        output.measured_nut_speed = settle_result.settled_nut_speed;
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        output.result = common::Ok();
        return output;
      }
      const bool force_current_stop_confirmed =
          output.force_current_reached && contact_stop_confirmed;
      const bool force_current_at_target =
          output.force_current_reached && output.target_reached &&
          options.allow_contact_success;
      if (force_current_stop_confirmed ||
          force_current_at_target ||
          (output.target_reached && !options.allow_contact_success)) {
        if ((force_current_stop_confirmed || force_current_at_target) &&
            options.allow_jam_success) {
          return completeEndpointMotion(
              output, "force_current_feedback", 0.0, last_feedback_.current,
              output.measured_nut_speed, current_nut_stroke_);
        }
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isError()) {
          output.result = settle_result.result;
          return output;
        }
        output.end_stroke = settle_result.settled_stroke;
        output.measured_nut_speed = settle_result.settled_nut_speed;
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        output.result = common::Ok();
        return output;
      }
      if (output.target_reached && options.allow_contact_success) {
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isError()) {
          output.result = settle_result.result;
          return output;
        }
        output.end_stroke = settle_result.settled_stroke;
        output.measured_nut_speed = settle_result.settled_nut_speed;
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        output.result = common::Ok();
        return output;
      }
    }

    const auto settle_result = stopAndWaitForSettledFeedback();
    if (settle_result.result.isOk()) {
      output.end_stroke = settle_result.settled_stroke;
      output.measured_nut_speed = settle_result.settled_nut_speed;
    } else {
      output.end_stroke = current_nut_stroke_;
      output.measured_nut_speed = measuredNutSpeed();
      output.measured_distance = common::Mm{
          std::abs(output.end_stroke.value - output.start_stroke.value)};
      output.result = settle_result.result;
      return output;
    }
    output.measured_distance = common::Mm{
        std::abs(output.end_stroke.value - output.start_stroke.value)};
    output.result = common::Result::error(
        common::ErrorCode::OperationTimedOut,
        "motion did not reach target or feedback condition before timeout");
    return output;
  }

  [[nodiscard]] ManualPositioningResult runManualPositioningUntil(
      common::Mm target_stroke, common::MmPerS nut_speed,
      common::A feedback_current_limit, common::S requested_timeout,
      const ManualPositioningRange& allowed_range) {
    ManualPositioningResult output{};
    output.start_stroke = current_nut_stroke_;
    output.start_motor_position = last_feedback_.position;
    output.target_motor_position = strokeToReferencedMotorPosition(target_stroke);
    output.end_motor_position = last_feedback_.position;

    const auto timeout = motionTimeoutFor(output.start_stroke, target_stroke,
                                          nut_speed, requested_timeout);
    const common::S control_period = manualPositioningControlPeriod();
    const double max_command_step_mm =
        std::max(std::abs(nut_speed.value) * control_period.value,
                 config_.self_check.fallback_nut_stroke_noise.value);
    output.max_command_step = common::Mm{max_command_step_mm};
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>{timeout.value});
    auto previous_command_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point over_current_since{};
    bool over_current_timer_running = false;
    common::Mm progress_reference_stroke = current_nut_stroke_;
    std::chrono::steady_clock::time_point no_progress_since{};
    bool no_progress_timer_running = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto cancel = checkOperationCancelled("manual positioning wait");
          cancel.isError()) {
        output.result = cancel;
        return output;
      }

      output.target_reached =
          isManualPositioningTargetReached(target_stroke, nut_speed);
      if (output.target_reached) {
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isError()) {
          output.result = settle_result.result;
          return output;
        }
        output.end_stroke = settle_result.settled_stroke;
        output.end_motor_position = settle_result.settled_motor_position;
        output.measured_nut_speed = settle_result.settled_nut_speed;
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        output.result = common::Ok();
        return output;
      }

      const auto command_time = std::chrono::steady_clock::now();
      const double elapsed_since_last_command_s = std::max(
          control_period.value,
          std::chrono::duration<double>{
              command_time - previous_command_time}.count());
      previous_command_time = command_time;
      const double allowed_step_mm =
          std::max(std::abs(nut_speed.value) * elapsed_since_last_command_s,
                   config_.self_check.fallback_nut_stroke_noise.value);
      output.max_command_step = common::Mm{
          std::max(output.max_command_step.value, allowed_step_mm)};
      const double remaining = target_stroke.value - current_nut_stroke_.value;
      const double signed_step =
          std::clamp(remaining, -allowed_step_mm, allowed_step_mm);
      const common::Mm step_target{
          current_nut_stroke_.value + signed_step};
      const auto command_result =
          sendLimitedPositionCommand(step_target, nut_speed,
                                     feedback_current_limit, allowed_range);
      if (command_result.isError()) {
        output.result = command_result;
        return output;
      }
      ++output.command_steps;

      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::duration<double>{control_period.value}));
      const auto feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        output.result = feedback_result;
        return output;
      }

      output.end_stroke = current_nut_stroke_;
      output.end_motor_position = last_feedback_.position;
      output.measured_nut_speed = measuredNutSpeed();
      output.measured_distance = common::Mm{
          std::abs(output.end_stroke.value - output.start_stroke.value)};
      output.peak_motor_current = common::A{std::max(
          std::abs(output.peak_motor_current.value),
          std::abs(last_feedback_.current.value))};
      output.target_reached =
          isManualPositioningTargetReached(target_stroke, nut_speed);
      const auto now = std::chrono::steady_clock::now();
      const bool over_feedback_limit =
          std::abs(last_feedback_.current.value) >
          std::abs(feedback_current_limit.value);
      if (over_feedback_limit && !over_current_timer_running) {
        over_current_since = now;
        over_current_timer_running = true;
      } else if (!over_feedback_limit) {
        over_current_timer_running = false;
      }
      const bool sustained_feedback_over_current =
          over_current_timer_running &&
          now - over_current_since >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{
                      config_.safety.contact_detection_time.value});
      const bool hard_feedback_over_current = feedbackCurrentExceedsHardLimit();
      if (sustained_feedback_over_current || hard_feedback_over_current) {
        output.current_limit_exceeded = true;
        output.trigger_motor_position = last_feedback_.position;
        output.trigger_motor_current = last_feedback_.current;
        output.trigger_motor_velocity = last_feedback_.velocity;
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isOk()) {
          output.end_stroke = settle_result.settled_stroke;
          output.measured_nut_speed = settle_result.settled_nut_speed;
          output.end_motor_position = settle_result.settled_motor_position;
        } else {
          (void)disableAfterMotion();
          output.end_stroke = current_nut_stroke_;
          output.measured_nut_speed = measuredNutSpeed();
          output.end_motor_position = last_feedback_.position;
        }
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        state_machine_.forceActiveStop();
        std::ostringstream message;
        message << "manual positioning feedback current exceeded limit"
                << " trigger_current_a=" << output.trigger_motor_current.value
                << " limit_a=" << feedback_current_limit.value
                << " hard_limit_a=" << config_.safety.max_motor_current.value
                << " sustained="
                << (sustained_feedback_over_current ? "true" : "false")
                << " hard="
                << (hard_feedback_over_current ? "true" : "false")
                << " target_mm=" << target_stroke.value
                << " start_mm=" << output.start_stroke.value
                << " end_mm=" << output.end_stroke.value
                << " target_motor_pos_rad="
                << output.target_motor_position.value
                << " start_motor_pos_rad="
                << output.start_motor_position.value
                << " trigger_motor_pos_rad="
                << output.trigger_motor_position.value
                << " end_motor_pos_rad=" << output.end_motor_position.value
                << " motor_delta_rad="
                << (output.end_motor_position.value -
                    output.start_motor_position.value)
                << " trigger_motor_vel_rad_s="
                << output.trigger_motor_velocity.value
                << " command_steps=" << output.command_steps
                << " max_command_step_mm=" << output.max_command_step.value
                << " motor_disabled=" << (motor_->isEnabled() ? 0 : 1);
        if (settle_result.result.isError()) {
          message << " settle_result="
                  << common::toString(settle_result.result.code());
          if (settle_result.result.hasMessage()) {
            message << " settle_message=" << settle_result.result.message();
          }
        }
        output.result = common::Result::error(
            common::ErrorCode::SafetyActiveStop, message.str());
        return output;
      }

      const double progress_sign = nut_speed.value >= 0.0 ? 1.0 : -1.0;
      const double progress_threshold = std::min(
          output.max_command_step.value * 0.5,
          std::max({profile_.noise_floor.nut_stroke_noise.value * 3.0,
                    config_.self_check.fallback_nut_stroke_noise.value,
                    0.002}));
      const double signed_progress =
          (current_nut_stroke_.value - progress_reference_stroke.value) *
          progress_sign;
      if (signed_progress > progress_threshold) {
        progress_reference_stroke = current_nut_stroke_;
        no_progress_timer_running = false;
      } else if (!no_progress_timer_running) {
        no_progress_since = now;
        no_progress_timer_running = true;
      }
      const bool no_position_progress =
          no_progress_timer_running &&
          now - no_progress_since >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{
                      config_.safety.contact_detection_time.value});

      safety::ContactJamResult contact{};
      if (no_position_progress) {
        contact = evaluateContactInRange(
            nut_speed, allowed_range.open_limit, allowed_range.closed_limit);
      }
      output.jam_detected = contact.jam_detected;
      output.stroke_limit_suspected = contact.stroke_limit_suspected;
      if (output.jam_detected || output.stroke_limit_suspected) {
        output.trigger_motor_position = last_feedback_.position;
        output.trigger_motor_current = last_feedback_.current;
        output.trigger_motor_velocity = last_feedback_.velocity;
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isOk()) {
          output.end_stroke = settle_result.settled_stroke;
          output.measured_nut_speed = settle_result.settled_nut_speed;
          output.end_motor_position = settle_result.settled_motor_position;
        } else {
          (void)disableAfterMotion();
          output.end_stroke = current_nut_stroke_;
          output.measured_nut_speed = measuredNutSpeed();
          output.end_motor_position = last_feedback_.position;
        }
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        state_machine_.forceActiveStop();
        std::ostringstream message;
        message << "manual positioning stopped by jam or stroke-limit feedback"
                << " target_mm=" << target_stroke.value
                << " start_mm=" << output.start_stroke.value
                << " end_mm=" << output.end_stroke.value
                << " target_motor_pos_rad="
                << output.target_motor_position.value
                << " start_motor_pos_rad="
                << output.start_motor_position.value
                << " trigger_motor_pos_rad="
                << output.trigger_motor_position.value
                << " end_motor_pos_rad=" << output.end_motor_position.value
                << " motor_delta_rad="
                << (output.end_motor_position.value -
                    output.start_motor_position.value)
                << " trigger_current_a="
                << output.trigger_motor_current.value
                << " trigger_motor_vel_rad_s="
                << output.trigger_motor_velocity.value
                << " command_steps=" << output.command_steps
                << " max_command_step_mm=" << output.max_command_step.value
                << " motor_disabled=" << (motor_->isEnabled() ? 0 : 1);
        if (settle_result.result.isError()) {
          message << " settle_result="
                  << common::toString(settle_result.result.code());
          if (settle_result.result.hasMessage()) {
            message << " settle_message=" << settle_result.result.message();
          }
        }
        output.result = common::Result::error(common::ErrorCode::SafetyJamDetected,
                                             message.str());
        return output;
      }
      if (output.target_reached) {
        const auto settle_result = stopAndWaitForSettledFeedback();
        if (settle_result.result.isError()) {
          output.result = settle_result.result;
          return output;
        }
        output.end_stroke = settle_result.settled_stroke;
        output.end_motor_position = settle_result.settled_motor_position;
        output.measured_nut_speed = settle_result.settled_nut_speed;
        output.measured_distance = common::Mm{
            std::abs(output.end_stroke.value - output.start_stroke.value)};
        output.result = common::Ok();
        return output;
      }
    }

    const auto settle_result = stopAndWaitForSettledFeedback();
    if (settle_result.result.isOk()) {
      output.end_stroke = settle_result.settled_stroke;
      output.measured_nut_speed = settle_result.settled_nut_speed;
      output.end_motor_position = settle_result.settled_motor_position;
    } else {
      output.end_stroke = current_nut_stroke_;
      output.measured_nut_speed = measuredNutSpeed();
      output.end_motor_position = last_feedback_.position;
    }
    output.measured_distance = common::Mm{
        std::abs(output.end_stroke.value - output.start_stroke.value)};
    std::ostringstream message;
    message << "manual positioning did not reach target before timeout"
            << " target_mm=" << target_stroke.value
            << " start_mm=" << output.start_stroke.value
            << " end_mm=" << output.end_stroke.value
            << " measured_delta_mm=" << output.measured_distance.value
            << " target_motor_pos_rad="
            << output.target_motor_position.value
            << " start_motor_pos_rad=" << output.start_motor_position.value
            << " end_motor_pos_rad=" << output.end_motor_position.value
            << " motor_delta_rad="
            << (output.end_motor_position.value -
                output.start_motor_position.value)
            << " timeout_s=" << timeout.value
            << " peak_current_a=" << output.peak_motor_current.value
            << " command_steps=" << output.command_steps
            << " max_command_step_mm=" << output.max_command_step.value;
    output.result = common::Result::error(common::ErrorCode::OperationTimedOut,
                                          message.str());
    return output;
  }

  [[nodiscard]] safety::ContactJamResult evaluateContact(
      common::MmPerS commanded_nut_speed) {
    // Uses measured velocity from feedback so a velocity drop can be observed.
    // Passing the command speed as measured speed would hide contact and jam
    // conditions.
    contact_detector_.setConfig(makeContactConfig(config_));
    const common::MmPerS measured_nut_speed = motorVelocityToNutSpeed(
        last_feedback_.velocity, config_.mechanism.lead_screw_pitch,
        effectiveNutPositionDirectionSign());
    return contact_detector_.detect(safety::ContactJamContext{
        last_feedback_.current,
        contactBaselineFor(commanded_nut_speed),
        measured_nut_speed,
        commanded_nut_speed,
        current_nut_stroke_,
        profile_.travel_limits.software_open_limit,
        profile_.travel_limits.software_closed_limit,
        common::S{config_.safety.contact_detection_time.value}});
  }

  [[nodiscard]] safety::ContactJamResult evaluateContactInRange(
      common::MmPerS commanded_nut_speed, common::Mm open_limit,
      common::Mm closed_limit) {
    contact_detector_.setConfig(makeContactConfig(config_));
    const common::MmPerS measured_nut_speed = motorVelocityToNutSpeed(
        last_feedback_.velocity, config_.mechanism.lead_screw_pitch,
        effectiveNutPositionDirectionSign());
    return contact_detector_.detect(safety::ContactJamContext{
        last_feedback_.current,
        contactBaselineFor(commanded_nut_speed),
        measured_nut_speed,
        commanded_nut_speed,
        current_nut_stroke_,
        open_limit,
        closed_limit,
        common::S{config_.safety.contact_detection_time.value}});
  }

  [[nodiscard]] bool feedbackCurrentExceedsHardLimit(common::A hard_limit) const {
    return hard_limit.value > 0.0 &&
           std::abs(last_feedback_.current.value) >
               std::abs(hard_limit.value);
  }

  [[nodiscard]] bool feedbackCurrentExceedsHardLimit() const {
    return feedbackCurrentExceedsHardLimit(config_.safety.max_motor_current);
  }

  [[nodiscard]] common::A selfCheckFeedbackHardCurrentLimit() const {
    const double self_check_limit =
        std::abs(config_.safety.self_check_feedback_hard_current_limit.value);
    if (self_check_limit > 0.0) {
      return common::A{self_check_limit};
    }
    return config_.safety.max_motor_current;
  }

  [[nodiscard]] common::A selfCheckFeedbackEmergencyCurrentLimit() const {
    const double emergency_limit = std::abs(
        config_.safety.self_check_feedback_emergency_current_limit.value);
    if (emergency_limit > 0.0) {
      return common::A{emergency_limit};
    }
    return {};
  }

  void feedPreBFeedbackRecorders(
      self_check::FrictionAnomalyDetector* detector,
      self_check::MotionDirection direction,
      common::MmPerS measured_nut_speed,
      bool record_trace) {
    if (direction == self_check::MotionDirection::Unknown) {
      return;
    }
    if (detector != nullptr) {
      detector->addSample(self_check::FrictionAnomalySample{
          current_nut_stroke_,
          last_feedback_.position,
          last_feedback_.current,
          measured_nut_speed,
          direction,
          last_feedback_.temperature,
          last_feedback_.timestamp});
    }
    if (record_trace) {
      recordPreBCurrentTracePoint(direction, measured_nut_speed);
    }
  }

  void beginPreBCurrentTraceSegment() {
    if (pre_b_current_trace_segment_id_ == std::numeric_limits<std::uint32_t>::max()) {
      pre_b_current_trace_segment_id_ = 0U;
    }
    ++pre_b_current_trace_segment_id_;
  }

  void recordPreBCurrentTracePoint(self_check::MotionDirection direction,
                                   common::MmPerS measured_nut_speed) {
    if (direction == self_check::MotionDirection::Unknown) {
      return;
    }
    const auto phase = pre_self_check_state_machine_.phase();
    const bool trace_phase =
        phase == state_machine::PreSelfCheckPhase::PreliminaryLimitSearch ||
        phase == state_machine::PreSelfCheckPhase::MultiRegionRoundTripLearning;
    if (!trace_phase) {
      return;
    }

    ++pre_b_trace_decimation_counter_;
    if (pre_b_trace_decimation_counter_ % 2U != 0U &&
        pre_b_current_trace_points_.size() >= kPreBCurrentTraceMaxPoints / 2U) {
      return;
    }

    if (pre_b_current_trace_points_.size() >= kPreBCurrentTraceMaxPoints) {
      pre_b_current_trace_points_.erase(pre_b_current_trace_points_.begin());
    }
    pre_b_current_trace_points_.push_back(PreBCurrentTracePoint{
        current_nut_stroke_,
        last_feedback_.current,
        measured_nut_speed,
        pre_b_current_trace_segment_id_,
        static_cast<std::uint8_t>(
            phase == state_machine::PreSelfCheckPhase::PreliminaryLimitSearch
                ? 1U
                : 2U),
        static_cast<std::uint8_t>(
            direction == self_check::MotionDirection::Opening ? 1U : 2U)});
  }

  [[nodiscard]] self_check::MotionDirection directionFromSignedNutSpeed(
      common::MmPerS nut_speed) const noexcept {
    if (nut_speed.value > 0.0) {
      return self_check::MotionDirection::Closing;
    }
    if (nut_speed.value < 0.0) {
      return self_check::MotionDirection::Opening;
    }
    return self_check::MotionDirection::Unknown;
  }

  void mergeFrictionAnomalyRecords(
      const std::vector<self_check::FrictionAnomalyRecord>& records) {
    if (records.empty()) {
      return;
    }
    for (const auto& record : records) {
      if (friction_anomaly_records_.size() >=
          config_.self_check.friction_anomaly_max_records) {
        break;
      }
      friction_anomaly_records_.push_back(record);
    }
    reportFrictionAnomalySummary();
  }

  void rebaseFrictionAnomalyRecordsToCurrentReference(const char* reason) {
    if (friction_anomaly_records_.empty()) {
      return;
    }

    for (auto& record : friction_anomaly_records_) {
      const common::Mm center =
          clampReferenceSearchStroke(referencedMotorPositionToStroke(
              record.motor_position));
      const double half_width = std::max(0.0, record.width.value) * 0.5;
      record.center_position = center;
      const common::Mm start_position =
          clampReferenceSearchStroke(common::Mm{center.value - half_width});
      const common::Mm end_position =
          clampReferenceSearchStroke(common::Mm{center.value + half_width});
      record.start_position =
          common::Mm{std::min(start_position.value, end_position.value)};
      record.end_position =
          common::Mm{std::max(start_position.value, end_position.value)};
      record.width = common::Mm{record.end_position.value -
                                record.start_position.value};
    }

    std::ostringstream message;
    message << "friction anomaly map rebased"
            << " reason=" << (reason != nullptr ? reason : "unspecified")
            << " records=" << friction_anomaly_records_.size()
            << " zero_motor_rad=" << stroke_reference_motor_position_.value
            << " zero_stroke_mm=" << stroke_reference_nut_stroke_.value;
    reportProgress(message.str());
  }

  [[nodiscard]] SelfCheckProbeFeedbackRisk evaluateSelfCheckProbeRisk(
      common::MmPerS commanded_nut_speed,
      const SelfCheckProbeOptions& options,
      bool motion_started_in_requested_direction,
      bool hard_current_stop_required) {
    SelfCheckProbeFeedbackRisk output{};
    if (hard_current_stop_required) {
      output.stop_required = true;
      output.jam_suspected = true;
      output.message =
          "pre-self-check probe stopped by sustained hard feedback current limit";
      return output;
    }

    if (options.stop_after_motion_start) {
      // BidirectionalMoveEnable is only a low-energy breakaway check. Once the
      // motor has produced a controlled micro-motion, generic velocity-drop
      // jam detection becomes a false positive source at very low speeds.
      if (motion_started_in_requested_direction) {
        return output;
      }
      const auto contact = evaluateContactInRange(
          commanded_nut_speed,
          config_.mechanism.theoretical_open_limit,
          config_.mechanism.theoretical_close_limit);
      output.limit_suspected = contact.stroke_limit_suspected;
      output.jam_suspected = contact.stroke_limit_suspected;
      output.stop_required = contact.stroke_limit_suspected;
      return output;
    }

    const auto contact = evaluateContact(commanded_nut_speed);
    output.velocity_collapsed =
        contact.contact_detected || contact.jam_detected ||
        contact.stroke_limit_suspected;
    output.limit_suspected = contact.stroke_limit_suspected;
    output.jam_suspected =
        contact.jam_detected || contact.stroke_limit_suspected;
    output.stop_required = output.jam_suspected &&
                           !options.suppress_soft_jam_stop;
    return output;
  }

  void reportProgress(const std::string& message) {
    if (progress_callback_) {
      progress_callback_(message);
    }
  }

  [[nodiscard]] std::string directionName(
      self_check::MotionDirection direction) const {
    switch (direction) {
      case self_check::MotionDirection::Opening:
        return "opening";
      case self_check::MotionDirection::Closing:
        return "closing";
      case self_check::MotionDirection::Unknown:
        break;
    }
    return "unknown";
  }

  void reportSelfCheckAttempt(self_check::MotionDirection direction,
                              common::Mm target_stroke,
                              common::MmPerS nut_speed,
                              common::A current_limit) {
    const common::Rad target_motor_position =
        strokeToReferencedMotorPosition(target_stroke);
    std::ostringstream message;
    message << "PreSelfCheck | probe attempt"
            << " direction=" << directionName(direction)
            << " target_stroke_mm=" << target_stroke.value
            << " target_motor_pos_rad=" << target_motor_position.value
            << " target_nut_speed_mm_s=" << nut_speed.value
            << " motor_velocity_rad_s="
            << nutSpeedToMotorVelocity(
                   nut_speed, config_.mechanism.lead_screw_pitch,
                   effectiveNutPositionDirectionSign())
                   .value
            << " current_limit_a=" << current_limit.value
            << " start_estimated_stroke_mm=" << current_nut_stroke_.value
            << " start_motor_pos_rad=" << last_feedback_.position.value;
    if (last_feedback_.raw_position_counts_valid) {
      message << " start_motor_raw_pos_counts="
              << last_feedback_.raw_position_counts;
    } else {
      message << " start_motor_raw_pos_counts=-";
    }
    reportProgress(message.str());
  }

  void reportSelfCheckResult(self_check::MotionDirection direction,
                             const SelfCheckMotionProbeResult& result) {
    std::ostringstream message;
    message << "PreSelfCheck | probe result"
            << " direction=" << directionName(direction)
            << " code=" << common::toString(result.result.code())
            << " start_estimated_mm=" << result.start_stroke.value
            << " end_estimated_mm=" << result.end_stroke.value
            << " start_motor_pos_rad=" << result.start_motor_position.value
            << " target_motor_pos_rad=" << result.target_motor_position.value
            << " target_motor_delta_rad="
            << result.target_motor_position.value -
                   result.start_motor_position.value
            << " end_motor_pos_rad=" << result.end_motor_position.value
            << " motor_delta_rad="
            << result.end_motor_position.value -
                   result.start_motor_position.value
            << " start_motor_raw_pos_counts=";
    if (result.start_raw_position_counts_valid) {
      message << result.start_raw_position_counts;
    } else {
      message << '-';
    }
    message << " end_motor_raw_pos_counts=";
    if (result.end_raw_position_counts_valid) {
      message << result.end_raw_position_counts;
    } else {
      message << '-';
    }
    message
            << " measured_mm=" << result.measured_distance.value
            << " avg_speed_mm_s=" << result.average_nut_speed.value
            << " max_current_a=" << result.max_motor_current.value
            << " last_current_a=" << result.last_motor_current.value
            << " hard_current_limit_a="
            << result.feedback_hard_current_limit.value
            << " emergency_current_limit_a="
            << result.feedback_emergency_current_limit.value
            << " hard_current_confirm_time_s="
            << result.hard_current_confirm_time.value
            << " hard_current_over_limit_s="
            << result.hard_current_over_limit_duration.value
            << " hard_current_transient="
            << (result.hard_current_transient_observed ? "true" : "false")
            << " hard_current_confirmed="
            << (result.hard_current_confirmed ? "true" : "false")
            << " hard_current_no_progress="
            << (result.hard_current_no_progress_confirmed ? "true" : "false")
            << " hard_current_progress_observed="
            << (result.hard_current_progress_observed ? "true" : "false")
            << " hard_current_progress_mm="
            << result.hard_current_progress_distance.value
            << " emergency_current="
            << (result.emergency_current_confirmed ? "true" : "false")
            << " velocity_collapsed="
            << (result.velocity_collapsed ? "true" : "false")
            << " sustained_no_progress="
            << (result.sustained_no_progress_confirmed ? "true" : "false")
            << " max_torque_nm=" << result.max_motor_torque.value
            << " last_torque_nm=" << result.last_motor_torque.value
            << " settled=" << (result.settled ? "true" : "false")
            << " settled_speed_mm_s=" << result.settled_nut_speed.value
            << " settled_position_delta_speed_mm_s="
            << result.settled_position_delta_speed.value
            << " settle_code=" << common::toString(result.settle_result.code())
            << " direction_ok=" << (result.sample.direction_matches ? "true" : "false")
            << " monotonic=" << (result.sample.position_monotonic ? "true" : "false")
            << " velocity_stable=" << (result.sample.velocity_stable ? "true" : "false")
            << " current_stable=" << (result.sample.current_stable ? "true" : "false")
            << " limit_suspected=" << (result.limit_suspected ? "true" : "false")
            << " jam_suspected=" << (result.jam_suspected ? "true" : "false");
    if (result.result.hasMessage()) {
      message << " message=" << result.result.message();
    }
    reportProgress(message.str());
  }

  void reportFrictionAnomalySummary() {
    std::ostringstream message;
    message << "PreSelfCheck | friction anomaly summary"
            << " candidates=" << friction_anomaly_records_.size();
    const auto* severe_record = mostSevereFrictionAnomalyRecord();
    if (severe_record != nullptr) {
      message << " max_ratio="
              << severe_record->current_excess_ratio.value
              << " center_mm=" << severe_record->center_position.value
              << " width_mm=" << severe_record->width.value
              << " peak_current_a=" << severe_record->peak_current.value
              << " baseline_current_a="
              << severe_record->baseline_current.value
              << " direction=" << directionName(severe_record->direction);
    }
    reportProgress(message.str());
    for (std::size_t index = 0; index < friction_anomaly_records_.size();
         ++index) {
      const auto& record = friction_anomaly_records_[index];
      std::ostringstream record_message;
      record_message << "PreSelfCheck | friction anomaly record"
                     << " index=" << index
                     << " center_mm=" << record.center_position.value
                     << " start_mm=" << record.start_position.value
                     << " end_mm=" << record.end_position.value
                     << " motor_pos_rad=" << record.motor_position.value
                     << " direction=" << directionName(record.direction)
                     << " ratio=" << record.current_excess_ratio.value
                     << " peak_current_a=" << record.peak_current.value
                     << " baseline_current_a="
                     << record.baseline_current.value
                     << " width_mm=" << record.width.value;
      reportProgress(record_message.str());
    }
  }

  void reportBreakawayAccepted(
      self_check::MotionDirection direction,
      const SelfCheckBreakawayCandidate& candidate) {
    const auto& sample = candidate.sample;
    std::ostringstream message;
    message << "PreSelfCheck | breakaway accepted"
            << " direction=" << directionName(direction)
            << " confidence="
            << (candidate.low_confidence_micro_motion ? "low_micro_motion"
                                                      : "normal")
            << " target_nut_speed_mm_s=" << sample.target_nut_speed.value
            << " measured_mm=" << sample.measured_distance.value
            << " avg_speed_mm_s=" << sample.average_nut_speed.value
            << " max_current_a=" << sample.max_motor_current.value
            << " max_torque_nm=" << sample.max_motor_torque.value
            << " current_stable=" << (sample.current_stable ? "true" : "false");
    reportProgress(message.str());
  }

  [[nodiscard]] const self_check::FrictionAnomalyRecord*
  mostSevereFrictionAnomalyRecord() const {
    const self_check::FrictionAnomalyRecord* best = nullptr;
    for (const auto& record : friction_anomaly_records_) {
      if (best == nullptr ||
          record.current_excess_ratio.value >
              best->current_excess_ratio.value) {
        best = &record;
      }
    }
    return best;
  }

  [[nodiscard]] common::Result updateFeedback() {
    // Feedback is the controller's only observation source. It updates the
    // stroke estimate and promotes motor fault feedback into the top-level fault
    // state.
    if (!motor_ || !motor_->isConnected()) {
      updateSnapshotFlags();
      return common::Ok();
    }
    MotorFeedback feedback{};
    const auto result = motor_->readFeedback(&feedback);
    if (result.isError()) {
      return result;
    }
    last_feedback_ = feedback;
    current_nut_stroke_ = updateNutPositionFromMotorFeedback(feedback);
    if (feedback.fault) {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      snapshot_.fault.severity = state_machine::FaultSeverity::Recoverable;
      snapshot_.fault.source = state_machine::FaultSource::Hardware;
      snapshot_.fault.code =
          common::toValue(common::ErrorCode::HardwareFault);
      state_machine_.forceFault();
    }
    return common::Ok();
  }

  [[nodiscard]] self_check::StructureProfile makeConservativeProfile() const {
    // Conservative defaults are safe fallback data, not learned data. The
    // quality marker lets downstream code distinguish fallback operation from
    // verified structure identification.
    self_check::StructureProfile profile{};
    profile.validity = StructureProfileValidity::ConservativeDefaults;
    profile.quality = self_check::IdentificationQuality::ConservativeDefault;
    profile.opening.minimum_stable_nut_speed =
        config_.self_check.min_speed_scan_start;
    profile.opening.low_speed_unstable_upper_bound =
        config_.self_check.min_speed_scan_stop;
    profile.opening.quality = self_check::IdentificationQuality::ConservativeDefault;
    profile.closing = profile.opening;

    profile.noise_floor.motor_position_noise =
        config_.self_check.fallback_motor_position_noise;
    profile.noise_floor.motor_velocity_noise =
        config_.self_check.fallback_motor_velocity_noise;
    profile.noise_floor.motor_current_noise =
        config_.self_check.fallback_motor_current_noise;
    profile.noise_floor.motor_torque_noise =
        config_.self_check.fallback_motor_torque_noise;
    profile.noise_floor.nut_stroke_noise =
        config_.self_check.fallback_nut_stroke_noise;
    profile.noise_floor.quality =
        self_check::IdentificationQuality::ConservativeDefault;

    profile.travel_limits.preliminary_open_limit =
        config_.mechanism.theoretical_open_limit;
    profile.travel_limits.preliminary_closed_limit =
        config_.mechanism.theoretical_close_limit;
    profile.travel_limits.safe_zone_open_limit =
        common::Mm{config_.mechanism.theoretical_open_limit.value +
                   config_.self_check.safe_zone_margin.value};
    profile.travel_limits.safe_zone_closed_limit =
        common::Mm{config_.mechanism.theoretical_close_limit.value -
                   config_.self_check.safe_zone_margin.value};
    profile.travel_limits.software_open_limit =
        common::Mm{config_.mechanism.theoretical_open_limit.value +
                   config_.self_check.software_limit_margin.value};
    profile.travel_limits.software_closed_limit =
        common::Mm{config_.mechanism.theoretical_close_limit.value -
                   config_.self_check.software_limit_margin.value};
    profile.travel_limits.learned_travel =
        common::Mm{config_.mechanism.theoretical_close_limit.value -
                   config_.mechanism.theoretical_open_limit.value};
    profile.travel_limits.quality =
        self_check::IdentificationQuality::ConservativeDefault;
    return profile;
  }

  [[nodiscard]] common::Result probeMotion(
      self_check::MotionDirection direction, common::Mm relative_distance,
      common::MmPerS target_nut_speed, common::A current_limit,
      SelfCheckProbeOptions options) {
    const auto probe = runSelfCheckMotionProbe(
        direction, relative_distance, target_nut_speed, current_limit,
        options);
    last_self_check_probe_ = probe;
    last_motion_sample_ = probe.sample;
    return probe.result;
  }

  [[nodiscard]] bool isUnsettledSelfCheckProbe(
      const SelfCheckMotionProbeResult& probe) const {
    return !probe.settled && probe.settle_result.isError();
  }

  [[nodiscard]] bool selfCheckProbeMessageContains(
      const SelfCheckMotionProbeResult& probe, const char* needle) const {
    return needle != nullptr && probe.result.hasMessage() &&
           probe.result.message().find(needle) != std::string::npos;
  }

  [[nodiscard]] bool isSelfCheckTerminalFeedbackStop(
      const SelfCheckMotionProbeResult& probe) const {
    if (probe.result.code() != common::ErrorCode::SafetyJamDetected) {
      return false;
    }
    // End-stop feedback stops must unload before final sampling. Holding the
    // current position can keep a lead screw wedged against the mechanical end.
    return probe.emergency_current_confirmed ||
           probe.hard_current_no_progress_confirmed ||
           probe.sustained_no_progress_confirmed ||
           selfCheckProbeMessageContains(
               probe, "emergency feedback current limit") ||
           selfCheckProbeMessageContains(
               probe, "sustained hard feedback current limit") ||
           selfCheckProbeMessageContains(
               probe, "continuous scan stopped by sustained no progress");
  }

  [[nodiscard]] SelfCheckMotionProbeResult runSelfCheckMotionProbe(
      self_check::MotionDirection direction, common::Mm relative_distance,
      common::MmPerS target_nut_speed, common::A current_limit,
      SelfCheckProbeOptions options) {
    // PreSelfCheck uses the normal gripper control path, not MotorBringup.
    // The command is sent as a mechanism-side target stroke with both nut-speed
    // and motor-current limits, then feedback is sampled until the small target
    // window, jam condition, or timeout is reached.
    SelfCheckMotionProbeResult output{};
    const auto feedback_before = updateFeedback();
    if (feedback_before.isError()) {
      output.result = feedback_before;
      return output;
    }

    output.start_stroke = current_nut_stroke_;
    output.start_motor_position = last_feedback_.position;
    output.start_raw_position_counts = last_feedback_.raw_position_counts;
    output.start_raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    const double signed_distance =
        direction == self_check::MotionDirection::Opening
            ? -std::abs(relative_distance.value)
            : std::abs(relative_distance.value);
    common::Mm target_stroke{output.start_stroke.value + signed_distance};
    if (!options.use_custom_stroke_limits) {
      target_stroke = common::Mm{std::clamp(
          target_stroke.value, config_.mechanism.theoretical_open_limit.value,
          config_.mechanism.theoretical_close_limit.value)};
    } else {
      const double guard_margin =
          std::max(config_.self_check.max_distance_error.value,
                   config_.self_check.fallback_nut_stroke_noise.value * 3.0);
      const double observed_low =
          std::min(output.start_stroke.value, target_stroke.value) -
          guard_margin;
      const double observed_high =
          std::max(output.start_stroke.value, target_stroke.value) +
          guard_margin;
      const double configured_low =
          std::min(options.custom_open_limit.value,
                   options.custom_closed_limit.value);
      const double configured_high =
          std::max(options.custom_open_limit.value,
                   options.custom_closed_limit.value);
      options.custom_open_limit =
          common::Mm{std::min(configured_low, observed_low)};
      options.custom_closed_limit =
          common::Mm{std::max(configured_high, observed_high)};
    }
    output.target_motor_position =
        strokeToReferencedMotorPosition(target_stroke);
    const common::MmPerS signed_speed{
        direction == self_check::MotionDirection::Opening
            ? -std::abs(target_nut_speed.value)
            : std::abs(target_nut_speed.value)};
    const common::A command_current_limit =
        selfCheckCommandCurrentLimit(current_limit, options.command_current_cap);
    output.feedback_hard_current_limit = selfCheckFeedbackHardCurrentLimit();
    output.feedback_emergency_current_limit =
        selfCheckFeedbackEmergencyCurrentLimit();
    output.hard_current_confirm_time = common::S{
        std::max(0.0, options.hard_current_confirm_time.value)};

    reportSelfCheckAttempt(direction, target_stroke, signed_speed,
                           command_current_limit);

    const auto timeout = motionTimeoutFor(output.start_stroke, target_stroke,
                                         signed_speed, {});
    const auto command_result =
        sendLimitedCommand(target_stroke, signed_speed, command_current_limit,
                           false, options.use_custom_stroke_limits,
                           options.custom_open_limit,
                           options.custom_closed_limit,
                           output.feedback_hard_current_limit);
    if (command_result.isError()) {
      if (command_result.code() == common::ErrorCode::SafetyActiveStop) {
        (void)disableAfterMotion();
      }
      output.result = command_result;
      return output;
    }
    if (options.record_pre_b_current_trace) {
      beginPreBCurrentTraceSegment();
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::duration<double>{timeout.value});
    double speed_sum = 0.0;
    double current_sum = 0.0;
    double torque_sum = 0.0;
    std::uint32_t sample_count = 0;
    common::Mm previous_stroke = current_nut_stroke_;
    bool monotonic = true;
    bool direction_matches = true;
    bool target_reached = false;
    common::Result feedback_result = common::Ok();
    std::chrono::steady_clock::time_point no_progress_since{};
    bool no_progress_timer_running = false;
    std::chrono::steady_clock::time_point scan_progress_since{};
    bool scan_progress_timer_running = false;
    common::Mm scan_progress_reference = current_nut_stroke_;
    std::chrono::steady_clock::time_point hard_current_since{};
    bool hard_current_timer_running = false;
    common::Mm hard_current_progress_reference = current_nut_stroke_;

    while (std::chrono::steady_clock::now() < deadline) {
      if (const auto cancel = checkOperationCancelled("pre-self-check probe wait");
          cancel.isError()) {
        output.result = cancel;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      feedback_result = updateFeedback();
      if (feedback_result.isError()) {
        output.result = feedback_result;
        break;
      }

      const common::MmPerS measured_speed = measuredNutSpeed();
      feedPreBFeedbackRecorders(options.friction_anomaly_detector, direction,
                                measured_speed,
                                options.record_pre_b_current_trace);
      const double abs_speed = std::abs(measured_speed.value);
      const double abs_current = std::abs(last_feedback_.current.value);
      const double abs_torque = std::abs(last_feedback_.torque.value);
      speed_sum += abs_speed;
      current_sum += abs_current;
      torque_sum += abs_torque;
      ++sample_count;
      output.max_nut_speed.value = std::max(output.max_nut_speed.value,
                                            abs_speed);
      output.max_motor_current.value = std::max(output.max_motor_current.value,
                                                abs_current);
      output.max_motor_torque.value = std::max(output.max_motor_torque.value,
                                               abs_torque);
      output.last_motor_current = last_feedback_.current;
      output.last_motor_torque = last_feedback_.torque;
      output.max_motor_temperature.value =
          std::max(output.max_motor_temperature.value,
                   last_feedback_.temperature.value);

      const bool over_feedback_emergency_current =
          feedbackCurrentExceedsHardLimit(output.feedback_emergency_current_limit);
      if (over_feedback_emergency_current) {
        output.hard_current_transient_observed = true;
        output.hard_current_confirmed = true;
        output.hard_current_no_progress_confirmed = true;
        output.emergency_current_confirmed = true;
        output.velocity_collapsed = true;
        output.limit_suspected = true;
        output.jam_suspected = true;
        output.result = common::Result::error(
            common::ErrorCode::SafetyJamDetected,
            "pre-self-check probe stopped by emergency feedback current limit");
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      const auto refresh_result = sendLimitedCommand(
          target_stroke, signed_speed, command_current_limit, false,
          options.use_custom_stroke_limits, options.custom_open_limit,
          options.custom_closed_limit, output.feedback_hard_current_limit);
      if (refresh_result.isError()) {
        if (refresh_result.code() == common::ErrorCode::SafetyActiveStop) {
          (void)disableAfterMotion();
        }
        output.result = refresh_result;
        break;
      }

      const double stroke_delta =
          current_nut_stroke_.value - previous_stroke.value;
      if (direction == self_check::MotionDirection::Opening) {
        if (stroke_delta > config_.self_check.max_distance_error.value) {
          monotonic = false;
        }
      } else if (stroke_delta < -config_.self_check.max_distance_error.value) {
        monotonic = false;
      }
      previous_stroke = current_nut_stroke_;

      const double total_delta =
          current_nut_stroke_.value - output.start_stroke.value;
      const double motion_start_threshold =
          options.stop_after_motion_start ? lowConfidenceBreakawayDistance()
                                          : normalBreakawayDistance();
      const double opposite_direction_threshold = std::max(
          motion_start_threshold,
          config_.self_check.max_distance_error.value * 0.25);
      if (direction == self_check::MotionDirection::Opening) {
        direction_matches =
            total_delta <= -motion_start_threshold ||
            std::abs(total_delta) < motion_start_threshold;
      } else {
        direction_matches =
            total_delta >= motion_start_threshold ||
            std::abs(total_delta) < motion_start_threshold;
      }
      const bool motion_started_in_requested_direction =
          direction == self_check::MotionDirection::Opening
              ? total_delta <= -motion_start_threshold
              : total_delta >= motion_start_threshold;
      const bool motion_started_opposite_direction =
          direction == self_check::MotionDirection::Opening
              ? total_delta >= opposite_direction_threshold
              : total_delta <= -opposite_direction_threshold;
      const bool over_feedback_hard_current =
          feedbackCurrentExceedsHardLimit(output.feedback_hard_current_limit);
      if (over_feedback_hard_current) {
        output.hard_current_transient_observed = true;
        if (!hard_current_timer_running) {
          hard_current_since = now;
          hard_current_progress_reference = current_nut_stroke_;
          hard_current_timer_running = true;
        }
        const double progress_sign =
            direction == self_check::MotionDirection::Opening ? -1.0 : 1.0;
        const double signed_hard_current_progress =
            (current_nut_stroke_.value -
             hard_current_progress_reference.value) *
            progress_sign;
        output.hard_current_progress_distance = common::Mm{std::max(
            output.hard_current_progress_distance.value,
            signed_hard_current_progress)};
        const bool hard_current_progress_made =
            madeProgressInCommandDirection(hard_current_progress_reference,
                                           signed_speed,
                                           selfCheckNoProgressThreshold());
        if (hard_current_progress_made) {
          output.hard_current_progress_observed = true;
          hard_current_progress_reference = current_nut_stroke_;
          hard_current_since = now;
        }
        const auto over_limit_duration = now - hard_current_since;
        const double over_limit_duration_s =
            std::chrono::duration<double>{over_limit_duration}.count();
        output.hard_current_over_limit_duration = common::S{std::max(
            output.hard_current_over_limit_duration.value,
            over_limit_duration_s)};
        output.hard_current_confirmed =
            output.hard_current_confirm_time.value <= 0.0 ||
            over_limit_duration >=
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>{
                        output.hard_current_confirm_time.value});
        output.hard_current_no_progress_confirmed =
            output.hard_current_confirmed && !hard_current_progress_made;
      } else {
        hard_current_timer_running = false;
      }
      if (options.early_progress_timeout.value > 0.0) {
        if (motion_started_in_requested_direction) {
          no_progress_timer_running = false;
        } else if (!no_progress_timer_running) {
          no_progress_since = std::chrono::steady_clock::now();
          no_progress_timer_running = true;
        } else {
          const auto no_progress_duration =
              std::chrono::steady_clock::now() - no_progress_since;
          if (no_progress_duration >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{
                      options.early_progress_timeout.value})) {
            output.result = common::Result::error(
                common::ErrorCode::OperationTimedOut,
                "pre-self-check soft-jam retry made no early progress");
            break;
          }
        }
      }
      bool sustained_no_progress_boundary = false;
      if (options.sustained_no_progress_timeout.value > 0.0) {
        const double progress_sign =
            direction == self_check::MotionDirection::Opening ? -1.0 : 1.0;
        const double signed_scan_progress =
            (current_nut_stroke_.value - scan_progress_reference.value) *
            progress_sign;
        const double progress_threshold = selfCheckNoProgressThreshold().value;
        if (signed_scan_progress > progress_threshold) {
          scan_progress_reference = current_nut_stroke_;
          scan_progress_timer_running = false;
        } else if (!scan_progress_timer_running) {
          scan_progress_since = std::chrono::steady_clock::now();
          scan_progress_timer_running = true;
        } else if (scan_progress_timer_running) {
          const auto no_scan_progress_duration =
              std::chrono::steady_clock::now() - scan_progress_since;
          if (no_scan_progress_duration >=
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>{
                      options.sustained_no_progress_timeout.value})) {
            sustained_no_progress_boundary = true;
            output.sustained_no_progress_confirmed = true;
            output.velocity_collapsed = true;
            output.limit_suspected = true;
            output.jam_suspected = true;
          }
        }
      }
      const auto risk = evaluateSelfCheckProbeRisk(
          signed_speed, options, motion_started_in_requested_direction,
          output.hard_current_no_progress_confirmed);
      output.jam_suspected = output.jam_suspected || risk.jam_suspected;
      output.limit_suspected = output.limit_suspected || risk.limit_suspected;
      output.velocity_collapsed =
          output.velocity_collapsed || risk.velocity_collapsed;
      if (risk.stop_required || sustained_no_progress_boundary) {
        output.result = common::Result::error(
            common::ErrorCode::SafetyJamDetected,
            sustained_no_progress_boundary
                ? "pre-self-check continuous scan stopped by sustained no progress"
                : risk.message);
        break;
      }
      if (motion_started_opposite_direction) {
        output.result = common::Result::error(
            common::ErrorCode::SelfCheckInconsistentFeedback,
            "pre-self-check probe moved opposite to requested direction");
        break;
      }
      if (options.stop_after_motion_start &&
          motion_started_in_requested_direction) {
        output.result = common::Result::error(
            common::ErrorCode::OperationTimedOut,
            "pre-self-check breakaway threshold reached before target window");
        break;
      }

      target_reached =
          isSelfCheckProbeTargetReached(output.start_stroke, target_stroke,
                                        signed_speed);
      if (target_reached) {
        output.result = common::Ok();
        break;
      }
    }
    output.target_reached = target_reached;

    if (sample_count > 0U) {
      const double count = static_cast<double>(sample_count);
      output.average_nut_speed = common::MmPerS{speed_sum / count};
      output.average_motor_current = common::A{current_sum / count};
      output.average_motor_torque = common::Nm{torque_sum / count};
    }
    const bool cancelled =
        output.result.code() == common::ErrorCode::SafetyActiveStop &&
        operationCancelRequested();
    const bool terminal_feedback_stop =
        isSelfCheckTerminalFeedbackStop(output);
    MotionSettleResult settle_result{};
    if (terminal_feedback_stop) {
      std::ostringstream settle_message;
      settle_message
          << "PreSelfCheck | probe terminal feedback stop"
          << " action=disable_and_wait"
          << " code=" << common::toString(output.result.code());
      if (output.result.hasMessage()) {
        settle_message << " message=" << output.result.message();
      }
      reportProgress(settle_message.str());
      settle_result = disableAndWaitForSelfCheckSettledFeedback();
    } else {
      settle_result =
          holdAndWaitForSelfCheckSettledFeedback(command_current_limit);
      if (settle_result.result.isError()) {
        std::ostringstream fallback_message;
        fallback_message
            << "PreSelfCheck | probe settle fallback"
            << " reason=" << settle_result.result.message()
            << " action=disable_and_wait";
        reportProgress(fallback_message.str());
        settle_result = disableAndWaitForSelfCheckSettledFeedback();
      }
    }
    output.end_stroke = settle_result.settled_stroke;
    output.end_motor_position = settle_result.settled_motor_position;
    output.end_raw_position_counts = settle_result.settled_raw_position_counts;
    output.end_raw_position_counts_valid =
        settle_result.settled_raw_position_counts_valid;
    output.settled = settle_result.settled;
    output.settled_nut_speed = settle_result.settled_nut_speed;
    output.settled_position_delta_speed =
        settle_result.settled_position_delta_speed;
    output.settle_result = settle_result.result;
    output.measured_distance = common::Mm{
        std::abs(output.end_stroke.value - output.start_stroke.value)};

    if (cancelled) {
      output.result = common::Result::error(
          common::ErrorCode::SafetyActiveStop,
          "pre-self-check probe cancelled by user stop");
    } else if (settle_result.result.isError()) {
      output.result = settle_result.result;
    }
    if (isHardSelfCheckNoProgressJamResult(output)) {
      (void)disableAfterMotion();
    }

    const double commanded_distance =
        std::abs(target_stroke.value - output.start_stroke.value);
    const double final_delta =
        output.end_stroke.value - output.start_stroke.value;
    const double motion_start_threshold =
        options.stop_after_motion_start ? lowConfidenceBreakawayDistance()
                                        : normalBreakawayDistance();
    const double opposite_direction_threshold = std::max(
        motion_start_threshold,
        config_.self_check.max_distance_error.value * 0.25);
    const bool final_opposite_direction_detected =
        direction == self_check::MotionDirection::Opening
            ? final_delta > opposite_direction_threshold
            : final_delta < -opposite_direction_threshold;
    const bool final_direction_matches =
        direction == self_check::MotionDirection::Opening
            ? (final_delta <= -motion_start_threshold ||
               std::abs(final_delta) < motion_start_threshold)
            : (final_delta >= motion_start_threshold ||
               std::abs(final_delta) < motion_start_threshold);
    if (!cancelled && settle_result.settled && final_opposite_direction_detected) {
      output.result = common::Result::error(
          common::ErrorCode::SelfCheckInconsistentFeedback,
          "pre-self-check settled feedback moved opposite to requested direction");
    }
    if (!cancelled && output.result.isOk() && !target_reached) {
      output.result = common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "pre-self-check probe did not reach target before timeout");
    }
    const bool position_monotonic =
        monotonic && final_direction_matches;
    const bool velocity_stable =
        output.average_nut_speed.value +
            config_.self_check.stable_speed_margin.value >=
        std::abs(signed_speed.value);
    const bool current_stable =
        output.max_motor_current.value <=
        command_current_limit.value + config_.self_check.max_current_ripple.value;
    output.sample = self_check::MotionIdentificationSample{
        direction,
        common::Mm{commanded_distance},
        output.measured_distance,
        common::MmPerS{std::abs(signed_speed.value)},
        output.average_nut_speed,
        output.max_nut_speed,
        output.average_motor_current,
        output.max_motor_current,
        output.average_motor_torque,
        output.max_motor_torque,
        output.max_motor_temperature,
        direction_matches && final_direction_matches,
        position_monotonic,
        velocity_stable,
        current_stable,
        output.limit_suspected,
        output.jam_suspected,
        output.result.code() == common::ErrorCode::SafetyActiveStop,
        output.settled};

    if (!operationCancelRequested() &&
        state_machine_.state() != GripperTopState::ActiveStop &&
        state_machine_.state() != GripperTopState::Fault && output.settled &&
        !isSelfCheckTerminalFeedbackStop(output)) {
      const auto enable_result = reenableAfterSelfCheckProbe();
      if (enable_result.isError() && output.result.isOk()) {
        output.result = enable_result;
      }
    }
    reportSelfCheckResult(direction, output);
    return output;
  }

  [[nodiscard]] common::Result runProgressiveBidirectionalProbe(
      self_check::SelfCheckInput* input) {
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }

    const auto paired_probe = runPairedBreakawayScan(input);
    if (paired_probe.isError()) {
      return paired_probe;
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result runPairedBreakawayScan(
      self_check::SelfCheckInput* input) {
    const double speed_start =
        std::abs(config_.self_check.min_speed_scan_start.value);
    const double speed_stop =
        std::abs(config_.self_check.min_speed_scan_stop.value);
    const double speed_step =
        std::abs(config_.self_check.min_speed_scan_step.value) > 0.0
            ? std::abs(config_.self_check.min_speed_scan_step.value)
            : std::max(0.1, speed_stop - speed_start);
    const double configured_current_start =
        std::abs(config_.self_check.static_friction_current_start.value);
    const double configured_current_stop =
        std::abs(config_.self_check.static_friction_current_stop.value);
    const double current_step =
        std::abs(config_.self_check.static_friction_current_step.value) > 0.0
            ? std::abs(config_.self_check.static_friction_current_step.value)
            : 0.05;
    const double safety_current =
        std::abs(config_.safety.self_check_current_limit.value);
    const double current_stop =
        std::min(safety_current,
                 configured_current_stop > 0.0 ? configured_current_stop
                                                : safety_current);
    const double current_start = selfCheckBreakawayStartCurrent(
        current_step, configured_current_start, current_stop);
    const common::Mm paired_probe_window{
        std::min(config_.self_check.max_probe_window.value,
                 std::max(config_.self_check.motion_start_distance.value * 4.0,
                          config_.self_check.min_measured_distance.value))};

    if (speed_stop <= 0.0 || current_stop <= 0.0 ||
        paired_probe_window.value <= 0.0 || current_start > current_stop) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "pre-self-check paired scan config must have positive speed, current, and probe window");
    }

    SelfCheckBreakawayCandidate closing{};
    SelfCheckBreakawayCandidate opening{};
    common::Result last_error = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "pre-self-check paired breakaway scan found no usable motion");

    std::ostringstream start_message;
    start_message << "PreSelfCheck | phase=BidirectionalMoveEnable"
                  << " | paired_scan speed_start_mm_s=" << speed_start
                  << " speed_stop_mm_s=" << speed_stop
                  << " current_start_a=" << current_start
                  << " current_stop_a=" << current_stop
                  << " probe_window_mm=" << paired_probe_window.value;
    reportProgress(start_message.str());

    for (double speed = speed_start; speed <= speed_stop + 1e-9;
         speed += speed_step) {
      if (const auto cancel = checkOperationCancelled("paired breakaway speed scan");
          cancel.isError()) {
        return cancel;
      }
      for (double current = current_start; current <= current_stop + 1e-9;
           current += current_step) {
        if (const auto cancel = checkOperationCancelled("paired breakaway current scan");
            cancel.isError()) {
          return cancel;
        }
        const common::A limited_current{std::min(current, current_stop)};
        if (!closing.found) {
          const auto close_result =
              probeMotion(self_check::MotionDirection::Closing,
                          paired_probe_window, common::MmPerS{speed},
                          limited_current,
                          SelfCheckProbeOptions{
                              true});
          if (isUnsettledSelfCheckProbe(last_self_check_probe_)) {
            return close_result;
          }
          if (!closing.found && !opening.found) {
            markEarlyBreakawayOppositeDriftBoundary(
                self_check::MotionDirection::Closing, limited_current.value);
          }
          if (opening.found &&
              completeOneSidedBreakawayAsOppositeDriftBoundary(
                  opening, self_check::MotionDirection::Closing,
                  limited_current.value)) {
            return common::Ok();
          }
          if (isUnrecoverableBreakawayProbeResult(close_result)) {
            return close_result;
          }
          if (acceptBreakawayCandidate(self_check::MotionDirection::Closing,
                                       limited_current, common::MmPerS{speed},
                                       input, &closing)) {
            reportBreakawayAccepted(self_check::MotionDirection::Closing,
                                    closing);
          } else if (close_result.isError()) {
            last_error = close_result;
          }
        }

        if (!opening.found) {
          const auto open_result =
              probeMotion(self_check::MotionDirection::Opening,
                          paired_probe_window, common::MmPerS{speed},
                          limited_current,
                          SelfCheckProbeOptions{
                              true});
          if (isUnsettledSelfCheckProbe(last_self_check_probe_)) {
            return open_result;
          }
          if (!closing.found && !opening.found) {
            markEarlyBreakawayOppositeDriftBoundary(
                self_check::MotionDirection::Opening, limited_current.value);
          }
          if (closing.found &&
              completeOneSidedBreakawayAsOppositeDriftBoundary(
                  closing, self_check::MotionDirection::Opening,
                  limited_current.value)) {
            return common::Ok();
          }
          if (isUnrecoverableBreakawayProbeResult(open_result)) {
            return open_result;
          }
          if (acceptBreakawayCandidate(self_check::MotionDirection::Opening,
                                       limited_current, common::MmPerS{speed},
                                       input, &opening)) {
            reportBreakawayAccepted(self_check::MotionDirection::Opening,
                                    opening);
          } else if (open_result.isError()) {
            last_error = open_result;
          }
        }

        if (closing.found && opening.found) {
          return common::Ok();
        }
        if (completeOneSidedBreakawayAsLowConfidenceBoundary(
                closing, opening, limited_current.value, current_stop)) {
          return common::Ok();
        }
      }
    }

    if (!closing.found && !opening.found) {
      return common::Result::error(
          common::ErrorCode::SelfCheckUnsafeMotion,
          "pre-self-check could not start motion in either direction with paired probes");
    }
    (void)last_error;
    return common::Result::error(
        common::ErrorCode::SelfCheckUnsafeMotion,
        "pre-self-check requires paired breakaway motion in both directions");
  }

  [[nodiscard]] bool acceptBreakawayCandidate(
      self_check::MotionDirection direction, common::A current_limit,
      common::MmPerS target_nut_speed,
      self_check::SelfCheckInput* input, SelfCheckBreakawayCandidate* candidate) {
    if (candidate == nullptr || candidate->found) {
      return false;
    }
    const auto& sample = last_motion_sample_;
    const double normal_start_distance = normalBreakawayDistance();
    const double low_confidence_distance = lowConfidenceBreakawayDistance();
    const double signed_delta =
        last_self_check_probe_.end_stroke.value -
        last_self_check_probe_.start_stroke.value;
    const bool low_confidence_direction_motion =
        direction == self_check::MotionDirection::Opening
            ? signed_delta <= -low_confidence_distance
            : signed_delta >= low_confidence_distance;
    const bool normal_direction_motion =
        direction == self_check::MotionDirection::Opening
            ? signed_delta <= -normal_start_distance
            : signed_delta >= normal_start_distance;
    const bool motion_started =
        sample.commanded_direction == direction && sample.settled &&
        sample.direction_matches && sample.position_monotonic &&
        low_confidence_direction_motion &&
        sample.measured_distance.value >= low_confidence_distance &&
        !sample.limit_triggered && !sample.jam_detected &&
        !sample.active_stop_triggered;
    if (!motion_started) {
      return false;
    }
    const bool command_current_within_self_check_envelope =
        current_limit.value <= config_.safety.self_check_current_limit.value +
                                   config_.self_check.max_current_ripple.value;
    const common::A hard_limit = selfCheckFeedbackHardCurrentLimit();
    const bool feedback_current_within_hard_limit =
        hard_limit.value <= 0.0 ||
        sample.max_motor_current.value <=
            hard_limit.value;
    if (!command_current_within_self_check_envelope ||
        !feedback_current_within_hard_limit) {
      return false;
    }

    candidate->found = true;
    candidate->sample = sample;
    candidate->current_limit = current_limit;
    candidate->target_nut_speed = target_nut_speed;
    candidate->low_confidence_micro_motion = !normal_direction_motion;
    candidate->sample.current_stable =
        current_limit.value <= config_.safety.self_check_current_limit.value +
                                   config_.self_check.max_current_ripple.value;
    auto learned_sample = candidate->sample;
    if (candidate->low_confidence_micro_motion) {
      learned_sample.max_motor_current.value =
          std::min(learned_sample.max_motor_current.value, current_limit.value);
      learned_sample.average_motor_current.value =
          std::min(learned_sample.average_motor_current.value,
                   current_limit.value);
    }
    (void)input;
    updateRuntimeBreakawayProfile(direction, learned_sample);
    return true;
  }

  [[nodiscard]] bool isUnrecoverableBreakawayProbeResult(
      const common::Result& result) const {
    // Only feedback contradiction, safety stops, hardware faults, or
    // communication loss block PreSelfCheck immediately. Plain timeout can mean
    // "no motion under low energy" and may be downgraded to a suspected
    // boundary during the first-pass precheck.
    return result.code() == common::ErrorCode::SelfCheckInconsistentFeedback ||
           result.code() == common::ErrorCode::SafetyActiveStop ||
           result.code() == common::ErrorCode::SafetyJamDetected ||
           result.code() == common::ErrorCode::SafetyContactDetected ||
           result.code() == common::ErrorCode::HardwareFault ||
           result.code() == common::ErrorCode::HardwareFeedbackInvalid ||
           result.code() == common::ErrorCode::HardwareRejectedCommand ||
           result.code() == common::ErrorCode::ConnectionLost ||
           result.code() == common::ErrorCode::ConnectionNotOpen ||
           result.code() == common::ErrorCode::FeedbackTimedOut;
  }

  [[nodiscard]] bool lastProbeLooksLikeLowConfidenceBoundary(
      self_check::MotionDirection missing_direction) const {
    const auto& sample = last_motion_sample_;
    const double start_distance = lowConfidenceBreakawayDistance();
    const bool no_effective_motion =
        sample.measured_distance.value < start_distance;
    const common::A hard_limit = selfCheckFeedbackHardCurrentLimit();
    const bool current_within_self_check_envelope =
        hard_limit.value <= 0.0 ||
        sample.max_motor_current.value <= hard_limit.value;
    return sample.commanded_direction == missing_direction && sample.settled &&
           sample.direction_matches && sample.position_monotonic &&
           no_effective_motion && current_within_self_check_envelope &&
           !sample.limit_triggered && !sample.jam_detected &&
           !sample.active_stop_triggered;
  }

  [[nodiscard]] bool completeOneSidedBreakawayAsLowConfidenceBoundary(
      const SelfCheckBreakawayCandidate& closing,
      const SelfCheckBreakawayCandidate& opening, double current_limit,
      double current_stop) {
    // PreSelfCheck is allowed to build a low-confidence temporary safe zone.
    // Once one direction is confirmed, repeatedly driving a non-moving opposite
    // direction is less informative and can push against a real boundary.
    if (closing.found == opening.found) {
      return false;
    }

    const auto confirmed_direction =
        closing.found ? self_check::MotionDirection::Closing
                      : self_check::MotionDirection::Opening;
    const auto missing_direction =
        closing.found ? self_check::MotionDirection::Opening
                      : self_check::MotionDirection::Closing;
    const SelfCheckBreakawayCandidate& confirmed =
        closing.found ? closing : opening;
    const double current_ceiling = std::min(
        current_stop, confirmed.current_limit.value +
                          std::abs(config_.self_check.max_current_ripple.value));
    if (current_limit + 1e-9 < current_ceiling ||
        !lastProbeLooksLikeLowConfidenceBoundary(missing_direction)) {
      return false;
    }

    if (missing_direction == self_check::MotionDirection::Opening) {
      pre_self_check_opening_boundary_suspected_ = true;
    } else {
      pre_self_check_closing_boundary_suspected_ = true;
    }

    std::ostringstream message;
    message << "PreSelfCheck | phase=BidirectionalMoveEnable | degraded"
            << " confirmed_direction=" << directionName(confirmed_direction)
            << " suspected_boundary_direction=" << directionName(missing_direction)
            << " reason=no_effective_motion_under_limited_scan"
            << " confirmed_current_a=" << confirmed.current_limit.value
            << " scan_current_a=" << current_limit
            << " current_ceiling_a=" << current_ceiling
            << " missing_measured_mm="
            << last_motion_sample_.measured_distance.value
            << " missing_max_current_a="
            << last_motion_sample_.max_motor_current.value;
    reportProgress(message.str());
    return true;
  }

  [[nodiscard]] bool lastProbeLooksLikeLowEnergyOppositeDrift(
      self_check::MotionDirection missing_direction,
      self_check::MotionDirection confirmed_direction) const {
    const auto& probe = last_self_check_probe_;
    const auto& sample = last_motion_sample_;
    const double start_distance = lowConfidenceBreakawayDistance();
    const double max_low_energy_drift = std::max(
        config_.self_check.max_distance_error.value,
        config_.self_check.motion_start_distance.value * 2.0);
    const double signed_delta =
        probe.end_stroke.value - probe.start_stroke.value;
    const bool drifted_toward_confirmed =
        confirmed_direction == self_check::MotionDirection::Closing
            ? signed_delta >= start_distance
            : signed_delta <= -start_distance;
    const bool commanded_missing_direction =
        sample.commanded_direction == missing_direction;
    const common::A hard_limit = selfCheckFeedbackHardCurrentLimit();
    const bool current_within_self_check_envelope =
        hard_limit.value <= 0.0 ||
        sample.max_motor_current.value <= hard_limit.value;
    return commanded_missing_direction && sample.settled &&
           !sample.direction_matches && drifted_toward_confirmed &&
           sample.measured_distance.value <= max_low_energy_drift &&
           current_within_self_check_envelope && !sample.limit_triggered &&
           !sample.jam_detected && !sample.active_stop_triggered;
  }

  [[nodiscard]] bool lastProbeLooksLikeEarlyLowEnergyOppositeDrift(
      self_check::MotionDirection requested_direction) const {
    const auto& probe = last_self_check_probe_;
    const auto& sample = last_motion_sample_;
    const double start_distance = lowConfidenceBreakawayDistance();
    const double max_low_energy_drift = std::max(
        config_.self_check.max_distance_error.value,
        config_.self_check.motion_start_distance.value * 2.0);
    const double signed_delta =
        probe.end_stroke.value - probe.start_stroke.value;
    const bool drifted_opposite =
        requested_direction == self_check::MotionDirection::Opening
            ? signed_delta >= start_distance
            : signed_delta <= -start_distance;
    const common::A hard_limit = selfCheckFeedbackHardCurrentLimit();
    const bool current_within_self_check_envelope =
        hard_limit.value <= 0.0 ||
        sample.max_motor_current.value <= hard_limit.value;
    return sample.commanded_direction == requested_direction &&
           sample.settled && !sample.direction_matches && drifted_opposite &&
           sample.measured_distance.value <= max_low_energy_drift &&
           current_within_self_check_envelope && !sample.limit_triggered &&
           !sample.jam_detected && !sample.active_stop_triggered;
  }

  bool markEarlyBreakawayOppositeDriftBoundary(
      self_check::MotionDirection requested_direction, double current_limit) {
    if (!lastProbeLooksLikeEarlyLowEnergyOppositeDrift(requested_direction)) {
      return false;
    }

    if (requested_direction == self_check::MotionDirection::Opening) {
      pre_self_check_opening_boundary_suspected_ = true;
    } else {
      pre_self_check_closing_boundary_suspected_ = true;
    }

    const auto drift_direction =
        requested_direction == self_check::MotionDirection::Opening
            ? self_check::MotionDirection::Closing
            : self_check::MotionDirection::Opening;
    std::ostringstream message;
    message << "PreSelfCheck | phase=BidirectionalMoveEnable | degraded"
            << " requested_direction=" << directionName(requested_direction)
            << " observed_drift_direction=" << directionName(drift_direction)
            << " suspected_boundary_direction="
            << directionName(requested_direction)
            << " reason=early_low_energy_opposite_drift"
            << " scan_current_a=" << current_limit
            << " measured_mm=" << last_motion_sample_.measured_distance.value
            << " max_current_a=" << last_motion_sample_.max_motor_current.value;
    reportProgress(message.str());
    return true;
  }

  [[nodiscard]] bool completeOneSidedBreakawayAsOppositeDriftBoundary(
      const SelfCheckBreakawayCandidate& confirmed,
      self_check::MotionDirection missing_direction, double current_limit) {
    const auto confirmed_direction = confirmed.sample.commanded_direction;
    if (!confirmed.found ||
        confirmed_direction == self_check::MotionDirection::Unknown ||
        !lastProbeLooksLikeLowEnergyOppositeDrift(missing_direction,
                                                  confirmed_direction)) {
      return false;
    }

    if (missing_direction == self_check::MotionDirection::Opening) {
      pre_self_check_opening_boundary_suspected_ = true;
    } else {
      pre_self_check_closing_boundary_suspected_ = true;
    }

    std::ostringstream message;
    message << "PreSelfCheck | phase=BidirectionalMoveEnable | degraded"
            << " confirmed_direction=" << directionName(confirmed_direction)
            << " suspected_boundary_direction=" << directionName(missing_direction)
            << " reason=low_energy_opposite_drift"
            << " confirmed_current_a=" << confirmed.current_limit.value
            << " scan_current_a=" << current_limit
            << " missing_measured_mm="
            << last_motion_sample_.measured_distance.value
            << " missing_max_current_a="
            << last_motion_sample_.max_motor_current.value;
    reportProgress(message.str());
    return true;
  }

  [[nodiscard]] double selfCheckBreakawayStartCurrent(
      double current_step, double configured_current_start,
      double current_stop) const {
    double learned_start = 0.0;
    if (profile_.closing.bootstrap_breakaway_sample_count > 0 &&
        profile_.closing.bootstrap_breakaway_current.value > 0.0) {
      learned_start = std::max(
          learned_start, profile_.closing.bootstrap_breakaway_current.value);
    }
    if (profile_.opening.bootstrap_breakaway_sample_count > 0 &&
        profile_.opening.bootstrap_breakaway_current.value > 0.0) {
      learned_start = std::max(
          learned_start, profile_.opening.bootstrap_breakaway_current.value);
    }
    if (learned_start > 0.0) {
      learned_start = std::max(current_step, learned_start - current_step);
    }
    const double requested_start =
        std::max({current_step, configured_current_start, learned_start});
    return std::min(requested_start, current_stop);
  }

  [[nodiscard]] double normalBreakawayDistance() const {
    return std::max(config_.self_check.motion_start_distance.value,
                    profile_.noise_floor.nut_stroke_noise.value * 3.0);
  }

  [[nodiscard]] double lowConfidenceBreakawayDistance() const {
    const double configured =
        config_.self_check.low_confidence_motion_distance.value > 0.0
            ? config_.self_check.low_confidence_motion_distance.value
            : config_.self_check.motion_start_distance.value;
    const double below_normal =
        std::min(config_.self_check.motion_start_distance.value, configured);
    return std::max(below_normal,
                    profile_.noise_floor.nut_stroke_noise.value * 3.0);
  }

  void updateRuntimeBreakawayProfile(
      self_check::MotionDirection direction,
      const self_check::MotionIdentificationSample& sample) {
    auto* directional_profile =
        direction == self_check::MotionDirection::Opening
            ? &profile_.opening
            : &profile_.closing;
    if (directional_profile->bootstrap_breakaway_sample_count == 0U) {
      directional_profile->bootstrap_breakaway_current = sample.max_motor_current;
      directional_profile->bootstrap_breakaway_torque = sample.max_motor_torque;
    } else {
      directional_profile->bootstrap_breakaway_current.value =
          std::max(directional_profile->bootstrap_breakaway_current.value,
                   sample.max_motor_current.value);
      directional_profile->bootstrap_breakaway_torque.value =
          std::max(directional_profile->bootstrap_breakaway_torque.value,
                   sample.max_motor_torque.value);
    }
    ++directional_profile->bootstrap_breakaway_sample_count;
    directional_profile->quality = self_check::IdentificationQuality::LowConfidence;
  }

  void mergeRuntimeBreakawaySeeds(self_check::StructureProfile* profile) const {
    if (profile == nullptr) {
      return;
    }
    mergeDirectionalBreakawaySeed(profile_.opening, &profile->opening);
    mergeDirectionalBreakawaySeed(profile_.closing, &profile->closing);
  }

  void mergeLoadedSelfCheckSeed(
      const self_check::StructureProfile& seed,
      self_check::StructureProfile* target) const {
    if (target == nullptr) {
      return;
    }
    mergeDirectionalProfileSeed(seed.opening, &target->opening);
    mergeDirectionalProfileSeed(seed.closing, &target->closing);

    if (target->motion_health.valid_sample_count == 0U &&
        seed.motion_health.valid_sample_count > 0U) {
      target->motion_health = seed.motion_health;
      target->motion_health.quality =
          self_check::IdentificationQuality::LowConfidence;
    }
    target->total_sample_count =
        std::max(target->total_sample_count, seed.total_sample_count);
    target->valid_sample_count =
        std::max(target->valid_sample_count, seed.valid_sample_count);
    target->rejected_sample_count =
        std::max(target->rejected_sample_count, seed.rejected_sample_count);
  }

  void mergeDirectionalBreakawaySeed(
      const self_check::DirectionalStructureProfile& source,
      self_check::DirectionalStructureProfile* target) const {
    if (target == nullptr || source.bootstrap_breakaway_sample_count == 0) {
      return;
    }
    target->bootstrap_breakaway_current.value =
        std::max(target->bootstrap_breakaway_current.value,
                 source.bootstrap_breakaway_current.value);
    target->bootstrap_breakaway_torque.value =
        std::max(target->bootstrap_breakaway_torque.value,
                 source.bootstrap_breakaway_torque.value);
    target->bootstrap_breakaway_sample_count =
        std::max(target->bootstrap_breakaway_sample_count,
                 source.bootstrap_breakaway_sample_count);
    if (target->quality == self_check::IdentificationQuality::Invalid ||
        target->quality == self_check::IdentificationQuality::ConservativeDefault) {
      target->quality = self_check::IdentificationQuality::LowConfidence;
    }
  }

  void mergeDirectionalProfileSeed(
      const self_check::DirectionalStructureProfile& source,
      self_check::DirectionalStructureProfile* target) const {
    if (target == nullptr) {
      return;
    }
    mergeDirectionalBreakawaySeed(source, target);
    if (source.static_friction_sample_count > 0U) {
      if (target->static_friction_sample_count == 0U ||
          source.static_friction_current.value >
              target->static_friction_current.value) {
        target->static_friction_current = source.static_friction_current;
        target->static_friction_torque = source.static_friction_torque;
      }
      target->static_friction_sample_count =
          std::max(target->static_friction_sample_count,
                   source.static_friction_sample_count);
      if (target->quality == self_check::IdentificationQuality::Invalid ||
          target->quality ==
              self_check::IdentificationQuality::ConservativeDefault) {
        target->quality = self_check::IdentificationQuality::LowConfidence;
      }
    }
    if (source.dynamic_friction_sample_count > 0U) {
      if (target->dynamic_friction_sample_count == 0U ||
          source.dynamic_friction_current_average.value >
              target->dynamic_friction_current_average.value) {
        target->dynamic_friction_current_average =
            source.dynamic_friction_current_average;
      }
      target->dynamic_friction_current_max.value = std::max(
          target->dynamic_friction_current_max.value,
          source.dynamic_friction_current_max.value);
      if (target->dynamic_friction_sample_count == 0U ||
          source.dynamic_friction_torque_average.value >
              target->dynamic_friction_torque_average.value) {
        target->dynamic_friction_torque_average =
            source.dynamic_friction_torque_average;
      }
      target->dynamic_friction_torque_max.value = std::max(
          target->dynamic_friction_torque_max.value,
          source.dynamic_friction_torque_max.value);
      target->dynamic_friction_sample_count =
          std::max(target->dynamic_friction_sample_count,
                   source.dynamic_friction_sample_count);
      if (target->quality == self_check::IdentificationQuality::Invalid ||
          target->quality ==
              self_check::IdentificationQuality::ConservativeDefault) {
        target->quality = self_check::IdentificationQuality::LowConfidence;
      }
    }
    if (source.stable_speed_sample_count > 0U) {
      target->stable_speed_sample_count =
          std::max(target->stable_speed_sample_count,
                   source.stable_speed_sample_count);
      if (source.low_speed_unstable_upper_bound.value >
          target->low_speed_unstable_upper_bound.value) {
        target->low_speed_unstable_upper_bound =
            source.low_speed_unstable_upper_bound;
      }
    }
  }

  void loadSelfCheckSeedIfAvailable() {
    if (config_.self_check.learned_profile_path.empty()) {
      return;
    }
    std::ifstream stream{config_.self_check.learned_profile_path};
    if (!stream) {
      return;
    }

    std::string line;
    bool loaded_any_field = false;
    int seed_version = 1;
    while (std::getline(stream, line)) {
      const auto separator = line.find('=');
      if (separator == std::string::npos) {
        continue;
      }
      const std::string key = line.substr(0, separator);
      const double value = parseSeedDouble(line.substr(separator + 1));
      if (key == "seed_version") {
        seed_version = static_cast<int>(std::floor(value + 0.5));
      }
      if (applySelfCheckSeedField(key, value)) {
        loaded_any_field = true;
      }
    }
    if (loaded_any_field) {
      finalizeLoadedSelfCheckSeed(seed_version);
    }
    reportProgress(std::string{"PreSelfCheck | loaded seed path="} +
                   config_.self_check.learned_profile_path);
  }

  void saveSelfCheckSeed() const {
    if (config_.self_check.learned_profile_path.empty()) {
      return;
    }
    const std::filesystem::path path{config_.self_check.learned_profile_path};
    if (path.has_parent_path()) {
      std::error_code error;
      std::filesystem::create_directories(path.parent_path(), error);
    }
    std::ofstream stream{path};
    if (!stream) {
      return;
    }
    stream << "seed_version=3\n";
    stream << "opening_breakaway_bootstrap_current_a="
           << profile_.opening.bootstrap_breakaway_current.value << '\n';
    stream << "opening_breakaway_bootstrap_torque_nm="
           << profile_.opening.bootstrap_breakaway_torque.value << '\n';
    stream << "opening_breakaway_bootstrap_sample_count="
           << profile_.opening.bootstrap_breakaway_sample_count << '\n';
    stream << "opening_static_friction_current_a="
           << profile_.opening.static_friction_current.value << '\n';
    stream << "opening_static_friction_torque_nm="
           << profile_.opening.static_friction_torque.value << '\n';
    stream << "opening_minimum_stable_nut_speed_mm_s="
           << profile_.opening.minimum_stable_nut_speed.value << '\n';
    stream << "opening_low_speed_unstable_upper_bound_mm_s="
           << profile_.opening.low_speed_unstable_upper_bound.value << '\n';
    stream << "opening_dynamic_friction_current_average_a="
           << profile_.opening.dynamic_friction_current_average.value << '\n';
    stream << "opening_dynamic_friction_current_max_a="
           << profile_.opening.dynamic_friction_current_max.value << '\n';
    stream << "opening_dynamic_friction_torque_average_nm="
           << profile_.opening.dynamic_friction_torque_average.value << '\n';
    stream << "opening_dynamic_friction_torque_max_nm="
           << profile_.opening.dynamic_friction_torque_max.value << '\n';
    stream << "opening_stable_speed_sample_count="
           << profile_.opening.stable_speed_sample_count << '\n';
    stream << "opening_static_friction_sample_count="
           << profile_.opening.static_friction_sample_count << '\n';
    stream << "opening_dynamic_friction_sample_count="
           << profile_.opening.dynamic_friction_sample_count << '\n';
    stream << "closing_breakaway_bootstrap_current_a="
           << profile_.closing.bootstrap_breakaway_current.value << '\n';
    stream << "closing_breakaway_bootstrap_torque_nm="
           << profile_.closing.bootstrap_breakaway_torque.value << '\n';
    stream << "closing_breakaway_bootstrap_sample_count="
           << profile_.closing.bootstrap_breakaway_sample_count << '\n';
    stream << "closing_static_friction_current_a="
           << profile_.closing.static_friction_current.value << '\n';
    stream << "closing_static_friction_torque_nm="
           << profile_.closing.static_friction_torque.value << '\n';
    stream << "closing_minimum_stable_nut_speed_mm_s="
           << profile_.closing.minimum_stable_nut_speed.value << '\n';
    stream << "closing_low_speed_unstable_upper_bound_mm_s="
           << profile_.closing.low_speed_unstable_upper_bound.value << '\n';
    stream << "closing_dynamic_friction_current_average_a="
           << profile_.closing.dynamic_friction_current_average.value << '\n';
    stream << "closing_dynamic_friction_current_max_a="
           << profile_.closing.dynamic_friction_current_max.value << '\n';
    stream << "closing_dynamic_friction_torque_average_nm="
           << profile_.closing.dynamic_friction_torque_average.value << '\n';
    stream << "closing_dynamic_friction_torque_max_nm="
           << profile_.closing.dynamic_friction_torque_max.value << '\n';
    stream << "closing_stable_speed_sample_count="
           << profile_.closing.stable_speed_sample_count << '\n';
    stream << "closing_static_friction_sample_count="
           << profile_.closing.static_friction_sample_count << '\n';
    stream << "closing_dynamic_friction_sample_count="
           << profile_.closing.dynamic_friction_sample_count << '\n';
    stream << "safe_zone_open_limit_mm="
           << profile_.travel_limits.safe_zone_open_limit.value << '\n';
    stream << "safe_zone_closed_limit_mm="
           << profile_.travel_limits.safe_zone_closed_limit.value << '\n';
    stream << "software_open_limit_mm="
           << profile_.travel_limits.software_open_limit.value << '\n';
    stream << "software_closed_limit_mm="
           << profile_.travel_limits.software_closed_limit.value << '\n';
    stream << "learned_travel_mm="
           << profile_.travel_limits.learned_travel.value << '\n';
    stream << "travel_valid_limit_sample_count="
           << profile_.travel_limits.valid_limit_sample_count << '\n';
    stream << "motion_health_valid_sample_count="
           << profile_.motion_health.valid_sample_count << '\n';
    stream << "motion_health_rejected_sample_count="
           << profile_.motion_health.rejected_sample_count << '\n';
    stream << "motion_health_max_velocity_tracking_error_mm_s="
           << profile_.motion_health.max_velocity_tracking_error.value << '\n';
    stream << "motion_health_max_current_ripple_a="
           << profile_.motion_health.max_current_ripple.value << '\n';
    stream << "motion_health_max_torque_ripple_nm="
           << profile_.motion_health.max_torque_ripple.value << '\n';
    stream << "motion_health_max_temperature_deg_c="
           << profile_.motion_health.max_temperature.value << '\n';
    stream << "valid_sample_count=" << profile_.valid_sample_count << '\n';
    stream << "total_sample_count=" << profile_.total_sample_count << '\n';
    stream << "rejected_sample_count=" << profile_.rejected_sample_count << '\n';
  }

  [[nodiscard]] double parseSeedDouble(const std::string& text) const {
    try {
      return std::stod(text);
    } catch (...) {
      return 0.0;
    }
  }

  [[nodiscard]] bool applySelfCheckSeedField(const std::string& key,
                                             double value) {
    if (!std::isfinite(value)) {
      return false;
    }
    if (key == "opening_breakaway_bootstrap_current_a") {
      profile_.opening.bootstrap_breakaway_current = common::A{value};
    } else if (key == "opening_breakaway_bootstrap_torque_nm") {
      profile_.opening.bootstrap_breakaway_torque = common::Nm{value};
    } else if (key == "opening_breakaway_bootstrap_sample_count") {
      profile_.opening.bootstrap_breakaway_sample_count = parseSeedCount(value);
    } else if (key == "opening_static_friction_current_a") {
      profile_.opening.static_friction_current = common::A{value};
    } else if (key == "opening_static_friction_torque_nm") {
      profile_.opening.static_friction_torque = common::Nm{value};
    } else if (key == "opening_minimum_stable_nut_speed_mm_s") {
      profile_.opening.minimum_stable_nut_speed = common::MmPerS{value};
    } else if (key == "opening_low_speed_unstable_upper_bound_mm_s") {
      profile_.opening.low_speed_unstable_upper_bound = common::MmPerS{value};
    } else if (key == "opening_dynamic_friction_current_average_a") {
      profile_.opening.dynamic_friction_current_average = common::A{value};
    } else if (key == "opening_dynamic_friction_current_max_a") {
      profile_.opening.dynamic_friction_current_max = common::A{value};
    } else if (key == "opening_dynamic_friction_torque_average_nm") {
      profile_.opening.dynamic_friction_torque_average = common::Nm{value};
    } else if (key == "opening_dynamic_friction_torque_max_nm") {
      profile_.opening.dynamic_friction_torque_max = common::Nm{value};
    } else if (key == "opening_stable_speed_sample_count") {
      profile_.opening.stable_speed_sample_count = parseSeedCount(value);
    } else if (key == "opening_static_friction_sample_count") {
      profile_.opening.static_friction_sample_count = parseSeedCount(value);
    } else if (key == "opening_dynamic_friction_sample_count") {
      profile_.opening.dynamic_friction_sample_count = parseSeedCount(value);
    } else if (key == "closing_breakaway_bootstrap_current_a") {
      profile_.closing.bootstrap_breakaway_current = common::A{value};
    } else if (key == "closing_breakaway_bootstrap_torque_nm") {
      profile_.closing.bootstrap_breakaway_torque = common::Nm{value};
    } else if (key == "closing_breakaway_bootstrap_sample_count") {
      profile_.closing.bootstrap_breakaway_sample_count = parseSeedCount(value);
    } else if (key == "closing_static_friction_current_a") {
      profile_.closing.static_friction_current = common::A{value};
    } else if (key == "closing_static_friction_torque_nm") {
      profile_.closing.static_friction_torque = common::Nm{value};
    } else if (key == "closing_minimum_stable_nut_speed_mm_s") {
      profile_.closing.minimum_stable_nut_speed = common::MmPerS{value};
    } else if (key == "closing_low_speed_unstable_upper_bound_mm_s") {
      profile_.closing.low_speed_unstable_upper_bound = common::MmPerS{value};
    } else if (key == "closing_dynamic_friction_current_average_a") {
      profile_.closing.dynamic_friction_current_average = common::A{value};
    } else if (key == "closing_dynamic_friction_current_max_a") {
      profile_.closing.dynamic_friction_current_max = common::A{value};
    } else if (key == "closing_dynamic_friction_torque_average_nm") {
      profile_.closing.dynamic_friction_torque_average = common::Nm{value};
    } else if (key == "closing_dynamic_friction_torque_max_nm") {
      profile_.closing.dynamic_friction_torque_max = common::Nm{value};
    } else if (key == "closing_stable_speed_sample_count") {
      profile_.closing.stable_speed_sample_count = parseSeedCount(value);
    } else if (key == "closing_static_friction_sample_count") {
      profile_.closing.static_friction_sample_count = parseSeedCount(value);
    } else if (key == "closing_dynamic_friction_sample_count") {
      profile_.closing.dynamic_friction_sample_count = parseSeedCount(value);
    } else if (key == "safe_zone_open_limit_mm") {
      profile_.travel_limits.safe_zone_open_limit = common::Mm{value};
    } else if (key == "safe_zone_closed_limit_mm") {
      profile_.travel_limits.safe_zone_closed_limit = common::Mm{value};
    } else if (key == "software_open_limit_mm") {
      profile_.travel_limits.software_open_limit = common::Mm{value};
    } else if (key == "software_closed_limit_mm") {
      profile_.travel_limits.software_closed_limit = common::Mm{value};
    } else if (key == "learned_travel_mm") {
      profile_.travel_limits.learned_travel = common::Mm{value};
    } else if (key == "travel_valid_limit_sample_count") {
      profile_.travel_limits.valid_limit_sample_count = parseSeedCount(value);
    } else if (key == "motion_health_valid_sample_count") {
      profile_.motion_health.valid_sample_count = parseSeedCount(value);
    } else if (key == "motion_health_rejected_sample_count") {
      profile_.motion_health.rejected_sample_count = parseSeedCount(value);
    } else if (key == "motion_health_max_velocity_tracking_error_mm_s") {
      profile_.motion_health.max_velocity_tracking_error = common::MmPerS{value};
    } else if (key == "motion_health_max_current_ripple_a") {
      profile_.motion_health.max_current_ripple = common::A{value};
    } else if (key == "motion_health_max_torque_ripple_nm") {
      profile_.motion_health.max_torque_ripple = common::Nm{value};
    } else if (key == "motion_health_max_temperature_deg_c") {
      profile_.motion_health.max_temperature = common::DegC{value};
    } else if (key == "valid_sample_count") {
      profile_.valid_sample_count = parseSeedCount(value);
    } else if (key == "total_sample_count") {
      profile_.total_sample_count = parseSeedCount(value);
    } else if (key == "rejected_sample_count") {
      profile_.rejected_sample_count = parseSeedCount(value);
    } else if (key == "seed_version") {
      return false;
    } else {
      return false;
    }
    markSeedLoadedQualities();
    return true;
  }

  [[nodiscard]] std::uint32_t parseSeedCount(double value) const {
    if (value <= 0.0) {
      return 0U;
    }
    const double bounded = std::min(
        std::floor(value + 0.5),
        static_cast<double>(std::numeric_limits<std::uint32_t>::max()));
    return static_cast<std::uint32_t>(bounded);
  }

  void markSeedLoadedQualities() {
    if (profile_.opening.bootstrap_breakaway_current.value > 0.0 ||
        profile_.opening.static_friction_current.value > 0.0 ||
        profile_.opening.dynamic_friction_current_max.value > 0.0 ||
        profile_.opening.minimum_stable_nut_speed.value > 0.0) {
      profile_.opening.quality = self_check::IdentificationQuality::LowConfidence;
    }
    if (profile_.closing.bootstrap_breakaway_current.value > 0.0 ||
        profile_.closing.static_friction_current.value > 0.0 ||
        profile_.closing.dynamic_friction_current_max.value > 0.0 ||
        profile_.closing.minimum_stable_nut_speed.value > 0.0) {
      profile_.closing.quality = self_check::IdentificationQuality::LowConfidence;
    }
    if (profile_.travel_limits.safe_zone_closed_limit.value >
            profile_.travel_limits.safe_zone_open_limit.value ||
        profile_.travel_limits.software_closed_limit.value >
            profile_.travel_limits.software_open_limit.value ||
        profile_.travel_limits.learned_travel.value > 0.0) {
      profile_.travel_limits.quality =
          self_check::IdentificationQuality::LowConfidence;
    }
    if (profile_.motion_health.valid_sample_count > 0 ||
        profile_.motion_health.max_temperature.value > 0.0) {
      profile_.motion_health.quality =
          self_check::IdentificationQuality::LowConfidence;
      if (profile_.motion_health.status ==
          self_check::MotionHealthStatus::Unknown) {
        profile_.motion_health.status = self_check::MotionHealthStatus::Healthy;
      }
    }
  }

  void finalizeLoadedSelfCheckSeed(int seed_version) {
    if (seed_version < 3) {
      migrateLegacyDirectionalLearningSeedToBootstrap(&profile_.opening);
      migrateLegacyDirectionalLearningSeedToBootstrap(&profile_.closing);
    }
    if (profile_.opening.bootstrap_breakaway_current.value > 0.0 &&
        profile_.opening.bootstrap_breakaway_sample_count == 0U) {
      profile_.opening.bootstrap_breakaway_sample_count = 1U;
    }
    if (profile_.closing.bootstrap_breakaway_current.value > 0.0 &&
        profile_.closing.bootstrap_breakaway_sample_count == 0U) {
      profile_.closing.bootstrap_breakaway_sample_count = 1U;
    }
    markSeedLoadedQualities();
    if (profile_.quality == self_check::IdentificationQuality::Invalid &&
        (profile_.opening.quality != self_check::IdentificationQuality::Invalid ||
         profile_.closing.quality != self_check::IdentificationQuality::Invalid ||
         profile_.travel_limits.quality !=
             self_check::IdentificationQuality::Invalid ||
         profile_.motion_health.quality !=
             self_check::IdentificationQuality::Invalid)) {
      profile_.quality = self_check::IdentificationQuality::LowConfidence;
    }
  }

  void migrateLegacyDirectionalLearningSeedToBootstrap(
      self_check::DirectionalStructureProfile* profile) const {
    if (profile == nullptr) {
      return;
    }
    if (profile->bootstrap_breakaway_current.value <= 0.0 &&
        profile->static_friction_current.value > 0.0) {
      profile->bootstrap_breakaway_current = profile->static_friction_current;
      profile->bootstrap_breakaway_torque = profile->static_friction_torque;
      profile->bootstrap_breakaway_sample_count =
          std::max<std::uint32_t>(1U, profile->static_friction_sample_count);
    }
    profile->static_friction_current = {};
    profile->static_friction_torque = {};
    profile->static_friction_sample_count = 0U;
    profile->minimum_stable_nut_speed = {};
    profile->dynamic_friction_current_average = {};
    profile->dynamic_friction_current_max = {};
    profile->dynamic_friction_torque_average = {};
    profile->dynamic_friction_torque_max = {};
    profile->stable_speed_sample_count = 0U;
    profile->dynamic_friction_sample_count = 0U;
  }

  [[nodiscard]] common::A selfCheckStableCurrentLimit() const {
    const double learned_current = std::max(
        profile_.opening.bootstrap_breakaway_sample_count > 0
            ? profile_.opening.bootstrap_breakaway_current.value
            : 0.0,
        profile_.closing.bootstrap_breakaway_sample_count > 0
            ? profile_.closing.bootstrap_breakaway_current.value
            : 0.0);
    const double requested =
        learned_current > 0.0
            ? learned_current + config_.self_check.max_current_ripple.value
            : config_.safety.self_check_current_limit.value;
    return common::A{std::min(requested,
                              config_.safety.self_check_current_limit.value)};
  }

  [[nodiscard]] bool isStableShortStrokeSample(
      const self_check::MotionIdentificationSample& sample,
      common::Mm target_distance) const {
    const double distance_error =
        std::abs(sample.measured_distance.value - target_distance.value);
    const double allowed_distance_error =
        std::min(config_.self_check.max_distance_error.value,
                 target_distance.value * 0.5);
    return sample.commanded_direction != self_check::MotionDirection::Unknown &&
           sample.settled &&
           sample.direction_matches && sample.position_monotonic &&
           sample.velocity_stable && sample.current_stable &&
           !sample.limit_triggered && !sample.jam_detected &&
           !sample.active_stop_triggered &&
           sample.measured_distance.value >=
               config_.self_check.motion_start_distance.value &&
           distance_error <= allowed_distance_error;
  }

  [[nodiscard]] common::Result summarizeStableShortStrokeFailure(
      const SelfCheckMotionProbeResult& closing,
      const SelfCheckMotionProbeResult& opening,
      common::Mm target_distance) const {
    std::ostringstream message;
    message << "stable short-stroke failed"
            << " target_mm=" << target_distance.value
            << " closing_code=" << common::toString(closing.result.code())
            << " closing_settled=" << (closing.settled ? "true" : "false")
            << " closing_measured_mm=" << closing.measured_distance.value
            << " closing_avg_speed_mm_s=" << closing.average_nut_speed.value
            << " closing_max_current_a=" << closing.max_motor_current.value
            << " closing_current_stable="
            << (closing.sample.current_stable ? "true" : "false")
            << " opening_code=" << common::toString(opening.result.code())
            << " opening_settled=" << (opening.settled ? "true" : "false")
            << " opening_measured_mm=" << opening.measured_distance.value
            << " opening_avg_speed_mm_s=" << opening.average_nut_speed.value
            << " opening_max_current_a=" << opening.max_motor_current.value
            << " opening_current_stable="
            << (opening.sample.current_stable ? "true" : "false");
    return common::Result::error(common::ErrorCode::OperationTimedOut,
                                 message.str());
  }

  void rememberBestSelfCheckMotionSample(
      const self_check::MotionIdentificationSample& candidate,
      self_check::MotionIdentificationSample* best) const {
    if (best == nullptr) {
      return;
    }
    if (!isUsableLowConfidenceMotionSample(candidate)) {
      return;
    }
    if (candidate.measured_distance.value > best->measured_distance.value) {
      *best = candidate;
    }
  }

  [[nodiscard]] bool isUsableLowConfidenceMotionSample(
      const self_check::MotionIdentificationSample& sample) const {
    return sample.commanded_direction != self_check::MotionDirection::Unknown &&
           sample.settled &&
           sample.direction_matches && sample.position_monotonic &&
           !sample.limit_triggered && !sample.jam_detected &&
           !sample.active_stop_triggered &&
           sample.measured_distance.value >=
               config_.self_check.motion_start_distance.value;
  }

  [[nodiscard]] bool isEndpointStartRetreatProbe(
      const SelfCheckMotionProbeResult& probe) const {
    const auto& sample = probe.sample;
    if (sample.commanded_direction == self_check::MotionDirection::Unknown ||
        !sample.settled || !sample.direction_matches ||
        !sample.position_monotonic || sample.active_stop_triggered) {
      return false;
    }
    if (probe.result.code() == common::ErrorCode::SelfCheckInconsistentFeedback ||
        probe.result.code() == common::ErrorCode::SafetyActiveStop ||
        probe.result.code() == common::ErrorCode::HardwareFault ||
        probe.result.code() == common::ErrorCode::HardwareFeedbackInvalid ||
        probe.result.code() == common::ErrorCode::HardwareRejectedCommand ||
        probe.result.code() == common::ErrorCode::ConnectionLost ||
        probe.result.code() == common::ErrorCode::ConnectionNotOpen ||
        probe.result.code() == common::ErrorCode::FeedbackTimedOut) {
      return false;
    }

    const double retreat_threshold = std::max(
        {config_.self_check.motion_start_distance.value,
         config_.self_check.low_confidence_motion_distance.value,
         config_.self_check.fallback_nut_stroke_noise.value * 3.0,
         0.02});
    if (sample.measured_distance.value < retreat_threshold) {
      return false;
    }

    return probe.result.isOk() ||
           probe.result.code() == common::ErrorCode::OperationTimedOut ||
           probe.result.code() == common::ErrorCode::SafetyJamDetected ||
           probe.result.code() == common::ErrorCode::SafetyContactDetected;
  }

  [[nodiscard]] bool isEndpointStartBoundaryProbe(
      const SelfCheckMotionProbeResult& probe) const {
    if (probe.sample.commanded_direction == self_check::MotionDirection::Unknown ||
        !probe.sample.settled || probe.sample.active_stop_triggered) {
      return false;
    }
    if (probe.result.code() == common::ErrorCode::SelfCheckInconsistentFeedback ||
        probe.result.code() == common::ErrorCode::SafetyActiveStop ||
        probe.result.code() == common::ErrorCode::HardwareFault ||
        probe.result.code() == common::ErrorCode::HardwareFeedbackInvalid ||
        probe.result.code() == common::ErrorCode::HardwareRejectedCommand ||
        probe.result.code() == common::ErrorCode::ConnectionLost ||
        probe.result.code() == common::ErrorCode::ConnectionNotOpen ||
        probe.result.code() == common::ErrorCode::FeedbackTimedOut) {
      return false;
    }
    return probe.hard_current_no_progress_confirmed ||
           probe.sustained_no_progress_confirmed || probe.velocity_collapsed ||
           probe.jam_suspected || probe.limit_suspected ||
           probe.result.code() == common::ErrorCode::SafetyJamDetected ||
           probe.result.code() == common::ErrorCode::SafetyContactDetected;
  }

  [[nodiscard]] std::optional<common::Result> acceptEndpointStartEscape(
      const SelfCheckMotionProbeResult& retreat_probe,
      const SelfCheckMotionProbeResult& boundary_probe,
      self_check::SelfCheckInput* input) {
    if (input == nullptr ||
        !isEndpointStartRetreatProbe(retreat_probe) ||
        !isEndpointStartBoundaryProbe(boundary_probe)) {
      return std::nullopt;
    }

    const auto retreat_direction = retreat_probe.sample.commanded_direction;
    const auto boundary_direction = boundary_probe.sample.commanded_direction;
    if (retreat_direction == boundary_direction ||
        boundary_direction == self_check::MotionDirection::Unknown) {
      return std::nullopt;
    }

    if (boundary_direction == self_check::MotionDirection::Opening) {
      pre_self_check_opening_boundary_suspected_ = true;
    } else if (boundary_direction == self_check::MotionDirection::Closing) {
      pre_self_check_closing_boundary_suspected_ = true;
    }

    updateRuntimeBreakawayProfile(retreat_direction, retreat_probe.sample);

    std::ostringstream message;
    message << "PreSelfCheck | phase=StableShortStrokeMotion"
            << " | endpoint_start_escape"
            << " retreat_direction=" << directionName(retreat_direction)
            << " suspected_boundary_direction="
            << directionName(boundary_direction)
            << " retreat_mm=" << retreat_probe.measured_distance.value
            << " boundary_code="
            << common::toString(boundary_probe.result.code())
            << " boundary_mm=" << boundary_probe.measured_distance.value
            << " boundary_max_current_a="
            << boundary_probe.max_motor_current.value;
    reportProgress(message.str());

    return common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "stable short-stroke endpoint start escaped; continuing with PreB");
  }

  [[nodiscard]] bool isUsablePreBMotionSample(
      const self_check::MotionIdentificationSample& sample) const {
    const double distance_error =
        std::abs(sample.measured_distance.value - sample.commanded_distance.value);
    const double allowed_error =
        std::max(config_.self_check.max_distance_error.value,
                 sample.commanded_distance.value * 0.35);
    return sample.commanded_direction != self_check::MotionDirection::Unknown &&
           sample.settled && sample.direction_matches &&
           sample.position_monotonic && !sample.limit_triggered &&
           !sample.jam_detected && !sample.active_stop_triggered &&
           sample.measured_distance.value >=
               config_.self_check.motion_start_distance.value &&
           distance_error <= allowed_error;
  }

  [[nodiscard]] common::Mm preBEffectiveProgressDistance() const {
    return common::Mm{std::max(
        {config_.self_check.motion_start_distance.value,
         config_.self_check.low_confidence_motion_distance.value,
         config_.self_check.min_measured_distance.value * 0.25})};
  }

  [[nodiscard]] common::Mm preBBoundaryReleaseDistance() const {
    const double configured =
        std::abs(config_.self_check.pre_b_boundary_release_distance.value);
    const double fallback = std::max(
        config_.self_check.safe_zone_margin.value,
        config_.self_check.min_measured_distance.value);
    return common::Mm{configured > 0.0 ? configured : std::max(0.0, fallback)};
  }

  [[nodiscard]] common::MmPerS preBBoundaryReleaseSpeed() const {
    const double requested = std::max(
        std::abs(config_.self_check.min_speed_scan_start.value),
        std::abs(config_.self_check.pre_b_boundary_release_speed.value));
    const double fallback =
        requested > 0.0 ? requested : std::abs(preBExpansionSpeed().value);
    const double safety_cap = std::abs(config_.safety.max_nut_speed.value);
    return common::MmPerS{safety_cap > 0.0 ? std::min(fallback, safety_cap)
                                           : fallback};
  }

  [[nodiscard]] common::A preBBoundaryReleaseCurrentLimit() const {
    const double configured = std::abs(
        config_.self_check.pre_b_boundary_release_current_limit.value);
    const double fallback = std::abs(config_.safety.self_check_current_limit.value);
    const double requested = configured > 0.0 ? configured : fallback;
    const double global_cap = std::abs(config_.safety.max_motor_current.value);
    return common::A{
        global_cap > 0.0 ? std::min(requested, global_cap) : requested};
  }

  [[nodiscard]] double frictionAnomalyAvoidMargin() const {
    return std::max(0.0,
                    config_.self_check.friction_anomaly_avoid_margin.value);
  }

  [[nodiscard]] double frictionAnomalyLearningAvoidRatio() const {
    const double configured =
        config_.self_check.friction_anomaly_learning_avoid_ratio.value;
    if (configured > 0.0) {
      return configured;
    }
    return config_.self_check.friction_anomaly_moderate_ratio.value;
  }

  [[nodiscard]] bool intervalOverlaps(double start_a, double end_a,
                                      double start_b, double end_b,
                                      double margin) const {
    const double a_low = std::min(start_a, end_a) - margin;
    const double a_high = std::max(start_a, end_a) + margin;
    const double b_low = std::min(start_b, end_b);
    const double b_high = std::max(start_b, end_b);
    return a_low <= b_high && b_low <= a_high;
  }

  [[nodiscard]] bool segmentOverlapsFrictionAnomaly(
      common::Mm start_position, common::Mm end_position,
      const std::vector<self_check::FrictionAnomalyRecord>& records,
      std::uint32_t first_record_index = 0U) const {
    if (first_record_index >= records.size()) {
      return false;
    }
    const double margin = frictionAnomalyAvoidMargin();
    const double avoid_ratio = frictionAnomalyLearningAvoidRatio();
    for (std::size_t index = first_record_index; index < records.size();
         ++index) {
      const auto& record = records[index];
      if (record.current_excess_ratio.value < avoid_ratio) {
        continue;
      }
      if (intervalOverlaps(record.start_position.value, record.end_position.value,
                           start_position.value, end_position.value, margin)) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] bool plannedSegmentOverlapsFrictionAnomaly(
      common::Mm start_position, self_check::MotionDirection direction,
      common::Mm distance,
      const self_check::FrictionAnomalyDetector* detector) const {
    if (detector == nullptr ||
        direction == self_check::MotionDirection::Unknown ||
        distance.value <= 0.0) {
      return false;
    }
    const common::Mm end_position{
        start_position.value +
        (direction == self_check::MotionDirection::Opening
             ? -std::abs(distance.value)
             : std::abs(distance.value))};
    return segmentOverlapsFrictionAnomaly(start_position, end_position,
                                          detector->records());
  }

  [[nodiscard]] bool probeSegmentOverlapsFrictionAnomaly(
      const SelfCheckMotionProbeResult& probe,
      const self_check::FrictionAnomalyDetector* detector,
      std::uint32_t first_record_index = 0U) const {
    if (detector == nullptr) {
      return false;
    }
    return segmentOverlapsFrictionAnomaly(probe.start_stroke, probe.end_stroke,
                                          detector->records(),
                                          first_record_index);
  }

  [[nodiscard]] std::optional<common::Mm> cleanLearningAnchorNear(
      common::Mm preferred_anchor, const ManualPositioningRange& search_range,
      const ManualPositioningRange& safe_zone, common::Mm sample_distance,
      const self_check::FrictionAnomalyDetector* detector) const {
    const double usable_margin =
        std::max({config_.self_check.max_distance_error.value,
                  config_.self_check.min_measured_distance.value * 0.5,
                  sample_distance.value});
    const double min_anchor =
        std::max(search_range.open_limit.value,
                 safe_zone.open_limit.value + usable_margin);
    const double max_anchor =
        std::min(search_range.closed_limit.value,
                 safe_zone.closed_limit.value - usable_margin);
    if (max_anchor < min_anchor) {
      return std::nullopt;
    }

    const std::vector<self_check::FrictionAnomalyRecord> empty_records{};
    const auto& records =
        detector != nullptr ? detector->records() : empty_records;
    std::vector<common::Mm> candidates{
        common::Mm{std::clamp(preferred_anchor.value, min_anchor, max_anchor)}};
    const double span = max_anchor - min_anchor;
    const std::uint32_t grid_count = 8U;
    for (std::uint32_t index = 0; index <= grid_count; ++index) {
      const double ratio = static_cast<double>(index) /
                           static_cast<double>(grid_count);
      candidates.push_back(common::Mm{min_anchor + span * ratio});
    }

    std::sort(candidates.begin(), candidates.end(),
              [preferred_anchor](common::Mm lhs, common::Mm rhs) {
                return std::abs(lhs.value - preferred_anchor.value) <
                       std::abs(rhs.value - preferred_anchor.value);
              });

    for (const auto& candidate : candidates) {
      const bool center_dirty = segmentOverlapsFrictionAnomaly(
          candidate, candidate, records);
      if (center_dirty) {
        continue;
      }
      const bool closing_dirty = plannedSegmentOverlapsFrictionAnomaly(
          candidate, self_check::MotionDirection::Closing, sample_distance,
          detector);
      const bool opening_dirty = plannedSegmentOverlapsFrictionAnomaly(
          candidate, self_check::MotionDirection::Opening, sample_distance,
          detector);
      if (!closing_dirty && !opening_dirty) {
        return candidate;
      }
    }
    return std::nullopt;
  }

  [[nodiscard]] ManualPositioningRange preBLearningRegionRange(
      const ManualPositioningRange& safe_zone, std::uint32_t region_bucket,
      std::uint32_t region_count) const {
    ManualPositioningRange range = safe_zone;
    const double bucket_count =
        static_cast<double>(std::max<std::uint32_t>(1U, region_count));
    const double bucket =
        static_cast<double>(std::min(region_bucket,
                                     std::max<std::uint32_t>(1U, region_count) -
                                         1U));
    const double width =
        safe_zone.closed_limit.value - safe_zone.open_limit.value;
    range.open_limit =
        common::Mm{safe_zone.open_limit.value + width * bucket / bucket_count};
    range.closed_limit = common::Mm{
        safe_zone.open_limit.value + width * (bucket + 1.0) / bucket_count};
    range.valid = range.closed_limit.value > range.open_limit.value;
    return range;
  }

  [[nodiscard]] std::vector<PreBLearningAnchorCandidate>
  buildPreBLearningAnchorCandidates(const ManualPositioningRange& safe_zone,
                                    common::Mm sample_distance,
                                    std::uint32_t minimum_regions) const {
    std::vector<PreBLearningAnchorCandidate> anchors;
    const double usable_margin =
        std::max({config_.self_check.max_distance_error.value,
                  config_.self_check.min_measured_distance.value * 0.5,
                  sample_distance.value});
    const double global_min_anchor = safe_zone.open_limit.value + usable_margin;
    const double global_max_anchor = safe_zone.closed_limit.value - usable_margin;
    if (global_max_anchor < global_min_anchor) {
      return anchors;
    }

    const std::uint32_t configured_count =
        config_.self_check.pre_b_learning_anchor_count;
    const std::uint32_t required_regions =
        std::max<std::uint32_t>(1U, minimum_regions);
    const std::uint32_t anchor_count =
        std::max<std::uint32_t>({configured_count, required_regions, 3U});
    anchors.reserve(anchor_count);

    std::vector<std::uint32_t> per_region(required_regions, 1U);
    for (std::uint32_t extra = required_regions; extra < anchor_count; ++extra) {
      ++per_region[extra % required_regions];
    }
    for (std::uint32_t region = 0; region < required_regions; ++region) {
      const auto region_range =
          preBLearningRegionRange(safe_zone, region, required_regions);
      const double min_anchor =
          std::max(region_range.open_limit.value, global_min_anchor);
      const double max_anchor =
          std::min(region_range.closed_limit.value, global_max_anchor);
      if (max_anchor < min_anchor) {
        continue;
      }
      const std::uint32_t region_anchor_count =
          std::max<std::uint32_t>(1U, per_region[region]);
      for (std::uint32_t index = 0; index < region_anchor_count; ++index) {
        const double ratio =
            region_anchor_count == 1U
                ? 0.5
                : static_cast<double>(index) /
                      static_cast<double>(region_anchor_count - 1U);
        anchors.push_back(PreBLearningAnchorCandidate{
            common::Mm{min_anchor + (max_anchor - min_anchor) * ratio},
            region});
      }
    }
    return anchors;
  }

  [[nodiscard]] std::size_t learningRegionBucket(
      common::Mm anchor, const ManualPositioningRange& safe_zone,
      std::uint32_t minimum_regions) const {
    const std::uint32_t bucket_count =
        std::max<std::uint32_t>(1U, minimum_regions);
    const double width =
        std::max(1e-9, safe_zone.closed_limit.value - safe_zone.open_limit.value);
    const double normalized =
        std::clamp((anchor.value - safe_zone.open_limit.value) / width, 0.0,
                   0.999999);
    return static_cast<std::size_t>(
        std::floor(normalized * static_cast<double>(bucket_count)));
  }

  [[nodiscard]] std::optional<common::Rad> activeMotorPositionWindowLimit()
      const {
    if (last_feedback_.runtime_limits_valid &&
        last_feedback_.runtime_position_limit.value > 0.0) {
      return common::Rad{std::abs(last_feedback_.runtime_position_limit.value)};
    }
    if (config_.motor.max_position.value > 0.0) {
      return common::Rad{std::abs(config_.motor.max_position.value)};
    }
    return std::nullopt;
  }

  [[nodiscard]] double positionCommandSign() const noexcept {
    return config_.motor.position_command_sign < 0 ? -1.0 : 1.0;
  }

  [[nodiscard]] bool preBTargetExceedsMotorPositionWindow(
      common::Rad target_motor_position, std::string* detail) const {
    const auto window_limit = activeMotorPositionWindowLimit();
    if (!window_limit.has_value()) {
      if (detail != nullptr) {
        *detail = " pmax_valid=false";
      }
      return false;
    }

    const double command_sign = positionCommandSign();
    const double limit = window_limit->value;
    const double current_command_position =
        last_feedback_.position.value * command_sign;
    const double target_command_position =
        target_motor_position.value * command_sign;
    const double remaining_toward_target =
        target_command_position >= current_command_position
            ? limit - current_command_position
            : current_command_position + limit;
    const double target_margin =
        std::min(target_command_position + limit,
                 limit - target_command_position);
    if (detail != nullptr) {
      std::ostringstream message;
      message << " pmax_valid=true"
              << " pmax_rad=" << limit
              << " pmax_source="
              << (last_feedback_.runtime_limits_valid ? "runtime" : "config")
              << " current_command_motor_pos_rad="
              << current_command_position
              << " target_command_motor_pos_rad="
              << target_command_position
              << " pmax_remaining_toward_target_rad="
              << remaining_toward_target
              << " target_pmax_margin_rad=" << target_margin;
      *detail = message.str();
    }
    return target_command_position < -limit - 1e-6 ||
           target_command_position > limit + 1e-6;
  }

  void reportPreBSearchStart(self_check::MotionDirection direction,
                             common::Mm expansion_start_stroke,
                             common::Mm relative_scan_target,
                             common::MmPerS search_speed,
                             common::Mm refine_step_distance,
                             double max_expansion) {
    const common::Rad theoretical_open_motor =
        strokeToReferencedMotorPosition(config_.mechanism.theoretical_open_limit);
    const common::Rad theoretical_close_motor =
        strokeToReferencedMotorPosition(config_.mechanism.theoretical_close_limit);
    std::string motor_window_detail;
    (void)preBTargetExceedsMotorPositionWindow(
        strokeToReferencedMotorPosition(relative_scan_target),
        &motor_window_detail);
    std::ostringstream message;
    message << "PreSelfCheck | pre_b_search_start"
            << " direction=" << directionName(direction)
            << " expansion_start_mm=" << expansion_start_stroke.value
            << " current_stroke_mm=" << current_nut_stroke_.value
            << " relative_scan_target_mm=" << relative_scan_target.value
            << " provisional_reference_mid_mm="
            << provisionalReferenceStroke().value
            << " theoretical_open_limit_mm="
            << config_.mechanism.theoretical_open_limit.value
            << " theoretical_close_limit_mm="
            << config_.mechanism.theoretical_close_limit.value
            << " search_speed_mm_s=" << search_speed.value
            << " mode=continuous_scan"
            << " boundary_refine_step_mm=" << refine_step_distance.value
            << " configured_boundary_step_mm="
            << config_.self_check.pre_b_boundary_step.value
            << " max_expansion_mm=" << max_expansion
            << " theoretical_open_motor_pos_rad="
            << theoretical_open_motor.value
            << " theoretical_close_motor_pos_rad="
            << theoretical_close_motor.value
            << " theoretical_motor_span_rev="
            << std::abs(theoretical_close_motor.value -
                        theoretical_open_motor.value) /
                   kTwoPi
            << " note=relative_pre_homing_coordinate_not_physical_midpoint"
            << motor_window_detail;
    reportProgress(message.str());
  }

  void reportPreBScanPlan(self_check::MotionDirection direction,
                          common::Mm target_stroke,
                          common::Mm scan_distance,
                          double remaining,
                          double expanded,
                          double expansion_remaining,
                          const std::string& motor_window_detail) {
    const common::Rad target_motor_position =
        strokeToReferencedMotorPosition(target_stroke);
    std::ostringstream message;
    message << "PreSelfCheck | pre_b_scan_plan"
            << " direction=" << directionName(direction)
            << " current_stroke_mm=" << current_nut_stroke_.value
            << " target_stroke_mm=" << target_stroke.value
            << " scan_distance_mm=" << scan_distance.value
            << " target_motor_pos_rad=" << target_motor_position.value
            << " target_motor_delta_rad="
            << (target_motor_position.value - last_feedback_.position.value)
            << " target_motor_delta_rev="
            << (target_motor_position.value - last_feedback_.position.value) /
                   kTwoPi
            << " remaining_mm=" << remaining
            << " expanded_mm=" << std::max(0.0, expanded)
            << " expansion_remaining_mm=" << expansion_remaining
            << motor_window_detail;
    reportProgress(message.str());
  }

  void reportPreBScanResult(self_check::MotionDirection direction,
                            const SelfCheckMotionProbeResult& probe,
                            common::Mm progress_result,
                            common::Mm observed_limit) {
    const double motor_delta =
        probe.end_motor_position.value - probe.start_motor_position.value;
    const double motor_delta_rev = motor_delta / kTwoPi;
    const double mm_per_rev =
        std::abs(motor_delta_rev) > 1e-9
            ? (probe.end_stroke.value - probe.start_stroke.value) /
                  motor_delta_rev
            : 0.0;
    std::ostringstream message;
    message << "PreSelfCheck | pre_b_scan_result"
            << " direction=" << directionName(direction)
            << " code=" << common::toString(probe.result.code())
            << " start_mm=" << probe.start_stroke.value
            << " end_mm=" << probe.end_stroke.value
            << " observed_limit_mm=" << observed_limit.value
            << " measured_mm=" << probe.measured_distance.value
            << " accepted_progress_mm=" << progress_result.value
            << " target_reached="
            << (probe.target_reached ? "true" : "false")
            << " settled=" << (probe.settled ? "true" : "false")
            << " direction_ok="
            << (probe.sample.direction_matches ? "true" : "false")
            << " monotonic="
            << (probe.sample.position_monotonic ? "true" : "false")
            << " limit_suspected="
            << (probe.limit_suspected ? "true" : "false")
            << " jam_suspected="
            << (probe.jam_suspected ? "true" : "false")
            << " max_current_a=" << probe.max_motor_current.value
            << " hard_current_limit_a="
            << probe.feedback_hard_current_limit.value
            << " emergency_current_limit_a="
            << probe.feedback_emergency_current_limit.value
            << " hard_current_confirm_time_s="
            << probe.hard_current_confirm_time.value
            << " hard_current_over_limit_s="
            << probe.hard_current_over_limit_duration.value
            << " hard_current_transient="
            << (probe.hard_current_transient_observed ? "true" : "false")
            << " hard_current_confirmed="
            << (probe.hard_current_confirmed ? "true" : "false")
            << " hard_current_no_progress="
            << (probe.hard_current_no_progress_confirmed ? "true" : "false")
            << " hard_current_progress_observed="
            << (probe.hard_current_progress_observed ? "true" : "false")
            << " hard_current_progress_mm="
            << probe.hard_current_progress_distance.value
            << " emergency_current="
            << (probe.emergency_current_confirmed ? "true" : "false")
            << " velocity_collapsed="
            << (probe.velocity_collapsed ? "true" : "false")
            << " sustained_no_progress="
            << (probe.sustained_no_progress_confirmed ? "true" : "false")
            << " physical_boundary_candidate="
            << (isPreBPhysicalBoundaryCandidate(probe) ? "true" : "false")
            << " motor_delta_rad=" << motor_delta
            << " motor_delta_rev=" << motor_delta_rev
            << " mm_per_rev_estimate=" << mm_per_rev;
    if (probe.result.hasMessage()) {
      message << " message=" << probe.result.message();
    }
    reportProgress(message.str());
  }

  [[nodiscard]] bool isPassThroughPreBMotionSample(
      const SelfCheckMotionProbeResult& probe) const {
    const auto& sample = probe.sample;
    if (sample.commanded_direction == self_check::MotionDirection::Unknown ||
        !sample.settled || !sample.direction_matches ||
        !sample.position_monotonic || sample.active_stop_triggered ||
        sample.measured_distance.value < preBEffectiveProgressDistance().value) {
      return false;
    }
    return !isPreBPhysicalBoundaryCandidate(probe);
  }

  [[nodiscard]] bool isEffectivePreBProgressSample(
      const SelfCheckMotionProbeResult& probe) const {
    const auto& sample = probe.sample;
    return sample.commanded_direction != self_check::MotionDirection::Unknown &&
           sample.settled && sample.direction_matches &&
           sample.position_monotonic && !sample.active_stop_triggered &&
           sample.measured_distance.value >=
               preBEffectiveProgressDistance().value;
  }

  void recordPreBMotionSample(
      const SelfCheckMotionProbeResult& probe,
      self_check::SelfCheckInput* input,
      PreSelfCheckExpansionBounds* expansion_bounds,
      const self_check::FrictionAnomalyDetector* friction_anomaly_detector =
          nullptr,
      std::uint32_t first_anomaly_record_index = 0U) {
    if (input == nullptr ||
        !(isUsablePreBMotionSample(probe.sample) ||
          isPassThroughPreBMotionSample(probe))) {
      return;
    }
    if (probeSegmentOverlapsFrictionAnomaly(probe, friction_anomaly_detector,
                                            first_anomaly_record_index)) {
      std::ostringstream message;
      message << "PreSelfCheck | pre_b_sample_skipped"
              << " reason=friction_anomaly_overlap"
              << " direction=" << directionName(probe.sample.commanded_direction)
              << " start_mm=" << probe.start_stroke.value
              << " end_mm=" << probe.end_stroke.value
              << " avoid_margin_mm=" << frictionAnomalyAvoidMargin()
              << " avoid_ratio=" << frictionAnomalyLearningAvoidRatio();
      reportProgress(message.str());
      return;
    }
    if (expansion_bounds == nullptr) {
      return;
    }
    if (probe.sample.commanded_direction == self_check::MotionDirection::Opening) {
      ++expansion_bounds->open_sample_count;
    } else if (probe.sample.commanded_direction ==
               self_check::MotionDirection::Closing) {
      ++expansion_bounds->closed_sample_count;
    }
  }

  [[nodiscard]] bool isFartherPreBObservedLimit(
      self_check::MotionDirection direction,
      common::Mm candidate_limit,
      common::Mm current_limit,
      common::Mm threshold = {}) const {
    const double required_delta =
        threshold.value > 0.0 ? threshold.value : selfCheckNoProgressThreshold().value;
    if (direction == self_check::MotionDirection::Opening) {
      return candidate_limit.value < current_limit.value - required_delta;
    }
    if (direction == self_check::MotionDirection::Closing) {
      return candidate_limit.value > current_limit.value + required_delta;
    }
    return false;
  }

  [[nodiscard]] common::Mm applyPreBProbeProgress(
      self_check::MotionDirection direction,
      const SelfCheckMotionProbeResult& probe,
      self_check::SelfCheckInput* input,
      PreSelfCheckExpansionBounds* expansion_bounds,
      common::Mm* observed_limit,
      const self_check::FrictionAnomalyDetector* friction_anomaly_detector =
          nullptr,
      std::uint32_t first_anomaly_record_index = 0U) {
    if (observed_limit == nullptr ||
        !isEffectivePreBProgressSample(probe)) {
      return common::Mm{0.0};
    }
    const common::Mm previous_observed_limit = *observed_limit;
    const bool frontier_advanced =
        isFartherPreBObservedLimit(direction, probe.end_stroke,
                                  previous_observed_limit);
    if (frontier_advanced) {
      *observed_limit = probe.end_stroke;
    }
    if (expansion_bounds != nullptr && frontier_advanced) {
      if (direction == self_check::MotionDirection::Opening) {
        expansion_bounds->open_limit = *observed_limit;
      } else {
        expansion_bounds->closed_limit = *observed_limit;
      }
    }
    if (isPassThroughPreBMotionSample(probe)) {
      recordPreBMotionSample(probe, input, expansion_bounds,
                             friction_anomaly_detector,
                             first_anomaly_record_index);
    }
    if (!frontier_advanced) {
      return common::Mm{0.0};
    }
    return common::Mm{std::abs(observed_limit->value -
                               previous_observed_limit.value)};
  }

  void markPreBMechanicalLimit(self_check::MotionDirection direction,
                               common::Mm observed_limit,
                               self_check::SelfCheckInput* input,
                               std::string reason) {
    if (direction == self_check::MotionDirection::Opening) {
      pre_self_check_opening_boundary_suspected_ = true;
    } else {
      pre_self_check_closing_boundary_suspected_ = true;
    }
    if (input != nullptr) {
      input->limit_samples.push_back(self_check::LimitObservationSample{
          direction, observed_limit, true, false, false});
    }
    reportProgress(std::string{"PreSelfCheck | suspected mechanical limit direction="} +
                   directionName(direction) +
                   " stroke_mm=" + std::to_string(observed_limit.value) +
                   " reason=" + std::move(reason));
  }

  [[nodiscard]] bool isHardSelfCheckJamResult(
      const SelfCheckMotionProbeResult& probe) const {
    return probe.result.code() == common::ErrorCode::SafetyJamDetected &&
           (probe.hard_current_confirmed ||
            probe.emergency_current_confirmed ||
            selfCheckProbeMessageContains(
                probe, "hard feedback current limit") ||
            selfCheckProbeMessageContains(
                probe, "emergency feedback current limit"));
  }

  [[nodiscard]] bool isHardSelfCheckNoProgressJamResult(
      const SelfCheckMotionProbeResult& probe) const {
    return isHardSelfCheckJamResult(probe) &&
           probe.hard_current_no_progress_confirmed;
  }

  [[nodiscard]] bool isPreBPhysicalBoundaryCandidate(
      const SelfCheckMotionProbeResult& probe) const {
    if ((probe.hard_current_no_progress_confirmed ||
         isHardSelfCheckNoProgressJamResult(probe))) {
      return true;
    }
    if (probe.sustained_no_progress_confirmed) {
      return true;
    }
    const bool no_effective_motion =
        probe.measured_distance.value < preBEffectiveProgressDistance().value;
    return no_effective_motion &&
           (probe.velocity_collapsed || probe.jam_suspected ||
            probe.limit_suspected);
  }

  [[nodiscard]] common::MmPerS preBExpansionSpeed() const {
    const double requested = std::max(
        std::abs(config_.self_check.min_speed_scan_start.value),
        std::abs(config_.self_check.pre_b_expansion_speed.value));
    const double safety_cap = std::abs(config_.safety.max_nut_speed.value);
    return common::MmPerS{safety_cap > 0.0 ? std::min(requested, safety_cap)
                                           : requested};
  }

  [[nodiscard]] common::Mm preBRelativeScanWindowDistance() const {
    const double total_theoretical_travel =
        std::abs(config_.mechanism.theoretical_close_limit.value -
                 config_.mechanism.theoretical_open_limit.value);
    const double configured =
        std::abs(config_.self_check.pre_b_max_expansion_distance.value);
    const double window =
        configured > 0.0 ? std::max(configured, total_theoretical_travel)
                         : total_theoretical_travel;
    return common::Mm{std::max(
        window,
        std::max(config_.self_check.motion_start_distance.value,
                 config_.self_check.min_measured_distance.value))};
  }

  [[nodiscard]] SelfCheckProbeOptions preBRelativeProbeOptions(
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      common::S early_progress_timeout = {},
      common::S sustained_no_progress_timeout = {}) const {
    const common::Mm window = preBRelativeScanWindowDistance();
    return SelfCheckProbeOptions{
        false,
        friction_anomaly_detector,
        true,
        early_progress_timeout,
        config_.self_check.pre_b_hard_current_confirm_time,
        sustained_no_progress_timeout,
        true,
        common::Mm{current_nut_stroke_.value - window.value},
        common::Mm{current_nut_stroke_.value + window.value},
        friction_anomaly_detector != nullptr};
  }

  [[nodiscard]] common::Mm preBTargetStrokeFromRelativeDistance(
      self_check::MotionDirection direction, common::Mm distance) const {
    return common::Mm{
        current_nut_stroke_.value +
        (direction == self_check::MotionDirection::Opening
             ? -std::abs(distance.value)
             : std::abs(distance.value))};
  }

  [[nodiscard]] common::Mm preBScanDistanceInsideMotorWindow(
      self_check::MotionDirection direction, common::Mm requested_distance,
      std::string* motor_window_detail) const {
    const double requested = std::max(0.0, requested_distance.value);
    if (requested <= 0.0) {
      return common::Mm{0.0};
    }

    const common::Mm requested_target =
        preBTargetStrokeFromRelativeDistance(direction, common::Mm{requested});
    std::string requested_detail;
    if (!preBTargetExceedsMotorPositionWindow(
            strokeToReferencedMotorPosition(requested_target),
            &requested_detail)) {
      if (motor_window_detail != nullptr) {
        *motor_window_detail = requested_detail;
      }
      return common::Mm{requested};
    }

    double low = 0.0;
    double high = requested;
    for (std::uint32_t iteration = 0; iteration < 48U; ++iteration) {
      const double mid = (low + high) * 0.5;
      const common::Mm mid_target =
          preBTargetStrokeFromRelativeDistance(direction, common::Mm{mid});
      if (preBTargetExceedsMotorPositionWindow(
              strokeToReferencedMotorPosition(mid_target), nullptr)) {
        high = mid;
      } else {
        low = mid;
      }
    }

    const common::Mm limited_distance{std::max(0.0, low)};
    const common::Mm limited_target =
        preBTargetStrokeFromRelativeDistance(direction, limited_distance);
    std::string limited_detail;
    (void)preBTargetExceedsMotorPositionWindow(
        strokeToReferencedMotorPosition(limited_target), &limited_detail);
    if (motor_window_detail != nullptr) {
      std::ostringstream message;
      message << limited_detail
              << " pmax_scan_distance_limited=true"
              << " requested_scan_distance_mm=" << requested
              << " limited_scan_distance_mm=" << limited_distance.value
              << " requested_target_stroke_mm=" << requested_target.value
              << " limited_target_stroke_mm=" << limited_target.value;
      *motor_window_detail = message.str();
    }
    return limited_distance;
  }

  [[nodiscard]] common::Mm travelLearningSearchTargetFromDistance(
      common::Mm distance) const {
    return common::Mm{config_.mechanism.theoretical_open_limit.value +
                      std::max(0.0, distance.value)};
  }

  [[nodiscard]] common::Mm travelLearningSearchDistanceInsideMotorWindow(
      common::Mm requested_distance, std::string* motor_window_detail) const {
    const double requested = std::max(0.0, requested_distance.value);
    if (requested <= 0.0) {
      return common::Mm{0.0};
    }

    const common::Mm requested_target =
        travelLearningSearchTargetFromDistance(common::Mm{requested});
    std::string requested_detail;
    if (!preBTargetExceedsMotorPositionWindow(
            strokeToReferencedMotorPosition(requested_target),
            &requested_detail)) {
      if (motor_window_detail != nullptr) {
        *motor_window_detail = requested_detail;
      }
      return common::Mm{requested};
    }

    double low = 0.0;
    double high = requested;
    for (std::uint32_t iteration = 0; iteration < 48U; ++iteration) {
      const double mid = (low + high) * 0.5;
      const common::Mm mid_target =
          travelLearningSearchTargetFromDistance(common::Mm{mid});
      if (preBTargetExceedsMotorPositionWindow(
              strokeToReferencedMotorPosition(mid_target), nullptr)) {
        high = mid;
      } else {
        low = mid;
      }
    }

    const common::Mm limited_distance{std::max(0.0, low)};
    const common::Mm limited_target =
        travelLearningSearchTargetFromDistance(limited_distance);
    std::string limited_detail;
    (void)preBTargetExceedsMotorPositionWindow(
        strokeToReferencedMotorPosition(limited_target), &limited_detail);
    if (motor_window_detail != nullptr) {
      std::ostringstream message;
      message << limited_detail
              << " pmax_travel_search_limited=true"
              << " requested_search_distance_mm=" << requested
              << " limited_search_distance_mm=" << limited_distance.value
              << " requested_target_stroke_mm=" << requested_target.value
              << " limited_target_stroke_mm=" << limited_target.value;
      *motor_window_detail = message.str();
    }
    return limited_distance;
  }

  [[nodiscard]] common::Mm preBBoundaryStepDistance() const {
    const double configured = std::abs(config_.self_check.pre_b_boundary_step.value);
    const double fallback = std::max(
        config_.self_check.motion_start_distance.value * 2.0,
        config_.self_check.min_measured_distance.value);
    return common::Mm{configured > 0.0 ? configured : fallback};
  }

  [[nodiscard]] double preBDynamicFrictionSpeedCap() const {
    const double configured_cap =
        std::max(std::abs(config_.self_check.min_speed_scan_stop.value),
                 preBExpansionSpeed().value);
    const double safety_cap = std::abs(config_.safety.max_nut_speed.value);
    return safety_cap > 0.0 ? std::min(configured_cap, safety_cap)
                            : configured_cap;
  }

  [[nodiscard]] PreBSoftJamRetryResult confirmPreBSoftJamPassThrough(
      self_check::MotionDirection direction,
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      PreSelfCheckExpansionBounds* expansion_bounds,
      common::Mm* observed_limit) {
    PreBSoftJamRetryResult output{};
    const double retry_distance = std::max(
        config_.self_check.motion_start_distance.value * 2.0,
        config_.self_check.pre_b_soft_jam_retry_distance.value);
    const double retry_speed = std::max(
        std::abs(config_.self_check.min_speed_scan_start.value),
        std::abs(config_.self_check.pre_b_soft_jam_retry_speed.value));
    if (retry_distance <= 0.0 || retry_speed <= 0.0) {
      output.result = common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "pre-self-check soft-jam retry config must be positive");
      return output;
    }

    std::ostringstream start_message;
    start_message << "PreSelfCheck | soft_jam_retry"
                  << " direction=" << directionName(direction)
                  << " distance_mm=" << retry_distance
                  << " speed_mm_s=" << retry_speed
                  << " current_limit_a="
                  << config_.safety.self_check_current_limit.value;
    reportProgress(start_message.str());

    const auto retry = runSelfCheckMotionProbe(
        direction, common::Mm{retry_distance}, common::MmPerS{retry_speed},
        config_.safety.self_check_current_limit,
        preBRelativeProbeOptions(
            friction_anomaly_detector,
            config_.self_check.pre_b_soft_jam_progress_timeout));
    last_self_check_probe_ = retry;
    last_motion_sample_ = retry.sample;
    if (isUnsettledSelfCheckProbe(retry)) {
      output.result = retry.result;
      return output;
    }
    output.confirmation_distance = applyPreBProbeProgress(
        direction, retry, input, expansion_bounds, observed_limit,
        friction_anomaly_detector);
    if (isPreBPhysicalBoundaryCandidate(retry) ||
        retry.result.code() == common::ErrorCode::SelfCheckInconsistentFeedback ||
        retry.result.code() == common::ErrorCode::SafetyActiveStop) {
      output.result = retry.result;
      return output;
    }
    output.passed_through =
        output.confirmation_distance.value >=
            config_.self_check.motion_start_distance.value &&
        isPassThroughPreBMotionSample(retry);
    return output;
  }

  [[nodiscard]] common::Result runStableShortStrokeSamples(
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector) {
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }
    if (pre_self_check_opening_boundary_suspected_ ||
        pre_self_check_closing_boundary_suspected_) {
      std::ostringstream message;
      message << "stable short-stroke skipped after one-sided low-confidence"
              << " boundary detection"
              << " opening_boundary_suspected="
              << (pre_self_check_opening_boundary_suspected_ ? "true" : "false")
              << " closing_boundary_suspected="
              << (pre_self_check_closing_boundary_suspected_ ? "true" : "false");
      return common::Result::error(common::ErrorCode::SelfCheckFailed,
                                   message.str());
    }
    const common::Mm sample_distance{
        std::min(config_.self_check.max_probe_window.value,
                 std::max(config_.self_check.stable_short_stroke_distance.value,
                          config_.self_check.motion_start_distance.value * 2.0))};
    const double speed_start =
        std::abs(config_.self_check.min_speed_scan_start.value);
    const double speed_stop =
        std::min(std::abs(config_.self_check.min_speed_scan_stop.value),
                 std::max(speed_start,
                          std::abs(config_.self_check.travel_learning_speed.value)));
    const double speed_step =
        std::abs(config_.self_check.min_speed_scan_step.value) > 0.0
            ? std::abs(config_.self_check.min_speed_scan_step.value)
            : std::max(0.1, speed_stop - speed_start);
    const double current_step =
        std::abs(config_.self_check.static_friction_current_step.value) > 0.0
            ? std::abs(config_.self_check.static_friction_current_step.value)
            : 0.05;
    const double current_start = std::min(
        selfCheckStableCurrentLimit().value,
        std::abs(config_.safety.self_check_current_limit.value));
    const double current_stop =
        std::abs(config_.safety.self_check_current_limit.value);
    if (sample_distance.value <= 0.0 || speed_stop <= 0.0 ||
        current_start <= 0.0 || current_stop <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "stable short-stroke config must have positive distance, speed, and current");
    }

    std::ostringstream start_message;
    start_message << "PreSelfCheck | phase=StableShortStrokeMotion"
                  << " | paired_scan distance_mm=" << sample_distance.value
                  << " speed_start_mm_s=" << speed_start
                  << " speed_stop_mm_s=" << speed_stop
                  << " current_start_a=" << current_start
                  << " current_stop_a=" << current_stop;
    reportProgress(start_message.str());

    common::Result last_error = common::Result::error(
        common::ErrorCode::SelfCheckFailed,
        "stable short-stroke motion was not found");
    self_check::MotionIdentificationSample best_closing{};
    self_check::MotionIdentificationSample best_opening{};
    for (double speed = speed_start; speed <= speed_stop + 1e-9;
         speed += speed_step) {
      if (const auto cancel = checkOperationCancelled("stable short-stroke speed scan");
          cancel.isError()) {
        return cancel;
      }
      for (double current = current_start; current <= current_stop + 1e-9;
           current += current_step) {
        if (const auto cancel = checkOperationCancelled("stable short-stroke current scan");
            cancel.isError()) {
          return cancel;
        }
        const common::A limited_current{std::min(current, current_stop)};
        const auto close_probe = runSelfCheckMotionProbe(
            self_check::MotionDirection::Closing, sample_distance,
            common::MmPerS{speed}, limited_current,
            SelfCheckProbeOptions{false, friction_anomaly_detector});
        if (isUnsettledSelfCheckProbe(close_probe)) {
          return close_probe.result;
        }
        if (close_probe.result.code() ==
            common::ErrorCode::SelfCheckInconsistentFeedback) {
          return close_probe.result;
        }
        const auto close_sample = close_probe.sample;
        const auto open_probe = runSelfCheckMotionProbe(
            self_check::MotionDirection::Opening, sample_distance,
            common::MmPerS{speed}, limited_current,
            SelfCheckProbeOptions{false, friction_anomaly_detector});
        if (isUnsettledSelfCheckProbe(open_probe)) {
          return open_probe.result;
        }
        if (open_probe.result.code() ==
            common::ErrorCode::SelfCheckInconsistentFeedback) {
          return open_probe.result;
        }
        const auto open_sample = open_probe.sample;
        if (auto endpoint_start = acceptEndpointStartEscape(
                close_probe, open_probe, input);
            endpoint_start.has_value()) {
          return *endpoint_start;
        }
        if (auto endpoint_start = acceptEndpointStartEscape(
                open_probe, close_probe, input);
            endpoint_start.has_value()) {
          return *endpoint_start;
        }
        rememberBestSelfCheckMotionSample(close_sample, &best_closing);
        rememberBestSelfCheckMotionSample(open_sample, &best_opening);

        const bool close_ok =
            isStableShortStrokeSample(close_sample, sample_distance);
        const bool open_ok =
            isStableShortStrokeSample(open_sample, sample_distance);
        if (close_ok && open_ok) {
          return common::Ok();
        }
        last_error = summarizeStableShortStrokeFailure(
            close_probe, open_probe, sample_distance);
      }
    }
    if (isUsableLowConfidenceMotionSample(best_closing) &&
        isUsableLowConfidenceMotionSample(best_opening)) {
      std::ostringstream message;
      message << "PreSelfCheck | phase=StableShortStrokeMotion"
              << " | low_confidence_samples_accepted"
              << " closing_mm=" << best_closing.measured_distance.value
              << " opening_mm=" << best_opening.measured_distance.value
              << " target_mm=" << sample_distance.value;
      reportProgress(message.str());
    }
    return last_error;
  }

  [[nodiscard]] common::Result runPreliminaryLimitSearch(
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      PreSelfCheckExpansionBounds* expansion_bounds) {
    if (input == nullptr || expansion_bounds == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check preliminary search output is null");
    }
    const common::Mm open_limit = config_.mechanism.theoretical_open_limit;
    const common::Mm closed_limit = config_.mechanism.theoretical_close_limit;
    const common::Mm theoretical_travel{closed_limit.value - open_limit.value};
    if (theoretical_travel.value <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "pre-self-check theoretical travel must be positive");
    }

    const double current_stroke = current_nut_stroke_.value;
    const common::Mm start_stroke{current_stroke};
    expansion_bounds->open_limit = start_stroke;
    expansion_bounds->closed_limit = start_stroke;
    std::optional<common::Result> first_degraded_result{};

    std::vector<self_check::MotionDirection> search_order{
        self_check::MotionDirection::Opening,
        self_check::MotionDirection::Closing};
    if (pre_self_check_opening_boundary_suspected_ &&
        !pre_self_check_closing_boundary_suspected_) {
      search_order = {self_check::MotionDirection::Closing,
                      self_check::MotionDirection::Opening};
    } else if (pre_self_check_closing_boundary_suspected_ &&
               !pre_self_check_opening_boundary_suspected_) {
      search_order = {self_check::MotionDirection::Opening,
                      self_check::MotionDirection::Closing};
    }
    {
      std::ostringstream message;
      message << "PreSelfCheck | phase=PreliminaryLimitSearch"
              << " | search_order first="
              << directionName(search_order.front())
              << " second=" << directionName(search_order.back())
              << " start_mm=" << start_stroke.value
              << " pre_a_open_boundary_suspected="
              << (pre_self_check_opening_boundary_suspected_ ? "true" : "false")
              << " pre_a_closed_boundary_suspected="
              << (pre_self_check_closing_boundary_suspected_ ? "true" : "false");
      reportProgress(message.str());
    }

    for (std::size_t search_index = 0; search_index < search_order.size();
         ++search_index) {
      if (expansion_bounds->boundary_release_failed) {
        std::ostringstream message;
        message << "PreSelfCheck | phase=PreliminaryLimitSearch"
                << " | second_direction_skipped"
                << " reason=boundary_release_failed"
                << " unreleased_boundary_direction="
                << directionName(expansion_bounds->unreleased_boundary_direction)
                << " action=keep_single_side_boundary_and_pre_a_window";
        reportProgress(message.str());
        break;
      }
      const auto direction = search_order[search_index];
      const common::Mm expansion_start = current_nut_stroke_;
      self_check::FrictionAnomalyDetector* search_detector =
          friction_anomaly_detector;
      if (search_index == 0U &&
          direction == self_check::MotionDirection::Opening) {
        search_detector = nullptr;
        reportProgress(
            "PreSelfCheck | phase=PreliminaryLimitSearch"
            " | initial_opening_probe_only"
            " | recording=disabled"
            " | reason=current_to_opening_structure_limit");
      }
      const auto search = searchPreliminaryLimit(
          direction, input, search_detector, expansion_start, expansion_bounds);
      if (search.result.isError()) {
        if (!canDegradePreBSegment(search.result)) {
          return search.result;
        }
        if (!first_degraded_result.has_value()) {
          first_degraded_result = search.result;
        }
      }

      const bool release_recommended =
          direction == self_check::MotionDirection::Opening
              ? (expansion_bounds->open_boundary_suspected ||
                 expansion_bounds->open_release_recommended)
              : (expansion_bounds->closed_boundary_suspected ||
                 expansion_bounds->closed_release_recommended);
      if (release_recommended) {
        const auto release_after_search = releasePreBStoppedDirection(
            direction, *expansion_bounds, input, nullptr,
            search_index == 0U ? "after_first_boundary_search"
                               : "after_second_boundary_search");
        if (release_after_search.isError()) {
          if (!canDegradePreBSegment(release_after_search)) {
            return release_after_search;
          }
          expansion_bounds->boundary_release_failed = true;
          expansion_bounds->unreleased_boundary_direction = direction;
          pre_b_mechanism_anomaly_ = true;
          std::ostringstream message;
          message << "PreSelfCheck | phase=PreliminaryLimitSearch"
                  << " | boundary_release_failed"
                  << " after_direction=" << directionName(direction)
                  << " search_index=" << search_index
                  << " action=skip_remaining_boundary_search_and_learning"
                  << " message=" << release_after_search.message();
          reportProgress(message.str());
          if (search_index + 1U < search_order.size()) {
            std::ostringstream skip_message;
            skip_message << "PreSelfCheck | phase=PreliminaryLimitSearch"
                         << " | second_direction_skipped"
                         << " reason=boundary_release_failed"
                         << " skipped_direction="
                         << directionName(search_order[search_index + 1U])
                         << " unreleased_boundary_direction="
                         << directionName(direction);
            reportProgress(skip_message.str());
          }
          if (!first_degraded_result.has_value()) {
            first_degraded_result = release_after_search;
          }
          break;
        }
      }
    }

    expansion_bounds->open_boundary_suspected =
        pre_self_check_opening_boundary_suspected_;
    expansion_bounds->closed_boundary_suspected =
        pre_self_check_closing_boundary_suspected_;
    expansion_bounds->open_release_recommended =
        expansion_bounds->open_release_recommended ||
        expansion_bounds->open_boundary_suspected;
    expansion_bounds->closed_release_recommended =
        expansion_bounds->closed_release_recommended ||
        expansion_bounds->closed_boundary_suspected;
    if (expansion_bounds->boundary_release_failed) {
      std::ostringstream message;
      message << "PreSelfCheck | phase=PreliminaryLimitSearch"
              << " | degraded reason=boundary_release_failed"
              << " unreleased_boundary_direction="
              << directionName(expansion_bounds->unreleased_boundary_direction)
              << " action=single_side_boundary_not_accepted_as_safe_zone";
      reportProgress(message.str());
      if (first_degraded_result.has_value()) {
        return *first_degraded_result;
      }
      return common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "pre-self-check boundary release failed before both boundaries were verified");
    }
    common::Mm observed_open_limit = expansion_bounds->open_limit;
    common::Mm observed_closed_limit = expansion_bounds->closed_limit;

    const common::Mm observed_travel{
        observed_closed_limit.value - observed_open_limit.value};
    if (observed_travel.value <
        std::max(config_.self_check.min_measured_distance.value,
                 config_.self_check.motion_start_distance.value * 2.0)) {
      if (first_degraded_result.has_value()) {
        return *first_degraded_result;
      }
      return common::Result::error(common::ErrorCode::SelfCheckFailed,
                                   "pre-self-check partial bounds are too narrow");
    }
    const common::Mm travel_error{
        std::abs(theoretical_travel.value - observed_travel.value)};
    input->limit_samples.push_back(self_check::LimitObservationSample{
        self_check::MotionDirection::Opening,
        observed_open_limit,
        true,
        false,
        false});
    input->limit_samples.push_back(self_check::LimitObservationSample{
        self_check::MotionDirection::Closing,
        observed_closed_limit,
        true,
        false,
        false});
    std::ostringstream message;
    message << "PreSelfCheck | phase=PreliminaryLimitSearch | limits accepted"
            << " open_mm=" << observed_open_limit.value
            << " close_mm=" << observed_closed_limit.value
            << " travel_mm=" << observed_travel.value
            << " travel_error_mm=" << travel_error.value
            << " theoretical_travel_mm=" << theoretical_travel.value
            << " open_boundary_suspected="
            << (expansion_bounds->open_boundary_suspected ? "true" : "false")
            << " closed_boundary_suspected="
            << (expansion_bounds->closed_boundary_suspected ? "true" : "false")
            << " note=relative_pre_homing_low_confidence_bounds";
    reportProgress(message.str());
    if (first_degraded_result.has_value()) {
      reportProgress(
          std::string{
              "PreSelfCheck | phase=PreliminaryLimitSearch | partial_bounds_retained | reason="} +
          first_degraded_result->message());
    }
    return common::Ok();
  }

  [[nodiscard]] bool canDegradePreBSegment(
      const common::Result& result) const {
    // PreB improves confidence and coverage, but it is not the first safety
    // gate. If it cannot enlarge the already proven PreA window, keep the PreA
    // conservative result. Feedback contradictions, operator stops, hard safety
    // events, hardware faults, and config errors still fail the workflow.
    return result.code() == common::ErrorCode::SelfCheckFailed ||
           result.code() == common::ErrorCode::OperationTimedOut ||
           isPreBInternalFeedbackStopResult(result);
  }

  [[nodiscard]] bool resultMessageContains(
      const common::Result& result, const char* needle) const {
    return needle != nullptr && result.hasMessage() &&
           result.message().find(needle) != std::string::npos;
  }

  [[nodiscard]] bool isPreBInternalFeedbackStopResult(
      const common::Result& result) const {
    if (result.code() != common::ErrorCode::SafetyJamDetected) {
      return false;
    }
    // PreB owns these feedback stops as boundary-search diagnostics. They are
    // degraded only inside PreB; operator stop, inconsistent feedback, hardware
    // faults, feedback timeouts, and config errors still fail the workflow.
    return resultMessageContains(
               result,
               "pre-self-check probe stopped by sustained hard feedback current limit") ||
           resultMessageContains(
               result,
               "pre-self-check probe stopped by emergency feedback current limit") ||
           resultMessageContains(
               result,
               "pre-self-check continuous scan stopped by sustained no progress");
  }

  [[nodiscard]] common::Result normalizePreBProbeResult(
      const SelfCheckMotionProbeResult& probe) const {
    if (probe.result.code() == common::ErrorCode::SafetyJamDetected &&
        isEffectivePreBProgressSample(probe) &&
        !isPreBPhysicalBoundaryCandidate(probe)) {
      return common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "pre-self-check card-point pass-through stopped before target");
    }
    return probe.result;
  }

  [[nodiscard]] bool isPreBCardPointPartialStop(
      const SelfCheckMotionProbeResult& probe,
      const common::Result& normalized_probe_result,
      common::Mm progress_result) const {
    return (normalized_probe_result.code() ==
                common::ErrorCode::OperationTimedOut ||
            normalized_probe_result.code() ==
                common::ErrorCode::SafetyJamDetected) &&
           progress_result.value >= preBEffectiveProgressDistance().value &&
           isEffectivePreBProgressSample(probe) &&
           !isPreBPhysicalBoundaryCandidate(probe);
  }

  void recommendPreBReleaseAfterStop(
      self_check::MotionDirection direction,
      PreSelfCheckExpansionBounds* expansion_bounds) {
    if (expansion_bounds == nullptr) {
      return;
    }
    if (direction == self_check::MotionDirection::Opening) {
      expansion_bounds->open_release_recommended = true;
    } else if (direction == self_check::MotionDirection::Closing) {
      expansion_bounds->closed_release_recommended = true;
    }
  }

  [[nodiscard]] PreSelfCheckExpansionBounds
  fallbackPreSelfCheckExpansionBounds() const {
    PreSelfCheckExpansionBounds bounds{};
    const double half_window =
        std::max({config_.self_check.motion_start_distance.value,
                  config_.self_check.low_confidence_motion_distance.value,
                  config_.self_check.stable_short_stroke_distance.value,
                  config_.self_check.min_measured_distance.value}) *
        0.5;
    bounds.open_limit = common::Mm{current_nut_stroke_.value - half_window};
    bounds.closed_limit = common::Mm{current_nut_stroke_.value + half_window};
    return bounds;
  }

  [[nodiscard]] SelfCheckProbeOptions preBLocalReleaseProbeOptions(
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      common::Mm target_stroke) const {
    auto options = preBRelativeProbeOptions(
        friction_anomaly_detector, {},
        config_.self_check.pre_b_soft_jam_progress_timeout);
    const double guard_margin =
        std::max(config_.self_check.max_distance_error.value,
                 config_.self_check.fallback_nut_stroke_noise.value * 3.0);
    options.custom_open_limit =
        common::Mm{std::min(current_nut_stroke_.value, target_stroke.value) -
                   guard_margin};
    options.custom_closed_limit =
        common::Mm{std::max(current_nut_stroke_.value, target_stroke.value) +
                   guard_margin};
    options.command_current_cap = preBBoundaryReleaseCurrentLimit();
    return options;
  }

  [[nodiscard]] PreSelfCheckDirectionalSearchResult searchPreliminaryLimit(
      self_check::MotionDirection direction, self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      common::Mm expansion_start_stroke,
      PreSelfCheckExpansionBounds* expansion_bounds) {
    PreSelfCheckDirectionalSearchResult output{};
    output.observed_limit = expansion_start_stroke;
    if (input == nullptr || expansion_bounds == nullptr) {
      output.result = common::Result::error(
          common::ErrorCode::InvalidArgument,
          "preliminary limit search output is null");
      return output;
    }

    const common::MmPerS search_speed = preBExpansionSpeed();
    const common::Mm refine_step_distance = preBBoundaryStepDistance();
    const double total_theoretical_travel =
        std::abs(config_.mechanism.theoretical_close_limit.value -
                 config_.mechanism.theoretical_open_limit.value);
    const double configured_max_expansion =
        std::abs(config_.self_check.pre_b_max_expansion_distance.value) > 0.0
            ? std::abs(config_.self_check.pre_b_max_expansion_distance.value)
            : total_theoretical_travel;
    const double max_expansion = std::max(0.0, configured_max_expansion);
    const common::Mm relative_scan_target{
        expansion_start_stroke.value +
        (direction == self_check::MotionDirection::Opening ? -max_expansion
                                                           : max_expansion)};

    reportPreBSearchStart(direction, expansion_start_stroke, relative_scan_target,
                          search_speed, refine_step_distance, max_expansion);

    if (max_expansion <= config_.self_check.max_distance_error.value) {
      input->limit_samples.push_back(self_check::LimitObservationSample{
          direction, output.observed_limit, true, false, false});
      reportProgress(std::string{"PreSelfCheck | no expansion room direction="} +
                     directionName(direction) +
                     " stroke_mm=" +
                     std::to_string(output.observed_limit.value));
      output.result = common::Ok();
      return output;
    }

    if (const auto cancel = checkOperationCancelled("preliminary limit search");
        cancel.isError()) {
      output.result = cancel;
      return output;
    }

    common::Result first_degraded_result =
        common::Result::error(common::ErrorCode::OperationTimedOut,
                              "pre-self-check preliminary scan made no progress");
    bool has_degraded_result = false;
    bool made_effective_progress = false;
    std::uint32_t segment_index = 0U;
    const std::uint32_t max_segments =
        static_cast<std::uint32_t>(
            std::ceil(max_expansion /
                      std::max(preBEffectiveProgressDistance().value, 0.05))) +
        4U;

    while (segment_index++ < max_segments) {
      if (const auto cancel = checkOperationCancelled("preliminary limit search");
          cancel.isError()) {
        output.result = cancel;
        return output;
      }

      const double expanded =
          std::max(0.0,
                   direction == self_check::MotionDirection::Opening
                       ? expansion_start_stroke.value -
                             output.observed_limit.value
                       : output.observed_limit.value -
                             expansion_start_stroke.value);
      const double expansion_remaining = max_expansion - expanded;
      if (expansion_remaining <= config_.self_check.max_distance_error.value) {
        input->limit_samples.push_back(self_check::LimitObservationSample{
            direction, output.observed_limit, true, false, false});
        reportProgress(
            std::string{"PreSelfCheck | expansion boundary direction="} +
            directionName(direction) + " stroke_mm=" +
            std::to_string(output.observed_limit.value) + " expanded_mm=" +
            std::to_string(expanded) +
            " note=relative_configured_pre_b_boundary");
        output.result = common::Ok();
        return output;
      }

      std::string motor_window_detail;
      const double distance_to_relative_target =
          std::abs(relative_scan_target.value - current_nut_stroke_.value);
      const common::Mm scan_distance =
          preBScanDistanceInsideMotorWindow(
              direction, common::Mm{distance_to_relative_target},
              &motor_window_detail);
      const common::Mm target_stroke =
          preBTargetStrokeFromRelativeDistance(direction, scan_distance);
      reportPreBScanPlan(direction, target_stroke, scan_distance,
                         distance_to_relative_target, expanded,
                         expansion_remaining, motor_window_detail);
      if (scan_distance.value <= config_.self_check.max_distance_error.value) {
        output.observed_limit = current_nut_stroke_;
        if (direction == self_check::MotionDirection::Opening) {
          expansion_bounds->open_limit = output.observed_limit;
        } else {
          expansion_bounds->closed_limit = output.observed_limit;
        }
        input->limit_samples.push_back(self_check::LimitObservationSample{
            direction, output.observed_limit, true, false, false});
        reportProgress(
            std::string{"PreSelfCheck | expansion boundary direction="} +
            directionName(direction) + " stroke_mm=" +
            std::to_string(output.observed_limit.value) +
            " note=damiao_pmax_position_window_no_scan_room" +
            motor_window_detail);
        output.result = common::Ok();
        return output;
      }

      const std::uint32_t anomaly_count_before =
          friction_anomaly_detector != nullptr
              ? static_cast<std::uint32_t>(
                    friction_anomaly_detector->records().size())
              : 0U;
      const auto probe = runSelfCheckMotionProbe(
          direction, scan_distance, search_speed,
          config_.safety.self_check_current_limit,
          preBRelativeProbeOptions(
              friction_anomaly_detector, {},
              config_.self_check.pre_b_soft_jam_progress_timeout));
      if (isUnsettledSelfCheckProbe(probe)) {
        output.result = probe.result;
        return output;
      }
      const auto accepted_progress_result =
          applyPreBProbeProgress(direction, probe, input, expansion_bounds,
                                 &output.observed_limit,
                                 friction_anomaly_detector,
                                 anomaly_count_before);
      const common::Mm probe_progress_result =
          isEffectivePreBProgressSample(probe) ? probe.measured_distance
                                               : common::Mm{};
      reportPreBScanResult(direction, probe, accepted_progress_result,
                           output.observed_limit);
      const common::Result normalized_probe_result =
          normalizePreBProbeResult(probe);
      if (accepted_progress_result.value >=
          preBEffectiveProgressDistance().value) {
        made_effective_progress = true;
      }

      if (normalized_probe_result.isOk()) {
        input->limit_samples.push_back(self_check::LimitObservationSample{
            direction, output.observed_limit, true, false, false});
        reportProgress(
            std::string{"PreSelfCheck | expansion boundary direction="} +
            directionName(direction) + " stroke_mm=" +
            std::to_string(output.observed_limit.value) + " expanded_mm=" +
            std::to_string(std::max(
                0.0,
                direction == self_check::MotionDirection::Opening
                    ? expansion_start_stroke.value -
                          output.observed_limit.value
                    : output.observed_limit.value -
                          expansion_start_stroke.value)) +
            " note=relative_configured_pre_b_boundary");
        output.result = common::Ok();
        return output;
      }

      if (isPreBCardPointPartialStop(probe, normalized_probe_result,
                                     accepted_progress_result)) {
        recommendPreBReleaseAfterStop(direction, expansion_bounds);
        has_degraded_result = true;
        first_degraded_result = normalized_probe_result;
        std::ostringstream message;
        message << "PreSelfCheck | pre_b_card_point_continue"
                << " direction=" << directionName(direction)
                << " segment=" << segment_index
                << " progress_mm=" << probe_progress_result.value
                << " accepted_progress_mm=" << accepted_progress_result.value
                << " observed_limit_mm=" << output.observed_limit.value
                << " remaining_before_mm=" << expansion_remaining
                << " result="
                << common::toString(normalized_probe_result.code());
        reportProgress(message.str());
        const auto release_result = releasePreBStoppedDirection(
            direction, *expansion_bounds, input, nullptr,
            "card_point_continue");
        if (release_result.isError()) {
          output.result = release_result;
          return output;
        }
        continue;
      }

      if (normalized_probe_result.code() ==
              common::ErrorCode::SafetyJamDetected &&
          probe.jam_suspected && !isPreBPhysicalBoundaryCandidate(probe)) {
        const double distance_to_relative_target =
            std::abs(output.observed_limit.value - target_stroke.value);
        if (distance_to_relative_target <=
            std::max(config_.self_check.max_distance_error.value,
                     config_.self_check.pre_b_soft_jam_retry_distance.value)) {
          recommendPreBReleaseAfterStop(direction, expansion_bounds);
          markPreBMechanicalLimit(direction, output.observed_limit, input,
                                  "soft_jam_near_relative_scan_target");
          output.result = common::Ok();
          return output;
        }
        const auto retry = confirmPreBSoftJamPassThrough(
            direction, input, friction_anomaly_detector, expansion_bounds,
            &output.observed_limit);
        if (retry.result.isError()) {
          output.result = retry.result;
          return output;
        }
        if (retry.passed_through) {
          recommendPreBReleaseAfterStop(direction, expansion_bounds);
          const auto release_result = releasePreBStoppedDirection(
              direction, *expansion_bounds, input, nullptr,
              "soft_jam_retry_passed_through");
          if (release_result.isError()) {
            output.result = release_result;
            return output;
          }
          continue;
        }
        const bool no_effective_motion =
            probe_progress_result.value <
                config_.self_check.min_measured_distance.value &&
            retry.confirmation_distance.value <
                config_.self_check.motion_start_distance.value;
        if (no_effective_motion || isPreBPhysicalBoundaryCandidate(probe)) {
          recommendPreBReleaseAfterStop(direction, expansion_bounds);
          markPreBMechanicalLimit(direction, output.observed_limit, input,
                                  no_effective_motion
                                      ? "soft_jam_retry_no_progress"
                                      : "continuous_scan_boundary_candidate");
          output.result = common::Ok();
          return output;
        }
      }

      if (isPreBPhysicalBoundaryCandidate(probe) &&
          (normalized_probe_result.code() ==
               common::ErrorCode::OperationTimedOut ||
           normalized_probe_result.code() ==
               common::ErrorCode::SafetyJamDetected)) {
        recommendPreBReleaseAfterStop(direction, expansion_bounds);
        markPreBMechanicalLimit(
            direction, output.observed_limit, input,
            std::string{common::toString(normalized_probe_result.code())});
        output.result = common::Ok();
        return output;
      }

      if (canDegradePreBSegment(normalized_probe_result)) {
        has_degraded_result = true;
        first_degraded_result = normalized_probe_result;
      } else {
        output.result = normalized_probe_result;
        return output;
      }
      if (!made_effective_progress) {
        output.result = normalized_probe_result;
        return output;
      }
    }

    output.result =
        has_degraded_result
            ? first_degraded_result
            : common::Result::error(
                  common::ErrorCode::OperationTimedOut,
                  "pre-self-check preliminary scan exceeded segment budget");
    return output;
  }

  [[nodiscard]] common::Result moveContinuouslyToPreBStroke(
      common::Mm target_stroke,
      const ManualPositioningRange& safe_zone,
      const char* reason) {
    if (!safe_zone.valid) {
      return common::Result::error(
          common::ErrorCode::SelfCheckFailed,
          "pre-self-check safe zone is too small for continuous reposition");
    }
    if (const auto cancel = checkOperationCancelled("continuous PreB reposition");
        cancel.isError()) {
      return cancel;
    }
    const double remaining = target_stroke.value - current_nut_stroke_.value;
    if (std::abs(remaining) <= config_.self_check.max_distance_error.value) {
      reportProgress(
          "PreSelfCheck | phase=MultiRegionRoundTripLearning"
          " | continuous_reposition skipped reason=target_already_reached");
      return common::Ok();
    }

    const auto direction = remaining < 0.0
                               ? self_check::MotionDirection::Opening
                               : self_check::MotionDirection::Closing;
    const common::Mm distance{std::abs(remaining)};
    const common::MmPerS move_speed = preBExpansionSpeed();
    auto options = preBRelativeProbeOptions(
        nullptr, {}, config_.self_check.pre_b_soft_jam_progress_timeout);
    options.record_pre_b_current_trace = true;
    const double guard_margin =
        std::max(config_.self_check.max_distance_error.value,
                 config_.self_check.fallback_nut_stroke_noise.value * 3.0);
    options.custom_open_limit = common::Mm{
        std::min({safe_zone.open_limit.value, current_nut_stroke_.value,
                  target_stroke.value}) -
        guard_margin};
    options.custom_closed_limit = common::Mm{
        std::max({safe_zone.closed_limit.value, current_nut_stroke_.value,
                  target_stroke.value}) +
        guard_margin};

    std::ostringstream start_message;
    start_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                  << " | continuous_reposition start"
                  << " reason=" << (reason != nullptr ? reason : "reposition")
                  << " direction=" << directionName(direction)
                  << " current_mm=" << current_nut_stroke_.value
                  << " target_mm=" << target_stroke.value
                  << " distance_mm=" << distance.value
                  << " speed_mm_s=" << move_speed.value
                  << " recording=ui_trace_only"
                  << " anomaly_recording=disabled"
                  << " learning_recording=disabled";
    reportProgress(start_message.str());

    const auto probe = runSelfCheckMotionProbe(
        direction, distance, move_speed,
        config_.safety.self_check_current_limit, options);
    if (isUnsettledSelfCheckProbe(probe)) {
      return probe.result;
    }

    std::ostringstream result_message;
    result_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                   << " | continuous_reposition result"
                   << " code=" << common::toString(probe.result.code())
                   << " reason=" << (reason != nullptr ? reason : "reposition")
                   << " start_mm=" << probe.start_stroke.value
                   << " end_mm=" << probe.end_stroke.value
                   << " measured_mm=" << probe.measured_distance.value
                   << " target_reached="
                   << (probe.target_reached ? "true" : "false")
                   << " hard_current_confirmed="
                   << (probe.hard_current_confirmed ? "true" : "false")
                   << " hard_current_no_progress="
                   << (probe.hard_current_no_progress_confirmed ? "true"
                                                                 : "false")
                   << " physical_boundary_candidate="
                   << (isPreBPhysicalBoundaryCandidate(probe) ? "true"
                                                              : "false");
    if (probe.result.hasMessage()) {
      result_message << " message=" << probe.result.message();
    }
    reportProgress(result_message.str());

    const common::Result normalized_probe_result =
        normalizePreBProbeResult(probe);
    if (normalized_probe_result.isError()) {
      return normalized_probe_result;
    }
    if (!probe.target_reached) {
      return common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "continuous PreB reposition did not reach target");
    }
    return common::Ok();
  }

  [[nodiscard]] common::Result moveToSelfCheckStroke(
      common::Mm target_stroke, self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector) {
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }
    const common::MmPerS move_speed = preBExpansionSpeed();
    const common::Mm max_step = preBBoundaryStepDistance();
    const double total_travel =
        std::max(std::abs(target_stroke.value - current_nut_stroke_.value),
                 preBRelativeScanWindowDistance().value);
    const std::uint32_t max_steps =
        static_cast<std::uint32_t>(
            std::ceil(total_travel / std::max(max_step.value, 0.1))) +
        4U;

    for (std::uint32_t step_index = 0; step_index < max_steps; ++step_index) {
      if (const auto cancel = checkOperationCancelled("self-check stroke move");
          cancel.isError()) {
        return cancel;
      }
      const double remaining = target_stroke.value - current_nut_stroke_.value;
      if (std::abs(remaining) <= config_.self_check.max_distance_error.value) {
        return common::Ok();
      }
      const auto direction = remaining < 0.0
                                 ? self_check::MotionDirection::Opening
                                 : self_check::MotionDirection::Closing;
      const common::Mm step_distance{
          std::min(std::abs(remaining), max_step.value)};
      const auto probe = runSelfCheckMotionProbe(
          direction, step_distance, move_speed,
          config_.safety.self_check_current_limit,
          preBRelativeProbeOptions(friction_anomaly_detector));
      if (isUnsettledSelfCheckProbe(probe)) {
        return probe.result;
      }
      if (isPassThroughPreBMotionSample(probe)) {
        recordPreBMotionSample(probe, input, nullptr,
                               friction_anomaly_detector);
      }
      const common::Result normalized_probe_result =
          normalizePreBProbeResult(probe);
      if (normalized_probe_result.code() == common::ErrorCode::SafetyJamDetected &&
          probe.jam_suspected && !isPreBPhysicalBoundaryCandidate(probe)) {
        common::Mm ignored_observed_limit = current_nut_stroke_;
        const auto retry = confirmPreBSoftJamPassThrough(
            direction, input, friction_anomaly_detector, nullptr,
            &ignored_observed_limit);
        if (retry.result.isError()) {
          return retry.result;
        }
        if (retry.passed_through) {
          continue;
        }
      }
      if (normalized_probe_result.isError()) {
        if ((canDegradePreBSegment(normalized_probe_result) ||
             normalized_probe_result.code() == common::ErrorCode::SafetyJamDetected) &&
            probe.sample.settled && probe.sample.direction_matches &&
            probe.sample.position_monotonic &&
            probe.measured_distance.value >=
                preBEffectiveProgressDistance().value &&
            !isPreBPhysicalBoundaryCandidate(probe)) {
          continue;
        }
        return normalized_probe_result;
      }
    }
    return common::Result::error(common::ErrorCode::OperationTimedOut,
                                 "failed to move back into self-check safe zone");
  }

  [[nodiscard]] common::Result releasePreBStoppedDirection(
      self_check::MotionDirection stopped_direction,
      const PreSelfCheckExpansionBounds& expansion_bounds,
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      const char* reason) {
    (void)expansion_bounds;
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }
    const double release_distance = preBBoundaryReleaseDistance().value;
    const double release_speed = preBBoundaryReleaseSpeed().value;
    if (release_distance <= 0.0 || release_speed <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "pre-self-check boundary release distance and speed must be positive");
    }

    const self_check::MotionDirection release_direction =
        stopped_direction == self_check::MotionDirection::Opening
            ? self_check::MotionDirection::Closing
            : self_check::MotionDirection::Opening;
    double released_distance = 0.0;
    common::Result first_degraded_result = common::Result::error(
        common::ErrorCode::OperationTimedOut,
        "pre-self-check boundary release degraded before completion");
    bool has_degraded_result = false;
    std::uint32_t attempt_index = 0U;

    while (released_distance + config_.self_check.max_distance_error.value <
           release_distance) {
      ++attempt_index;
      if (const auto cancel =
              checkOperationCancelled("pre-self-check boundary release");
          cancel.isError()) {
        return cancel;
      }
      const double remaining_distance = release_distance - released_distance;
      const double signed_distance =
          release_direction == self_check::MotionDirection::Opening
              ? -remaining_distance
              : remaining_distance;
      const common::Mm target_stroke{
          current_nut_stroke_.value + signed_distance};
      const common::Mm actual_distance{
          std::abs(target_stroke.value - current_nut_stroke_.value)};
      const common::A release_current = preBBoundaryReleaseCurrentLimit();
      std::ostringstream start_message;
      start_message << "PreSelfCheck | phase=BoundaryRelease"
                    << " | start reason="
                    << (reason != nullptr ? reason : "directional_release")
                    << " attempt=" << attempt_index
                    << " stopped_direction=" << directionName(stopped_direction)
                    << " release_direction=" << directionName(release_direction)
                    << " current_mm=" << current_nut_stroke_.value
                    << " target_mm=" << target_stroke.value
                    << " distance_mm=" << actual_distance.value
                    << " released_total_mm=" << released_distance
                    << " speed_mm_s=" << release_speed
                    << " current_limit_a=" << release_current.value;
      reportProgress(start_message.str());

      if (actual_distance.value <= config_.self_check.max_distance_error.value) {
        reportProgress(
            "PreSelfCheck | phase=BoundaryRelease | skipped reason=no_release_room");
        return has_degraded_result ? first_degraded_result : common::Ok();
      }

    const std::uint32_t anomaly_count_before =
        friction_anomaly_detector != nullptr
            ? static_cast<std::uint32_t>(
                  friction_anomaly_detector->records().size())
            : 0U;
      const auto release_probe = runSelfCheckMotionProbe(
          release_direction, actual_distance, common::MmPerS{release_speed},
          release_current,
          preBLocalReleaseProbeOptions(friction_anomaly_detector,
                                       target_stroke));
    if (isUnsettledSelfCheckProbe(release_probe)) {
      return release_probe.result;
    }
    if (isPassThroughPreBMotionSample(release_probe)) {
      recordPreBMotionSample(release_probe, input, nullptr,
                             friction_anomaly_detector,
                             anomaly_count_before);
    }
      const double effective_release = std::min(
          release_probe.measured_distance.value, actual_distance.value);
      released_distance += std::max(0.0, effective_release);

    std::ostringstream result_message;
    result_message << "PreSelfCheck | phase=BoundaryRelease"
                   << " | result code="
                   << common::toString(release_probe.result.code())
                   << " reason="
                   << (reason != nullptr ? reason : "directional_release")
                   << " attempt=" << attempt_index
                   << " start_mm=" << release_probe.start_stroke.value
                   << " end_mm=" << release_probe.end_stroke.value
                   << " measured_mm=" << release_probe.measured_distance.value
                   << " released_total_mm=" << released_distance
                   << " current_limit_a=" << release_current.value
                   << " target_reached="
                   << (release_probe.target_reached ? "true" : "false")
                   << " hard_current_confirmed="
                   << (release_probe.hard_current_confirmed ? "true" : "false")
                   << " hard_current_no_progress="
                   << (release_probe.hard_current_no_progress_confirmed ? "true"
                                                                         : "false")
                   << " anomaly_overlap="
                   << (probeSegmentOverlapsFrictionAnomaly(
                           release_probe, friction_anomaly_detector,
                           anomaly_count_before)
                           ? "true"
                           : "false");
    if (release_probe.result.hasMessage()) {
      result_message << " message=" << release_probe.result.message();
    }
    reportProgress(result_message.str());

    const common::Result normalized_release_result =
        normalizePreBProbeResult(release_probe);
    if (normalized_release_result.code() ==
            common::ErrorCode::SelfCheckInconsistentFeedback ||
        normalized_release_result.code() == common::ErrorCode::SafetyActiveStop) {
      return normalized_release_result;
    }
    if (isSelfCheckTerminalFeedbackStop(release_probe) ||
        isPreBPhysicalBoundaryCandidate(release_probe)) {
        const bool made_release_progress =
            release_probe.sample.settled &&
            release_probe.sample.direction_matches &&
            release_probe.sample.position_monotonic &&
            release_probe.measured_distance.value >=
                preBEffectiveProgressDistance().value;
        if (made_release_progress &&
            released_distance + config_.self_check.max_distance_error.value <
                release_distance) {
          has_degraded_result = true;
          if (!first_degraded_result.hasMessage()) {
            first_degraded_result = normalized_release_result;
          }
          std::ostringstream retry_message;
          retry_message
              << "PreSelfCheck | phase=BoundaryRelease"
              << " | partial_release_continue"
              << " reason=" << (reason != nullptr ? reason : "directional_release")
              << " attempt=" << attempt_index
              << " measured_mm=" << release_probe.measured_distance.value
              << " released_total_mm=" << released_distance
              << " required_mm=" << release_distance;
          reportProgress(retry_message.str());
          continue;
        }
      std::ostringstream degraded_message;
      degraded_message
          << "PreSelfCheck | phase=BoundaryRelease"
          << " | release_failed_degraded"
          << " reason=" << (reason != nullptr ? reason : "directional_release")
          << " attempt=" << attempt_index
          << " released_total_mm=" << released_distance
          << " required_mm=" << release_distance
          << " code=" << common::toString(normalized_release_result.code())
          << " terminal_feedback_stop="
          << (isSelfCheckTerminalFeedbackStop(release_probe) ? "true" : "false")
          << " physical_boundary_candidate="
          << (isPreBPhysicalBoundaryCandidate(release_probe) ? "true" : "false");
      if (normalized_release_result.hasMessage()) {
        degraded_message << " message=" << normalized_release_result.message();
      }
      reportProgress(degraded_message.str());
      return common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "pre-self-check boundary release degraded after terminal feedback stop");
    }
    if (normalized_release_result.isError() &&
        !canDegradePreBSegment(normalized_release_result)) {
      return normalized_release_result;
    }
      if (normalized_release_result.isError()) {
        has_degraded_result = true;
        if (!first_degraded_result.hasMessage()) {
          first_degraded_result = normalized_release_result;
        }
      }
      if (release_probe.target_reached ||
          released_distance + config_.self_check.max_distance_error.value >=
              release_distance) {
    return common::Ok();
  }
    }
    return has_degraded_result ? first_degraded_result : common::Ok();
  }

  [[nodiscard]] common::Result releasePreBMechanicalBoundary(
      const PreSelfCheckExpansionBounds& expansion_bounds,
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector) {
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }
    if (!expansion_bounds.open_boundary_suspected &&
        !expansion_bounds.closed_boundary_suspected &&
        !expansion_bounds.open_release_recommended &&
        !expansion_bounds.closed_release_recommended) {
      return common::Ok();
    }

    const auto safe_zone = preSelfCheckSafeZoneFromBounds(expansion_bounds);
    if (!safe_zone.valid) {
      return common::Result::error(
          common::ErrorCode::SelfCheckFailed,
          "pre-self-check safe zone is too small for boundary release");
    }
    const double release_distance = preBBoundaryReleaseDistance().value;
    const double release_speed = preBBoundaryReleaseSpeed().value;
    if (release_distance <= 0.0 || release_speed <= 0.0) {
      return common::Result::error(
          common::ErrorCode::ConfigInvalid,
          "pre-self-check boundary release distance and speed must be positive");
    }

    const double open_distance =
        std::abs(current_nut_stroke_.value - expansion_bounds.open_limit.value);
    const double closed_distance =
        std::abs(current_nut_stroke_.value - expansion_bounds.closed_limit.value);
    const bool open_stop_detected =
        expansion_bounds.open_boundary_suspected ||
        expansion_bounds.open_release_recommended;
    const bool closed_stop_detected =
        expansion_bounds.closed_boundary_suspected ||
        expansion_bounds.closed_release_recommended;
    const bool release_from_open =
        open_stop_detected &&
        (!closed_stop_detected || open_distance <= closed_distance);
    const self_check::MotionDirection release_direction =
        release_from_open ? self_check::MotionDirection::Closing
                          : self_check::MotionDirection::Opening;
    const double signed_distance =
        release_direction == self_check::MotionDirection::Opening
            ? -release_distance
            : release_distance;
    const common::Mm target_stroke{std::clamp(
        current_nut_stroke_.value + signed_distance, safe_zone.open_limit.value,
        safe_zone.closed_limit.value)};
    const common::Mm actual_distance{
        std::abs(target_stroke.value - current_nut_stroke_.value)};
    std::ostringstream start_message;
    start_message << "PreSelfCheck | phase=BoundaryRelease"
                  << " | start release_direction="
                  << directionName(release_direction)
                  << " current_mm=" << current_nut_stroke_.value
                  << " target_mm=" << target_stroke.value
                  << " distance_mm=" << actual_distance.value
                  << " speed_mm_s=" << release_speed
                  << " current_limit_a="
                  << preBBoundaryReleaseCurrentLimit().value
                  << " open_boundary_suspected="
                  << (expansion_bounds.open_boundary_suspected ? "true" : "false")
                  << " closed_boundary_suspected="
                  << (expansion_bounds.closed_boundary_suspected ? "true"
                                                                : "false")
                  << " open_release_recommended="
                  << (expansion_bounds.open_release_recommended ? "true"
                                                                : "false")
                  << " closed_release_recommended="
                  << (expansion_bounds.closed_release_recommended ? "true"
                                                                  : "false");
    reportProgress(start_message.str());

    if (actual_distance.value <= config_.self_check.max_distance_error.value) {
      reportProgress(
          "PreSelfCheck | phase=BoundaryRelease | skipped reason=no_release_room");
      return common::Ok();
    }

    const common::A release_current = preBBoundaryReleaseCurrentLimit();
    const std::uint32_t anomaly_count_before =
        friction_anomaly_detector != nullptr
            ? static_cast<std::uint32_t>(
                  friction_anomaly_detector->records().size())
            : 0U;
    auto release_options = preBRelativeProbeOptions(
        friction_anomaly_detector, {},
        config_.self_check.pre_b_soft_jam_progress_timeout);
    release_options.command_current_cap = release_current;
    const auto release_probe = runSelfCheckMotionProbe(
        release_direction, actual_distance, common::MmPerS{release_speed},
        release_current, release_options);
    if (isUnsettledSelfCheckProbe(release_probe)) {
      return release_probe.result;
    }

    std::ostringstream result_message;
    result_message << "PreSelfCheck | phase=BoundaryRelease"
                   << " | result code="
                   << common::toString(release_probe.result.code())
                   << " start_mm=" << release_probe.start_stroke.value
                   << " end_mm=" << release_probe.end_stroke.value
                   << " measured_mm=" << release_probe.measured_distance.value
                   << " current_limit_a=" << release_current.value
                   << " target_reached="
                   << (release_probe.target_reached ? "true" : "false")
                   << " hard_current_confirmed="
                   << (release_probe.hard_current_confirmed ? "true" : "false")
                   << " hard_current_no_progress="
                   << (release_probe.hard_current_no_progress_confirmed ? "true"
                                                                         : "false")
                   << " anomaly_overlap="
                   << (probeSegmentOverlapsFrictionAnomaly(
                           release_probe, friction_anomaly_detector,
                           anomaly_count_before)
                           ? "true"
                           : "false");
    if (release_probe.result.hasMessage()) {
      result_message << " message=" << release_probe.result.message();
    }
    reportProgress(result_message.str());

    const common::Result normalized_release_result =
        normalizePreBProbeResult(release_probe);
    if (normalized_release_result.code() ==
            common::ErrorCode::SelfCheckInconsistentFeedback ||
        normalized_release_result.code() == common::ErrorCode::SafetyActiveStop) {
      return normalized_release_result;
    }
    if (isSelfCheckTerminalFeedbackStop(release_probe) ||
        isPreBPhysicalBoundaryCandidate(release_probe)) {
      std::ostringstream degraded_message;
      degraded_message
          << "PreSelfCheck | phase=BoundaryRelease"
          << " | release_failed_degraded"
          << " code=" << common::toString(normalized_release_result.code())
          << " terminal_feedback_stop="
          << (isSelfCheckTerminalFeedbackStop(release_probe) ? "true" : "false")
          << " physical_boundary_candidate="
          << (isPreBPhysicalBoundaryCandidate(release_probe) ? "true" : "false");
      if (normalized_release_result.hasMessage()) {
        degraded_message << " message=" << normalized_release_result.message();
      }
      reportProgress(degraded_message.str());
      return common::Result::error(
          common::ErrorCode::OperationTimedOut,
          "pre-self-check boundary release degraded after terminal feedback stop");
    }
    if (normalized_release_result.isError() &&
        !canDegradePreBSegment(normalized_release_result)) {
      return normalized_release_result;
    }
    return common::Ok();
  }

  [[nodiscard]] bool acceptPreBLearningProbe(
      const SelfCheckMotionProbeResult& probe,
      self_check::MotionDirection direction, self_check::SelfCheckInput* input,
      const self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      std::uint32_t anomaly_record_count_before,
      std::uint32_t* accepted_samples) {
    if (input == nullptr) {
      return false;
    }
    const std::uint32_t anomaly_record_count_after =
        friction_anomaly_detector != nullptr
            ? static_cast<std::uint32_t>(
                  friction_anomaly_detector->records().size())
            : anomaly_record_count_before;
    if (probeSegmentOverlapsFrictionAnomaly(probe, friction_anomaly_detector) ||
        probeSegmentOverlapsFrictionAnomaly(probe, friction_anomaly_detector,
                                            anomaly_record_count_before)) {
      std::ostringstream message;
      message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
              << " | sample_skipped reason=friction_anomaly_overlap"
              << " direction=" << directionName(direction)
              << " start_mm=" << probe.start_stroke.value
              << " end_mm=" << probe.end_stroke.value
              << " records_before=" << anomaly_record_count_before
              << " records_after=" << anomaly_record_count_after
              << " avoid_margin_mm=" << frictionAnomalyAvoidMargin()
              << " avoid_ratio=" << frictionAnomalyLearningAvoidRatio();
      reportProgress(message.str());
      return false;
    }
    if (!isUsablePreBMotionSample(probe.sample)) {
      return false;
    }
    input->motion_samples.push_back(probe.sample);
    input->static_friction_samples.push_back(
        makeStaticFrictionSample(direction, probe.sample));
    input->dynamic_friction_samples.push_back(makeDynamicFrictionSample(
        direction, probe.sample));
    if (accepted_samples != nullptr) {
      ++(*accepted_samples);
    }
    return true;
  }

  [[nodiscard]] common::Result runPreBLearningProbe(
      self_check::MotionDirection direction, common::Mm sample_distance,
      common::MmPerS speed, self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      std::uint32_t* accepted_samples) {
    if (plannedSegmentOverlapsFrictionAnomaly(
            current_nut_stroke_, direction, sample_distance,
            friction_anomaly_detector)) {
      std::ostringstream message;
      message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
              << " | planned_sample_skipped reason=friction_anomaly_overlap"
              << " direction=" << directionName(direction)
              << " start_mm=" << current_nut_stroke_.value
              << " distance_mm=" << sample_distance.value
              << " avoid_margin_mm=" << frictionAnomalyAvoidMargin()
              << " avoid_ratio=" << frictionAnomalyLearningAvoidRatio();
      reportProgress(message.str());
      return common::Ok();
    }

    const std::uint32_t anomaly_count_before =
        friction_anomaly_detector != nullptr
            ? static_cast<std::uint32_t>(
                  friction_anomaly_detector->records().size())
            : 0U;
    const auto probe_result = probeMotion(
        direction, sample_distance, speed, config_.safety.self_check_current_limit,
        preBRelativeProbeOptions(friction_anomaly_detector));
    if (isUnsettledSelfCheckProbe(last_self_check_probe_)) {
      return probe_result;
    }
    const common::Result normalized_probe_result =
        normalizePreBProbeResult(last_self_check_probe_);
    if (normalized_probe_result.isError() &&
        !canDegradePreBSegment(normalized_probe_result)) {
      return normalized_probe_result;
    }
    (void)acceptPreBLearningProbe(last_self_check_probe_, direction, input,
                                  friction_anomaly_detector,
                                  anomaly_count_before, accepted_samples);
    return common::Ok();
  }

  [[nodiscard]] ManualPositioningRange preSelfCheckSafeZoneFromBounds(
      const PreSelfCheckExpansionBounds& expansion_bounds) const {
    ManualPositioningRange range{};
    range.use_software_limits = false;
    range.low_confidence_window = true;
    range.confidence = "pre_self_check_continuous_pre_b";

    double raw_open =
        std::min(expansion_bounds.open_limit.value,
                 expansion_bounds.closed_limit.value);
    double raw_closed =
        std::max(expansion_bounds.open_limit.value,
                 expansion_bounds.closed_limit.value);

    const double raw_width = raw_closed - raw_open;
    const double minimum_safe_width = std::max(
        config_.self_check.min_measured_distance.value,
        config_.self_check.motion_start_distance.value * 2.0);
    if (raw_width < minimum_safe_width) {
      return range;
    }

    // PreB bounds are low-confidence observed bounds. Keep the configured
    // margin when there is room, but reduce it instead of falling back to the
    // full theoretical range when the gripper starts close to one end stop.
    const double configured_margin =
        std::max(0.0, config_.self_check.safe_zone_margin.value);
    const double adaptive_margin =
        std::min(configured_margin,
                 std::max(0.0, (raw_width - minimum_safe_width) * 0.5));
    range.open_limit = common::Mm{raw_open + adaptive_margin};
    range.closed_limit = common::Mm{raw_closed - adaptive_margin};
    range.valid =
        range.closed_limit.value - range.open_limit.value >=
        minimum_safe_width;
    return range;
  }

  void applyPreSelfCheckExpansionBoundsToProfile(
      const PreSelfCheckExpansionBounds& expansion_bounds,
      self_check::StructureProfile* profile) const {
    if (profile == nullptr) {
      return;
    }

    double observed_open =
        std::min(expansion_bounds.open_limit.value,
                 expansion_bounds.closed_limit.value);
    double observed_closed =
        std::max(expansion_bounds.open_limit.value,
                 expansion_bounds.closed_limit.value);

    const double observed_width = observed_closed - observed_open;
    const double minimum_software_width = std::max(
        config_.self_check.min_measured_distance.value,
        config_.self_check.motion_start_distance.value * 2.0);
    if (observed_width < minimum_software_width) {
      return;
    }

    profile->travel_limits.preliminary_open_limit = common::Mm{observed_open};
    profile->travel_limits.preliminary_closed_limit =
        common::Mm{observed_closed};
    profile->travel_limits.learned_travel = common::Mm{observed_width};
    const double theoretical_open = config_.mechanism.theoretical_open_limit.value;
    const double theoretical_closed =
        config_.mechanism.theoretical_close_limit.value;
    profile->travel_limits.theoretical_travel_error =
        common::Mm{std::abs((theoretical_closed - theoretical_open) -
                            observed_width)};
    profile->travel_limits.valid_limit_sample_count = 2;
    profile->travel_limits.quality = self_check::IdentificationQuality::LowConfidence;

    const auto safe_zone = preSelfCheckSafeZoneFromBounds(expansion_bounds);
    if (safe_zone.valid) {
      profile->travel_limits.safe_zone_open_limit = safe_zone.open_limit;
      profile->travel_limits.safe_zone_closed_limit = safe_zone.closed_limit;
    } else {
      profile->travel_limits.safe_zone_open_limit = common::Mm{observed_open};
      profile->travel_limits.safe_zone_closed_limit = common::Mm{observed_closed};
    }

    const double software_margin =
        std::min(std::max(0.0, config_.self_check.software_limit_margin.value),
                 std::max(0.0,
                          (observed_width - minimum_software_width) * 0.5));
    profile->travel_limits.software_open_limit =
        common::Mm{observed_open + software_margin};
    profile->travel_limits.software_closed_limit =
        common::Mm{observed_closed - software_margin};
  }

  [[nodiscard]] common::Result runSafeZoneRoundTripSamples(
      self_check::SelfCheckInput* input,
      self_check::FrictionAnomalyDetector* friction_anomaly_detector,
      const PreSelfCheckExpansionBounds& expansion_bounds) {
    if (input == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "self-check input pointer is null");
    }
    const auto safe_zone = preSelfCheckSafeZoneFromBounds(expansion_bounds);
    if (!safe_zone.valid) {
      return common::Result::error(common::ErrorCode::SelfCheckFailed,
                                   "pre-self-check safe zone is too small for round-trip sampling");
    }
    const std::size_t motion_sample_count_before =
        input->motion_samples.size();
    const std::size_t static_sample_count_before =
        input->static_friction_samples.size();
    const std::size_t dynamic_sample_count_before =
        input->dynamic_friction_samples.size();
    const auto move_to_safe_open = moveContinuouslyToPreBStroke(
        safe_zone.open_limit, safe_zone,
        "return_to_open_soft_limit_before_learning");
    if (move_to_safe_open.isError()) {
      return move_to_safe_open;
    }

    const double safe_zone_width =
        safe_zone.closed_limit.value - safe_zone.open_limit.value;
    const common::Mm sample_distance{
        std::min(safe_zone_width * 0.25,
                 std::max(config_.self_check.min_measured_distance.value,
                          config_.self_check.max_probe_window.value))};
    if (sample_distance.value < config_.self_check.min_measured_distance.value) {
      return common::Result::error(
          common::ErrorCode::SelfCheckFailed,
          "pre-self-check safe zone is narrower than the minimum measured distance");
    }
    const double speed_cap = preBDynamicFrictionSpeedCap();
    const std::uint32_t minimum_learning_regions =
        std::max<std::uint32_t>(
            3U, config_.self_check.pre_b_min_learning_regions);
    const std::vector<PreBLearningAnchorCandidate> preferred_anchors =
        buildPreBLearningAnchorCandidates(safe_zone, sample_distance,
                                          minimum_learning_regions);
    std::vector<PreBLearningAnchorCandidate> anchors;
    anchors.reserve(preferred_anchors.size());
    const double duplicate_anchor_distance =
        std::max(config_.self_check.max_distance_error.value,
                 sample_distance.value * 0.5);
    for (const auto& preferred_anchor : preferred_anchors) {
      const auto region_range = preBLearningRegionRange(
          safe_zone, preferred_anchor.region_bucket, minimum_learning_regions);
      if (!region_range.valid ||
          region_range.closed_limit.value - region_range.open_limit.value <
              sample_distance.value * 2.0) {
        std::ostringstream message;
        message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                << " | region_skipped reason=insufficient_region_room"
                << " region_bucket=" << preferred_anchor.region_bucket
                << " region_open_mm=" << region_range.open_limit.value
                << " region_closed_mm=" << region_range.closed_limit.value
                << " sample_distance_mm=" << sample_distance.value;
        reportProgress(message.str());
        continue;
      }
      const auto clean_anchor = cleanLearningAnchorNear(
          preferred_anchor.anchor, region_range, safe_zone, sample_distance,
          friction_anomaly_detector);
      if (clean_anchor.has_value()) {
        const auto duplicate = std::find_if(
            anchors.begin(), anchors.end(),
            [clean_anchor, duplicate_anchor_distance,
             preferred_anchor](const PreBLearningAnchorCandidate& existing) {
              return existing.region_bucket == preferred_anchor.region_bucket &&
                     std::abs(existing.anchor.value - clean_anchor->value) <
                     duplicate_anchor_distance;
            });
        if (duplicate == anchors.end()) {
          anchors.push_back(
              PreBLearningAnchorCandidate{*clean_anchor,
                                          preferred_anchor.region_bucket});
        }
      } else {
        std::ostringstream message;
        message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                << " | region_skipped reason=friction_anomaly_coverage"
                << " region_bucket=" << preferred_anchor.region_bucket
                << " preferred_mm=" << preferred_anchor.anchor.value
                << " region_open_mm=" << region_range.open_limit.value
                << " region_closed_mm=" << region_range.closed_limit.value
                << " avoid_margin_mm=" << frictionAnomalyAvoidMargin()
                << " avoid_ratio=" << frictionAnomalyLearningAvoidRatio();
        reportProgress(message.str());
      }
    }
    if (anchors.empty()) {
      return common::Result::error(
          common::ErrorCode::SelfCheckFailed,
          "no clean safe-zone anchor remains after friction anomaly filtering");
    }

    std::ostringstream start_message;
    start_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                  << " | safe_zone open_mm=" << safe_zone.open_limit.value
                  << " closed_mm=" << safe_zone.closed_limit.value
                  << " sample_distance_mm=" << sample_distance.value
                  << " reposition_speed_mm_s=" << preBExpansionSpeed().value
                  << " speed_cap_mm_s=" << speed_cap
                  << " preferred_anchor_count=" << preferred_anchors.size()
                  << " clean_anchor_count=" << anchors.size()
                  << " min_learning_regions=" << minimum_learning_regions
                  << " anomaly_avoid_margin_mm=" << frictionAnomalyAvoidMargin()
                  << " anomaly_avoid_ratio="
                  << frictionAnomalyLearningAvoidRatio()
                  << " open_expansion_samples="
                  << expansion_bounds.open_sample_count
                  << " closed_expansion_samples="
                  << expansion_bounds.closed_sample_count;
    reportProgress(start_message.str());

    std::uint32_t accepted_samples = 0;
    std::set<std::size_t> accepted_regions;
    std::uint32_t anchor_index = 0;
    for (const auto& anchor_candidate : anchors) {
      ++anchor_index;
      const common::Mm anchor = anchor_candidate.anchor;
      if (anchor.value < safe_zone.open_limit.value ||
          anchor.value > safe_zone.closed_limit.value) {
        continue;
      }
      const std::uint32_t accepted_before_anchor = accepted_samples;
      const std::size_t region_bucket = anchor_candidate.region_bucket;
      std::ostringstream anchor_message;
      anchor_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                     << " | anchor_start index=" << anchor_index
                     << " anchor_mm=" << anchor.value
                     << " region_bucket=" << region_bucket;
      reportProgress(anchor_message.str());

      const auto move_to_anchor =
          moveToSelfCheckStroke(anchor, input, friction_anomaly_detector);
      if (move_to_anchor.isError()) {
        if (canDegradePreBSegment(move_to_anchor)) {
          std::ostringstream message;
          message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                  << " | anchor_skipped reason=reposition_degraded"
                  << " index=" << anchor_index
                  << " anchor_mm=" << anchor.value
                  << " message=" << move_to_anchor.message();
          reportProgress(message.str());
          continue;
        }
        return move_to_anchor;
      }

      for (const auto& speed : config_.self_check.dynamic_friction_speeds) {
        if (const auto cancel =
                checkOperationCancelled("safe-zone round-trip samples");
            cancel.isError()) {
          return cancel;
        }
        const double abs_speed = std::abs(speed.value);
        if (abs_speed <= 0.0 ||
            abs_speed > speed_cap) {
          continue;
        }

        const double closing_room =
            safe_zone.closed_limit.value - current_nut_stroke_.value;
        if (closing_room >= sample_distance.value) {
          const auto close_result = runPreBLearningProbe(
              self_check::MotionDirection::Closing, sample_distance,
              common::MmPerS{abs_speed}, input, friction_anomaly_detector,
              &accepted_samples);
          if (close_result.isError()) {
            return close_result;
          }
        }

        const double opening_room =
            current_nut_stroke_.value - safe_zone.open_limit.value;
        if (opening_room >= sample_distance.value) {
          const auto open_result = runPreBLearningProbe(
              self_check::MotionDirection::Opening, sample_distance,
              common::MmPerS{abs_speed}, input, friction_anomaly_detector,
              &accepted_samples);
          if (open_result.isError()) {
            return open_result;
          }
        }
      }
      const std::uint32_t anchor_accepted_samples =
          accepted_samples - accepted_before_anchor;
      if (anchor_accepted_samples > 0U) {
        accepted_regions.insert(region_bucket);
      }
      std::ostringstream result_message;
      result_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                     << " | anchor_result index=" << anchor_index
                     << " anchor_mm=" << anchor.value
                     << " region_bucket=" << region_bucket
                     << " accepted_samples=" << anchor_accepted_samples
                     << " learned_regions=" << accepted_regions.size();
      reportProgress(result_message.str());
    }

    if (accepted_regions.size() < minimum_learning_regions) {
      input->motion_samples.resize(motion_sample_count_before);
      input->static_friction_samples.resize(static_sample_count_before);
      input->dynamic_friction_samples.resize(dynamic_sample_count_before);
      std::ostringstream message;
      message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
              << " | insufficient_learning_regions"
              << " accepted_samples=" << accepted_samples
              << " learned_regions=" << accepted_regions.size()
              << " required_regions=" << minimum_learning_regions
              << " clean_anchor_count=" << anchors.size()
              << " mechanism_anomaly=true"
              << " action=rollback_final_model_samples";
      reportProgress(message.str());
      pre_b_mechanism_anomaly_ = true;
      return common::Result::error(
          common::ErrorCode::SelfCheckFailed,
          "pre-self-check did not learn enough clean safe-zone regions");
    }

    std::ostringstream complete_message;
    complete_message << "PreSelfCheck | phase=MultiRegionRoundTripLearning"
                     << " | completed accepted_samples=" << accepted_samples
                     << " learned_regions=" << accepted_regions.size()
                     << " required_regions=" << minimum_learning_regions;
    reportProgress(complete_message.str());
    return common::Ok();
  }

  void collectFeedbackNoiseSamples(self_check::SelfCheckInput* input) {
    if (input == nullptr) {
      return;
    }
    if (operationCancelRequested()) {
      return;
    }
    reportProgress("PreSelfCheck | phase=LimitedProbe | collecting still feedback noise");
    const auto initial = updateFeedback();
    if (initial.isError()) {
      input->noise_samples.push_back(makeFallbackNoiseSample());
      return;
    }
    const common::Rad reference_position = last_feedback_.position;
    const common::Mm reference_stroke = current_nut_stroke_;
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>{
                std::max(0.02, config_.self_check.feedback_noise_sample_time.value)});
    std::uint32_t sample_count = 0;
    while (std::chrono::steady_clock::now() < deadline) {
      if (operationCancelRequested()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
      if (updateFeedback().isError()) {
        break;
      }
      input->noise_samples.push_back(self_check::FeedbackNoiseSample{
          common::Rad{last_feedback_.position.value - reference_position.value},
          last_feedback_.velocity,
          last_feedback_.current,
          last_feedback_.torque,
          common::Mm{current_nut_stroke_.value - reference_stroke.value}});
      ++sample_count;
    }
    if (sample_count == 0U) {
      input->noise_samples.push_back(makeFallbackNoiseSample());
    }
  }

  [[nodiscard]] self_check::FeedbackNoiseSample makeFallbackNoiseSample() const {
    return self_check::FeedbackNoiseSample{
        config_.self_check.fallback_motor_position_noise,
        config_.self_check.fallback_motor_velocity_noise,
        config_.self_check.fallback_motor_current_noise,
        config_.self_check.fallback_motor_torque_noise,
        config_.self_check.fallback_nut_stroke_noise};
  }

  [[nodiscard]] self_check::StaticFrictionSample makeStaticFrictionSample(
      self_check::MotionDirection direction,
      const self_check::MotionIdentificationSample& sample) const {
    return self_check::StaticFrictionSample{
        direction,
        sample.average_motor_current,
        sample.average_motor_torque,
        sample.measured_distance.value >= config_.self_check.min_measured_distance.value,
        true,
        sample.limit_triggered};
  }

  [[nodiscard]] self_check::DynamicFrictionSample makeDynamicFrictionSample(
      self_check::MotionDirection direction,
      const self_check::MotionIdentificationSample& sample) const {
    return self_check::DynamicFrictionSample{
        direction,
        sample.average_nut_speed,
        config_.self_check.min_speed_scan_start,
        sample.average_motor_current,
        sample.max_motor_current,
        sample.average_motor_torque,
        sample.max_motor_torque,
        true,
        sample.velocity_stable,
        sample.current_stable,
        sample.limit_triggered,
        sample.jam_detected};
  }

  [[nodiscard]] self_check::MotionHealthSample makeMotionHealthSample(
      const MotionWaitResult& motion,
      common::MmPerS target_nut_speed) const {
    const bool inside_limits =
        motion.start_stroke.value >=
            profile_.travel_limits.software_open_limit.value &&
        motion.start_stroke.value <=
            profile_.travel_limits.software_closed_limit.value &&
        motion.end_stroke.value >=
            profile_.travel_limits.software_open_limit.value &&
        motion.end_stroke.value <=
            profile_.travel_limits.software_closed_limit.value;
    const auto direction = target_nut_speed.value >= 0.0
                               ? self_check::MotionDirection::Closing
                               : self_check::MotionDirection::Opening;
    const double signed_delta =
        motion.end_stroke.value - motion.start_stroke.value;
    const bool position_monotonic =
        direction == self_check::MotionDirection::Closing
            ? signed_delta >= -config_.self_check.max_distance_error.value
            : signed_delta <= config_.self_check.max_distance_error.value;
    return self_check::MotionHealthSample{
        direction,
        motion.start_stroke,
        motion.end_stroke,
        common::MmPerS{std::abs(target_nut_speed.value)},
        motion.max_velocity_tracking_error,
        motion.max_current_ripple,
        motion.max_torque_ripple,
        motion.max_motor_temperature,
        inside_limits,
        position_monotonic,
        true,
        motion.jam_detected,
        false,
    };
  }

  void updateSnapshotFlags() {
    std::lock_guard<std::mutex> lock{snapshot_mutex_};
    const auto nut_encoder_feedback = nut_position_encoder_.feedback();
    snapshot_.top_state = state_machine_.state();
    snapshot_.pre_self_check_phase = pre_self_check_state_machine_.phase();
    snapshot_.structure_profile_validity = profile_.validity;
    snapshot_.configured = configured_;
    snapshot_.connected = motor_ && motor_->isConnected();
    snapshot_.enabled = motor_ && motor_->isEnabled();
    snapshot_.motor_bringup_active = motor_bringup_active_;
    snapshot_.motor_bringup_unloaded_confirmed =
        motor_bringup_unloaded_confirmed_;
    snapshot_.pre_self_check_completed = pre_self_check_completed_;
    snapshot_.pre_b_mechanism_anomaly = pre_b_mechanism_anomaly_;
    snapshot_.homed = homed_;
    snapshot_.travel_limits_learned = travel_limits_learned_;
    snapshot_.motion_health_checked = motion_health_checked_;
    snapshot_.contact_detected = contact_detected_;
    snapshot_.nut_stroke = current_nut_stroke_;
    snapshot_.gripper_angle = kinematics_.strokeToAngle(current_nut_stroke_);
    snapshot_.estimated_clamp_force = estimated_clamp_force_;
    snapshot_.motor.position = last_feedback_.position;
    snapshot_.motor.wrapped_position = last_feedback_.wrapped_position;
    snapshot_.motor.wrapped_position_valid =
        last_feedback_.wrapped_position_valid;
    snapshot_.motor.velocity = last_feedback_.velocity;
    snapshot_.motor.current = last_feedback_.current;
    snapshot_.motor.torque = last_feedback_.torque;
    snapshot_.motor.temperature = last_feedback_.temperature;
    snapshot_.motor.raw_position_counts = last_feedback_.raw_position_counts;
    snapshot_.motor.raw_position_counts_valid =
        last_feedback_.raw_position_counts_valid;
    snapshot_.motor.raw_feedback_frame_id =
        last_feedback_.raw_feedback_frame_id;
    snapshot_.motor.raw_feedback_frame_length =
        last_feedback_.raw_feedback_frame_length;
    snapshot_.motor.raw_feedback_frame_data =
        last_feedback_.raw_feedback_frame_data;
    snapshot_.motor.raw_feedback_frame_valid =
        last_feedback_.raw_feedback_frame_valid;
    snapshot_.motor.runtime_position_limit =
        last_feedback_.runtime_position_limit;
    snapshot_.motor.runtime_velocity_limit =
        last_feedback_.runtime_velocity_limit;
    snapshot_.motor.runtime_torque_limit =
        last_feedback_.runtime_torque_limit;
    snapshot_.motor.runtime_limits_valid =
        last_feedback_.runtime_limits_valid;
    snapshot_.motor.enabled = motor_ && motor_->isEnabled();
    snapshot_.motor.fault = last_feedback_.fault;
    snapshot_.nut_encoder.nut_position =
        nut_encoder_feedback.nut_position;
    snapshot_.nut_encoder.nut_velocity =
        nut_encoder_feedback.nut_velocity;
    snapshot_.nut_encoder.zero_motor_position =
        nut_encoder_feedback.zero_motor_position;
    snapshot_.nut_encoder.zero_nut_position =
        nut_encoder_feedback.zero_nut_position;
    snapshot_.nut_encoder.motor_position =
        nut_encoder_feedback.motor_position;
    snapshot_.nut_encoder.motor_delta =
        nut_encoder_feedback.motor_delta;
    snapshot_.nut_encoder.motor_delta_revolutions =
        nut_encoder_feedback.motor_delta_revolutions;
    snapshot_.nut_encoder.millimeters_per_revolution_estimate =
        nut_encoder_feedback.millimeters_per_revolution_estimate;
    snapshot_.nut_encoder.fresh =
        nut_position_encoder_.isFresh(last_feedback_.timestamp);
    snapshot_.pre_b_current_trace = pre_b_current_trace_points_;
    snapshot_.learned_parameters.opening.breakaway_current =
        profile_.opening.bootstrap_breakaway_current;
    snapshot_.learned_parameters.opening.breakaway_torque =
        profile_.opening.bootstrap_breakaway_torque;
    snapshot_.learned_parameters.opening.static_friction_current =
        profile_.opening.static_friction_current;
    snapshot_.learned_parameters.opening.static_friction_torque =
        profile_.opening.static_friction_torque;
    snapshot_.learned_parameters.opening.minimum_stable_nut_speed =
        profile_.opening.stable_speed_sample_count > 0U
            ? profile_.opening.minimum_stable_nut_speed
            : common::MmPerS{};
    snapshot_.learned_parameters.opening.dynamic_friction_current_average =
        profile_.opening.dynamic_friction_current_average;
    snapshot_.learned_parameters.opening.dynamic_friction_current_max =
        profile_.opening.dynamic_friction_current_max;
    snapshot_.learned_parameters.opening.dynamic_friction_torque_average =
        profile_.opening.dynamic_friction_torque_average;
    snapshot_.learned_parameters.opening.dynamic_friction_torque_max =
        profile_.opening.dynamic_friction_torque_max;
    snapshot_.learned_parameters.opening.breakaway_sample_count =
        profile_.opening.bootstrap_breakaway_sample_count;
    snapshot_.learned_parameters.opening.static_friction_sample_count =
        profile_.opening.static_friction_sample_count;
    snapshot_.learned_parameters.opening.stable_speed_sample_count =
        profile_.opening.stable_speed_sample_count;
    snapshot_.learned_parameters.opening.dynamic_friction_sample_count =
        profile_.opening.dynamic_friction_sample_count;
    snapshot_.learned_parameters.closing.breakaway_current =
        profile_.closing.bootstrap_breakaway_current;
    snapshot_.learned_parameters.closing.breakaway_torque =
        profile_.closing.bootstrap_breakaway_torque;
    snapshot_.learned_parameters.closing.static_friction_current =
        profile_.closing.static_friction_current;
    snapshot_.learned_parameters.closing.static_friction_torque =
        profile_.closing.static_friction_torque;
    snapshot_.learned_parameters.closing.minimum_stable_nut_speed =
        profile_.closing.stable_speed_sample_count > 0U
            ? profile_.closing.minimum_stable_nut_speed
            : common::MmPerS{};
    snapshot_.learned_parameters.closing.dynamic_friction_current_average =
        profile_.closing.dynamic_friction_current_average;
    snapshot_.learned_parameters.closing.dynamic_friction_current_max =
        profile_.closing.dynamic_friction_current_max;
    snapshot_.learned_parameters.closing.dynamic_friction_torque_average =
        profile_.closing.dynamic_friction_torque_average;
    snapshot_.learned_parameters.closing.dynamic_friction_torque_max =
        profile_.closing.dynamic_friction_torque_max;
    snapshot_.learned_parameters.closing.breakaway_sample_count =
        profile_.closing.bootstrap_breakaway_sample_count;
    snapshot_.learned_parameters.closing.static_friction_sample_count =
        profile_.closing.static_friction_sample_count;
    snapshot_.learned_parameters.closing.stable_speed_sample_count =
        profile_.closing.stable_speed_sample_count;
    snapshot_.learned_parameters.closing.dynamic_friction_sample_count =
        profile_.closing.dynamic_friction_sample_count;
    snapshot_.learned_parameters.safe_zone_open_limit =
        profile_.travel_limits.safe_zone_open_limit;
    snapshot_.learned_parameters.safe_zone_closed_limit =
        profile_.travel_limits.safe_zone_closed_limit;
    snapshot_.learned_parameters.software_open_limit =
        profile_.travel_limits.software_open_limit;
    snapshot_.learned_parameters.software_closed_limit =
        profile_.travel_limits.software_closed_limit;
    snapshot_.learned_parameters.learned_travel =
        profile_.travel_limits.learned_travel;
    snapshot_.friction_anomaly = {};
    snapshot_.friction_anomaly.record_count =
        static_cast<std::uint32_t>(friction_anomaly_records_.size());
    if (const auto* severe_record = mostSevereFrictionAnomalyRecord();
        severe_record != nullptr) {
      snapshot_.friction_anomaly.max_current_excess_ratio =
          severe_record->current_excess_ratio;
      snapshot_.friction_anomaly.severe_center_position =
          severe_record->center_position;
      snapshot_.friction_anomaly.severe_width = severe_record->width;
      snapshot_.friction_anomaly.severe_peak_current =
          severe_record->peak_current;
      snapshot_.friction_anomaly.severe_baseline_current =
          severe_record->baseline_current;
      snapshot_.friction_anomaly.has_severe_record =
          severe_record->severity ==
          self_check::FrictionAnomalySeverity::Severe;
    }
    const auto manual_range = currentManualPositioningRange();
    snapshot_.manual_nut_stroke_range.open_limit = manual_range.open_limit;
    snapshot_.manual_nut_stroke_range.closed_limit = manual_range.closed_limit;
    snapshot_.manual_nut_stroke_range.valid = manual_range.valid;
    snapshot_.manual_nut_stroke_range.use_software_limits =
        manual_range.use_software_limits;
    snapshot_.manual_nut_stroke_range.low_confidence_window =
        manual_range.low_confidence_window;
    snapshot_.manual_nut_stroke_range.confidence = manual_range.confidence;
  }

  void resetOperationFlagsForDisconnect() {
    pre_self_check_completed_ = false;
    homed_ = false;
    travel_limits_learned_ = false;
    motion_health_checked_ = false;
    motor_bringup_active_ = false;
    motor_bringup_unloaded_confirmed_ = false;
    pre_b_mechanism_anomaly_ = false;
    contact_detected_ = false;
    estimated_clamp_force_ = {};
    current_nut_stroke_ = {};
    stroke_reference_valid_ = false;
    stroke_reference_motor_position_ = {};
    stroke_reference_nut_stroke_ = {};
    nut_position_encoder_.configure(makeNutPositionEncoderConfig(
        config_, provisionalReferenceStroke()));
    motor_zero_reference_valid_ = false;
    motor_zero_reference_ = {};
    pre_b_current_trace_points_.clear();
    pre_b_trace_decimation_counter_ = 0U;
    pre_b_current_trace_segment_id_ = 0U;
    updateSnapshotFlags();
  }

  common::Result finish(const common::Result& result) {
    {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      snapshot_.last_result_code = common::toValue(result.code());
    }
    updateSnapshotFlags();
    return result;
  }

  common::Result fail(const common::Result& result) {
    {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      snapshot_.last_result_code = common::toValue(result.code());
    }
    if (result.code() == common::ErrorCode::SafetyActiveStop ||
        result.code() == common::ErrorCode::SafetyJamDetected ||
        result.code() == common::ErrorCode::SafetyContactDetected) {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      snapshot_.fault.severity = state_machine::FaultSeverity::Recoverable;
      snapshot_.fault.source = state_machine::FaultSource::Safety;
      snapshot_.fault.code = common::toValue(result.code());
    }
    updateSnapshotFlags();
    return result;
  }

  common::Result finishFailureWithoutFault(const common::Result& result) {
    {
      std::lock_guard<std::mutex> lock{snapshot_mutex_};
      snapshot_.last_result_code = common::toValue(result.code());
    }
    updateSnapshotFlags();
    return result;
  }

  common::Result fail(common::ErrorCode code, std::string message) {
    return fail(common::Result::error(code, std::move(message)));
  }

  std::unique_ptr<hardware_interface::MotorInterface> motor_{};
  config::GripperConfig config_{};
  std::atomic_bool operation_cancel_requested_{false};
  bool configured_{false};
  bool pre_self_check_completed_{false};
  bool pre_self_check_opening_boundary_suspected_{false};
  bool pre_self_check_closing_boundary_suspected_{false};
  bool pre_b_mechanism_anomaly_{false};
  bool homed_{false};
  bool travel_limits_learned_{false};
  bool motion_health_checked_{false};
  bool motor_bringup_active_{false};
  bool motor_bringup_unloaded_confirmed_{false};
  bool contact_detected_{false};
  common::Mm current_nut_stroke_{};
  bool stroke_reference_valid_{false};
  common::Rad stroke_reference_motor_position_{};
  common::Mm stroke_reference_nut_stroke_{};
  LeadScrewNutPositionEncoder nut_position_encoder_{};
  bool motor_zero_reference_valid_{false};
  common::Rad motor_zero_reference_{};
  common::N estimated_clamp_force_{};
  common::MmPerS previous_commanded_nut_speed_{};
  MotorFeedback last_feedback_{};
  SelfCheckMotionProbeResult last_self_check_probe_{};
  self_check::MotionIdentificationSample last_motion_sample_{};
  std::vector<self_check::FrictionAnomalyRecord> friction_anomaly_records_{};
  std::vector<PreBCurrentTracePoint> pre_b_current_trace_points_{};
  std::uint64_t pre_b_trace_decimation_counter_{0U};
  std::uint32_t pre_b_current_trace_segment_id_{0U};
  ProgressCallback progress_callback_{};
  mutable std::mutex snapshot_mutex_{};
  GripperStateSnapshot snapshot_{};
  state_machine::GripperStateMachine state_machine_{};
  state_machine::PreSelfCheckStateMachine pre_self_check_state_machine_{};
  self_check::StructureProfile profile_{};
  safety::SafetyLimiter safety_limiter_{};
  safety::ContactJamDetector contact_detector_{};
  mechanism::GripperKinematics kinematics_{};
  calibration::ForceMapper force_mapper_{};
};

std::unique_ptr<GripperController> createGripperController(
    std::unique_ptr<hardware_interface::MotorInterface> motor) {
  return std::make_unique<DefaultGripperController>(std::move(motor));
}

}  // namespace gripper::controller
