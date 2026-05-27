#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "common/units.hpp"
#include "controller/state_machine/fault_state.hpp"
#include "controller/state_machine/gripper_state_machine.hpp"
#include "controller/state_machine/self_check_state.hpp"

namespace gripper::controller {

enum class StructureProfileValidity : std::uint8_t {
  Unknown,
  ConservativeDefaults,
  PreSelfCheckCompleted,
  Homed,
  TravelLimitsLearned,
  MotionHealthChecked,
};

enum class ContactDetectionPolicy : std::uint8_t {
  CurrentRiseAndVelocityDrop,
  TorqueRiseAndVelocityDrop,
  StrokeContextAndMotorFeedback,
  DisabledForDebug,
};

enum class CommandSource : std::uint8_t {
  Unknown,
  Ui,
  Commander,
  Programmatic,
  Safety,
};

enum class ClampSpeedMode : std::uint8_t {
  // Keep nut linear speed constant. Gripper angular speed will vary with
  // mechanism geometry.
  NutLinearSpeed,
  // Keep target gripper angular speed approximately constant by converting
  // omega to nut speed through the mechanism model.
  GripperAngularSpeed,
};

struct MotorBringupSessionRequest {
  // Bring-up jog commands are intended for first hardware communication tests.
  // The caller must confirm that the motor is unloaded or the mechanism has
  // been mechanically made safe before any motion command is accepted.
  bool unloaded_or_structure_removed_confirmed{false};
  CommandSource source{CommandSource::Unknown};
};

struct MotorBringupJogCommand {
  // Relative motor-side motion. Positive/negative direction is intentionally
  // motor-side so the first test can establish actual mechanism direction.
  common::Rad relative_motor_position{};
  // Used only when relative_motor_position is zero and the controller falls
  // back to the configured default pulse distance.
  int motor_direction_sign{1};
  common::RadPerS max_motor_velocity{};
  common::A max_motor_current{};
  common::S pulse_duration{};
  CommandSource source{CommandSource::Unknown};
};

struct MotorBringupPositionMoveCommand {
  // Relative motor-side revolutions for unloaded encoder/lead-screw mapping
  // checks. Positive/negative direction remains motor-side.
  common::Ratio relative_motor_revolutions{};
  common::RadPerS max_motor_velocity{};
  // Position mode does not embed a current limit in the Damiao PV command.
  // The controller monitors feedback current and disables output if this limit
  // or the global hard current limit is exceeded.
  common::A feedback_current_limit{};
  common::S timeout{};
  CommandSource source{CommandSource::Unknown};
};

struct ClampForceCommand {
  common::N target_force{};
  ClampSpeedMode speed_mode{ClampSpeedMode::GripperAngularSpeed};
  // Nut speed upper limit in mm/s. In NutLinearSpeed mode it is also the target.
  common::MmPerS max_nut_speed{};
  // Target gripper angular speed in rad/s for GripperAngularSpeed mode.
  common::RadPerS target_gripper_angular_speed{};
  common::A max_motor_current{};
  common::Nm max_motor_torque{};
  common::S timeout{};
  CommandSource source{CommandSource::Unknown};
};

struct ClampSpeedCommand {
  common::MmPerS target_nut_speed{};
  common::A max_motor_current{};
  common::Nm max_motor_torque{};
  common::S timeout{};
  ContactDetectionPolicy contact_policy{
      ContactDetectionPolicy::CurrentRiseAndVelocityDrop};
  CommandSource source{CommandSource::Unknown};
};

struct ReleaseCommand {
  common::MmPerS release_nut_speed{};
  common::Mm release_distance{};
  common::S timeout{};
  CommandSource source{CommandSource::Unknown};
};

struct MoveNutStrokeCommand {
  // Absolute mechanism-side nut stroke target in mm.
  common::Mm target_nut_stroke{};
  // Positive command limit in mm/s. Direction is derived from the target and
  // current stroke inside the controller.
  common::MmPerS max_nut_speed{};
  // Optional command current cap in A. Zero selects the conservative default
  // for the current confidence level.
  common::A max_motor_current{};
  // Allows the full low-confidence pre-self-check boundary to be used for
  // encoder-to-lead-screw travel verification when the mechanism is unloaded
  // or removed. Without this confirmation, low-confidence manual positioning
  // stays in a small window around the current estimated stroke.
  bool unloaded_or_structure_removed_confirmed{false};
  common::S timeout{};
  CommandSource source{CommandSource::Unknown};
};

struct MotorFeedbackSummary {
  common::Rad position{};
  common::Rad wrapped_position{};
  bool wrapped_position_valid{false};
  common::RadPerS velocity{};
  common::A current{};
  common::Nm torque{};
  common::DegC temperature{};
  std::uint32_t raw_position_counts{0};
  bool raw_position_counts_valid{false};
  std::uint32_t raw_feedback_frame_id{0};
  std::uint8_t raw_feedback_frame_length{0};
  std::array<std::uint8_t, 64> raw_feedback_frame_data{};
  bool raw_feedback_frame_valid{false};
  common::Rad runtime_position_limit{};
  common::RadPerS runtime_velocity_limit{};
  common::Nm runtime_torque_limit{};
  bool runtime_limits_valid{false};
  bool enabled{false};
  bool fault{false};
};

struct NutPositionEncoderSummary {
  common::Mm nut_position{};
  common::MmPerS nut_velocity{};
  common::Rad zero_motor_position{};
  common::Mm zero_nut_position{};
  common::Rad motor_position{};
  common::Rad motor_delta{};
  common::Ratio motor_delta_revolutions{};
  common::Ratio millimeters_per_revolution_estimate{};
  bool fresh{false};
};

struct GripperFaultSummary {
  state_machine::FaultSeverity severity{state_machine::FaultSeverity::None};
  state_machine::FaultSource source{state_machine::FaultSource::None};
  std::uint32_t code{0};
};

struct DirectionalLearnedParameters {
  common::A breakaway_current{};
  common::Nm breakaway_torque{};
  common::A static_friction_current{};
  common::Nm static_friction_torque{};
  common::MmPerS minimum_stable_nut_speed{};
  common::A dynamic_friction_current_average{};
  common::A dynamic_friction_current_max{};
  common::Nm dynamic_friction_torque_average{};
  common::Nm dynamic_friction_torque_max{};
  std::uint32_t breakaway_sample_count{0};
  std::uint32_t static_friction_sample_count{0};
  std::uint32_t stable_speed_sample_count{0};
  std::uint32_t dynamic_friction_sample_count{0};
};

struct StructureLearnedParametersSummary {
  DirectionalLearnedParameters opening{};
  DirectionalLearnedParameters closing{};
  common::Mm safe_zone_open_limit{};
  common::Mm safe_zone_closed_limit{};
  common::Mm software_open_limit{};
  common::Mm software_closed_limit{};
  common::Mm learned_travel{};
};

struct FrictionAnomalySummary {
  std::uint32_t record_count{0};
  common::Ratio max_current_excess_ratio{};
  common::Mm severe_center_position{};
  common::Mm severe_width{};
  common::A severe_peak_current{};
  common::A severe_baseline_current{};
  bool has_severe_record{false};
};

struct PreBCurrentTracePoint {
  common::Mm stroke{};
  common::A motor_current{};
  common::MmPerS nut_speed{};
  std::uint32_t segment_id{0};
  std::uint8_t phase{0};
  std::uint8_t direction{0};
};

struct ManualNutStrokeRangeSummary {
  common::Mm open_limit{};
  common::Mm closed_limit{};
  bool valid{false};
  bool use_software_limits{false};
  bool low_confidence_window{false};
  const char* confidence{"none"};
};

struct GripperStateSnapshot {
  state_machine::GripperTopState top_state{
      state_machine::GripperTopState::Disconnected};
  state_machine::PreSelfCheckPhase pre_self_check_phase{
      state_machine::PreSelfCheckPhase::Idle};
  StructureProfileValidity structure_profile_validity{
      StructureProfileValidity::Unknown};

  bool configured{false};
  bool connected{false};
  bool enabled{false};
  bool motor_bringup_active{false};
  bool motor_bringup_unloaded_confirmed{false};
  bool pre_self_check_completed{false};
  bool pre_b_mechanism_anomaly{false};
  bool homed{false};
  bool travel_limits_learned{false};
  bool motion_health_checked{false};
  bool contact_detected{false};

  common::Mm nut_stroke{};
  common::Rad gripper_angle{};
  common::N estimated_clamp_force{};

  MotorFeedbackSummary motor{};
  NutPositionEncoderSummary nut_encoder{};
  StructureLearnedParametersSummary learned_parameters{};
  FrictionAnomalySummary friction_anomaly{};
  std::vector<PreBCurrentTracePoint> pre_b_current_trace{};
  ManualNutStrokeRangeSummary manual_nut_stroke_range{};
  GripperFaultSummary fault{};
  std::uint32_t last_result_code{0};
};

}  // namespace gripper::controller
