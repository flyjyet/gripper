#pragma once

#include <cstdint>
#include <string>

namespace gripper_control {

enum class MotorMode {
  Disabled,
  Position,
  Velocity,
  Current,
  Torque,
};

enum class GripperState {
  Disabled,
  PowerOnSelfCheck,
  HomingOpenStop,
  FrictionIdentify,
  Ready,
  GuardedClosing,
  ContactDetected,
  ForceBuild,
  UnloadBeforeDisable,
  ClampDoneDisabled,
  UnlockBeforeOpen,
  Opening,
  AntiJamRelease,
  FaultStop,
};

enum class GripperFault {
  None,
  FeedbackTimeout,
  EncoderFault,
  DriveFault,
  OverTemperature,
  OverCurrent,
  PositionLimit,
  HomingFailed,
  ContactNotFound,
  AbnormalJam,
  AntiJamFailed,
  CalibrationInvalid,
};

struct MotorFeedback {
  double timestamp_s = 0.0;
  double stroke_mm = 0.0;
  double velocity_mm_s = 0.0;
  double current_A = 0.0;
  double torque_Nm = 0.0;
  double temperature_C = 25.0;
  std::uint32_t fault_bits = 0;
  bool encoder_ok = true;
  bool drive_enabled = false;
};

struct MotorCommand {
  MotorMode mode = MotorMode::Disabled;
  double target_position_mm = 0.0;
  double target_velocity_mm_s = 0.0;
  double target_current_A = 0.0;
  double target_torque_Nm = 0.0;
  double current_limit_A = 0.0;
};

struct ClampCommand {
  double target_force_per_side_N = 150.0;
  double close_speed_mm_s = 0.2;
  double max_current_A = 1.5;
  double timeout_s = 8.0;
  bool known_cable_mode = false;
};

struct OpenCommand {
  double open_speed_mm_s = 0.8;
  double max_current_A = 1.5;
  double timeout_s = 8.0;
};

struct GripperStatus {
  GripperState state = GripperState::Disabled;
  GripperFault fault = GripperFault::None;
  double stroke_mm = 0.0;
  double velocity_mm_s = 0.0;
  double current_A = 0.0;
  double torque_Nm = 0.0;
  double temperature_C = 25.0;
  double estimated_force_per_side_N = 0.0;
  bool homed = false;
  bool friction_identified = false;
  bool motor_enabled = false;
  std::string fault_text;
};

struct GripperLimits {
  double stroke_min_mm = 0.0;
  double stroke_max_mm = 16.0;
  double v_close_unknown_max_mm_s = 0.3;
  double v_close_known_max_mm_s = 0.8;
  double v_contact_search_max_mm_s = 0.2;
  double v_open_max_mm_s = 0.8;
  double v_homing_max_mm_s = 0.2;
  double v_unlock_max_mm_s = 0.08;
  double current_abs_max_A = 2.0;
  double current_homing_max_A = 0.35;
  double current_selfcheck_max_A = 0.35;
  double current_close_guard_max_A = 1.5;
  double current_unlock_max_A = 0.4;
  double current_anti_jam_max_A = 0.5;
  double current_slew_rate_A_s = 2.0;
  double accel_limit_mm_s2 = 1.0;
  double over_temperature_C = 75.0;
  double derate_temperature_C = 60.0;
};

struct JamDetectionParams {
  double speed_cmd_min_mm_s = 0.02;
  double speed_stall_threshold_mm_s = 0.03;
  double position_delta_min_mm = 0.005;
  double current_contact_delta_A = 0.18;
  double current_stall_delta_A = 0.35;
  double contact_debounce_s = 0.10;
  double stall_debounce_s = 0.20;
  double position_window_s = 0.10;
};

struct FrictionParams {
  double current_close_A = 0.10;
  double current_open_A = 0.08;
  double torque_close_Nm = 0.05;
  double torque_open_Nm = 0.04;
  bool valid = false;
};

struct ForceMapParams {
  double baseline_current_A = 0.30;
  double current_per_N = 0.0045;
  double max_target_force_N = 200.0;
};

const char* toString(GripperState state);
const char* toString(GripperFault fault);

}  // namespace gripper_control
