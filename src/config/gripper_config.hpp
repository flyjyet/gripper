#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "common/units.hpp"

namespace gripper::config {

enum class AdapterType {
  Simulated,
  DmUsb2FdcanDual,
};

enum class MotorControlMode {
  Unknown,
  PositionForce,
};

enum class EncoderUnwrapSource {
  ProtocolPosition,
  RawPositionCounts,
};

enum class MotionDirection {
  Open = -1,
  Close = 1,
};

struct AdapterConfig {
  AdapterType type{AdapterType::Simulated};
  std::string channel{"can0"};
  std::string driver_library_path{"src/third_party/damiao/bin/libdm_device.dll"};
  std::uint32_t device_index{0};
  std::uint32_t channel_index{0};
  bool fdcan_enabled{true};
  bool brs_enabled{true};
  std::uint32_t nominal_bitrate_bps{1000000};
  std::uint32_t data_bitrate_bps{5000000};
};

struct MotorConfig {
  std::uint32_t motor_id{0x01};
  std::uint32_t host_id{0x11};
  MotorControlMode control_mode{MotorControlMode::PositionForce};
  int direction_sign{-1};
  int position_command_sign{1};
  common::Ratio encoder_scale{1.0};
  EncoderUnwrapSource encoder_unwrap_source{EncoderUnwrapSource::ProtocolPosition};
  // Zero means derive the protocol-position unwrap range from runtime P_MAX
  // when real Damiao hardware is connected.
  common::Rad encoder_wrap_range{0.0};
  std::uint32_t encoder_raw_count_range{65536};
  common::Rad max_position{65.0};
  common::RadPerS max_velocity{20.0};
  common::Nm max_torque{28.0};
  common::A max_phase_current{20.0};
  common::Ratio torque_per_amp{0.625};
  bool auto_switch_mode{true};
  bool motor_frames_canfd{true};
  bool command_id_includes_mode_offset{true};
  bool continuous_encoder_enabled{true};
  common::S feedback_poll_period{0.02};
  common::S feedback_stale_timeout{0.2};
};

struct MechanismConfig {
  // Lead screw travel per revolution, in mm/rev.
  common::Mm lead_screw_pitch{2.0};
  // Conservative usable nut travel, in mm.
  common::Mm usable_travel{16.0};
  // Theoretical software limits before runtime learning, in mm.
  common::Mm theoretical_open_limit{0.0};
  common::Mm theoretical_close_limit{16.0};
  // First-order kinematic approximation used before a measured model exists.
  common::Mm zero_angle_stroke{0.0};
  common::Rad zero_stroke_gripper_angle{0.0};
  common::Ratio gripper_angle_per_nut_stroke{0.03};
  common::Ratio gripper_angular_speed_per_nut_speed{0.03};
};

struct NutPositionEncoderConfig {
  // Multiplies motor.direction_sign for the virtual nut-position encoder and
  // controller-side nut-position conversion. Use -1 when unloaded bring-up
  // proves that the virtual nut position increases opposite to the expected
  // nut closing direction, without changing the Damiao command sign.
  int direction_sign{-1};
};

struct SelfCheckConfig {
  // Runtime-learned seed used to start the next PreSelfCheck closer to the last
  // successful breakaway point. Empty disables persistence.
  std::string learned_profile_path{"log/pre_self_check_profile_seed.txt"};
  // Maximum relative probe window during pre-self-check, in mm.
  common::Mm max_probe_window{1.0};
  common::MmPerS min_speed_scan_start{0.2};
  common::MmPerS min_speed_scan_stop{2.0};
  common::MmPerS min_speed_scan_step{0.2};
  // PreA bootstrap breakaway scan range, in A. This only helps the current
  // boot cycle start moving; final static friction is learned later from clean
  // multi-region motion samples.
  common::A static_friction_current_start{0.1};
  common::A static_friction_current_stop{0.4};
  common::A static_friction_current_step{0.05};
  // Minimum stroke change accepted as "motion started" during breakaway scan.
  common::Mm motion_start_distance{0.05};
  // Lower repeatable micro-motion threshold accepted only as low-confidence
  // breakaway during the first PreSelfCheck scan.
  common::Mm low_confidence_motion_distance{0.02};
  // Short stable stroke target used after breakaway is confirmed.
  common::Mm stable_short_stroke_distance{0.3};
  // Test speeds for dynamic friction identification, in mm/s.
  std::vector<common::MmPerS> dynamic_friction_speeds{{0.6}, {1.0}, {1.2}, {1.5}};
  common::S feedback_noise_sample_time{1.0};
  // Time allowed after a stop command for residual motion to settle.
  common::S motion_settle_timeout{5.0};
  // Continuous still time required before final position is sampled.
  common::S motion_settle_stable_time{0.1};
  // Maximum nut speed accepted as still during self-check settle, in mm/s.
  common::MmPerS motion_settle_speed_threshold{0.03};
  // Low speed envelope used by PositionForce hold during probe-to-probe settle.
  common::MmPerS motion_hold_speed{0.2};
  // Current envelope used to stop and hold between self-check probes.
  common::A motion_hold_current{1.5};
  common::MmPerS travel_learning_speed{1.2};
  // Closing-side relative search distance after homing. This is an expected
  // actual-travel search budget, not the legacy theoretical close limit.
  common::Mm travel_learning_search_distance{20.0};
  common::Mm software_limit_margin{0.5};
  std::vector<common::MmPerS> motion_health_check_speeds{{1.0}, {2.0}, {4.0}};
  common::Mm min_measured_distance{0.5};
  common::Mm max_distance_error{0.2};
  common::MmPerS stable_speed_margin{0.2};
  common::Mm max_theoretical_travel_error{2.0};
  common::Mm safe_zone_margin{1.0};
  // Maximum one-sided PreB expansion in temporary pre-homing coordinates, in mm.
  // Values near expected actual travel let PreSelfCheck build a low-confidence
  // full-stroke model without treating legacy theoretical limits as verified.
  common::Mm pre_b_max_expansion_distance{20.0};
  // Step size used by PreB boundary expansion. Keep this separate from the
  // generic probe window so physical-end search never becomes a single large
  // move.
  common::Mm pre_b_boundary_step{1.0};
  // PreB expansion and safe-zone repositioning speed. This is intentionally
  // separate from formal TravelLearning speed so self-check can pass local
  // friction spots without changing the later high-confidence learning policy.
  common::MmPerS pre_b_expansion_speed{1.0};
  // After a physical boundary candidate is found, PreB backs away before
  // learning so dynamic-friction samples are not collected from a wedged end
  // stop.
  common::Mm pre_b_boundary_release_distance{1.0};
  common::MmPerS pre_b_boundary_release_speed{1.0};
  // Command-current envelope used only for PreB boundary release. It can be
  // slightly higher than the normal scan current to unwind a wedged end stop,
  // but is still capped by safety.max_motor_current.
  common::A pre_b_boundary_release_current_limit{2.0};
  // Controlled confirmation probe used when PreB sees a soft jam/contact
  // signal away from the theoretical guard. It distinguishes a pass-through
  // friction spot from a true no-progress limit.
  common::Mm pre_b_soft_jam_retry_distance{1.0};
  common::MmPerS pre_b_soft_jam_retry_speed{1.0};
  common::S pre_b_soft_jam_progress_timeout{0.6};
  // PreB-only confirmation time for feedback current above the self-check hard
  // threshold. Short motor-driver feedback spikes are logged but do not stop a
  // near-full-travel scan unless the over-limit state persists.
  common::S pre_b_hard_current_confirm_time{0.25};
  // Minimum number of position-separated clean regions required before PreB
  // dynamic/static friction samples are trusted as a structure baseline.
  std::uint32_t pre_b_min_learning_regions{3};
  // Candidate anchors distributed across the low-confidence safe zone. This is
  // intentionally larger than the minimum so local friction anomalies can be
  // skipped without collapsing learning to one edge.
  std::uint32_t pre_b_learning_anchor_count{7};
  common::Rad fallback_motor_position_noise{0.001};
  common::RadPerS fallback_motor_velocity_noise{0.001};
  common::A fallback_motor_current_noise{0.01};
  common::Nm fallback_motor_torque_noise{0.01};
  common::Mm fallback_nut_stroke_noise{0.01};
  common::MmPerS max_velocity_tracking_error{0.5};
  common::A max_current_ripple{0.1};
  common::Nm max_torque_ripple{0.05};
  common::DegC max_motor_temperature{70.0};
  bool friction_anomaly_enabled{true};
  common::Ratio friction_anomaly_current_ratio_threshold{2.0};
  common::Mm friction_anomaly_sliding_window_distance{1.0};
  common::Mm friction_anomaly_min_width{0.05};
  std::uint32_t friction_anomaly_min_confirmations{2};
  common::Ratio friction_anomaly_minor_ratio{2.0};
  common::Ratio friction_anomaly_moderate_ratio{3.0};
  std::uint32_t friction_anomaly_max_records{20};
  common::A friction_anomaly_min_baseline_current{0.02};
  // Multi-speed structure learning skips anchors and sample segments that
  // overlap a detected local friction anomaly plus this distance margin.
  common::Mm friction_anomaly_avoid_margin{0.3};
  // Only anomaly records at or above this excess-current ratio are excluded
  // from multi-speed structure learning. Minor candidates remain diagnostics.
  common::Ratio friction_anomaly_learning_avoid_ratio{3.0};
};

struct SafetyConfig {
  // Global feedback hard stop. Operation-specific command limits can be lower,
  // but measured feedback current above this value stops motion immediately.
  common::A max_motor_current{2.0};
  common::A self_check_current_limit{1.9};
  // Feedback hard stop used only by PreSelfCheck probes. It allows short
  // hardware-measured current peaks while keeping the command current limited.
  common::A self_check_feedback_hard_current_limit{2.5};
  // Immediate PreSelfCheck feedback stop for extreme end-stop wedging. This
  // caps unsafe spikes without changing the normal sustained-current filter.
  common::A self_check_feedback_emergency_current_limit{3.0};
  common::A homing_current_limit{0.5};
  common::A travel_learning_current_limit{1.5};
  common::A manual_positioning_current_limit{1.5};
  common::A clamp_current_limit{1.0};
  common::MmPerS max_nut_speed{8.0};
  common::MmPerS2 max_nut_acceleration{40.0};
  common::A contact_current_rise_threshold{0.3};
  common::A jam_current_threshold{0.8};
  common::MmPerS jam_speed_threshold{0.1};
  common::Mm stroke_limit_margin{0.3};
  common::S contact_detection_time{0.1};
  common::S command_timeout{0.2};
};

struct MotorBringupConfig {
  // Motor-side defaults for unloaded communication/direction testing. These
  // limits are independent from clamp/recovery limits and still cannot exceed
  // the global hard current protection.
  // In velocity bring-up jog, relative_motor_position is used only as a signed
  // direction window; motion distance is determined by velocity * duration.
  common::Rad default_relative_motor_position{0.2};
  common::Rad max_relative_motor_position{1.0};
  // Relative position-mode move used to verify motor multi-turn feedback and
  // lead-screw mapping during unloaded bring-up. One motor revolution should
  // map to one lead-screw pitch of nut travel.
  common::Ratio default_relative_motor_revolutions{1.0};
  common::Ratio max_relative_motor_revolutions{2.0};
  common::RadPerS default_motor_velocity{0.5};
  common::RadPerS max_motor_velocity{5.0};
  common::A default_motor_current{1.5};
  common::A max_motor_current{2.0};
  common::S default_pulse_duration{0.1};
  common::S max_pulse_duration{2.0};
  common::S default_position_move_timeout{0.0};
  common::S max_position_move_timeout{20.0};
};

struct HomingConfig {
  MotionDirection direction{MotionDirection::Open};
  common::MmPerS homing_speed{0.8};
  common::A homing_current{0.4};
  common::S jam_confirm_time{0.2};
  common::Mm backoff_distance{0.5};
};

struct ClampConfig {
  common::N target_force{20.0};
  common::N min_target_force{0.0};
  common::N max_target_force{200.0};
  // Nut speed upper limit in mm/s. Also used as the legacy nut-speed target.
  common::MmPerS target_nut_speed{1.0};
  // Default target gripper angular speed in rad/s for force clamp.
  common::RadPerS target_gripper_angular_speed{0.03};
  common::Mm approach_distance{2.0};
  common::MmPerS approach_nut_speed{0.5};
  common::Mm release_distance{1.0};
  common::Ratio torque_per_force{0.01};
  common::Ratio current_per_torque{1.0};
  common::Nm torque_offset{0.0};
  common::A current_offset{0.0};
  common::Nm max_motor_torque{3.5};
};

struct UiConfig {
  common::S refresh_period{0.05};
  std::uint32_t log_capacity{1000};
  common::N default_target_force{20.0};
  common::MmPerS default_clamp_speed{1.0};
};

struct GripperConfig {
  AdapterConfig adapter{};
  MotorConfig motor{};
  MechanismConfig mechanism{};
  NutPositionEncoderConfig nut_position_encoder{};
  SelfCheckConfig self_check{};
  SafetyConfig safety{};
  MotorBringupConfig motor_bringup{};
  HomingConfig homing{};
  ClampConfig clamp{};
  UiConfig ui{};
};

}  // namespace gripper::config
