#include "config/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <utility>

namespace gripper::config {
namespace {

[[nodiscard]] std::string trim(std::string text) {
  auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(),
             text.end());
  return text;
}

[[nodiscard]] std::string stripComment(std::string text) {
  const auto pos = text.find('#');
  if (pos != std::string::npos) {
    text.erase(pos);
  }
  return trim(std::move(text));
}

[[nodiscard]] bool parseBool(const std::string& value, bool fallback) {
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return fallback;
}

[[nodiscard]] double parseDouble(const std::string& value, double fallback) {
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

[[nodiscard]] std::uint32_t parseU32(const std::string& value,
                                     std::uint32_t fallback) {
  try {
    std::size_t parsed_chars = 0;
    const auto parsed = std::stoul(value, &parsed_chars, 0);
    return parsed_chars == value.size() ? static_cast<std::uint32_t>(parsed)
                                        : fallback;
  } catch (...) {
    return fallback;
  }
}

[[nodiscard]] std::vector<common::MmPerS> parseSpeedList(
    std::string value, const std::vector<common::MmPerS>& fallback) {
  value = trim(std::move(value));
  if (value.size() < 2 || value.front() != '[' || value.back() != ']') {
    return fallback;
  }

  value = value.substr(1, value.size() - 2);
  std::vector<common::MmPerS> speeds;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = trim(item);
    if (!item.empty()) {
      speeds.push_back(common::MmPerS{parseDouble(item, 0.0)});
    }
  }
  return speeds.empty() ? fallback : speeds;
}

void applyField(const std::string& section, const std::string& key,
                const std::string& value, GripperConfig* config) {
  if (config == nullptr) {
    return;
  }

  if (section == "adapter") {
    if (key == "type") {
      config->adapter.type =
          value == "dm_usb2fdcan_dual" ? AdapterType::DmUsb2FdcanDual
                                       : AdapterType::Simulated;
    } else if (key == "channel") {
      config->adapter.channel = value;
    } else if (key == "driver_library_path") {
      config->adapter.driver_library_path = value;
    } else if (key == "device_index") {
      config->adapter.device_index =
          parseU32(value, config->adapter.device_index);
    } else if (key == "channel_index") {
      config->adapter.channel_index =
          parseU32(value, config->adapter.channel_index);
    } else if (key == "fdcan_enabled") {
      config->adapter.fdcan_enabled =
          parseBool(value, config->adapter.fdcan_enabled);
    } else if (key == "brs_enabled") {
      config->adapter.brs_enabled = parseBool(value, config->adapter.brs_enabled);
    } else if (key == "nominal_bitrate_bps") {
      config->adapter.nominal_bitrate_bps =
          parseU32(value, config->adapter.nominal_bitrate_bps);
    } else if (key == "data_bitrate_bps") {
      config->adapter.data_bitrate_bps =
          parseU32(value, config->adapter.data_bitrate_bps);
    }
  } else if (section == "motor") {
    if (key == "motor_id") {
      config->motor.motor_id = parseU32(value, config->motor.motor_id);
    } else if (key == "host_id") {
      config->motor.host_id = parseU32(value, config->motor.host_id);
    } else if (key == "direction_sign") {
      config->motor.direction_sign =
          static_cast<int>(parseDouble(value, config->motor.direction_sign));
    } else if (key == "position_command_sign") {
      config->motor.position_command_sign = static_cast<int>(
          parseDouble(value, config->motor.position_command_sign));
    } else if (key == "encoder_scale_ratio") {
      config->motor.encoder_scale =
          common::Ratio{parseDouble(value, config->motor.encoder_scale.value)};
    } else if (key == "encoder_unwrap_source") {
      config->motor.encoder_unwrap_source =
          value == "protocol_position"
              ? EncoderUnwrapSource::ProtocolPosition
              : EncoderUnwrapSource::RawPositionCounts;
    } else if (key == "encoder_wrap_range_rad") {
      config->motor.encoder_wrap_range = common::Rad{
          parseDouble(value, config->motor.encoder_wrap_range.value)};
    } else if (key == "encoder_raw_count_range") {
      config->motor.encoder_raw_count_range =
          parseU32(value, config->motor.encoder_raw_count_range);
    } else if (key == "max_position_rad") {
      config->motor.max_position =
          common::Rad{parseDouble(value, config->motor.max_position.value)};
    } else if (key == "max_velocity_rad_s") {
      config->motor.max_velocity =
          common::RadPerS{parseDouble(value, config->motor.max_velocity.value)};
    } else if (key == "max_torque_nm") {
      config->motor.max_torque =
          common::Nm{parseDouble(value, config->motor.max_torque.value)};
    } else if (key == "max_phase_current_a") {
      config->motor.max_phase_current =
          common::A{parseDouble(value, config->motor.max_phase_current.value)};
    } else if (key == "torque_per_amp_nm_per_a") {
      config->motor.torque_per_amp =
          common::Ratio{parseDouble(value, config->motor.torque_per_amp.value)};
    } else if (key == "auto_switch_mode") {
      config->motor.auto_switch_mode =
          parseBool(value, config->motor.auto_switch_mode);
    } else if (key == "motor_frames_canfd") {
      config->motor.motor_frames_canfd =
          parseBool(value, config->motor.motor_frames_canfd);
    } else if (key == "command_id_includes_mode_offset") {
      config->motor.command_id_includes_mode_offset =
          parseBool(value, config->motor.command_id_includes_mode_offset);
    } else if (key == "continuous_encoder_enabled") {
      config->motor.continuous_encoder_enabled =
          parseBool(value, config->motor.continuous_encoder_enabled);
    } else if (key == "feedback_poll_period_s") {
      config->motor.feedback_poll_period =
          common::S{parseDouble(value, config->motor.feedback_poll_period.value)};
    } else if (key == "feedback_stale_timeout_s") {
      config->motor.feedback_stale_timeout = common::S{
          parseDouble(value, config->motor.feedback_stale_timeout.value)};
    }
  } else if (section == "mechanism") {
    if (key == "lead_screw_pitch_mm_per_rev") {
      config->mechanism.lead_screw_pitch =
          common::Mm{parseDouble(value, config->mechanism.lead_screw_pitch.value)};
    } else if (key == "usable_travel_mm") {
      config->mechanism.usable_travel =
          common::Mm{parseDouble(value, config->mechanism.usable_travel.value)};
    } else if (key == "theoretical_open_limit_mm") {
      config->mechanism.theoretical_open_limit = common::Mm{
          parseDouble(value, config->mechanism.theoretical_open_limit.value)};
    } else if (key == "theoretical_close_limit_mm") {
      config->mechanism.theoretical_close_limit = common::Mm{
          parseDouble(value, config->mechanism.theoretical_close_limit.value)};
    } else if (key == "zero_angle_stroke_mm") {
      config->mechanism.zero_angle_stroke = common::Mm{
          parseDouble(value, config->mechanism.zero_angle_stroke.value)};
    } else if (key == "zero_stroke_gripper_angle_rad") {
      config->mechanism.zero_stroke_gripper_angle = common::Rad{parseDouble(
          value, config->mechanism.zero_stroke_gripper_angle.value)};
    } else if (key == "gripper_angle_per_nut_stroke_rad_per_mm") {
      config->mechanism.gripper_angle_per_nut_stroke = common::Ratio{
          parseDouble(value, config->mechanism.gripper_angle_per_nut_stroke.value)};
    } else if (key == "gripper_angular_speed_per_nut_speed_ratio") {
      config->mechanism.gripper_angular_speed_per_nut_speed = common::Ratio{
          parseDouble(value,
                      config->mechanism.gripper_angular_speed_per_nut_speed.value)};
    }
  } else if (section == "nut_position_encoder") {
    if (key == "direction_sign") {
      config->nut_position_encoder.direction_sign = static_cast<int>(
          parseDouble(value, config->nut_position_encoder.direction_sign));
    }
  } else if (section == "self_check") {
    if (key == "learned_profile_path") {
      config->self_check.learned_profile_path = value;
    } else if (key == "max_probe_window_mm") {
      config->self_check.max_probe_window = common::Mm{
          parseDouble(value, config->self_check.max_probe_window.value)};
    } else if (key == "min_speed_scan_start_mm_s") {
      config->self_check.min_speed_scan_start = common::MmPerS{
          parseDouble(value, config->self_check.min_speed_scan_start.value)};
    } else if (key == "min_speed_scan_stop_mm_s") {
      config->self_check.min_speed_scan_stop = common::MmPerS{
          parseDouble(value, config->self_check.min_speed_scan_stop.value)};
    } else if (key == "min_speed_scan_step_mm_s") {
      config->self_check.min_speed_scan_step = common::MmPerS{
          parseDouble(value, config->self_check.min_speed_scan_step.value)};
    } else if (key == "static_friction_current_start_a") {
      config->self_check.static_friction_current_start = common::A{parseDouble(
          value, config->self_check.static_friction_current_start.value)};
    } else if (key == "static_friction_current_stop_a") {
      config->self_check.static_friction_current_stop = common::A{parseDouble(
          value, config->self_check.static_friction_current_stop.value)};
    } else if (key == "static_friction_current_step_a") {
      config->self_check.static_friction_current_step = common::A{parseDouble(
          value, config->self_check.static_friction_current_step.value)};
    } else if (key == "motion_start_distance_mm") {
      config->self_check.motion_start_distance = common::Mm{
          parseDouble(value, config->self_check.motion_start_distance.value)};
    } else if (key == "low_confidence_motion_distance_mm") {
      config->self_check.low_confidence_motion_distance = common::Mm{
          parseDouble(value,
                      config->self_check.low_confidence_motion_distance.value)};
    } else if (key == "stable_short_stroke_distance_mm") {
      config->self_check.stable_short_stroke_distance = common::Mm{parseDouble(
          value, config->self_check.stable_short_stroke_distance.value)};
    } else if (key == "dynamic_friction_speeds_mm_s") {
      config->self_check.dynamic_friction_speeds =
          parseSpeedList(value, config->self_check.dynamic_friction_speeds);
    } else if (key == "motion_settle_timeout_s") {
      config->self_check.motion_settle_timeout = common::S{
          parseDouble(value, config->self_check.motion_settle_timeout.value)};
    } else if (key == "motion_settle_stable_time_s") {
      config->self_check.motion_settle_stable_time = common::S{parseDouble(
          value, config->self_check.motion_settle_stable_time.value)};
    } else if (key == "motion_settle_speed_threshold_mm_s") {
      config->self_check.motion_settle_speed_threshold = common::MmPerS{
          parseDouble(value,
                      config->self_check.motion_settle_speed_threshold.value)};
    } else if (key == "motion_hold_speed_mm_s") {
      config->self_check.motion_hold_speed = common::MmPerS{
          parseDouble(value, config->self_check.motion_hold_speed.value)};
    } else if (key == "motion_hold_current_a") {
      config->self_check.motion_hold_current = common::A{
          parseDouble(value, config->self_check.motion_hold_current.value)};
    } else if (key == "travel_learning_speed_mm_s") {
      config->self_check.travel_learning_speed = common::MmPerS{
          parseDouble(value, config->self_check.travel_learning_speed.value)};
    } else if (key == "travel_learning_search_distance_mm") {
      config->self_check.travel_learning_search_distance = common::Mm{
          parseDouble(value,
                      config->self_check.travel_learning_search_distance.value)};
    } else if (key == "software_limit_margin_mm") {
      config->self_check.software_limit_margin = common::Mm{parseDouble(
          value, config->self_check.software_limit_margin.value)};
    } else if (key == "motion_health_check_speeds_mm_s") {
      config->self_check.motion_health_check_speeds =
          parseSpeedList(value, config->self_check.motion_health_check_speeds);
    } else if (key == "min_measured_distance_mm") {
      config->self_check.min_measured_distance = common::Mm{
          parseDouble(value, config->self_check.min_measured_distance.value)};
    } else if (key == "max_distance_error_mm") {
      config->self_check.max_distance_error = common::Mm{
          parseDouble(value, config->self_check.max_distance_error.value)};
    } else if (key == "stable_speed_margin_mm_s") {
      config->self_check.stable_speed_margin = common::MmPerS{
          parseDouble(value, config->self_check.stable_speed_margin.value)};
    } else if (key == "max_theoretical_travel_error_mm") {
      config->self_check.max_theoretical_travel_error = common::Mm{parseDouble(
          value, config->self_check.max_theoretical_travel_error.value)};
    } else if (key == "safe_zone_margin_mm") {
      config->self_check.safe_zone_margin =
          common::Mm{parseDouble(value, config->self_check.safe_zone_margin.value)};
    } else if (key == "pre_b_max_expansion_distance_mm") {
      config->self_check.pre_b_max_expansion_distance = common::Mm{parseDouble(
          value, config->self_check.pre_b_max_expansion_distance.value)};
    } else if (key == "pre_b_boundary_step_mm") {
      config->self_check.pre_b_boundary_step = common::Mm{parseDouble(
          value, config->self_check.pre_b_boundary_step.value)};
    } else if (key == "pre_b_expansion_speed_mm_s") {
      config->self_check.pre_b_expansion_speed = common::MmPerS{
          parseDouble(value, config->self_check.pre_b_expansion_speed.value)};
    } else if (key == "pre_b_boundary_release_distance_mm") {
      config->self_check.pre_b_boundary_release_distance = common::Mm{
          parseDouble(value,
                      config->self_check.pre_b_boundary_release_distance.value)};
    } else if (key == "pre_b_boundary_release_speed_mm_s") {
      config->self_check.pre_b_boundary_release_speed = common::MmPerS{
          parseDouble(value,
                      config->self_check.pre_b_boundary_release_speed.value)};
    } else if (key == "pre_b_boundary_release_current_limit_a") {
      config->self_check.pre_b_boundary_release_current_limit = common::A{
          parseDouble(
              value,
              config->self_check.pre_b_boundary_release_current_limit.value)};
    } else if (key == "pre_b_soft_jam_retry_distance_mm") {
      config->self_check.pre_b_soft_jam_retry_distance = common::Mm{parseDouble(
          value, config->self_check.pre_b_soft_jam_retry_distance.value)};
    } else if (key == "pre_b_soft_jam_retry_speed_mm_s") {
      config->self_check.pre_b_soft_jam_retry_speed = common::MmPerS{parseDouble(
          value, config->self_check.pre_b_soft_jam_retry_speed.value)};
    } else if (key == "pre_b_soft_jam_progress_timeout_s") {
      config->self_check.pre_b_soft_jam_progress_timeout = common::S{parseDouble(
          value, config->self_check.pre_b_soft_jam_progress_timeout.value)};
    } else if (key == "pre_b_hard_current_confirm_time_s") {
      config->self_check.pre_b_hard_current_confirm_time = common::S{parseDouble(
          value, config->self_check.pre_b_hard_current_confirm_time.value)};
    } else if (key == "pre_b_min_learning_regions") {
      config->self_check.pre_b_min_learning_regions =
          parseU32(value, config->self_check.pre_b_min_learning_regions);
    } else if (key == "pre_b_learning_anchor_count") {
      config->self_check.pre_b_learning_anchor_count =
          parseU32(value, config->self_check.pre_b_learning_anchor_count);
    } else if (key == "max_velocity_tracking_error_mm_s") {
      config->self_check.max_velocity_tracking_error = common::MmPerS{parseDouble(
          value, config->self_check.max_velocity_tracking_error.value)};
    } else if (key == "max_current_ripple_a") {
      config->self_check.max_current_ripple = common::A{
          parseDouble(value, config->self_check.max_current_ripple.value)};
    } else if (key == "max_torque_ripple_nm") {
      config->self_check.max_torque_ripple = common::Nm{
          parseDouble(value, config->self_check.max_torque_ripple.value)};
    } else if (key == "max_motor_temperature_deg_c") {
      config->self_check.max_motor_temperature = common::DegC{parseDouble(
          value, config->self_check.max_motor_temperature.value)};
    } else if (key == "friction_anomaly_enabled") {
      config->self_check.friction_anomaly_enabled =
          parseBool(value, config->self_check.friction_anomaly_enabled);
    } else if (key == "friction_anomaly_current_ratio_threshold") {
      config->self_check.friction_anomaly_current_ratio_threshold =
          common::Ratio{parseDouble(
              value,
              config->self_check.friction_anomaly_current_ratio_threshold.value)};
    } else if (key == "friction_anomaly_sliding_window_distance_mm") {
      config->self_check.friction_anomaly_sliding_window_distance =
          common::Mm{parseDouble(
              value,
              config->self_check.friction_anomaly_sliding_window_distance.value)};
    } else if (key == "friction_anomaly_min_width_mm") {
      config->self_check.friction_anomaly_min_width = common::Mm{parseDouble(
          value, config->self_check.friction_anomaly_min_width.value)};
    } else if (key == "friction_anomaly_min_confirmations") {
      config->self_check.friction_anomaly_min_confirmations = parseU32(
          value, config->self_check.friction_anomaly_min_confirmations);
    } else if (key == "friction_anomaly_minor_ratio") {
      config->self_check.friction_anomaly_minor_ratio =
          common::Ratio{parseDouble(
              value, config->self_check.friction_anomaly_minor_ratio.value)};
    } else if (key == "friction_anomaly_moderate_ratio") {
      config->self_check.friction_anomaly_moderate_ratio =
          common::Ratio{parseDouble(
              value, config->self_check.friction_anomaly_moderate_ratio.value)};
    } else if (key == "friction_anomaly_max_records") {
      config->self_check.friction_anomaly_max_records =
          parseU32(value, config->self_check.friction_anomaly_max_records);
    } else if (key == "friction_anomaly_min_baseline_current_a") {
      config->self_check.friction_anomaly_min_baseline_current =
          common::A{parseDouble(
              value,
              config->self_check.friction_anomaly_min_baseline_current.value)};
    } else if (key == "friction_anomaly_avoid_margin_mm") {
      config->self_check.friction_anomaly_avoid_margin =
          common::Mm{parseDouble(
              value, config->self_check.friction_anomaly_avoid_margin.value)};
    } else if (key == "friction_anomaly_learning_avoid_ratio") {
      config->self_check.friction_anomaly_learning_avoid_ratio =
          common::Ratio{parseDouble(
              value,
              config->self_check.friction_anomaly_learning_avoid_ratio.value)};
    }
  } else if (section == "safety") {
    if (key == "max_motor_current_a") {
      config->safety.max_motor_current =
          common::A{parseDouble(value, config->safety.max_motor_current.value)};
    } else if (key == "self_check_current_limit_a") {
      config->safety.self_check_current_limit = common::A{
          parseDouble(value, config->safety.self_check_current_limit.value)};
    } else if (key == "self_check_feedback_hard_current_limit_a") {
      config->safety.self_check_feedback_hard_current_limit =
          common::A{parseDouble(
              value,
              config->safety.self_check_feedback_hard_current_limit.value)};
    } else if (key == "self_check_feedback_emergency_current_limit_a") {
      config->safety.self_check_feedback_emergency_current_limit =
          common::A{parseDouble(
              value,
              config->safety.self_check_feedback_emergency_current_limit.value)};
    } else if (key == "homing_current_limit_a") {
      config->safety.homing_current_limit =
          common::A{parseDouble(value, config->safety.homing_current_limit.value)};
    } else if (key == "travel_learning_current_limit_a") {
      config->safety.travel_learning_current_limit = common::A{parseDouble(
          value, config->safety.travel_learning_current_limit.value)};
    } else if (key == "manual_positioning_current_limit_a") {
      config->safety.manual_positioning_current_limit = common::A{parseDouble(
          value, config->safety.manual_positioning_current_limit.value)};
    } else if (key == "clamp_current_limit_a") {
      config->safety.clamp_current_limit =
          common::A{parseDouble(value, config->safety.clamp_current_limit.value)};
    } else if (key == "max_nut_speed_mm_s") {
      config->safety.max_nut_speed =
          common::MmPerS{parseDouble(value, config->safety.max_nut_speed.value)};
    } else if (key == "max_nut_acceleration_mm_s2") {
      config->safety.max_nut_acceleration = common::MmPerS2{
          parseDouble(value, config->safety.max_nut_acceleration.value)};
    } else if (key == "contact_current_rise_threshold_a") {
      config->safety.contact_current_rise_threshold = common::A{parseDouble(
          value, config->safety.contact_current_rise_threshold.value)};
    } else if (key == "jam_current_threshold_a") {
      config->safety.jam_current_threshold =
          common::A{parseDouble(value, config->safety.jam_current_threshold.value)};
    } else if (key == "jam_speed_threshold_mm_s") {
      config->safety.jam_speed_threshold = common::MmPerS{
          parseDouble(value, config->safety.jam_speed_threshold.value)};
    } else if (key == "stroke_limit_margin_mm") {
      config->safety.stroke_limit_margin =
          common::Mm{parseDouble(value, config->safety.stroke_limit_margin.value)};
    } else if (key == "contact_detection_time_s") {
      config->safety.contact_detection_time = common::S{
          parseDouble(value, config->safety.contact_detection_time.value)};
    } else if (key == "command_timeout_s") {
      config->safety.command_timeout =
          common::S{parseDouble(value, config->safety.command_timeout.value)};
    }
  } else if (section == "motor_bringup") {
    if (key == "default_relative_motor_position_rad") {
      config->motor_bringup.default_relative_motor_position = common::Rad{
          parseDouble(value,
                      config->motor_bringup.default_relative_motor_position.value)};
    } else if (key == "max_relative_motor_position_rad") {
      config->motor_bringup.max_relative_motor_position = common::Rad{parseDouble(
          value, config->motor_bringup.max_relative_motor_position.value)};
    } else if (key == "default_relative_motor_revolutions") {
      config->motor_bringup.default_relative_motor_revolutions =
          common::Ratio{parseDouble(
              value,
              config->motor_bringup.default_relative_motor_revolutions.value)};
    } else if (key == "max_relative_motor_revolutions") {
      config->motor_bringup.max_relative_motor_revolutions =
          common::Ratio{parseDouble(
              value, config->motor_bringup.max_relative_motor_revolutions.value)};
    } else if (key == "default_motor_velocity_rad_s") {
      config->motor_bringup.default_motor_velocity = common::RadPerS{parseDouble(
          value, config->motor_bringup.default_motor_velocity.value)};
    } else if (key == "max_motor_velocity_rad_s") {
      config->motor_bringup.max_motor_velocity = common::RadPerS{parseDouble(
          value, config->motor_bringup.max_motor_velocity.value)};
    } else if (key == "default_motor_current_a") {
      config->motor_bringup.default_motor_current = common::A{parseDouble(
          value, config->motor_bringup.default_motor_current.value)};
    } else if (key == "max_motor_current_a") {
      config->motor_bringup.max_motor_current = common::A{
          parseDouble(value, config->motor_bringup.max_motor_current.value)};
    } else if (key == "default_pulse_duration_s") {
      config->motor_bringup.default_pulse_duration = common::S{parseDouble(
          value, config->motor_bringup.default_pulse_duration.value)};
    } else if (key == "max_pulse_duration_s") {
      config->motor_bringup.max_pulse_duration = common::S{
          parseDouble(value, config->motor_bringup.max_pulse_duration.value)};
    } else if (key == "default_position_move_timeout_s") {
      config->motor_bringup.default_position_move_timeout =
          common::S{parseDouble(
              value,
              config->motor_bringup.default_position_move_timeout.value)};
    } else if (key == "max_position_move_timeout_s") {
      config->motor_bringup.max_position_move_timeout =
          common::S{parseDouble(
              value, config->motor_bringup.max_position_move_timeout.value)};
    }
  } else if (section == "homing") {
    if (key == "direction") {
      config->homing.direction =
          value == "close" ? MotionDirection::Close : MotionDirection::Open;
    } else if (key == "homing_speed_mm_s") {
      config->homing.homing_speed =
          common::MmPerS{parseDouble(value, config->homing.homing_speed.value)};
    } else if (key == "homing_current_a") {
      config->homing.homing_current =
          common::A{parseDouble(value, config->homing.homing_current.value)};
    } else if (key == "jam_confirm_time_s") {
      config->homing.jam_confirm_time =
          common::S{parseDouble(value, config->homing.jam_confirm_time.value)};
    } else if (key == "backoff_distance_mm") {
      config->homing.backoff_distance =
          common::Mm{parseDouble(value, config->homing.backoff_distance.value)};
    }
  } else if (section == "clamp") {
    if (key == "target_force_n") {
      config->clamp.target_force =
          common::N{parseDouble(value, config->clamp.target_force.value)};
    } else if (key == "min_target_force_n") {
      config->clamp.min_target_force =
          common::N{parseDouble(value, config->clamp.min_target_force.value)};
    } else if (key == "max_target_force_n") {
      config->clamp.max_target_force =
          common::N{parseDouble(value, config->clamp.max_target_force.value)};
    } else if (key == "target_nut_speed_mm_s") {
      config->clamp.target_nut_speed =
          common::MmPerS{parseDouble(value, config->clamp.target_nut_speed.value)};
    } else if (key == "target_gripper_angular_speed_rad_s") {
      config->clamp.target_gripper_angular_speed = common::RadPerS{
          parseDouble(value, config->clamp.target_gripper_angular_speed.value)};
    } else if (key == "approach_distance_mm") {
      config->clamp.approach_distance =
          common::Mm{parseDouble(value, config->clamp.approach_distance.value)};
    } else if (key == "approach_nut_speed_mm_s") {
      config->clamp.approach_nut_speed = common::MmPerS{
          parseDouble(value, config->clamp.approach_nut_speed.value)};
    } else if (key == "release_distance_mm") {
      config->clamp.release_distance =
          common::Mm{parseDouble(value, config->clamp.release_distance.value)};
    } else if (key == "torque_per_force_nm_per_n") {
      config->clamp.torque_per_force =
          common::Ratio{parseDouble(value, config->clamp.torque_per_force.value)};
    } else if (key == "current_per_torque_a_per_nm") {
      config->clamp.current_per_torque =
          common::Ratio{parseDouble(value, config->clamp.current_per_torque.value)};
    } else if (key == "torque_offset_nm") {
      config->clamp.torque_offset =
          common::Nm{parseDouble(value, config->clamp.torque_offset.value)};
    } else if (key == "current_offset_a") {
      config->clamp.current_offset =
          common::A{parseDouble(value, config->clamp.current_offset.value)};
    } else if (key == "max_motor_torque_nm") {
      config->clamp.max_motor_torque =
          common::Nm{parseDouble(value, config->clamp.max_motor_torque.value)};
    }
  } else if (section == "ui") {
    if (key == "refresh_period_s") {
      config->ui.refresh_period =
          common::S{parseDouble(value, config->ui.refresh_period.value)};
    } else if (key == "log_capacity") {
      config->ui.log_capacity = parseU32(value, config->ui.log_capacity);
    } else if (key == "default_target_force_n") {
      config->ui.default_target_force =
          common::N{parseDouble(value, config->ui.default_target_force.value)};
    } else if (key == "default_clamp_speed_mm_s") {
      config->ui.default_clamp_speed = common::MmPerS{
          parseDouble(value, config->ui.default_clamp_speed.value)};
    }
  }
}

}  // namespace

GripperConfig defaultConfig() {
  return GripperConfig{};
}

ConfigLoadResult loadFromFile(const std::filesystem::path& path) {
  ConfigLoadResult result{};
  result.config = defaultConfig();

  std::ifstream file(path);
  if (!file.is_open()) {
    result.code = common::ErrorCode::ConfigMissing;
    result.message = "configuration file not found: " + path.string();
    return result;
  }

  std::string section;
  std::string line;
  while (std::getline(file, line)) {
    line = stripComment(std::move(line));
    if (line.empty()) {
      continue;
    }

    if (line.back() == ':' && line.find(':') == line.size() - 1) {
      section = trim(line.substr(0, line.size() - 1));
      continue;
    }

    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    const std::string value = trim(line.substr(colon + 1));
    applyField(section, key, value, &result.config);
  }

  result.code = common::ErrorCode::Ok;
  result.message = "configuration loaded: " + path.string();
  return result;
}

}  // namespace gripper::config
