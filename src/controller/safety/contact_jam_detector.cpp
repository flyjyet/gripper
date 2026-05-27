#include "controller/safety/contact_jam_detector.hpp"

#include <algorithm>
#include <cmath>

namespace gripper::controller::safety {
namespace {

[[nodiscard]] double absValue(double value) {
  return std::abs(value);
}

[[nodiscard]] bool isNearStrokeLimit(const ContactJamContext& context,
                                     common::Mm margin) {
  const double min_stroke =
      std::min(context.min_nut_stroke.value, context.max_nut_stroke.value);
  const double max_stroke =
      std::max(context.min_nut_stroke.value, context.max_nut_stroke.value);
  return context.current_nut_stroke.value <= min_stroke + margin.value ||
         context.current_nut_stroke.value >= max_stroke - margin.value;
}

}  // namespace

ContactJamDetector::ContactJamDetector(ContactJamDetectorConfig config)
    : config_(config) {}

void ContactJamDetector::setConfig(const ContactJamDetectorConfig& config) {
  config_ = config;
}

const ContactJamDetectorConfig& ContactJamDetector::config() const noexcept {
  return config_;
}

ContactJamResult ContactJamDetector::detect(
    const ContactJamContext& context) const {
  ContactJamResult result{};

  if (context.elapsed_in_motion.value < config_.minimum_detection_time.value) {
    return result;
  }

  const double current_rise =
      absValue(context.motor_current.value) -
      absValue(context.baseline_motion_current.value);
  const bool velocity_dropped =
      absValue(context.measured_nut_speed.value) <=
          absValue(config_.velocity_drop_threshold.value) &&
      absValue(context.commanded_nut_speed.value) >
          absValue(config_.velocity_drop_threshold.value);
  const bool near_stroke_limit = isNearStrokeLimit(context,
                                                  config_.stroke_limit_margin);

  if (near_stroke_limit && velocity_dropped &&
      current_rise >= config_.jam_current_rise.value) {
    result.state = ContactJamState::StrokeLimitSuspected;
    result.stroke_limit_suspected = true;
    result.jam_detected = config_.jam_detection_enabled;
    result.active_stop_required = config_.jam_detection_enabled;
    result.active_stop_reason = state_machine::ActiveStopReason::ContactOrJam;
    return result;
  }

  if (config_.jam_detection_enabled && velocity_dropped &&
      current_rise >= config_.jam_current_rise.value) {
    result.state = ContactJamState::JamDetected;
    result.jam_detected = true;
    result.active_stop_required = true;
    result.active_stop_reason = state_machine::ActiveStopReason::ContactOrJam;
    return result;
  }

  if (config_.contact_detection_enabled && velocity_dropped &&
      current_rise >= config_.contact_current_rise.value) {
    result.state = ContactJamState::ContactDetected;
    result.contact_detected = true;
    result.active_stop_required = true;
    result.active_stop_reason = state_machine::ActiveStopReason::ContactOrJam;
  }

  return result;
}

}  // namespace gripper::controller::safety
