#include "controller/self_check/friction_anomaly_map.hpp"

#include <algorithm>
#include <cmath>

namespace gripper::controller::self_check {
namespace {

[[nodiscard]] double absValue(double value) {
  return std::abs(value);
}

[[nodiscard]] common::Mm sortedStart(common::Mm lhs, common::Mm rhs) {
  return common::Mm{std::min(lhs.value, rhs.value)};
}

[[nodiscard]] common::Mm sortedEnd(common::Mm lhs, common::Mm rhs) {
  return common::Mm{std::max(lhs.value, rhs.value)};
}

}  // namespace

FrictionAnomalyMap::FrictionAnomalyMap(std::uint32_t max_records)
    : max_records_(max_records) {}

bool FrictionAnomalyMap::addRecord(const FrictionAnomalyRecord& record) {
  if (max_records_ == 0 || records_.size() >= max_records_) {
    return false;
  }
  records_.push_back(record);
  return true;
}

void FrictionAnomalyMap::clear() {
  records_.clear();
}

const std::vector<FrictionAnomalyRecord>& FrictionAnomalyMap::records() const {
  return records_;
}

std::uint32_t FrictionAnomalyMap::maxRecordCount() const noexcept {
  return max_records_;
}

FrictionAnomalyDetector::FrictionAnomalyDetector(
    FrictionAnomalyDetectorConfig config)
    : config_(config), map_(config.max_records) {}

void FrictionAnomalyDetector::reset() {
  map_.clear();
  window_.clear();
  active_ = {};
  last_direction_ = MotionDirection::Unknown;
  in_anomaly_ = false;
}

void FrictionAnomalyDetector::addSample(
    const FrictionAnomalySample& sample) {
  if (!config_.enabled || sample.direction == MotionDirection::Unknown) {
    return;
  }

  if (last_direction_ != MotionDirection::Unknown &&
      sample.direction != last_direction_) {
    dropActiveAnomaly();
    window_.clear();
  }
  last_direction_ = sample.direction;

  if (!in_anomaly_) {
    if (hasUsableBaseline()) {
      const common::A baseline = baselineCurrent();
      const double ratio = absValue(sample.motor_current.value) / baseline.value;
      if (ratio >= config_.current_ratio_threshold.value) {
        enterAnomaly(sample, baseline);
        return;
      }
    }
    appendWindowSample(sample);
    return;
  }

  updateActivePeak(sample);
  const double ratio =
      active_.baseline_current.value > 0.0
          ? absValue(sample.motor_current.value) / active_.baseline_current.value
          : 0.0;
  if (ratio < config_.current_ratio_threshold.value) {
    closeAnomaly(sample);
    appendWindowSample(sample);
  }
}

void FrictionAnomalyDetector::finish() {
  dropActiveAnomaly();
}

const std::vector<FrictionAnomalyRecord>& FrictionAnomalyDetector::records()
    const {
  return map_.records();
}

bool FrictionAnomalyDetector::inAnomaly() const noexcept {
  return in_anomaly_;
}

bool FrictionAnomalyDetector::hasUsableBaseline() const {
  if (window_.size() < 3) {
    return false;
  }

  double min_position = window_.front().nut_position.value;
  double max_position = window_.front().nut_position.value;
  for (const auto& sample : window_) {
    min_position = std::min(min_position, sample.nut_position.value);
    max_position = std::max(max_position, sample.nut_position.value);
  }

  const double required_distance =
      std::max(config_.min_width.value,
               config_.sliding_window_distance.value * 0.5);
  if (required_distance > 0.0 &&
      max_position - min_position < required_distance) {
    return false;
  }

  return baselineCurrent().value >= config_.min_baseline_current.value;
}

common::A FrictionAnomalyDetector::baselineCurrent() const {
  std::vector<double> currents;
  currents.reserve(window_.size());
  for (const auto& sample : window_) {
    currents.push_back(absValue(sample.motor_current.value));
  }
  std::sort(currents.begin(), currents.end());
  if (currents.empty()) {
    return {};
  }

  const std::size_t middle = currents.size() / 2U;
  if (currents.size() % 2U == 1U) {
    return common::A{currents[middle]};
  }
  return common::A{(currents[middle - 1U] + currents[middle]) * 0.5};
}

FrictionAnomalySeverity FrictionAnomalyDetector::severityFor(
    common::Ratio current_excess_ratio) const {
  if (current_excess_ratio.value >= config_.moderate_ratio.value) {
    return FrictionAnomalySeverity::Severe;
  }
  if (current_excess_ratio.value >= config_.minor_ratio.value) {
    return FrictionAnomalySeverity::Moderate;
  }
  return FrictionAnomalySeverity::Minor;
}

FrictionAnomalyRecord FrictionAnomalyDetector::makeRecord(
    const FrictionAnomalySample& recovery_sample) const {
  const common::Mm physical_start =
      sortedStart(active_.traversal_start_position, recovery_sample.nut_position);
  const common::Mm physical_end =
      sortedEnd(active_.traversal_start_position, recovery_sample.nut_position);
  const common::Mm width{physical_end.value - physical_start.value};
  const common::Ratio ratio{
      active_.baseline_current.value > 0.0
          ? active_.peak_current.value / active_.baseline_current.value
          : 0.0};

  FrictionAnomalyRecord record{};
  record.start_position = physical_start;
  record.end_position = physical_end;
  record.center_position =
      common::Mm{(physical_start.value + physical_end.value) * 0.5};
  record.motor_position = active_.peak_motor_position;
  record.direction = active_.direction;
  record.baseline_current = active_.baseline_current;
  record.peak_current = active_.peak_current;
  record.current_excess_ratio = ratio;
  record.width = width;
  record.occurrence_count = 1;
  record.severity = severityFor(ratio);
  record.confirmation_state =
      config_.min_confirmations <= 1U
          ? FrictionAnomalyConfirmationState::Confirmed
          : FrictionAnomalyConfirmationState::Candidate;
  record.sample_speed =
      common::MmPerS{absValue(active_.peak_sample_speed.value)};
  record.temperature = active_.peak_temperature;
  record.updated = recovery_sample.timestamp;
  return record;
}

void FrictionAnomalyDetector::enterAnomaly(
    const FrictionAnomalySample& sample, common::A baseline_current) {
  active_ = {};
  active_.traversal_start_position = sample.nut_position;
  active_.peak_motor_position = sample.motor_position;
  active_.baseline_current = baseline_current;
  active_.peak_current = common::A{absValue(sample.motor_current.value)};
  active_.peak_ratio = common::Ratio{
      baseline_current.value > 0.0 ? active_.peak_current.value /
                                         baseline_current.value
                                   : 0.0};
  active_.peak_sample_speed = sample.nut_speed;
  active_.peak_temperature = sample.motor_temperature;
  active_.updated = sample.timestamp;
  active_.direction = sample.direction;
  in_anomaly_ = true;
}

void FrictionAnomalyDetector::updateActivePeak(
    const FrictionAnomalySample& sample) {
  const double current = absValue(sample.motor_current.value);
  if (current <= active_.peak_current.value) {
    return;
  }

  active_.peak_current = common::A{current};
  active_.peak_motor_position = sample.motor_position;
  active_.peak_ratio = common::Ratio{
      active_.baseline_current.value > 0.0
          ? active_.peak_current.value / active_.baseline_current.value
          : 0.0};
  active_.peak_sample_speed = sample.nut_speed;
  active_.peak_temperature = sample.motor_temperature;
  active_.updated = sample.timestamp;
}

void FrictionAnomalyDetector::closeAnomaly(
    const FrictionAnomalySample& recovery_sample) {
  const FrictionAnomalyRecord record = makeRecord(recovery_sample);
  if (record.width.value >= config_.min_width.value) {
    (void)map_.addRecord(record);
  }
  dropActiveAnomaly();
}

void FrictionAnomalyDetector::appendWindowSample(
    const FrictionAnomalySample& sample) {
  window_.push_back(sample);
  if (config_.sliding_window_distance.value <= 0.0) {
    while (window_.size() > 16U) {
      window_.pop_front();
    }
    return;
  }

  while (window_.size() > 1U &&
         absValue(window_.back().nut_position.value -
                  window_.front().nut_position.value) >
             config_.sliding_window_distance.value) {
    window_.pop_front();
  }
}

void FrictionAnomalyDetector::dropActiveAnomaly() {
  active_ = {};
  in_anomaly_ = false;
}

}  // namespace gripper::controller::self_check
