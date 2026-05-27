#pragma once

// Detects local lead-screw friction current peaks from externally supplied
// motion samples. This module does not command hardware, persist data, or
// decide whether a live event is a mechanical limit or jam; it only records
// candidate position-current anomalies for later self-check stages.

#include <cstdint>
#include <deque>
#include <vector>

#include "common/timestamp.hpp"
#include "common/units.hpp"
#include "controller/self_check/structure_profile.hpp"

namespace gripper::controller::self_check {

enum class FrictionAnomalySeverity : std::uint8_t {
  Minor,
  Moderate,
  Severe,
};

enum class FrictionAnomalyConfirmationState : std::uint8_t {
  Candidate,
  Confirmed,
};

struct FrictionAnomalyDetectorConfig {
  bool enabled{true};
  common::Ratio current_ratio_threshold{2.0};
  common::Mm sliding_window_distance{1.0};
  common::Mm min_width{0.05};
  std::uint32_t min_confirmations{2};
  common::Ratio minor_ratio{2.0};
  common::Ratio moderate_ratio{3.0};
  std::uint32_t max_records{20};
  common::A min_baseline_current{0.02};
};

struct FrictionAnomalySample {
  common::Mm nut_position{};
  common::Rad motor_position{};
  common::A motor_current{};
  common::MmPerS nut_speed{};
  MotionDirection direction{MotionDirection::Unknown};
  common::DegC motor_temperature{};
  common::Timestamp timestamp{};
};

struct FrictionAnomalyRecord {
  common::Mm start_position{};
  common::Mm end_position{};
  common::Mm center_position{};
  common::Rad motor_position{};
  MotionDirection direction{MotionDirection::Unknown};
  common::A baseline_current{};
  common::A peak_current{};
  common::Ratio current_excess_ratio{};
  common::Mm width{};
  std::uint32_t occurrence_count{0};
  FrictionAnomalySeverity severity{FrictionAnomalySeverity::Minor};
  FrictionAnomalyConfirmationState confirmation_state{
      FrictionAnomalyConfirmationState::Candidate};
  common::MmPerS sample_speed{};
  common::DegC temperature{};
  common::Timestamp updated{};
};

class FrictionAnomalyMap {
 public:
  explicit FrictionAnomalyMap(std::uint32_t max_records = 20);

  [[nodiscard]] bool addRecord(const FrictionAnomalyRecord& record);
  void clear();

  [[nodiscard]] const std::vector<FrictionAnomalyRecord>& records() const;
  [[nodiscard]] std::uint32_t maxRecordCount() const noexcept;

 private:
  std::uint32_t max_records_{20};
  std::vector<FrictionAnomalyRecord> records_{};
};

class FrictionAnomalyDetector {
 public:
  explicit FrictionAnomalyDetector(
      FrictionAnomalyDetectorConfig config = {});

  void reset();
  void addSample(const FrictionAnomalySample& sample);
  void finish();

  [[nodiscard]] const std::vector<FrictionAnomalyRecord>& records() const;
  [[nodiscard]] bool inAnomaly() const noexcept;

 private:
  struct ActiveAnomaly {
    common::Mm traversal_start_position{};
    common::Rad peak_motor_position{};
    common::A baseline_current{};
    common::A peak_current{};
    common::Ratio peak_ratio{};
    common::MmPerS peak_sample_speed{};
    common::DegC peak_temperature{};
    common::Timestamp updated{};
    MotionDirection direction{MotionDirection::Unknown};
  };

  [[nodiscard]] bool hasUsableBaseline() const;
  [[nodiscard]] common::A baselineCurrent() const;
  [[nodiscard]] FrictionAnomalySeverity severityFor(
      common::Ratio current_excess_ratio) const;
  [[nodiscard]] FrictionAnomalyRecord makeRecord(
      const FrictionAnomalySample& recovery_sample) const;
  void enterAnomaly(const FrictionAnomalySample& sample,
                    common::A baseline_current);
  void updateActivePeak(const FrictionAnomalySample& sample);
  void closeAnomaly(const FrictionAnomalySample& recovery_sample);
  void appendWindowSample(const FrictionAnomalySample& sample);
  void dropActiveAnomaly();

  FrictionAnomalyDetectorConfig config_{};
  FrictionAnomalyMap map_{};
  std::deque<FrictionAnomalySample> window_{};
  ActiveAnomaly active_{};
  MotionDirection last_direction_{MotionDirection::Unknown};
  bool in_anomaly_{false};
};

}  // namespace gripper::controller::self_check
