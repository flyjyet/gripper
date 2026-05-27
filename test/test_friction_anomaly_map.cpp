#include "controller/self_check/friction_anomaly_map.hpp"

#include <cmath>
#include <iostream>

#include "test_utils.hpp"

namespace common = gripper::common;
namespace self_check = gripper::controller::self_check;

namespace {

self_check::FrictionAnomalyDetectorConfig testConfig() {
  self_check::FrictionAnomalyDetectorConfig config{};
  config.enabled = true;
  config.current_ratio_threshold = common::Ratio{1.5};
  config.sliding_window_distance = common::Mm{0.5};
  config.min_width = common::Mm{0.05};
  config.min_confirmations = 2;
  config.minor_ratio = common::Ratio{1.5};
  config.moderate_ratio = common::Ratio{3.0};
  config.max_records = 20;
  config.min_baseline_current = common::A{0.02};
  return config;
}

self_check::FrictionAnomalySample sample(
    double nut_position_mm, double current_a,
    self_check::MotionDirection direction = self_check::MotionDirection::Closing,
    std::int64_t index = 0) {
  self_check::FrictionAnomalySample output{};
  output.nut_position = common::Mm{nut_position_mm};
  output.motor_position = common::Rad{nut_position_mm * 3.14159265358979323846};
  output.motor_current = common::A{current_a};
  output.nut_speed = common::MmPerS{1.0};
  output.direction = direction;
  output.motor_temperature = common::DegC{32.0};
  output.timestamp = common::Timestamp::fromNanoseconds(index * 1000000);
  return output;
}

void feedBaseline(self_check::FrictionAnomalyDetector* detector,
                  self_check::MotionDirection direction =
                      self_check::MotionDirection::Closing) {
  detector->addSample(sample(0.0, 0.10, direction, 0));
  detector->addSample(sample(0.2, 0.11, direction, 1));
  detector->addSample(sample(0.4, 0.10, direction, 2));
  detector->addSample(sample(0.6, 0.09, direction, 3));
}

}  // namespace

static int test_smooth_current_does_not_create_record() {
  self_check::FrictionAnomalyDetector detector{testConfig()};
  for (int i = 0; i < 20; ++i) {
    detector.addSample(sample(static_cast<double>(i) * 0.1,
                              0.10 + (i % 2 == 0 ? 0.005 : -0.005),
                              self_check::MotionDirection::Closing, i));
  }
  detector.finish();

  TEST_ASSERT(detector.records().empty(),
              "smooth current must not create friction anomaly records");
  return 0;
}

static int test_current_peak_with_recovery_creates_candidate() {
  self_check::FrictionAnomalyDetector detector{testConfig()};
  feedBaseline(&detector);

  detector.addSample(sample(0.75, 0.22, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(0.85, 0.31, self_check::MotionDirection::Closing, 5));
  detector.addSample(sample(1.00, 0.11, self_check::MotionDirection::Closing, 6));

  TEST_ASSERT_EQ(detector.records().size(), static_cast<std::size_t>(1),
                 "one recovered current peak must be recorded");
  const auto& record = detector.records().front();
  TEST_ASSERT(record.direction == self_check::MotionDirection::Closing,
              "detected record must keep the sample direction");
  TEST_ASSERT(std::abs(record.baseline_current.value - 0.10) < 0.02,
              "baseline must come from the sliding current window");
  TEST_ASSERT(std::abs(record.peak_current.value - 0.31) < 1e-9,
              "peak current must be retained");
  TEST_ASSERT(record.current_excess_ratio.value > 2.5,
              "current excess ratio must reflect peak over baseline");
  TEST_ASSERT(record.width.value >= 0.05,
              "record width must span the recovered anomaly segment");
  TEST_ASSERT(record.confirmation_state ==
                  self_check::FrictionAnomalyConfirmationState::Candidate,
              "first-pass detector must create candidates by default");
  return 0;
}

static int test_narrow_peak_is_filtered_by_width() {
  auto config = testConfig();
  config.min_width = common::Mm{0.10};
  self_check::FrictionAnomalyDetector detector{config};
  feedBaseline(&detector);

  detector.addSample(sample(0.70, 0.25, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(0.74, 0.10, self_check::MotionDirection::Closing, 5));

  TEST_ASSERT(detector.records().empty(),
              "peak narrower than configured width must be ignored");
  return 0;
}

static int test_severity_thresholds_are_applied() {
  auto config = testConfig();
  config.current_ratio_threshold = common::Ratio{1.2};
  self_check::FrictionAnomalyDetector detector{config};
  feedBaseline(&detector);

  detector.addSample(sample(0.75, 0.13, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(0.90, 0.10, self_check::MotionDirection::Closing, 5));
  detector.addSample(sample(1.10, 0.10, self_check::MotionDirection::Closing, 6));
  detector.addSample(sample(1.30, 0.10, self_check::MotionDirection::Closing, 7));
  detector.addSample(sample(1.50, 0.10, self_check::MotionDirection::Closing, 8));
  detector.addSample(sample(1.70, 0.20, self_check::MotionDirection::Closing, 9));
  detector.addSample(sample(1.85, 0.10, self_check::MotionDirection::Closing, 10));
  detector.addSample(sample(2.05, 0.10, self_check::MotionDirection::Closing, 11));
  detector.addSample(sample(2.25, 0.10, self_check::MotionDirection::Closing, 12));
  detector.addSample(sample(2.45, 0.10, self_check::MotionDirection::Closing, 13));
  detector.addSample(sample(2.65, 0.35, self_check::MotionDirection::Closing, 14));
  detector.addSample(sample(2.80, 0.10, self_check::MotionDirection::Closing, 15));

  TEST_ASSERT_EQ(detector.records().size(), static_cast<std::size_t>(3),
                 "three recovered peaks must be recorded");
  TEST_ASSERT(detector.records()[0].severity ==
                  self_check::FrictionAnomalySeverity::Minor,
              "ratio below minor threshold must be minor");
  TEST_ASSERT(detector.records()[1].severity ==
                  self_check::FrictionAnomalySeverity::Moderate,
              "ratio above minor threshold must be moderate");
  TEST_ASSERT(detector.records()[2].severity ==
                  self_check::FrictionAnomalySeverity::Severe,
              "ratio above moderate threshold must be severe");
  return 0;
}

static int test_direction_change_keeps_records_separate() {
  self_check::FrictionAnomalyDetector detector{testConfig()};
  feedBaseline(&detector, self_check::MotionDirection::Closing);
  detector.addSample(sample(0.80, 0.25, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(1.00, 0.10, self_check::MotionDirection::Closing, 5));

  feedBaseline(&detector, self_check::MotionDirection::Opening);
  detector.addSample(sample(0.20, 0.24, self_check::MotionDirection::Opening, 10));
  detector.addSample(sample(0.00, 0.10, self_check::MotionDirection::Opening, 11));

  TEST_ASSERT_EQ(detector.records().size(), static_cast<std::size_t>(2),
                 "records from both directions must be retained");
  TEST_ASSERT(detector.records()[0].direction ==
                  self_check::MotionDirection::Closing,
              "first record must be closing");
  TEST_ASSERT(detector.records()[1].direction ==
                  self_check::MotionDirection::Opening,
              "second record must be opening");
  return 0;
}

static int test_disabled_detector_records_nothing() {
  auto config = testConfig();
  config.enabled = false;
  self_check::FrictionAnomalyDetector detector{config};
  feedBaseline(&detector);
  detector.addSample(sample(0.75, 0.30, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(1.00, 0.10, self_check::MotionDirection::Closing, 5));

  TEST_ASSERT(detector.records().empty(),
              "disabled detector must not retain records");
  return 0;
}

static int test_unrecovered_peak_is_dropped_on_finish() {
  self_check::FrictionAnomalyDetector detector{testConfig()};
  feedBaseline(&detector);
  detector.addSample(sample(0.75, 0.24, self_check::MotionDirection::Closing, 4));
  detector.addSample(sample(0.95, 0.30, self_check::MotionDirection::Closing, 5));
  detector.finish();

  TEST_ASSERT(detector.records().empty(),
              "current rise without recovery must not be classified as anomaly");
  TEST_ASSERT(!detector.inAnomaly(), "finish must clear active anomaly state");
  return 0;
}

int main() {
  std::cout << "test_friction_anomaly_map" << std::endl;

  RUN_TEST(smooth_current_does_not_create_record);
  RUN_TEST(current_peak_with_recovery_creates_candidate);
  RUN_TEST(narrow_peak_is_filtered_by_width);
  RUN_TEST(severity_thresholds_are_applied);
  RUN_TEST(direction_change_keeps_records_separate);
  RUN_TEST(disabled_detector_records_nothing);
  RUN_TEST(unrecovered_peak_is_dropped_on_finish);

  std::cout << "  all passed" << std::endl;
  return 0;
}
