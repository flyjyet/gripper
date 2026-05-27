#include "controller/nut_position_encoder/nut_position_encoder.hpp"

#include <cmath>
#include <iostream>

#include "test_utils.hpp"

namespace encoder = gripper::controller::nut_position_encoder;
namespace common = gripper::common;
namespace hardware = gripper::hardware_interface;

namespace {

constexpr double kTwoPi = 6.28318530717958647692;

hardware::MotorFeedback makeFeedback(double motor_position_rad,
                                     double motor_velocity_rad_s,
                                     std::int64_t timestamp_ns) {
  hardware::MotorFeedback feedback{};
  feedback.position = common::Rad{motor_position_rad};
  feedback.velocity = common::RadPerS{motor_velocity_rad_s};
  feedback.current = common::A{0.12};
  feedback.torque = common::Nm{0.075};
  feedback.wrapped_position = common::Rad{motor_position_rad};
  feedback.wrapped_position_valid = true;
  feedback.raw_position_counts = 1234;
  feedback.raw_position_counts_valid = true;
  feedback.timestamp = common::Timestamp::fromNanoseconds(timestamp_ns);
  return feedback;
}

int test_two_pi_motor_motion_maps_to_two_mm_nut_motion() {
  encoder::LeadScrewNutPositionEncoder nut_encoder{
      encoder::NutPositionEncoderConfig{common::Mm{2.0}, 1, common::S{0.2},
                                        common::Mm{0.0}}};

  nut_encoder.resetZero(common::Rad{10.0}, common::Mm{0.0});
  nut_encoder.update(makeFeedback(10.0 + kTwoPi, kTwoPi, 1000000000));
  const auto feedback = nut_encoder.feedback();

  TEST_ASSERT(std::abs(feedback.nut_position.value - 2.0) < 1e-9,
              "one motor revolution must move nut by the 2 mm lead");
  TEST_ASSERT(std::abs(feedback.nut_velocity.value - 2.0) < 1e-9,
              "one revolution per second must map to 2 mm/s");
  TEST_ASSERT(std::abs(feedback.motor_delta_revolutions.value - 1.0) < 1e-9,
              "motor delta revolution estimate must be one");
  TEST_ASSERT(feedback.last_motor_feedback.raw_position_counts_valid,
              "raw feedback diagnostics must be preserved");
  return 0;
}

int test_direction_sign_reverses_nut_motion() {
  encoder::LeadScrewNutPositionEncoder nut_encoder{
      encoder::NutPositionEncoderConfig{common::Mm{2.0}, -1, common::S{0.2},
                                        common::Mm{0.0}}};

  nut_encoder.resetZero(common::Rad{0.0}, common::Mm{5.0});
  nut_encoder.update(makeFeedback(kTwoPi * 0.5, kTwoPi * 0.5, 1000000000));

  const auto feedback = nut_encoder.feedback();
  TEST_ASSERT(std::abs(feedback.nut_position.value - 4.0) < 1e-9,
              "negative direction must subtract nut travel from zero point");
  TEST_ASSERT(std::abs(feedback.nut_velocity.value + 1.0) < 1e-9,
              "negative direction must reverse nut velocity");
  return 0;
}

int test_reset_zero_changes_reference_without_touching_hardware() {
  encoder::LeadScrewNutPositionEncoder nut_encoder{
      encoder::NutPositionEncoderConfig{common::Mm{2.0}, 1, common::S{0.2},
                                        common::Mm{0.0}}};

  nut_encoder.resetZero(common::Rad{3.0}, common::Mm{8.0});
  nut_encoder.update(makeFeedback(3.0 + kTwoPi * 0.25, 0.0, 1000000000));

  const auto feedback = nut_encoder.feedback();
  TEST_ASSERT(std::abs(feedback.nut_position.value - 8.5) < 1e-9,
              "reset zero must shift the nut reference point");
  TEST_ASSERT(std::abs(feedback.zero_motor_position.value - 3.0) < 1e-9,
              "zero motor position must be recorded for diagnostics");
  TEST_ASSERT(std::abs(feedback.zero_nut_position.value - 8.0) < 1e-9,
              "zero nut position must be recorded for diagnostics");
  return 0;
}

int test_first_update_uses_startup_reference() {
  encoder::LeadScrewNutPositionEncoder nut_encoder{};
  nut_encoder.configure(encoder::NutPositionEncoderConfig{
      common::Mm{2.0}, 1, common::S{0.2}, common::Mm{6.0}});

  nut_encoder.update(makeFeedback(12.0, 0.0, 1000000000));

  const auto feedback = nut_encoder.feedback();
  TEST_ASSERT(std::abs(feedback.nut_position.value - 6.0) < 1e-9,
              "first update must establish the configured startup nut position");
  TEST_ASSERT(std::abs(feedback.zero_motor_position.value - 12.0) < 1e-9,
              "first update must use the first motor position as zero");
  return 0;
}

int test_feedback_freshness_uses_timestamp_timeout() {
  encoder::LeadScrewNutPositionEncoder nut_encoder{
      encoder::NutPositionEncoderConfig{common::Mm{2.0}, 1, common::S{0.2},
                                        common::Mm{0.0}}};

  TEST_ASSERT(!nut_encoder.isFresh(common::Timestamp::fromNanoseconds(1)),
              "encoder without feedback cannot be fresh");

  nut_encoder.update(makeFeedback(0.0, 0.0, 1000000000));

  TEST_ASSERT(nut_encoder.isFresh(common::Timestamp::fromNanoseconds(1100000000)),
              "feedback younger than stale timeout must be fresh");
  TEST_ASSERT(!nut_encoder.isFresh(common::Timestamp::fromNanoseconds(1300000000)),
              "feedback older than stale timeout must be stale");
  return 0;
}

}  // namespace

int main() {
  std::cout << "test_nut_position_encoder" << std::endl;
  RUN_TEST(two_pi_motor_motion_maps_to_two_mm_nut_motion);
  RUN_TEST(direction_sign_reverses_nut_motion);
  RUN_TEST(reset_zero_changes_reference_without_touching_hardware);
  RUN_TEST(first_update_uses_startup_reference);
  RUN_TEST(feedback_freshness_uses_timestamp_timeout);
  std::cout << "  all passed" << std::endl;
  return 0;
}
