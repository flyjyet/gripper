#include "hardware_interface/virtual_encoder.hpp"

#include <cmath>
#include <iostream>

#include "test_utils.hpp"

namespace hardware = gripper::hardware_interface;

namespace {

int test_forward_wrap_continues_position() {
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{25.0}, true}};

  (void)encoder.update(gripper::common::Rad{11.8});
  const auto sample = encoder.update(gripper::common::Rad{-12.0});

  TEST_ASSERT(sample.wrap_corrected, "forward wrap must be corrected");
  TEST_ASSERT(std::abs(sample.continuous_position.value - 13.0) < 1e-9,
              "forward wrapped position must continue above positive range");
  return 0;
}

int test_backward_wrap_continues_position() {
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{25.0}, true}};

  (void)encoder.update(gripper::common::Rad{-11.8});
  const auto sample = encoder.update(gripper::common::Rad{12.0});

  TEST_ASSERT(sample.wrap_corrected, "backward wrap must be corrected");
  TEST_ASSERT(std::abs(sample.continuous_position.value + 13.0) < 1e-9,
              "backward wrapped position must continue below negative range");
  return 0;
}

int test_disabled_encoder_tracks_wrapped_position() {
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{25.0}, false}};

  (void)encoder.update(gripper::common::Rad{11.8});
  const auto sample = encoder.update(gripper::common::Rad{-12.0});

  TEST_ASSERT(!sample.wrap_corrected, "disabled encoder must not unwrap");
  TEST_ASSERT(std::abs(sample.continuous_position.value + 12.0) < 1e-9,
              "disabled encoder returns the latest wrapped position");
  return 0;
}

int test_multiple_wraps_accumulate_continuous_position() {
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{25.0}, true}};

  const double samples[] = {
      0.0, 8.0, -9.0, -1.0, 7.0, -10.0, -2.0, 6.0};
  hardware::VirtualEncoderSample sample{};
  bool saw_wrap = false;
  for (const double wrapped : samples) {
    sample = encoder.update(gripper::common::Rad{wrapped});
    saw_wrap = saw_wrap || sample.wrap_corrected;
  }

  TEST_ASSERT(std::abs(sample.continuous_position.value - 56.0) < 1e-9,
              "encoder must accumulate across repeated forward wraps");
  TEST_ASSERT(saw_wrap, "repeated wrap samples must be corrected");
  return 0;
}

int test_single_turn_encoder_wraps_at_two_pi() {
  constexpr double two_pi = 6.28318530717958647692;
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{two_pi}, true}};

  (void)encoder.update(gripper::common::Rad{6.1});
  const auto sample = encoder.update(gripper::common::Rad{0.1});

  TEST_ASSERT(sample.wrap_corrected, "single-turn wrap must be corrected");
  TEST_ASSERT(std::abs(sample.continuous_position.value - (6.1 + 0.28318530717958623)) <
                  1e-9,
              "single-turn encoder must continue above one revolution");
  return 0;
}

int test_raw_count_unwrap_preserves_protocol_initial_position() {
  constexpr double two_pi = 6.28318530717958647692;
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{two_pi}, true}};

  (void)encoder.update(gripper::common::Rad{6.1},
                       gripper::common::Rad{0.45});
  const auto sample = encoder.update(gripper::common::Rad{0.1},
                                     gripper::common::Rad{0.0});

  TEST_ASSERT(sample.wrap_corrected, "raw-count wrap must be corrected");
  TEST_ASSERT(std::abs(sample.continuous_position.value -
                       (0.45 + 0.28318530717958623)) < 1e-9,
              "continuous position must keep the protocol initial baseline");
  return 0;
}

int test_protocol_position_wrap_uses_motor_position_range() {
  hardware::MultiTurnVirtualEncoder encoder{
      hardware::VirtualEncoderConfig{gripper::common::Rad{25.0}, true}};

  (void)encoder.update(gripper::common::Rad{12.2});
  const auto sample = encoder.update(gripper::common::Rad{-12.3});

  TEST_ASSERT(sample.wrap_corrected,
              "protocol-position wrap at +/-P_MAX must be corrected");
  TEST_ASSERT(std::abs(sample.continuous_position.value - 12.7) < 1e-9,
              "protocol-position encoder must accumulate with 2*P_MAX range");
  return 0;
}

}  // namespace

int main() {
  std::cout << "test_virtual_encoder" << std::endl;
  RUN_TEST(forward_wrap_continues_position);
  RUN_TEST(backward_wrap_continues_position);
  RUN_TEST(disabled_encoder_tracks_wrapped_position);
  RUN_TEST(multiple_wraps_accumulate_continuous_position);
  RUN_TEST(single_turn_encoder_wraps_at_two_pi);
  RUN_TEST(raw_count_unwrap_preserves_protocol_initial_position);
  RUN_TEST(protocol_position_wrap_uses_motor_position_range);
  std::cout << "  all passed" << std::endl;
  return 0;
}
