#include "hardware_interface/damiao/damiao_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>

#include "test_utils.hpp"

namespace dm = gripper::hardware_interface::damiao;

namespace {

float readFloatLe(const gripper::hardware_interface::CanFrame& frame,
                  std::size_t offset) {
  std::uint32_t raw = static_cast<std::uint32_t>(frame.data[offset]) |
                      (static_cast<std::uint32_t>(frame.data[offset + 1]) << 8U) |
                      (static_cast<std::uint32_t>(frame.data[offset + 2]) << 16U) |
                      (static_cast<std::uint32_t>(frame.data[offset + 3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void writeFloatLe(float value, gripper::hardware_interface::CanFrame* frame,
                  std::size_t offset) {
  std::uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(value));
  frame->data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
  frame->data[offset + 1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame->data[offset + 2] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
  frame->data[offset + 3] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
}

int test_enable_disable_use_mode_offset() {
  dm::DamiaoIds ids{0x08, 0x18};
  dm::DamiaoFrameOptions options{};
  options.command_id_includes_mode_offset = true;

  const auto enable = dm::DamiaoProtocol::makeEnableCommand(
      ids, dm::DamiaoControlMode::PositionForce, options);
  TEST_ASSERT_EQ(enable.id, 0x308U, "enable id must include position-force offset");
  TEST_ASSERT_EQ(static_cast<int>(enable.data_length), 8, "enable dlc");
  for (int i = 0; i < 7; ++i) {
    TEST_ASSERT_EQ(static_cast<int>(enable.data[i]), 0xFF, "enable prefix");
  }
  TEST_ASSERT_EQ(static_cast<int>(enable.data[7]), 0xFC, "enable command byte");

  const auto disable = dm::DamiaoProtocol::makeDisabledCommand(
      ids, dm::DamiaoControlMode::PositionForce, options);
  TEST_ASSERT_EQ(disable.id, 0x308U, "disable id must include current mode offset");
  TEST_ASSERT_EQ(static_cast<int>(disable.data[7]), 0xFD, "disable command byte");
  return 0;
}

int test_control_mode_write() {
  dm::DamiaoIds ids{0x08, 0x18};
  dm::DamiaoFrameOptions options{};
  const auto frame = dm::DamiaoProtocol::makeSetControlModeCommand(
      ids, dm::DamiaoControlMode::PositionForce, options);

  TEST_ASSERT_EQ(frame.id, 0x7FFU, "control mode command id");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[0]), 0x08, "motor id low byte");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[2]), 0x55, "write register op");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[3]), 0x0A, "CTRL_MODE RID");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[4]), 0x04, "position-force mode");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[5]), 0x00, "mode byte 1");
  return 0;
}

int test_register_read() {
  dm::DamiaoIds ids{0x08, 0x18};
  dm::DamiaoFrameOptions options{};
  const auto frame = dm::DamiaoProtocol::makeReadRegisterCommand(
      ids, 0x08, options);

  TEST_ASSERT_EQ(frame.id, 0x7FFU, "register read command id");
  TEST_ASSERT_EQ(static_cast<int>(frame.data_length), 8, "register read dlc");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[0]), 0x08, "motor id low byte");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[2]), 0x33, "read register op");
  TEST_ASSERT_EQ(static_cast<int>(frame.data[3]), 0x08, "master id RID");
  return 0;
}

int test_position_force_command() {
  dm::DamiaoIds ids{0x08, 0x18};
  dm::DamiaoFrameOptions options{};
  const auto frame = dm::DamiaoProtocol::makePositionForceCommand(
      ids, gripper::common::Rad{1.25}, gripper::common::RadPerS{2.5},
      gripper::common::A{2.5}, gripper::common::A{20.0}, options);

  TEST_ASSERT_EQ(frame.id, 0x308U, "position-force command id");
  TEST_ASSERT_EQ(static_cast<int>(frame.data_length), 8, "position-force dlc");
  TEST_ASSERT(std::abs(readFloatLe(frame, 0) - 1.25F) < 1e-6F,
              "position must be little-endian float");
  const auto velocity_u =
      static_cast<std::uint16_t>(frame.data[4] | (frame.data[5] << 8U));
  const auto current_u =
      static_cast<std::uint16_t>(frame.data[6] | (frame.data[7] << 8U));
  TEST_ASSERT_EQ(static_cast<int>(velocity_u), 250, "speed scaled by 100");
  TEST_ASSERT_EQ(static_cast<int>(current_u), 1250,
                 "current normalized to max phase current");
  return 0;
}

int test_feedback_parse() {
  dm::DamiaoIds ids{0x08, 0x18};
  dm::DamiaoMotorLimits limits{};
  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x18;
  frame.data_length = 8;
  frame.data[0] = static_cast<std::uint8_t>((1U << 4U) | 0x08U);
  frame.data[1] = 0x80;
  frame.data[2] = 0x00;
  frame.data[3] = 0x80;
  frame.data[4] = 0x08;
  frame.data[5] = 0x00;
  frame.data[6] = 31;
  frame.data[7] = 32;

  dm::DamiaoFeedback parsed{};
  TEST_ASSERT(dm::DamiaoProtocol::parseFeedback(frame, ids, limits, &parsed),
              "feedback must parse");
  TEST_ASSERT_EQ(parsed.motor_id.value, 0x08U, "parsed motor id");
  TEST_ASSERT(parsed.enabled, "error code 1 means enabled");
  TEST_ASSERT(!parsed.fault, "enabled feedback is not fault");
  TEST_ASSERT(std::abs(parsed.position.value) < 0.01, "mid position near zero");
  TEST_ASSERT(std::abs(parsed.velocity.value) < 0.02, "mid speed near zero");
  TEST_ASSERT(std::abs(parsed.torque.value) < 0.02, "mid torque near zero");
  TEST_ASSERT_EQ(static_cast<int>(parsed.rotor_temperature.value), 32,
                 "rotor temperature");
  return 0;
}

int test_register_response_is_not_feedback() {
  dm::DamiaoIds ids{0x01, 0x11};
  dm::DamiaoMotorLimits limits{};
  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x11;
  frame.data_length = 8;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  frame.data[2] = 0x55;
  frame.data[3] = 0x0A;
  frame.data[4] = 0x04;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  dm::DamiaoFeedback parsed{};
  TEST_ASSERT(!dm::DamiaoProtocol::parseFeedback(frame, ids, limits, &parsed),
              "CTRL_MODE register response must not parse as motor feedback");
  return 0;
}

int test_unknown_register_response_is_not_feedback() {
  dm::DamiaoIds ids{0x01, 0x11};
  dm::DamiaoMotorLimits limits{};
  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x11;
  frame.data_length = 8;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  frame.data[2] = 0x33;
  frame.data[3] = 0x2B;
  frame.data[4] = 0x10;
  frame.data[5] = 0x20;
  frame.data[6] = 0x30;
  frame.data[7] = 0x40;

  dm::DamiaoFeedback parsed{};
  TEST_ASSERT(!dm::DamiaoProtocol::parseFeedback(frame, ids, limits, &parsed),
              "any register response must not parse as motor feedback");
  return 0;
}

int test_parse_float_register_response() {
  dm::DamiaoIds ids{0x01, 0x11};
  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x11;
  frame.data_length = 8;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  frame.data[2] = 0x33;
  frame.data[3] = 0x15;
  writeFloatLe(50.0F, &frame, 4);

  dm::DamiaoRegisterValue parsed{};
  TEST_ASSERT(dm::DamiaoProtocol::parseRegisterResponse(frame, ids, &parsed),
              "P_MAX register response must parse");
  TEST_ASSERT_EQ(parsed.motor_id.value, 0x01U, "parsed register motor id");
  TEST_ASSERT_EQ(static_cast<int>(parsed.register_id), 0x15,
                 "parsed register id");
  TEST_ASSERT(!parsed.is_integer, "P_MAX uses float encoding");
  TEST_ASSERT(std::abs(parsed.value - 50.0) < 1e-6,
              "P_MAX float value must decode");
  return 0;
}

int test_parse_integer_register_response() {
  dm::DamiaoIds ids{0x01, 0x11};
  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x11;
  frame.data_length = 8;
  frame.data[0] = 0x01;
  frame.data[1] = 0x00;
  frame.data[2] = 0x33;
  frame.data[3] = 0x08;
  frame.data[4] = 0x11;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  dm::DamiaoRegisterValue parsed{};
  TEST_ASSERT(dm::DamiaoProtocol::parseRegisterResponse(frame, ids, &parsed),
              "MASTER_ID register response must parse");
  TEST_ASSERT(parsed.is_integer, "MASTER_ID uses integer encoding");
  TEST_ASSERT_EQ(parsed.raw_u32, 0x11U, "raw integer value");
  TEST_ASSERT(std::abs(parsed.value - 17.0) < 1e-9,
              "integer register value must decode");
  return 0;
}

int test_feedback_parse_uses_runtime_pmax() {
  dm::DamiaoIds ids{0x01, 0x11};
  dm::DamiaoMotorLimits limits{};
  limits.position = gripper::common::Rad{50.0};
  limits.velocity = gripper::common::RadPerS{20.0};
  limits.torque = gripper::common::Nm{28.0};

  gripper::hardware_interface::CanFrame frame{};
  frame.id = 0x11;
  frame.data_length = 8;
  frame.data[0] = static_cast<std::uint8_t>((1U << 4U) | 0x01U);
  frame.data[1] = 0x73;
  frame.data[2] = 0x32;
  frame.data[3] = 0x80;
  frame.data[4] = 0x08;
  frame.data[5] = 0x00;
  frame.data[6] = 31;
  frame.data[7] = 32;

  dm::DamiaoFeedback parsed{};
  TEST_ASSERT(dm::DamiaoProtocol::parseFeedback(frame, ids, limits, &parsed),
              "feedback must parse with runtime limits");
  TEST_ASSERT(std::abs(parsed.position.value + 5.001144) < 0.002,
              "feedback position must decode with runtime P_MAX");
  return 0;
}

}  // namespace

int main() {
  std::cout << "test_damiao_protocol" << std::endl;
  RUN_TEST(enable_disable_use_mode_offset);
  RUN_TEST(control_mode_write);
  RUN_TEST(register_read);
  RUN_TEST(position_force_command);
  RUN_TEST(feedback_parse);
  RUN_TEST(register_response_is_not_feedback);
  RUN_TEST(unknown_register_response_is_not_feedback);
  RUN_TEST(parse_float_register_response);
  RUN_TEST(parse_integer_register_response);
  RUN_TEST(feedback_parse_uses_runtime_pmax);
  std::cout << "  all passed" << std::endl;
  return 0;
}
