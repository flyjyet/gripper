#include <cassert>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <iostream>

#include "gripper_control/damiao/dm_j4310_protocol.hpp"
#include "gripper_control/force_command_mapper.hpp"
#include "gripper_control/safety_limiter.hpp"

using gripper_control::damiao::CanFrame;
using gripper_control::damiao::DmErrorCode;
using gripper_control::damiao::DmFeedback;
using gripper_control::damiao::DmRegister;
using gripper_control::damiao::LinearMappingRange;
using gripper_control::damiao::floatToUint;
using gripper_control::damiao::packDisable;
using gripper_control::damiao::packEnable;
using gripper_control::damiao::packPositionForceCommand;
using gripper_control::damiao::packRegisterRead;
using gripper_control::damiao::packRegisterWriteUint32;
using gripper_control::damiao::packVelocityCommand;
using gripper_control::damiao::parseFeedback;
using gripper_control::damiao::parseRegisterReply;
using gripper_control::ForceCommandMapper;
using gripper_control::FrictionParams;
using gripper_control::GripperLimits;
using gripper_control::SafetyLimiter;

namespace {

float readFloatLe(const CanFrame& frame, std::size_t offset) {
  std::uint32_t raw = static_cast<std::uint32_t>(frame.data[offset + 0]) |
                      (static_cast<std::uint32_t>(frame.data[offset + 1]) << 8) |
                      (static_cast<std::uint32_t>(frame.data[offset + 2]) << 16) |
                      (static_cast<std::uint32_t>(frame.data[offset + 3]) << 24);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void testDamiaoProtocol() {
  const auto enable = packEnable(0x01);
  assert(enable.id == 0x01);
  assert(enable.dlc == 8);
  for (int i = 0; i < 7; ++i) {
    assert(enable.data[i] == 0xFF);
  }
  assert(enable.data[7] == 0xFC);

  const auto disable = packDisable(0x01);
  assert(disable.data[7] == 0xFD);

  const auto velocity = packVelocityCommand(1, 3.5);
  assert(velocity.id == 0x201);
  assert(velocity.dlc == 4);
  assert(std::abs(readFloatLe(velocity, 0) - 3.5F) < 1e-6F);

  const auto pvt = packPositionForceCommand(2, 1.25, 2.5, 0.125);
  assert(pvt.id == 0x302);
  assert(pvt.dlc == 8);
  assert(std::abs(readFloatLe(pvt, 0) - 1.25F) < 1e-6F);
  const std::uint16_t v_u = static_cast<std::uint16_t>(pvt.data[4] | (pvt.data[5] << 8));
  const std::uint16_t i_u = static_cast<std::uint16_t>(pvt.data[6] | (pvt.data[7] << 8));
  assert(v_u == 250);
  assert(i_u == 1250);

  const LinearMappingRange range;
  CanFrame feedback;
  feedback.id = 0;
  feedback.dlc = 8;
  const auto p_u = floatToUint(1.0, -range.p_max_rad, range.p_max_rad, 16);
  const auto v_u12 = floatToUint(2.0, -range.v_max_rad_s, range.v_max_rad_s, 12);
  const auto t_u12 = floatToUint(3.0, -range.t_max_Nm, range.t_max_Nm, 12);
  feedback.data[0] = static_cast<std::uint8_t>((static_cast<std::uint8_t>(DmErrorCode::Enabled) << 4) | 0x01);
  feedback.data[1] = static_cast<std::uint8_t>((p_u >> 8) & 0xFF);
  feedback.data[2] = static_cast<std::uint8_t>(p_u & 0xFF);
  feedback.data[3] = static_cast<std::uint8_t>((v_u12 >> 4) & 0xFF);
  feedback.data[4] = static_cast<std::uint8_t>(((v_u12 & 0x0F) << 4) | ((t_u12 >> 8) & 0x0F));
  feedback.data[5] = static_cast<std::uint8_t>(t_u12 & 0xFF);
  feedback.data[6] = 31;
  feedback.data[7] = 32;

  DmFeedback parsed;
  assert(parseFeedback(feedback, 0, range, parsed));
  assert(parsed.motor_id == 1);
  assert(parsed.error == DmErrorCode::Enabled);
  assert(std::abs(parsed.position_rad - 1.0) < 0.01);
  assert(std::abs(parsed.velocity_rad_s - 2.0) < 0.05);
  assert(std::abs(parsed.torque_Nm - 3.0) < 0.05);
  assert(parsed.mos_temperature_C == 31.0);
  assert(parsed.rotor_temperature_C == 32.0);

  const auto reg_read = packRegisterRead(0x01, DmRegister::ControlMode);
  assert(reg_read.id == 0x7FF);
  assert(reg_read.dlc == 8);
  assert(reg_read.data[0] == 0x01);
  assert(reg_read.data[2] == 0x33);
  assert(reg_read.data[3] == 0x0A);
  assert(reg_read.data[4] == 0x00);
  assert(reg_read.data[7] == 0x00);

  CanFrame reg_reply = packRegisterWriteUint32(0x01, DmRegister::ControlMode, 3);
  reg_reply.id = 0;
  std::uint32_t mode = 0;
  assert(parseRegisterReply(reg_reply, 0x01, 0x55, DmRegister::ControlMode, mode));
  assert(mode == 3);
}

}  // namespace

int main() {
  GripperLimits limits;
  limits.current_abs_max_A = 2.0;
  limits.v_close_unknown_max_mm_s = 0.3;
  SafetyLimiter limiter(limits);

  assert(std::abs(limiter.clampStroke(-1.0) - 0.0) < 1e-9);
  assert(std::abs(limiter.clampStroke(20.0) - 16.0) < 1e-9);
  assert(std::abs(limiter.clampCurrent(5.0, 1.5) - 1.5) < 1e-9);
  assert(std::abs(limiter.clampVelocity(1.0, 0.3) - 0.3) < 1e-9);

  ForceCommandMapper mapper;
  FrictionParams friction;
  friction.current_close_A = 0.10;
  friction.valid = true;
  const double current = mapper.targetCurrentForForce(150.0, 8.0, friction);
  assert(current > 0.0);
  const double force = mapper.estimateForceFromCurrent(current, 8.0, friction);
  assert(std::abs(force - 150.0) < 1e-6);

  testDamiaoProtocol();

  std::cout << "gripper_control_unit_tests passed\n";
  return 0;
}
