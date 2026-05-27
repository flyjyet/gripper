#include "gripper_control/damiao/dm_j4310_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace gripper_control::damiao {

namespace {

constexpr std::uint32_t kPositionVelocityBaseId = 0x100;
constexpr std::uint32_t kVelocityBaseId = 0x200;
constexpr std::uint32_t kPvtBaseId = 0x300;
constexpr std::uint32_t kRegisterAccessId = 0x7FF;

void putFloatLe(CanFrame& frame, std::size_t offset, float value) {
  std::uint32_t raw = 0;
  static_assert(sizeof(raw) == sizeof(value), "float must be 32-bit");
  std::memcpy(&raw, &value, sizeof(value));
  frame.data[offset + 0] = static_cast<std::uint8_t>(raw & 0xFF);
  frame.data[offset + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
  frame.data[offset + 2] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
  frame.data[offset + 3] = static_cast<std::uint8_t>((raw >> 24) & 0xFF);
}

float readFloatLe(const CanFrame& frame, std::size_t offset) {
  const std::uint32_t raw = static_cast<std::uint32_t>(frame.data[offset + 0]) |
                            (static_cast<std::uint32_t>(frame.data[offset + 1]) << 8) |
                            (static_cast<std::uint32_t>(frame.data[offset + 2]) << 16) |
                            (static_cast<std::uint32_t>(frame.data[offset + 3]) << 24);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void putUint16Le(CanFrame& frame, std::size_t offset, std::uint16_t value) {
  frame.data[offset + 0] = static_cast<std::uint8_t>(value & 0xFF);
  frame.data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
}

void putUint32Le(CanFrame& frame, std::size_t offset, std::uint32_t value) {
  frame.data[offset + 0] = static_cast<std::uint8_t>(value & 0xFF);
  frame.data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  frame.data[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFF);
  frame.data[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFF);
}

std::uint32_t readUint32Le(const CanFrame& frame, std::size_t offset) {
  return static_cast<std::uint32_t>(frame.data[offset + 0]) |
         (static_cast<std::uint32_t>(frame.data[offset + 1]) << 8) |
         (static_cast<std::uint32_t>(frame.data[offset + 2]) << 16) |
         (static_cast<std::uint32_t>(frame.data[offset + 3]) << 24);
}

CanFrame specialCommand(std::uint32_t control_id, std::uint8_t suffix) {
  CanFrame frame;
  frame.id = control_id;
  frame.dlc = 8;
  frame.data.fill(0xFF);
  frame.data[7] = suffix;
  return frame;
}

std::uint16_t normalizedToU16(double normalized, double scale) {
  const auto scaled = static_cast<int>(std::lround(std::clamp(normalized, 0.0, 1.0) * scale));
  return static_cast<std::uint16_t>(std::clamp(scaled, 0, static_cast<int>(scale)));
}

}  // namespace

std::uint16_t floatToUint(double value, double min, double max, std::uint8_t bits) {
  if (bits == 0 || bits > 16 || !(max > min)) {
    return 0;
  }
  const double span = max - min;
  const double max_int = static_cast<double>((1U << bits) - 1U);
  const double clamped = std::clamp(value, min, max);
  return static_cast<std::uint16_t>(std::lround((clamped - min) * max_int / span));
}

double uintToFloat(std::uint16_t value, double min, double max, std::uint8_t bits) {
  if (bits == 0 || bits > 16 || !(max > min)) {
    return min;
  }
  const std::uint16_t mask = static_cast<std::uint16_t>((1U << bits) - 1U);
  const double max_int = static_cast<double>(mask);
  return static_cast<double>(value & mask) * (max - min) / max_int + min;
}

CanFrame packEnable(std::uint32_t control_id) { return specialCommand(control_id, 0xFC); }

CanFrame packDisable(std::uint32_t control_id) { return specialCommand(control_id, 0xFD); }

CanFrame packSaveZero(std::uint32_t control_id) { return specialCommand(control_id, 0xFE); }

CanFrame packClearError(std::uint32_t control_id) { return specialCommand(control_id, 0xFB); }

CanFrame packVelocityCommand(std::uint8_t motor_id, double velocity_rad_s) {
  CanFrame frame;
  frame.id = kVelocityBaseId + motor_id;
  frame.dlc = 4;
  putFloatLe(frame, 0, static_cast<float>(velocity_rad_s));
  return frame;
}

CanFrame packPositionVelocityCommand(std::uint8_t motor_id,
                                     double position_rad,
                                     double velocity_rad_s) {
  CanFrame frame;
  frame.id = kPositionVelocityBaseId + motor_id;
  frame.dlc = 8;
  putFloatLe(frame, 0, static_cast<float>(position_rad));
  putFloatLe(frame, 4, static_cast<float>(velocity_rad_s));
  return frame;
}

CanFrame packPositionForceCommand(std::uint8_t motor_id,
                                  double position_rad,
                                  double velocity_limit_rad_s,
                                  double current_limit_normalized) {
  CanFrame frame;
  frame.id = kPvtBaseId + motor_id;
  frame.dlc = 8;
  putFloatLe(frame, 0, static_cast<float>(position_rad));
  const double velocity_norm = std::clamp(std::abs(velocity_limit_rad_s) / 100.0, 0.0, 1.0);
  putUint16Le(frame, 4, normalizedToU16(velocity_norm, 10000.0));
  putUint16Le(frame, 6, normalizedToU16(current_limit_normalized, 10000.0));
  return frame;
}

CanFrame packMitCommand(std::uint8_t motor_id,
                        double position_rad,
                        double velocity_rad_s,
                        double kp,
                        double kd,
                        double torque_Nm,
                        const LinearMappingRange& range) {
  const auto p = floatToUint(position_rad, -range.p_max_rad, range.p_max_rad, 16);
  const auto v = floatToUint(velocity_rad_s, -range.v_max_rad_s, range.v_max_rad_s, 12);
  const auto kp_u = floatToUint(kp, 0.0, 500.0, 12);
  const auto kd_u = floatToUint(kd, 0.0, 5.0, 12);
  const auto t = floatToUint(torque_Nm, -range.t_max_Nm, range.t_max_Nm, 12);

  CanFrame frame;
  frame.id = motor_id;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>((p >> 8) & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>(p & 0xFF);
  frame.data[2] = static_cast<std::uint8_t>((v >> 4) & 0xFF);
  frame.data[3] = static_cast<std::uint8_t>(((v & 0x0F) << 4) | ((kp_u >> 8) & 0x0F));
  frame.data[4] = static_cast<std::uint8_t>(kp_u & 0xFF);
  frame.data[5] = static_cast<std::uint8_t>((kd_u >> 4) & 0xFF);
  frame.data[6] = static_cast<std::uint8_t>(((kd_u & 0x0F) << 4) | ((t >> 8) & 0x0F));
  frame.data[7] = static_cast<std::uint8_t>(t & 0xFF);
  return frame;
}

CanFrame packRegisterRead(std::uint16_t can_id, DmRegister reg) {
  CanFrame frame;
  frame.id = kRegisterAccessId;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(can_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  frame.data[2] = 0x33;
  frame.data[3] = static_cast<std::uint8_t>(reg);
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;
  return frame;
}

CanFrame packRegisterWriteFloat(std::uint16_t can_id, DmRegister reg, float value) {
  CanFrame frame;
  frame.id = kRegisterAccessId;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(can_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  frame.data[2] = 0x55;
  frame.data[3] = static_cast<std::uint8_t>(reg);
  putFloatLe(frame, 4, value);
  return frame;
}

CanFrame packRegisterWriteUint32(std::uint16_t can_id, DmRegister reg, std::uint32_t value) {
  CanFrame frame;
  frame.id = kRegisterAccessId;
  frame.dlc = 8;
  frame.data[0] = static_cast<std::uint8_t>(can_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  frame.data[2] = 0x55;
  frame.data[3] = static_cast<std::uint8_t>(reg);
  putUint32Le(frame, 4, value);
  return frame;
}

CanFrame packRegisterStore(std::uint16_t can_id) {
  CanFrame frame;
  frame.id = kRegisterAccessId;
  frame.dlc = 4;
  frame.data[0] = static_cast<std::uint8_t>(can_id & 0xFF);
  frame.data[1] = static_cast<std::uint8_t>((can_id >> 8) & 0xFF);
  frame.data[2] = 0xAA;
  frame.data[3] = 0x01;
  return frame;
}

bool parseFeedback(const CanFrame& frame,
                   std::uint8_t expected_master_id,
                   const LinearMappingRange& range,
                   DmFeedback& feedback) {
  if (frame.extended || frame.id != expected_master_id || frame.dlc < 8) {
    return false;
  }

  const auto pos_u =
      static_cast<std::uint16_t>((static_cast<std::uint16_t>(frame.data[1]) << 8) | frame.data[2]);
  const auto vel_u = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame.data[3]) << 4) | ((frame.data[4] >> 4) & 0x0F));
  const auto tor_u = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(frame.data[4] & 0x0F) << 8) | frame.data[5]);

  feedback.master_id = static_cast<std::uint8_t>(frame.id & 0xFF);
  feedback.motor_id = static_cast<std::uint8_t>(frame.data[0] & 0x0F);
  feedback.error = static_cast<DmErrorCode>((frame.data[0] >> 4) & 0x0F);
  feedback.position_rad = uintToFloat(pos_u, -range.p_max_rad, range.p_max_rad, 16);
  feedback.velocity_rad_s = uintToFloat(vel_u, -range.v_max_rad_s, range.v_max_rad_s, 12);
  feedback.torque_Nm = uintToFloat(tor_u, -range.t_max_Nm, range.t_max_Nm, 12);
  feedback.mos_temperature_C = static_cast<double>(frame.data[6]);
  feedback.rotor_temperature_C = static_cast<double>(frame.data[7]);
  return true;
}

bool parseRegisterReply(const CanFrame& frame,
                        std::uint16_t expected_can_id,
                        std::uint8_t op,
                        DmRegister expected_register,
                        float& value) {
  if (frame.extended || frame.dlc < 8) {
    return false;
  }
  const std::uint16_t can_id =
      static_cast<std::uint16_t>(frame.data[0] | (static_cast<std::uint16_t>(frame.data[1]) << 8));
  if (can_id != expected_can_id || frame.data[2] != op ||
      frame.data[3] != static_cast<std::uint8_t>(expected_register)) {
    return false;
  }
  value = readFloatLe(frame, 4);
  return std::isfinite(value);
}

bool parseRegisterReply(const CanFrame& frame,
                        std::uint16_t expected_can_id,
                        std::uint8_t op,
                        DmRegister expected_register,
                        std::uint32_t& value) {
  if (frame.extended || frame.dlc < 8) {
    return false;
  }
  const std::uint16_t can_id =
      static_cast<std::uint16_t>(frame.data[0] | (static_cast<std::uint16_t>(frame.data[1]) << 8));
  if (can_id != expected_can_id || frame.data[2] != op ||
      frame.data[3] != static_cast<std::uint8_t>(expected_register)) {
    return false;
  }
  value = readUint32Le(frame, 4);
  return true;
}

}  // namespace gripper_control::damiao
