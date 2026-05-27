#include "hardware_interface/damiao/damiao_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace gripper::hardware_interface::damiao {
namespace {

constexpr std::uint32_t kRegisterCommandId = 0x7FF;
constexpr std::uint8_t kRegisterRead = 0x33;
constexpr std::uint8_t kRegisterWrite = 0x55;
constexpr std::uint8_t kRefresh = 0xCC;
constexpr std::uint8_t kControlModeRegister = 0x0A;
constexpr std::uint8_t kEnableCommand = 0xFC;
constexpr std::uint8_t kDisableCommand = 0xFD;
constexpr std::uint8_t kSetZeroCommand = 0xFE;
constexpr std::uint8_t kClearFaultCommand = 0xFB;

[[nodiscard]] CanFrame makeFrame(std::uint32_t id, std::uint8_t length,
                                 const DamiaoFrameOptions& options) {
  CanFrame frame{};
  frame.id = id;
  frame.id_format = CanIdFormat::Standard11Bit;
  frame.payload_mode = options.motor_frames_canfd ? CanPayloadMode::CanFd
                                                  : CanPayloadMode::ClassicCan;
  frame.bitrate_switch =
      options.motor_frames_canfd ? options.bitrate_switch : false;
  frame.data_length = length;
  return frame;
}

[[nodiscard]] CanFrame makeControlCommand(DamiaoIds ids, DamiaoControlMode mode,
                                          std::uint8_t command,
                                          const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(DamiaoProtocol::commandId(ids, mode, options), 8,
                             options);
  for (std::uint8_t i = 0; i < 7; ++i) {
    frame.data[i] = 0xFF;
  }
  frame.data[7] = command;
  return frame;
}

void writeU32Le(std::uint32_t value, std::uint8_t* output) {
  output[0] = static_cast<std::uint8_t>(value & 0xFFU);
  output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
  output[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
  output[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void writeFloatLe(float value, std::uint8_t* output) {
  std::array<std::uint8_t, sizeof(float)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(value));
  std::copy(bytes.begin(), bytes.end(), output);
}

[[nodiscard]] double clampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

[[nodiscard]] std::uint16_t doubleToUint(double value, double low, double high,
                                         int bits) {
  if (high <= low) {
    return 0;
  }
  value = clampValue(value, low, high);
  const double max_int = static_cast<double>((1U << bits) - 1U);
  return static_cast<std::uint16_t>(
      std::round((value - low) * max_int / (high - low)));
}

[[nodiscard]] double uintToDouble(std::uint16_t value, double low, double high,
                                  int bits) {
  if (high <= low) {
    return 0.0;
  }
  const double max_int = static_cast<double>((1U << bits) - 1U);
  return static_cast<double>(value) * (high - low) / max_int + low;
}

[[nodiscard]] double positiveOrDefault(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

[[nodiscard]] float readFloatLe(const std::uint8_t* input) {
  float value = 0.0F;
  std::array<std::uint8_t, sizeof(float)> bytes{};
  std::copy(input, input + bytes.size(), bytes.begin());
  std::memcpy(&value, bytes.data(), sizeof(value));
  return value;
}

[[nodiscard]] std::uint32_t readU32Le(const std::uint8_t* input) {
  return static_cast<std::uint32_t>(input[0]) |
         (static_cast<std::uint32_t>(input[1]) << 8U) |
         (static_cast<std::uint32_t>(input[2]) << 16U) |
         (static_cast<std::uint32_t>(input[3]) << 24U);
}

[[nodiscard]] bool registerUsesIntegerEncoding(std::uint8_t register_id) {
  return (register_id >= 7U && register_id <= 10U) ||
         (register_id >= 13U && register_id <= 16U) ||
         (register_id >= 35U && register_id <= 36U);
}

[[nodiscard]] bool isRegisterResponseFrame(const CanFrame& frame,
                                           DamiaoIds ids) {
  if (frame.data_length < 4U) {
    return false;
  }
  const std::uint8_t expected_id_low =
      static_cast<std::uint8_t>(ids.motor_id & 0xFFU);
  const std::uint8_t expected_id_high =
      static_cast<std::uint8_t>((ids.motor_id >> 8U) & 0xFFU);
  const bool id_matches =
      frame.data[0] == expected_id_low && frame.data[1] == expected_id_high;
  const bool register_op =
      frame.data[2] == kRegisterRead || frame.data[2] == kRegisterWrite;
  return id_matches && register_op;
}

}  // namespace

bool DamiaoProtocol::isValidCanFdFrame(const CanFrame& frame) noexcept {
  if (frame.payload_mode == CanPayloadMode::ClassicCan) {
    return frame.data_length <= 8U;
  }
  return frame.data_length <= 64U;
}

std::uint32_t DamiaoProtocol::modeOffset(DamiaoControlMode mode) noexcept {
  switch (mode) {
    case DamiaoControlMode::Mit:
      return 0x000;
    case DamiaoControlMode::PositionVelocity:
      return 0x100;
    case DamiaoControlMode::Velocity:
      return 0x200;
    case DamiaoControlMode::PositionForce:
      return 0x300;
  }
  return 0;
}

std::uint32_t DamiaoProtocol::commandId(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) noexcept {
  if (!options.command_id_includes_mode_offset) {
    return ids.motor_id;
  }
  return ids.motor_id + modeOffset(mode);
}

CanFrame DamiaoProtocol::makeEnableCommand(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) {
  return makeControlCommand(ids, mode, kEnableCommand, options);
}

CanFrame DamiaoProtocol::makeDisabledCommand(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) {
  return makeControlCommand(ids, mode, kDisableCommand, options);
}

CanFrame DamiaoProtocol::makeDisabledCommand(DamiaoIds ids) {
  return makeDisabledCommand(ids, DamiaoControlMode::PositionForce,
                             DamiaoFrameOptions{});
}

CanFrame DamiaoProtocol::makeClearFaultCommand(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) {
  return makeControlCommand(ids, mode, kClearFaultCommand, options);
}

CanFrame DamiaoProtocol::makeSetZeroCommand(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) {
  return makeControlCommand(ids, mode, kSetZeroCommand, options);
}

CanFrame DamiaoProtocol::makeSetControlModeCommand(
    DamiaoIds ids, DamiaoControlMode mode,
    const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(kRegisterCommandId, 8, options);
  frame.data[0] = static_cast<std::uint8_t>(ids.motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((ids.motor_id >> 8U) & 0xFFU);
  frame.data[2] = kRegisterWrite;
  frame.data[3] = kControlModeRegister;
  writeU32Le(static_cast<std::uint32_t>(mode), &frame.data[4]);
  return frame;
}

CanFrame DamiaoProtocol::makeReadRegisterCommand(
    DamiaoIds ids, std::uint8_t register_id,
    const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(kRegisterCommandId, 8, options);
  frame.data[0] = static_cast<std::uint8_t>(ids.motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((ids.motor_id >> 8U) & 0xFFU);
  frame.data[2] = kRegisterRead;
  frame.data[3] = register_id;
  return frame;
}

CanFrame DamiaoProtocol::makeRefreshCommand(
    DamiaoIds ids, const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(kRegisterCommandId, 8, options);
  frame.data[0] = static_cast<std::uint8_t>(ids.motor_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>((ids.motor_id >> 8U) & 0xFFU);
  frame.data[2] = kRefresh;
  return frame;
}

CanFrame DamiaoProtocol::makeVelocityCommand(
    DamiaoIds ids, common::RadPerS velocity,
    const DamiaoFrameOptions& options) {
  CanFrame frame =
      makeFrame(ids.motor_id + modeOffset(DamiaoControlMode::Velocity), 4,
                options);
  writeFloatLe(static_cast<float>(velocity.value), frame.data.data());
  return frame;
}

CanFrame DamiaoProtocol::makePositionVelocityCommand(
    DamiaoIds ids, common::Rad position, common::RadPerS velocity,
    const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(
      ids.motor_id + modeOffset(DamiaoControlMode::PositionVelocity), 8,
      options);
  writeFloatLe(static_cast<float>(position.value), frame.data.data());
  writeFloatLe(static_cast<float>(velocity.value), &frame.data[4]);
  return frame;
}

CanFrame DamiaoProtocol::makePositionForceCommand(
    DamiaoIds ids, common::Rad position, common::RadPerS velocity_limit,
    common::A current_limit, common::A max_phase_current,
    const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(
      ids.motor_id + modeOffset(DamiaoControlMode::PositionForce), 8, options);
  writeFloatLe(static_cast<float>(position.value), frame.data.data());

  const double velocity_scaled = std::abs(velocity_limit.value) * 100.0;
  const auto velocity_uint = static_cast<std::uint16_t>(
      clampValue(std::round(velocity_scaled), 0.0, 10000.0));
  const double max_current = positiveOrDefault(max_phase_current.value, 20.0);
  const double current_norm =
      clampValue(std::abs(current_limit.value) / max_current, 0.0, 1.0);
  const auto current_uint = static_cast<std::uint16_t>(
      clampValue(std::round(current_norm * 10000.0), 0.0, 10000.0));

  frame.data[4] = static_cast<std::uint8_t>(velocity_uint & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((velocity_uint >> 8U) & 0xFFU);
  frame.data[6] = static_cast<std::uint8_t>(current_uint & 0xFFU);
  frame.data[7] = static_cast<std::uint8_t>((current_uint >> 8U) & 0xFFU);
  return frame;
}

CanFrame DamiaoProtocol::makeMitTorqueCommand(
    DamiaoIds ids, common::Nm torque, DamiaoMotorLimits limits,
    const DamiaoFrameOptions& options) {
  CanFrame frame = makeFrame(ids.motor_id, 8, options);
  const double p_max = positiveOrDefault(limits.position.value, 12.5);
  const double v_max = positiveOrDefault(limits.velocity.value, 30.0);
  const double t_max = positiveOrDefault(limits.torque.value, 10.0);

  const auto p = doubleToUint(0.0, -p_max, p_max, 16);
  const auto v = doubleToUint(0.0, -v_max, v_max, 12);
  const auto kp = doubleToUint(0.0, 0.0, 500.0, 12);
  const auto kd = doubleToUint(0.0, 0.0, 5.0, 12);
  const auto t = doubleToUint(torque.value, -t_max, t_max, 12);

  frame.data[0] = static_cast<std::uint8_t>((p >> 8U) & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(p & 0xFFU);
  frame.data[2] = static_cast<std::uint8_t>((v >> 4U) & 0xFFU);
  frame.data[3] =
      static_cast<std::uint8_t>(((v & 0x0FU) << 4U) | ((kp >> 8U) & 0x0FU));
  frame.data[4] = static_cast<std::uint8_t>(kp & 0xFFU);
  frame.data[5] = static_cast<std::uint8_t>((kd >> 4U) & 0xFFU);
  frame.data[6] =
      static_cast<std::uint8_t>(((kd & 0x0FU) << 4U) | ((t >> 8U) & 0x0FU));
  frame.data[7] = static_cast<std::uint8_t>(t & 0xFFU);
  return frame;
}

bool DamiaoProtocol::parseFeedback(const CanFrame& frame, DamiaoIds ids,
                                   DamiaoMotorLimits limits,
                                   DamiaoFeedback* feedback) {
  if (feedback == nullptr || frame.data_length < 8U) {
    return false;
  }
  if (ids.host_id != 0U && frame.id != ids.host_id) {
    return false;
  }
  if (isRegisterResponseFrame(frame, ids)) {
    return false;
  }

  const std::uint8_t motor_id = frame.data[0] & 0x0FU;
  if (ids.motor_id != 0U && motor_id != (ids.motor_id & 0x0FU)) {
    return false;
  }

  const auto q_uint =
      static_cast<std::uint16_t>((frame.data[1] << 8U) | frame.data[2]);
  const auto dq_uint = static_cast<std::uint16_t>(
      (frame.data[3] << 4U) | (frame.data[4] >> 4U));
  const auto tau_uint = static_cast<std::uint16_t>(
      ((frame.data[4] & 0x0FU) << 8U) | frame.data[5]);

  const double p_max = positiveOrDefault(limits.position.value, 12.5);
  const double v_max = positiveOrDefault(limits.velocity.value, 30.0);
  const double t_max = positiveOrDefault(limits.torque.value, 10.0);
  const std::uint8_t error_code = (frame.data[0] >> 4U) & 0x0FU;

  feedback->motor_id = MotorId{motor_id};
  feedback->error_code = error_code;
  feedback->raw_position_counts = q_uint;
  feedback->raw_velocity_counts = dq_uint;
  feedback->raw_torque_counts = tau_uint;
  feedback->position = common::Rad{uintToDouble(q_uint, -p_max, p_max, 16)};
  feedback->velocity =
      common::RadPerS{uintToDouble(dq_uint, -v_max, v_max, 12)};
  feedback->torque = common::Nm{uintToDouble(tau_uint, -t_max, t_max, 12)};
  feedback->mos_temperature = common::DegC{static_cast<double>(frame.data[6])};
  feedback->rotor_temperature =
      common::DegC{static_cast<double>(frame.data[7])};
  feedback->enabled = error_code == 1U;
  feedback->fault = error_code != 0U && error_code != 1U;
  return true;
}

bool DamiaoProtocol::parseRegisterResponse(const CanFrame& frame, DamiaoIds ids,
                                           DamiaoRegisterValue* value) {
  if (value == nullptr || frame.data_length < 8U ||
      !isRegisterResponseFrame(frame, ids)) {
    return false;
  }

  const std::uint16_t motor_id =
      static_cast<std::uint16_t>(frame.data[0]) |
      (static_cast<std::uint16_t>(frame.data[1]) << 8U);
  const std::uint8_t register_id = frame.data[3];
  const bool is_integer = registerUsesIntegerEncoding(register_id);
  const std::uint32_t raw_u32 = readU32Le(&frame.data[4]);

  value->motor_id = MotorId{motor_id};
  value->operation = frame.data[2];
  value->register_id = register_id;
  value->raw_u32 = raw_u32;
  value->is_integer = is_integer;
  value->value = is_integer ? static_cast<double>(raw_u32)
                            : static_cast<double>(readFloatLe(&frame.data[4]));
  return true;
}

}  // namespace gripper::hardware_interface::damiao
