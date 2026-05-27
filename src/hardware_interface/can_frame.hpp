#pragma once

#include <array>
#include <cstdint>

#include "common/timestamp.hpp"

namespace gripper::hardware_interface {

enum class CanIdFormat : std::uint8_t {
  Standard11Bit,
  Extended29Bit,
};

enum class CanPayloadMode : std::uint8_t {
  ClassicCan,
  CanFd,
};

// CAN or CAN-FD frame independent of any adapter SDK. data_length is bytes and
// must not exceed 8 for Classic CAN or 64 for CAN-FD.
struct CanFrame {
  std::uint32_t id{0};
  CanIdFormat id_format{CanIdFormat::Standard11Bit};
  CanPayloadMode payload_mode{CanPayloadMode::ClassicCan};
  bool bitrate_switch{false};
  std::uint8_t data_length{0};
  std::array<std::uint8_t, 64> data{};
  common::Timestamp timestamp{};
};

}  // namespace gripper::hardware_interface
