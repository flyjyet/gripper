#pragma once

#include <cstdint>
#include <string>

namespace gripper::hardware_interface {

enum class AdapterType : std::uint8_t {
  Unknown,
  UsbCanFd,
  SocketCan,
  Simulated,
};

// CAN adapter configuration. Bit rates are expressed in bit/s. The structure
// does not carry SDK handles or upper-layer settings.
struct AdapterConfig {
  AdapterType adapter_type{AdapterType::Unknown};
  std::string device_name{};
  std::string driver_library_path{};
  std::uint32_t device_index{0};
  std::uint32_t channel_index{0};
  std::uint32_t nominal_bitrate{0};
  std::uint32_t data_bitrate{0};
  bool can_fd_enabled{false};
  bool bitrate_switch_enabled{false};
};

}  // namespace gripper::hardware_interface
