#pragma once

#include <array>
#include <cstdint>

namespace gripper_control::damiao {

struct CanFrame {
  std::uint32_t id = 0;
  std::uint8_t dlc = 8;
  std::array<std::uint8_t, 8> data{};
  bool extended = false;
};

class CanBus {
 public:
  virtual ~CanBus() = default;

  virtual bool send(const CanFrame& frame) = 0;
  virtual bool receive(CanFrame& frame, int timeout_ms) = 0;
};

}  // namespace gripper_control::damiao
