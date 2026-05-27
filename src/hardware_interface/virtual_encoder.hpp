#pragma once

#include "common/units.hpp"

namespace gripper::hardware_interface {

struct VirtualEncoderConfig {
  // One full wrap of the physical encoder signal after conversion to radians.
  // For the DM-J4310P-2EC output-side single-turn encoder this is 2*pi rad.
  common::Rad wrap_range{};
  bool enabled{true};
};

struct VirtualEncoderSample {
  common::Rad wrapped_position{};
  common::Rad continuous_position{};
  common::Rad incremental_delta{};
  bool initialized{false};
  bool wrap_corrected{false};
};

// Maps bounded single-frame encoder feedback to a continuous multi-turn
// position. Callers must feed samples frequently enough that the motor cannot
// travel more than half of wrap_range between two adjacent updates.
class MultiTurnVirtualEncoder {
 public:
  MultiTurnVirtualEncoder() = default;
  explicit MultiTurnVirtualEncoder(VirtualEncoderConfig config);

  void setConfig(VirtualEncoderConfig config) noexcept;
  void reset() noexcept;
  [[nodiscard]] VirtualEncoderSample update(common::Rad wrapped_position) noexcept;
  [[nodiscard]] VirtualEncoderSample update(
      common::Rad wrapped_position,
      common::Rad initial_continuous_position) noexcept;

  [[nodiscard]] bool initialized() const noexcept;
  [[nodiscard]] common::Rad continuousPosition() const noexcept;
  [[nodiscard]] common::Rad lastWrappedPosition() const noexcept;

 private:
  VirtualEncoderConfig config_{};
  bool initialized_{false};
  common::Rad last_wrapped_position_{};
  common::Rad continuous_position_{};
};

}  // namespace gripper::hardware_interface
