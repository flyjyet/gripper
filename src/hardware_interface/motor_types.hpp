#pragma once

#include <array>
#include <cstdint>

#include "common/timestamp.hpp"
#include "common/units.hpp"

namespace gripper::hardware_interface {

// Control mode requested by upper layers. The hardware implementation maps
// these generic modes to device-specific protocol details.
enum class MotorControlMode : std::uint8_t {
  Disabled,
  Position,
  Velocity,
  Current,
  Torque,
  PositionVelocityTorque,
};

// Stable motor identity used by hardware implementations and logs.
struct MotorId {
  std::uint32_t value{0};
};

// Command values are expressed in motor-side units:
// - position: rad
// - velocity: rad/s
// - current: A
// - torque: Nm
// - temperature_limit: deg C, optional safety metadata for implementations
//   that support command-side thermal limits.
struct MotorCommand {
  MotorControlMode control_mode{MotorControlMode::Disabled};
  common::Rad target_position{};
  common::RadPerS target_velocity{};
  common::A target_current{};
  common::Nm target_torque{};
  common::DegC temperature_limit{};
  bool enable{false};
  bool clear_fault{false};
  common::Timestamp timestamp{};
};

// Feedback values are expressed in motor-side units:
// - position: continuous multi-turn rad when the hardware implementation can
//   unwrap bounded encoder feedback
// - velocity: rad/s
// - current: A
// - torque: Nm
// - temperature: deg C
struct MotorFeedback {
  common::Rad position{};
  // Optional raw single-frame encoder position after vendor conversion. This
  // stays bounded by the vendor feedback range and is diagnostic only.
  common::Rad wrapped_position{};
  bool wrapped_position_valid{false};
  common::RadPerS velocity{};
  common::A current{};
  common::Nm torque{};
  common::DegC temperature{};
  // Optional vendor raw encoder position. Damiao feedback exposes this as the
  // 16-bit q_uint value before conversion to radians; simulators may leave it
  // invalid.
  std::uint32_t raw_position_counts{0};
  bool raw_position_counts_valid{false};
  // Optional raw vendor feedback frame. This is diagnostic data for UI/logging
  // only; controller logic must not parse protocol bytes from this snapshot.
  std::uint32_t raw_feedback_frame_id{0};
  std::uint8_t raw_feedback_frame_length{0};
  std::array<std::uint8_t, 64> raw_feedback_frame_data{};
  bool raw_feedback_frame_valid{false};
  // Optional runtime device limits used to decode and bound vendor commands.
  // Damiao reports these as P_MAX/VMAX/TMAX registers during connect.
  common::Rad runtime_position_limit{};
  common::RadPerS runtime_velocity_limit{};
  common::Nm runtime_torque_limit{};
  bool runtime_limits_valid{false};
  bool enabled{false};
  bool fault{false};
  common::Timestamp timestamp{};
};

}  // namespace gripper::hardware_interface
