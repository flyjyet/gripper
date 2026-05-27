#pragma once

#include <cstdint>

#include "common/units.hpp"
#include "hardware_interface/can_frame.hpp"
#include "hardware_interface/motor_types.hpp"

namespace gripper::hardware_interface::damiao {

enum class DamiaoControlMode : std::uint8_t {
  Mit = 1,
  PositionVelocity = 2,
  Velocity = 3,
  PositionForce = 4,
};

struct DamiaoIds {
  std::uint32_t motor_id{0};
  std::uint32_t host_id{0};
};

struct DamiaoFrameOptions {
  bool motor_frames_canfd{true};
  bool bitrate_switch{true};
  bool command_id_includes_mode_offset{false};
};

struct DamiaoMotorLimits {
  common::Rad position{12.5};
  common::RadPerS velocity{30.0};
  common::Nm torque{10.0};
};

struct DamiaoFeedback {
  MotorId motor_id{};
  std::uint8_t error_code{0};
  std::uint16_t raw_position_counts{0};
  std::uint16_t raw_velocity_counts{0};
  std::uint16_t raw_torque_counts{0};
  common::Rad position{};
  common::RadPerS velocity{};
  common::Nm torque{};
  common::DegC mos_temperature{};
  common::DegC rotor_temperature{};
  bool enabled{false};
  bool fault{false};
};

struct DamiaoRegisterValue {
  MotorId motor_id{};
  std::uint8_t operation{0};
  std::uint8_t register_id{0};
  double value{0.0};
  std::uint32_t raw_u32{0};
  bool is_integer{false};
};

// DM-J4310 compatible CAN/CAN-FD packet helpers. The helpers intentionally
// return project CanFrame values instead of exposing any vendor SDK type.
class DamiaoProtocol {
 public:
  [[nodiscard]] static bool isValidCanFdFrame(const CanFrame& frame) noexcept;
  [[nodiscard]] static std::uint32_t modeOffset(DamiaoControlMode mode) noexcept;
  [[nodiscard]] static std::uint32_t commandId(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options) noexcept;
  [[nodiscard]] static CanFrame makeEnableCommand(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeDisabledCommand(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeDisabledCommand(DamiaoIds ids);
  [[nodiscard]] static CanFrame makeClearFaultCommand(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeSetZeroCommand(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeSetControlModeCommand(
      DamiaoIds ids, DamiaoControlMode mode,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeReadRegisterCommand(
      DamiaoIds ids, std::uint8_t register_id,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeRefreshCommand(
      DamiaoIds ids, const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeVelocityCommand(
      DamiaoIds ids, common::RadPerS velocity,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makePositionVelocityCommand(
      DamiaoIds ids, common::Rad position, common::RadPerS velocity,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makePositionForceCommand(
      DamiaoIds ids, common::Rad position, common::RadPerS velocity_limit,
      common::A current_limit, common::A max_phase_current,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static CanFrame makeMitTorqueCommand(
      DamiaoIds ids, common::Nm torque, DamiaoMotorLimits limits,
      const DamiaoFrameOptions& options);
  [[nodiscard]] static bool parseFeedback(const CanFrame& frame, DamiaoIds ids,
                                          DamiaoMotorLimits limits,
                                          DamiaoFeedback* feedback);
  [[nodiscard]] static bool parseRegisterResponse(
      const CanFrame& frame, DamiaoIds ids, DamiaoRegisterValue* value);
};

}  // namespace gripper::hardware_interface::damiao
