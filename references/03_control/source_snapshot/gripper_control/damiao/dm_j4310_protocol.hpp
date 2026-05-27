#pragma once

#include <cstdint>

#include "gripper_control/damiao/can_bus.hpp"

namespace gripper_control::damiao {

enum class DmControlMode : std::uint32_t {
  Mit = 1,
  PositionVelocity = 2,
  Velocity = 3,
  PositionForce = 4,
};

enum class DmRegister : std::uint8_t {
  MaxSpeed = 0x07,
  MasterId = 0x08,
  EscId = 0x09,
  ControlMode = 0x0A,
  CanBaudRate = 0x23,
  IMax = 0x3B,
  BusVoltage = 0x3C,
  MosTemperature = 0x3D,
  MotorTemperature = 0x3E,
  MotorPosition = 0x50,
  OutputPosition = 0x51,
};

enum class DmErrorCode : std::uint8_t {
  Disabled = 0x0,
  Enabled = 0x1,
  OverVoltage = 0x8,
  UnderVoltage = 0x9,
  OverCurrent = 0xA,
  MosOverTemperature = 0xB,
  MotorOverTemperature = 0xC,
  CommunicationLost = 0xD,
  Overload = 0xE,
};

struct LinearMappingRange {
  double p_max_rad = 12.5;
  double v_max_rad_s = 30.0;
  double t_max_Nm = 10.0;
};

struct DmFeedback {
  std::uint8_t master_id = 0;
  std::uint8_t motor_id = 0;
  DmErrorCode error = DmErrorCode::Disabled;
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  double torque_Nm = 0.0;
  double mos_temperature_C = 0.0;
  double rotor_temperature_C = 0.0;
};

std::uint16_t floatToUint(double value, double min, double max, std::uint8_t bits);
double uintToFloat(std::uint16_t value, double min, double max, std::uint8_t bits);

CanFrame packEnable(std::uint32_t control_id);
CanFrame packDisable(std::uint32_t control_id);
CanFrame packSaveZero(std::uint32_t control_id);
CanFrame packClearError(std::uint32_t control_id);

CanFrame packVelocityCommand(std::uint8_t motor_id, double velocity_rad_s);
CanFrame packPositionVelocityCommand(std::uint8_t motor_id,
                                     double position_rad,
                                     double velocity_rad_s);
CanFrame packPositionForceCommand(std::uint8_t motor_id,
                                  double position_rad,
                                  double velocity_limit_rad_s,
                                  double current_limit_normalized);
CanFrame packMitCommand(std::uint8_t motor_id,
                        double position_rad,
                        double velocity_rad_s,
                        double kp,
                        double kd,
                        double torque_Nm,
                        const LinearMappingRange& range);

CanFrame packRegisterRead(std::uint16_t can_id, DmRegister reg);
CanFrame packRegisterWriteFloat(std::uint16_t can_id, DmRegister reg, float value);
CanFrame packRegisterWriteUint32(std::uint16_t can_id, DmRegister reg, std::uint32_t value);
CanFrame packRegisterStore(std::uint16_t can_id);

bool parseFeedback(const CanFrame& frame,
                   std::uint8_t expected_master_id,
                   const LinearMappingRange& range,
                   DmFeedback& feedback);

bool parseRegisterReply(const CanFrame& frame,
                        std::uint16_t expected_can_id,
                        std::uint8_t op,
                        DmRegister expected_register,
                        float& value);
bool parseRegisterReply(const CanFrame& frame,
                        std::uint16_t expected_can_id,
                        std::uint8_t op,
                        DmRegister expected_register,
                        std::uint32_t& value);

}  // namespace gripper_control::damiao
