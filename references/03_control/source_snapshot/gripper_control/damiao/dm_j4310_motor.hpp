#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "gripper_control/damiao/can_bus.hpp"
#include "gripper_control/damiao/dm_j4310_protocol.hpp"
#include "gripper_control/motor_interface.hpp"

namespace gripper_control::damiao {

struct DmJ4310Config {
  std::uint8_t motor_id = 1;
  std::uint8_t master_id = 0;
  double screw_lead_mm = 2.0;
  double gear_ratio = 10.0;
  double max_phase_current_A = 20.0;
  double torque_per_amp_Nm = 0.625;
  double stroke_sign = 1.0;
  double feedback_timeout_s = 0.20;
  bool switch_mode_on_command = false;
  bool use_pvt_for_current_limit = true;
  LinearMappingRange mapping;
};

class DmJ4310Motor : public MotorInterface {
 public:
  DmJ4310Motor(std::shared_ptr<CanBus> bus, DmJ4310Config config = {});

  bool enable() override;
  bool disable() override;
  bool resetFault() override;
  bool sendCommand(const MotorCommand& command) override;
  bool readFeedback(MotorFeedback& feedback) override;
  bool setStrokeZero() override;

  bool switchMode(DmControlMode mode);
  bool readRegisterFloat(DmRegister reg, float& value);
  bool readRegisterUint32(DmRegister reg, std::uint32_t& value);
  bool writeRegisterFloat(DmRegister reg, float value);
  bool writeRegisterUint32(DmRegister reg, std::uint32_t value);

  const DmJ4310Config& config() const;

 private:
  double nowSeconds() const;
  std::uint32_t specialCommandId() const;
  double strokeToOutputRad(double stroke_mm) const;
  double outputRadToStroke(double output_rad) const;
  double velocityMmSToOutputRadS(double velocity_mm_s) const;
  double outputRadSToVelocityMmS(double velocity_rad_s) const;
  double currentToOutputTorque(double current_A) const;
  double currentLimitNormalized(double current_A) const;
  bool sendStopForCurrentMode();
  bool waitForFeedback(DmFeedback& feedback, int timeout_ms);
  void applyParsedFeedback(const DmFeedback& dm_feedback, MotorFeedback& feedback);
  std::uint32_t mapDriveFault(DmErrorCode error) const;

  std::shared_ptr<CanBus> bus_;
  DmJ4310Config config_;
  MotorFeedback last_feedback_;
  double zero_output_rad_ = 0.0;
  double last_output_rad_ = 0.0;
  bool have_zero_ = false;
  bool enabled_commanded_ = false;
  DmControlMode current_mode_ = DmControlMode::Velocity;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace gripper_control::damiao
