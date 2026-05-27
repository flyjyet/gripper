#pragma once

#include <memory>
#include <string>

#include "gripper_control/contact_jam_detector.hpp"
#include "gripper_control/force_command_mapper.hpp"
#include "gripper_control/friction_identifier.hpp"
#include "gripper_control/motor_interface.hpp"
#include "gripper_control/safety_limiter.hpp"
#include "gripper_control/types.hpp"

namespace gripper_control {

class GripperController {
 public:
  explicit GripperController(std::shared_ptr<MotorInterface> motor);

  void setLimits(const GripperLimits& limits);
  void setJamDetectionParams(const JamDetectionParams& params);
  void setForceMapParams(const ForceMapParams& params);

  bool initialize(double now_s);
  bool clampForce(const ClampCommand& command, double now_s);
  bool open(const OpenCommand& command, double now_s);
  bool stop();
  bool disable();
  bool resetFault();

  void update(double now_s);
  GripperStatus getStatus() const;

 private:
  void enterState(GripperState state, double now_s);
  void enterFault(GripperFault fault, const std::string& text, double now_s);
  bool readFeedback();
  bool sendVelocity(double velocity_mm_s, double velocity_limit_mm_s, double current_limit_A);
  bool sendCurrent(double current_A, double current_limit_A);
  double elapsed(double now_s) const;
  void updatePowerOnSelfCheck(double now_s);
  void updateHoming(double now_s);
  void updateFrictionIdentify(double now_s);
  void updateGuardedClosing(double now_s);
  void updateForceBuild(double now_s);
  void updateUnloadBeforeDisable(double now_s);
  void updateUnlockBeforeOpen(double now_s);
  void updateOpening(double now_s);
  void updateAntiJamRelease(double now_s);

  std::shared_ptr<MotorInterface> motor_;
  SafetyLimiter limiter_;
  ContactJamDetector detector_;
  ForceCommandMapper force_mapper_;
  FrictionIdentifier friction_identifier_;
  FrictionParams friction_;
  MotorFeedback feedback_;
  GripperStatus status_;

  ClampCommand active_clamp_;
  OpenCommand active_open_;
  double state_started_s_ = 0.0;
  double last_commanded_velocity_mm_s_ = 0.0;
  double contact_stroke_mm_ = 0.0;
  double target_current_A_ = 0.0;
  double unload_target_stroke_mm_ = 0.0;
  int anti_jam_attempt_ = 0;
};

}  // namespace gripper_control
