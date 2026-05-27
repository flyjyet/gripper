#pragma once

#include <memory>
#include <string>

#include "common/result.hpp"
#include "config/gripper_config.hpp"
#include "controller/gripper_controller.hpp"
#include "ui/runtime_log_model.hpp"
#include "ui/ui_view_model.hpp"

namespace gripper::ui {

class UiController {
 public:
  explicit UiController(std::unique_ptr<controller::GripperController> controller);

  [[nodiscard]] common::Result configure(
      const gripper::config::GripperConfig& config);
  [[nodiscard]] common::Result configureDefault();
  [[nodiscard]] common::Result connect();
  [[nodiscard]] common::Result disconnect();
  [[nodiscard]] common::Result enable();
  [[nodiscard]] common::Result disable();
  [[nodiscard]] common::Result enterMotorBringupMode(bool unloaded_confirmed);
  [[nodiscard]] common::Result exitMotorBringupMode();
  [[nodiscard]] common::Result refreshMotorBringupFeedback();
  [[nodiscard]] common::Result runMotorBringupCommunicationProbe();
  [[nodiscard]] common::Result enableMotorBringupOutput();
  [[nodiscard]] common::Result disableMotorBringupOutput();
  [[nodiscard]] common::Result jogMotorBringup(
      common::Rad relative_motor_position, common::RadPerS max_motor_velocity,
      common::A max_motor_current, common::S pulse_duration);
  [[nodiscard]] common::Result moveMotorBringupRelativeTurns(
      common::Ratio relative_motor_revolutions,
      common::RadPerS max_motor_velocity,
      common::A feedback_current_limit, common::S timeout);
  [[nodiscard]] common::Result runPreSelfCheck();
  [[nodiscard]] common::Result home();
  [[nodiscard]] common::Result learnTravelLimits();
  [[nodiscard]] common::Result runMotionHealthCheck();
  [[nodiscard]] common::Result clampForce(common::N target_force,
                                          common::MmPerS speed);
  [[nodiscard]] common::Result release();
  [[nodiscard]] common::Result moveNutStroke(common::Mm target_nut_stroke,
                                             common::MmPerS speed,
                                             bool unloaded_confirmed = false);
  [[nodiscard]] common::Result stop();
  [[nodiscard]] common::Result clearFault();
  [[nodiscard]] common::Result update();
  void appendLog(std::string message);

  [[nodiscard]] UiViewModel viewModel() const;
  [[nodiscard]] const RuntimeLogModel& logModel() const noexcept;
  [[nodiscard]] bool hasConfig() const noexcept;
  [[nodiscard]] const config::GripperConfig& config() const noexcept;

 private:
  common::Result logResult(const std::string& action, common::Result result);

  std::unique_ptr<controller::GripperController> controller_{};
  RuntimeLogModel log_model_{};
  config::GripperConfig config_{};
  bool config_valid_{false};
};

}  // namespace gripper::ui
