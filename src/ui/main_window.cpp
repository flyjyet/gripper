#include "ui/main_window.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace gripper::ui {
namespace {

[[nodiscard]] double readOptionalDouble(std::istringstream& input,
                                        double fallback) {
  double value = fallback;
  if (input >> value) {
    return value;
  }
  return fallback;
}

void printHelp(std::ostream& output) {
  output << "commands:\n"
         << "  status\n"
         << "  connect\n"
         << "  disconnect\n"
         << "  bringup_enter_confirm_unloaded\n"
         << "  bringup_can_probe\n"
         << "  bringup_feedback\n"
         << "  bringup_enable\n"
         << "  bringup_disable\n"
         << "  bringup_jog_pos [rad] [rad_s] [amp] [sec]\n"
         << "  bringup_jog_neg [rad] [rad_s] [amp] [sec]\n"
         << "  bringup_turns [rev] [rad_s] [amp] [timeout_s]\n"
         << "  bringup_exit\n"
         << "  enable\n"
         << "  selfcheck\n"
         << "  home\n"
         << "  learn\n"
         << "  health\n"
         << "  clamp\n"
         << "  release\n"
         << "  stop\n"
         << "  clear_fault\n"
         << "  log\n"
         << "  quit\n";
}

}  // namespace

MainWindow::MainWindow(UiController* controller) : controller_(controller) {}

int MainWindow::run(std::istream& input, std::ostream& output) {
  if (controller_ == nullptr) {
    return 1;
  }

  output << "gripper console UI\n";
  printHelp(output);

  std::string line;
  while (output << "> " && std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    std::istringstream line_stream{line};
    std::string command;
    line_stream >> command;
    if (command == "quit" || command == "exit") {
      break;
    }
    if (command == "status") {
      printStatus(output);
    } else if (command == "connect") {
      (void)controller_->connect();
    } else if (command == "disconnect") {
      (void)controller_->disconnect();
    } else if (command == "bringup_enter_confirm_unloaded") {
      (void)controller_->enterMotorBringupMode(true);
    } else if (command == "bringup_can_probe") {
      (void)controller_->runMotorBringupCommunicationProbe();
      printLog(output);
    } else if (command == "bringup_feedback") {
      (void)controller_->refreshMotorBringupFeedback();
      printStatus(output);
    } else if (command == "bringup_enable") {
      (void)controller_->enableMotorBringupOutput();
    } else if (command == "bringup_disable") {
      (void)controller_->disableMotorBringupOutput();
    } else if (command == "bringup_jog_pos" ||
               command == "bringup_jog_neg") {
      const double sign = command == "bringup_jog_neg" ? -1.0 : 1.0;
      const double relative_rad =
          std::abs(readOptionalDouble(line_stream, 0.0));
      const double velocity_rad_s = readOptionalDouble(line_stream, 0.0);
      const double current_a = readOptionalDouble(line_stream, 0.0);
      const double duration_s = readOptionalDouble(line_stream, 0.0);
      (void)controller_->jogMotorBringup(
          common::Rad{sign * relative_rad}, common::RadPerS{velocity_rad_s},
          common::A{current_a}, common::S{duration_s});
      printStatus(output);
    } else if (command == "bringup_turns") {
      const double revolutions = readOptionalDouble(line_stream, 1.0);
      const double velocity_rad_s = readOptionalDouble(line_stream, 1.0);
      const double current_a = readOptionalDouble(line_stream, 1.5);
      const double timeout_s = readOptionalDouble(line_stream, 0.0);
      (void)controller_->moveMotorBringupRelativeTurns(
          common::Ratio{revolutions}, common::RadPerS{velocity_rad_s},
          common::A{current_a}, common::S{timeout_s});
      printStatus(output);
    } else if (command == "bringup_exit") {
      (void)controller_->exitMotorBringupMode();
    } else if (command == "enable") {
      (void)controller_->enable();
    } else if (command == "selfcheck") {
      (void)controller_->runPreSelfCheck();
    } else if (command == "home") {
      (void)controller_->home();
    } else if (command == "learn") {
      (void)controller_->learnTravelLimits();
    } else if (command == "health") {
      (void)controller_->runMotionHealthCheck();
    } else if (command == "clamp") {
      (void)controller_->clampForce(common::N{20.0}, common::MmPerS{1.0});
    } else if (command == "release") {
      (void)controller_->release();
    } else if (command == "stop") {
      (void)controller_->stop();
    } else if (command == "clear_fault") {
      (void)controller_->clearFault();
    } else if (command == "log") {
      printLog(output);
    } else if (command == "help") {
      printHelp(output);
    } else {
      output << "unknown command\n";
    }
  }
  (void)controller_->disconnect();
  return 0;
}

void MainWindow::printStatus(std::ostream& output) const {
  const auto view = controller_->viewModel();
  output << "controller_state=" << view.status_text
         << " connected=" << (view.state.connected ? "true" : "false")
         << " motor_enabled=" << (view.state.enabled ? "true" : "false")
         << " bringup=" << (view.state.motor_bringup_active ? "true" : "false")
         << " stroke_mm=" << std::fixed << std::setprecision(3)
         << view.state.nut_stroke.value
         << " angle_rad=" << view.state.gripper_angle.value
         << " motor_pos_rad=" << view.state.motor.position.value
         << " motor_vel_rad_s=" << view.state.motor.velocity.value
         << " motor_current_a=" << view.state.motor.current.value
         << " motor_torque_nm=" << view.state.motor.torque.value
         << " temp_c=" << view.state.motor.temperature.value
         << " force_n=" << view.state.estimated_clamp_force.value << '\n';
}

void MainWindow::printLog(std::ostream& output) const {
  for (const auto& entry : controller_->logModel().entries()) {
    output << entry.message << '\n';
  }
}

}  // namespace gripper::ui
