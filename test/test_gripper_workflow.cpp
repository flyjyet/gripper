#include "controller/gripper_controller.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "config/config_loader.hpp"
#include "hardware_interface/simulated/simulated_motor.hpp"
#include "test_utils.hpp"

namespace controller = gripper::controller;
namespace hardware = gripper::hardware_interface;
namespace common = gripper::common;

class SlowFeedbackMotor final : public hardware::MotorInterface {
 public:
  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "slow test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "slow test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "slow test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "slow test motor is not enabled");
    }
    ++command_count_;
    last_command_ = command;
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "slow test motor is not connected");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    feedback_.timestamp = common::Timestamp::fromNanoseconds(
        timestamp_ns_.fetch_add(20000000) + 20000000);
    feedback_.enabled = enabled_;
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

  [[nodiscard]] std::uint32_t commandCount() const noexcept {
    return command_count_.load();
  }

 private:
  bool connected_{false};
  bool enabled_{false};
  hardware::MotorCommand last_command_{};
  hardware::MotorFeedback feedback_{};
  std::atomic<std::uint32_t> command_count_{0};
  std::atomic<std::int64_t> timestamp_ns_{0};
};

class MicroMotionMotor final : public hardware::MotorInterface {
 public:
  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "micro-motion test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "micro-motion test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "micro-motion test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "micro-motion test motor is not enabled");
    }
    if (!command.enable ||
        command.control_mode == hardware::MotorControlMode::Disabled) {
      enabled_ = false;
      feedback_.enabled = false;
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
      return common::Ok();
    }

    const double delta = command.target_position.value - feedback_.position.value;
    const double step =
        std::clamp(delta, -micro_step_rad_, micro_step_rad_);
    feedback_.position = common::Rad{feedback_.position.value + step};
    feedback_.velocity = {};
    feedback_.current = common::A{std::min(std::abs(command.target_current.value),
                                           current_limit_a_)};
    feedback_.torque = common::Nm{feedback_.current.value * 0.5};
    feedback_.enabled = true;
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "micro-motion test motor is not connected");
    }
    feedback_.timestamp = common::Timestamp::fromNanoseconds(
        timestamp_ns_.fetch_add(20000000) + 20000000);
    feedback_.enabled = enabled_;
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

 private:
  bool connected_{false};
  bool enabled_{false};
  hardware::MotorFeedback feedback_{};
  std::atomic<std::int64_t> timestamp_ns_{0};
  double micro_step_rad_{0.07};
  double current_limit_a_{0.35};
};

class EndpointStartMotor final : public hardware::MotorInterface {
 public:
  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    feedback_.position = common::Rad{open_stop_position_rad_};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    feedback_.enabled = false;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "endpoint-start test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "endpoint-start test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "endpoint-start test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "endpoint-start test motor is not enabled");
    }
    if (!command.enable ||
        command.control_mode == hardware::MotorControlMode::Disabled) {
      enabled_ = false;
      feedback_.enabled = false;
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
      feedback_.timestamp = nextTimestamp();
      return common::Ok();
    }

    const double current = feedback_.position.value;
    const double target = command.target_position.value;
    const double delta = target - current;
    if (std::abs(delta) < 1e-9) {
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
    } else if (target < current) {
      moveOpening(command);
    } else {
      moveClosing(command);
    }
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "endpoint-start test motor is not connected");
    }
    feedback_.enabled = enabled_;
    feedback_.timestamp = nextTimestamp();
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

 private:
  [[nodiscard]] common::Timestamp nextTimestamp() noexcept {
    timestamp_ns_ += 20000000;
    return common::Timestamp::fromNanoseconds(timestamp_ns_);
  }

  void moveOpening(const hardware::MotorCommand& command) {
    const double current = feedback_.position.value;
    if (current <= open_stop_position_rad_ + 1e-9) {
      feedback_.position = common::Rad{open_stop_position_rad_};
      feedback_.velocity = {};
      feedback_.current = common::A{-hard_feedback_current_a_};
      feedback_.torque = common::Nm{-hard_feedback_current_a_};
      return;
    }
    const double next_position =
        std::max(open_stop_position_rad_,
                 current - std::abs(command.target_position.value - current));
    feedback_.position = common::Rad{next_position};
    feedback_.velocity =
        common::RadPerS{-std::abs(command.target_velocity.value)};
    feedback_.current =
        common::A{-std::min(std::abs(command.target_current.value), moving_current_a_)};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  void moveClosing(const hardware::MotorCommand& command) {
    const double current = feedback_.position.value;
    const double target = command.target_position.value;
    const bool low_energy_probe =
        std::abs(command.target_current.value) <= low_energy_current_limit_a_;
    const double allowed_position =
        low_energy_probe ? low_energy_retreat_rad_ : stable_retreat_limit_rad_;
    if (current >= allowed_position - 1e-9) {
      feedback_.position = common::Rad{allowed_position};
      feedback_.velocity = {};
      feedback_.current = common::A{hard_feedback_current_a_};
      feedback_.torque = common::Nm{hard_feedback_current_a_};
      return;
    }
    feedback_.position =
        common::Rad{std::min(target, allowed_position)};
    feedback_.velocity =
        common::RadPerS{std::abs(command.target_velocity.value)};
    feedback_.current =
        common::A{std::min(std::abs(command.target_current.value), moving_current_a_)};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  bool connected_{false};
  bool enabled_{false};
  hardware::MotorFeedback feedback_{};
  std::int64_t timestamp_ns_{0};
  double open_stop_position_rad_{0.0};
  double low_energy_retreat_rad_{0.17};
  double stable_retreat_limit_rad_{0.36};
  double low_energy_current_limit_a_{0.5};
  double moving_current_a_{1.0};
  double hard_feedback_current_a_{3.0};
};

class RemoteOpenStopReleaseFailMotor final : public hardware::MotorInterface {
 public:
  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    feedback_.position = common::Rad{0.0};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    feedback_.enabled = false;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "remote open-stop test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "remote open-stop test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "remote open-stop test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "remote open-stop test motor is not enabled");
    }
    if (!command.enable ||
        command.control_mode == hardware::MotorControlMode::Disabled) {
      enabled_ = false;
      feedback_.enabled = false;
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
      feedback_.timestamp = nextTimestamp();
      return common::Ok();
    }

    const double current = feedback_.position.value;
    const double target = command.target_position.value;
    if (target < current) {
      moveOpening(command);
    } else if (target > current) {
      moveClosing(command);
    } else {
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
    }
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "remote open-stop test motor is not connected");
    }
    feedback_.enabled = enabled_;
    feedback_.timestamp = nextTimestamp();
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

 private:
  [[nodiscard]] common::Timestamp nextTimestamp() noexcept {
    timestamp_ns_ += 20000000;
    return common::Timestamp::fromNanoseconds(timestamp_ns_);
  }

  void moveOpening(const hardware::MotorCommand& command) {
    const double current = feedback_.position.value;
    const bool crosses_open_stop =
        command.target_position.value <= open_stop_position_rad_ &&
        current >= open_stop_position_rad_;
    if (crosses_open_stop || current <= open_stop_position_rad_ + 1e-9) {
      wedged_at_open_stop_ = true;
      feedback_.position = common::Rad{open_stop_position_rad_};
      feedback_.velocity = {};
      feedback_.current = common::A{-hard_feedback_current_a_};
      feedback_.torque = common::Nm{-hard_feedback_current_a_};
      return;
    }
    feedback_.position = command.target_position;
    feedback_.velocity =
        common::RadPerS{-std::abs(command.target_velocity.value)};
    feedback_.current = common::A{-moving_current_a_};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  void moveClosing(const hardware::MotorCommand& command) {
    if (wedged_at_open_stop_) {
      if (!partial_release_used_) {
        partial_release_used_ = true;
        feedback_.position =
            common::Rad{feedback_.position.value + partial_release_rad_};
      }
      feedback_.velocity = {};
      feedback_.current = common::A{hard_feedback_current_a_};
      feedback_.torque = common::Nm{hard_feedback_current_a_};
      return;
    }
    feedback_.position = command.target_position;
    feedback_.velocity =
        common::RadPerS{std::abs(command.target_velocity.value)};
    feedback_.current = common::A{moving_current_a_};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  bool connected_{false};
  bool enabled_{false};
  bool wedged_at_open_stop_{false};
  bool partial_release_used_{false};
  hardware::MotorFeedback feedback_{};
  std::int64_t timestamp_ns_{0};
  double open_stop_position_rad_{-25.0};
  double partial_release_rad_{0.7};
  double moving_current_a_{0.8};
  double hard_feedback_current_a_{5.0};
};

class SingleOpenStopReleaseSucceedsMotor final : public hardware::MotorInterface {
 public:
  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    feedback_.position = common::Rad{0.0};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    feedback_.enabled = false;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(
          common::ErrorCode::ConnectionNotOpen,
          "single-open-stop test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(
          common::ErrorCode::ConnectionNotOpen,
          "single-open-stop test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(
          common::ErrorCode::ConnectionNotOpen,
          "single-open-stop test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "single-open-stop test motor is not enabled");
    }
    if (!command.enable ||
        command.control_mode == hardware::MotorControlMode::Disabled) {
      enabled_ = false;
      feedback_.enabled = false;
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
      feedback_.timestamp = nextTimestamp();
      return common::Ok();
    }

    const double current = feedback_.position.value;
    const double target = command.target_position.value;
    if (target < current) {
      moveOpening(command);
    } else if (target > current) {
      moveClosing(command);
    } else {
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
    }
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(
          common::ErrorCode::ConnectionNotOpen,
          "single-open-stop test motor is not connected");
    }
    feedback_.enabled = enabled_;
    feedback_.timestamp = nextTimestamp();
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

 private:
  [[nodiscard]] common::Timestamp nextTimestamp() noexcept {
    timestamp_ns_ += 20000000;
    return common::Timestamp::fromNanoseconds(timestamp_ns_);
  }

  void moveOpening(const hardware::MotorCommand& command) {
    const double current = feedback_.position.value;
    const bool crosses_open_stop =
        command.target_position.value <= open_stop_position_rad_ &&
        current >= open_stop_position_rad_;
    if (crosses_open_stop || current <= open_stop_position_rad_ + 1e-9) {
      feedback_.position = common::Rad{open_stop_position_rad_};
      feedback_.velocity = {};
      feedback_.current = common::A{-hard_feedback_current_a_};
      feedback_.torque = common::Nm{-hard_feedback_current_a_};
      return;
    }
    feedback_.position = command.target_position;
    feedback_.velocity =
        common::RadPerS{-std::abs(command.target_velocity.value)};
    feedback_.current = common::A{-moving_current_a_};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  void moveClosing(const hardware::MotorCommand& command) {
    feedback_.position = command.target_position;
    feedback_.velocity =
        common::RadPerS{std::abs(command.target_velocity.value)};
    feedback_.current = common::A{moving_current_a_};
    feedback_.torque = common::Nm{feedback_.current.value};
  }

  bool connected_{false};
  bool enabled_{false};
  hardware::MotorFeedback feedback_{};
  std::int64_t timestamp_ns_{0};
  double open_stop_position_rad_{-25.0};
  double moving_current_a_{0.8};
  double hard_feedback_current_a_{5.0};
};

class OpenStopSimulatedMotor final : public hardware::MotorInterface {
 public:
  OpenStopSimulatedMotor() = default;

  [[nodiscard]] common::Result connect() override {
    connected_ = true;
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disconnect() override {
    enabled_ = false;
    connected_ = false;
    feedback_.enabled = false;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result enable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "open-stop test motor is not connected");
    }
    enabled_ = true;
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result disable() override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "open-stop test motor is not connected");
    }
    enabled_ = false;
    feedback_.enabled = false;
    feedback_.velocity = {};
    feedback_.current = {};
    feedback_.torque = {};
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result sendCommand(
      const hardware::MotorCommand& command) override {
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "open-stop test motor is not connected");
    }
    if (!enabled_ && command.enable) {
      return common::Result::error(common::ErrorCode::ControlNotReady,
                                   "open-stop test motor is not enabled");
    }

    if (!command.enable ||
        command.control_mode == hardware::MotorControlMode::Disabled) {
      enabled_ = false;
      feedback_.enabled = false;
      feedback_.velocity = {};
      feedback_.current = {};
      feedback_.torque = {};
      feedback_.timestamp = nextTimestamp();
      return common::Ok();
    }

    const double target = command.target_position.value;
    const double current = feedback_.position.value;
    const bool moving_toward_open_stop = target < current;
    const bool crosses_open_stop = moving_toward_open_stop &&
                                   target <= open_stop_position_rad_ &&
                                   current >= open_stop_position_rad_;
    if (crosses_open_stop) {
      feedback_.position = common::Rad{open_stop_position_rad_};
      feedback_.velocity = {};
      feedback_.current = common::A{std::abs(command.target_current.value)};
      feedback_.torque = common::Nm{feedback_.current.value};
    } else {
      feedback_.position = command.target_position;
      feedback_.velocity = command.target_velocity;
      feedback_.current = command.target_current;
      feedback_.torque =
          command.target_torque.value != 0.0 ? command.target_torque
                                             : common::Nm{command.target_current.value};
    }
    feedback_.enabled = true;
    feedback_.timestamp = nextTimestamp();
    return common::Ok();
  }

  [[nodiscard]] common::Result readFeedback(
      hardware::MotorFeedback* feedback) override {
    if (feedback == nullptr) {
      return common::Result::error(common::ErrorCode::InvalidArgument,
                                   "feedback pointer is null");
    }
    if (!connected_) {
      return common::Result::error(common::ErrorCode::ConnectionNotOpen,
                                   "open-stop test motor is not connected");
    }
    feedback_.enabled = enabled_;
    feedback_.timestamp = nextTimestamp();
    *feedback = feedback_;
    return common::Ok();
  }

  [[nodiscard]] bool isConnected() const noexcept override {
    return connected_;
  }

  [[nodiscard]] bool isEnabled() const noexcept override {
    return enabled_;
  }

 private:
  [[nodiscard]] common::Timestamp nextTimestamp() noexcept {
    timestamp_ns_ += 1000000;
    return common::Timestamp::fromNanoseconds(timestamp_ns_);
  }

  bool connected_{false};
  bool enabled_{false};
  hardware::MotorFeedback feedback_{};
  std::int64_t timestamp_ns_{0};
  double open_stop_position_rad_{-25.0};
};

static std::unique_ptr<controller::GripperController> makeController() {
  return controller::createGripperController(
      std::make_unique<OpenStopSimulatedMotor>());
}

static gripper::config::GripperConfig workflowConfig() {
  return gripper::config::defaultConfig();
}

static gripper::config::GripperConfig fastWorkflowConfig() {
  auto config = workflowConfig();
  config.self_check.learned_profile_path.clear();
  config.self_check.pre_b_max_expansion_distance = common::Mm{2.0};
  config.self_check.travel_learning_search_distance = common::Mm{20.0};
  config.self_check.pre_b_expansion_speed = common::MmPerS{2.0};
  config.self_check.pre_b_boundary_release_speed = common::MmPerS{2.0};
  config.self_check.pre_b_soft_jam_retry_speed = common::MmPerS{2.0};
  config.self_check.dynamic_friction_speeds = {common::MmPerS{1.0}};
  config.self_check.motion_settle_timeout = common::S{0.5};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.self_check.feedback_noise_sample_time = common::S{0.02};
  config.homing.jam_confirm_time = common::S{0.02};
  return config;
}

static int configureAndConnect(controller::GripperController* gripper) {
  const auto config = fastWorkflowConfig();
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  return 0;
}

static std::filesystem::path tempSeedPath(const char* name) {
  return std::filesystem::temp_directory_path() / name;
}

static bool textContains(const std::string& text, const std::string& needle) {
  return text.find(needle) != std::string::npos;
}

static std::string readTextFile(const std::filesystem::path& path) {
  std::ifstream stream{path};
  std::ostringstream output;
  output << stream.rdbuf();
  return output.str();
}

static int test_full_simulated_workflow_reaches_ready_clamps_and_releases() {
  auto gripper = makeController();
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");

  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");
  TEST_ASSERT(gripper->runPreSelfCheck().isOk(),
              "pre-self-check must succeed");
  TEST_ASSERT(gripper->homeOpenStop().isOk(), "homing must succeed");
  auto state = gripper->state();
  TEST_ASSERT(state.homed, "homing flag must be set");
  TEST_ASSERT(!state.enabled, "homing must release and disable motor output");
  TEST_ASSERT(state.nut_stroke.value > 0.0,
              "homing must back off from the open stop after zeroing");
  TEST_ASSERT(gripper->learnTravelLimits().isOk(),
              "travel learning must succeed");
  state = gripper->state();
  TEST_ASSERT(state.learned_parameters.software_closed_limit.value > 16.0,
              "travel learning search must not stop at the old 16mm reference");
  TEST_ASSERT(std::abs(state.learned_parameters.learned_travel.value - 20.0) <
                  0.5,
              "travel learning should use the configured 20mm search distance");
  TEST_ASSERT(gripper->runMotionHealthCheck().isOk(),
              "motion health check must succeed");

  state = gripper->state();
  TEST_ASSERT(state.motion_health_checked, "health flag must be set");
  TEST_ASSERT(!state.enabled,
              "health check should leave motor output disabled");

  controller::ClampForceCommand clamp{};
  clamp.target_force = common::N{20.0};
  clamp.speed_mode = controller::ClampSpeedMode::NutLinearSpeed;
  clamp.max_nut_speed = common::MmPerS{1.0};
  TEST_ASSERT(gripper->clampByForce(clamp).isOk(),
              "target-force clamp must succeed with simulated force proxy");

  state = gripper->state();
  TEST_ASSERT(!state.enabled, "clamp must disable output after completion");
  TEST_ASSERT(state.estimated_clamp_force.value > 0.0,
              "clamp should report estimated force");

  TEST_ASSERT(gripper->release().isOk(), "release must succeed");
  state = gripper->state();
  TEST_ASSERT(!state.enabled, "release must disable output");
  TEST_ASSERT(state.estimated_clamp_force.value == 0.0,
              "release must clear estimated force");
  return 0;
}

static int test_clamp_requires_travel_learning() {
  auto gripper = makeController();
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  controller::ClampForceCommand clamp{};
  clamp.target_force = common::N{20.0};
  const auto result = gripper->clampByForce(clamp);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::ControlNotReady),
                 "clamp before travel learning must be rejected");
  return 0;
}

static int test_homing_requires_current_pre_self_check() {
  auto gripper = makeController();
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  const auto result = gripper->homeOpenStop();
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::ControlNotReady),
                 "homing before current PreSelfCheck must be rejected");
  TEST_ASSERT(!gripper->state().homed,
              "rejected homing must not set the homed flag");
  return 0;
}

static int test_manual_nut_stroke_move_requires_pre_self_check() {
  auto gripper = makeController();
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  controller::MoveNutStrokeCommand move{};
  move.target_nut_stroke = common::Mm{8.0};
  move.max_nut_speed = common::MmPerS{1.0};
  const auto result = gripper->moveToNutStroke(move);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::ControlNotReady),
                 "manual positioning before PreSelfCheck must be rejected");
  return 0;
}

static int test_manual_nut_stroke_move_uses_safe_zone_after_pre_self_check() {
  auto gripper = makeController();
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");
  TEST_ASSERT(gripper->runPreSelfCheck().isOk(),
              "pre-self-check must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(state.manual_nut_stroke_range.valid,
              "manual positioning range must be exposed after PreSelfCheck");
  TEST_ASSERT(state.manual_nut_stroke_range.low_confidence_window,
              "range before MotionHealthCheck must be a low-confidence window");
  controller::MoveNutStrokeCommand move{};
  move.target_nut_stroke = common::Mm{
      (state.manual_nut_stroke_range.open_limit.value +
       state.manual_nut_stroke_range.closed_limit.value) *
      0.5};
  move.max_nut_speed = common::MmPerS{1.0};
  TEST_ASSERT(gripper->moveToNutStroke(move).isOk(),
              "manual positioning inside exposed PreSelfCheck window must succeed");
  TEST_ASSERT(!gripper->state().enabled,
              "manual positioning must disable output after completion");

  const auto after_move_state = gripper->state();
  TEST_ASSERT(after_move_state.manual_nut_stroke_range.valid,
              "manual positioning range must remain exposed after a move");
  controller::MoveNutStrokeCommand out_of_range{};
  out_of_range.target_nut_stroke = common::Mm{
      after_move_state.manual_nut_stroke_range.closed_limit.value + 1.0};
  out_of_range.max_nut_speed = common::MmPerS{1.0};
  const auto rejected = gripper->moveToNutStroke(out_of_range);
  TEST_ASSERT_EQ(static_cast<int>(rejected.code()),
                 static_cast<int>(common::ErrorCode::OutOfRange),
                 "manual positioning outside exposed window must be rejected");
  TEST_ASSERT(!gripper->state().enabled,
              "out-of-range manual positioning must not enable motor output");

  controller::MoveNutStrokeCommand unloaded_move{};
  unloaded_move.target_nut_stroke =
      state.learned_parameters.safe_zone_closed_limit;
  unloaded_move.max_nut_speed = common::MmPerS{1.0};
  unloaded_move.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->moveToNutStroke(unloaded_move).isOk(),
              "unloaded-confirmed manual positioning may use full low-confidence boundary");
  TEST_ASSERT(!gripper->state().enabled,
              "unloaded-confirmed manual positioning must disable output after completion");
  return 0;
}

static int test_default_damiao_encoder_config_uses_protocol_position_range() {
  const auto config = gripper::config::defaultConfig();
  TEST_ASSERT(
      config.motor.encoder_unwrap_source ==
          gripper::config::EncoderUnwrapSource::ProtocolPosition,
      "default hardware config must unwrap Damiao protocol position");
  TEST_ASSERT(std::abs(config.motor.encoder_wrap_range.value) < 1e-9,
              "default wrap range must be derived from runtime P_MAX");
  TEST_ASSERT(std::abs(config.motor.max_position.value - 65.0) < 1e-9,
              "fallback P_MAX should match the current field setup");
  TEST_ASSERT(config.nut_position_encoder.direction_sign == -1,
              "default virtual nut encoder direction should match current bring-up finding");
  TEST_ASSERT(std::abs(config.safety.manual_positioning_current_limit.value -
                       1.5) < 1e-9,
              "manual slider positioning current default should support hardware bring-up");
  TEST_ASSERT(std::abs(config.safety.max_motor_current.value - 2.0) < 1e-9,
              "global hard current protection should stay above manual positioning limit");
  TEST_ASSERT(std::abs(config.safety.self_check_current_limit.value - 1.9) <
                  1e-9,
              "self-check command current should support hardware PreB scans");
  TEST_ASSERT(
      std::abs(config.self_check.pre_b_expansion_speed.value - 1.0) < 1e-9,
      "PreB expansion speed should stay in the hardware-friendly scan band");
  TEST_ASSERT(config.self_check.pre_b_expansion_speed.value >
                  config.self_check.min_speed_scan_start.value,
              "PreB expansion speed should be independent from the breakaway crawl");
  TEST_ASSERT(std::abs(config.self_check.pre_b_soft_jam_retry_speed.value -
                       1.0) < 1e-9,
              "PreB soft-jam retry speed should match the continuous scan band");
  TEST_ASSERT(std::abs(config.self_check.pre_b_soft_jam_retry_distance.value -
                       1.0) < 1e-9,
              "PreB soft-jam retry should cover larger local card points");
  TEST_ASSERT(config.self_check.dynamic_friction_speeds.size() == 4U,
              "PreB dynamic friction scan should include the faster pass-through speed");
  TEST_ASSERT(std::abs(config.self_check.dynamic_friction_speeds[3].value -
                       1.5) < 1e-9,
              "PreB dynamic friction scan should stay below the earlier 15rad/s equivalent");
  TEST_ASSERT(
      std::abs(config.safety.self_check_feedback_hard_current_limit.value -
               2.5) < 1e-9,
      "self-check feedback hard stop should allow short pass-through peaks");
  TEST_ASSERT(
      std::abs(config.safety.self_check_feedback_emergency_current_limit.value -
               3.0) < 1e-9,
      "self-check emergency feedback stop should stop end-stop peaks earlier");
  TEST_ASSERT(std::abs(config.self_check.pre_b_boundary_step.value - 1.0) <
                  1e-9,
              "PreB boundary refine should use explicit small steps");
  TEST_ASSERT(
      std::abs(config.self_check.pre_b_boundary_release_distance.value - 1.0) <
          1e-9,
      "PreB should actively release a physical boundary before learning");
  TEST_ASSERT(
      std::abs(config.self_check.pre_b_boundary_release_speed.value - 1.0) <
          1e-9,
      "PreB boundary release speed should stay in the scan band");
  TEST_ASSERT(
      std::abs(config.self_check.pre_b_boundary_release_current_limit.value -
               2.0) < 1e-9,
      "PreB boundary release should have its own current envelope");
  TEST_ASSERT(
      std::abs(config.self_check.travel_learning_speed.value - 1.2) < 1e-9,
      "travel learning speed should have enough momentum for local card points");
  TEST_ASSERT(std::abs(config.self_check.travel_learning_search_distance.value -
                       20.0) < 1e-9,
              "travel learning search should cover the current 18-20mm field travel");
  TEST_ASSERT(std::abs(config.safety.travel_learning_current_limit.value -
                       1.5) < 1e-9,
              "travel learning current should exceed the early conservative card-point limit");
  TEST_ASSERT(std::abs(config.self_check.pre_b_max_expansion_distance.value -
                       20.0) < 1e-9,
              "PreB expansion should use the current 18-20mm field travel scale");
  TEST_ASSERT(
      std::abs(config.self_check.pre_b_hard_current_confirm_time.value - 0.25) <
          1e-9,
      "PreB hard feedback current should require sustained confirmation");
  TEST_ASSERT(
      std::abs(config.self_check.friction_anomaly_avoid_margin.value - 0.3) <
          1e-9,
      "PreB learning should avoid local friction anomalies");
  TEST_ASSERT(
      std::abs(config.self_check.friction_anomaly_current_ratio_threshold.value -
               2.0) < 1e-9,
      "Friction anomaly detection should default to obvious current spikes");
  TEST_ASSERT(
      std::abs(config.self_check.friction_anomaly_minor_ratio.value - 2.0) <
          1e-9,
      "Minor friction anomalies should not include ordinary 1.5x ripple");
  TEST_ASSERT(
      std::abs(config.self_check.friction_anomaly_learning_avoid_ratio.value -
               3.0) < 1e-9,
      "PreB learning should only avoid moderate or severe friction anomalies");
  TEST_ASSERT(config.self_check.pre_b_min_learning_regions == 3U,
              "PreB learning must require at least three clean regions");
  TEST_ASSERT(config.self_check.pre_b_learning_anchor_count >=
                  config.self_check.pre_b_min_learning_regions,
              "PreB learning should try extra anchors so anomalies can be skipped");
  return 0;
}

static int test_stop_cancels_running_pre_self_check() {
  auto motor = std::make_unique<SlowFeedbackMotor>();
  auto* motor_ptr = motor.get();
  auto gripper = controller::createGripperController(std::move(motor));
  TEST_ASSERT(configureAndConnect(gripper.get()) == 0,
              "configure/connect helper must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  common::Result self_check_result =
      common::Result::error(common::ErrorCode::Unknown, "not finished");
  std::thread worker{[&]() { self_check_result = gripper->runPreSelfCheck(); }};

  std::this_thread::sleep_for(std::chrono::milliseconds{80});
  const auto stop_result = gripper->stop();
  TEST_ASSERT_EQ(static_cast<int>(stop_result.code()),
                 static_cast<int>(common::ErrorCode::SafetyActiveStop),
                 "stop must enter active stop");
  worker.join();

  TEST_ASSERT_EQ(static_cast<int>(self_check_result.code()),
                 static_cast<int>(common::ErrorCode::SafetyActiveStop),
                 "running PreSelfCheck must observe stop cancellation");
  TEST_ASSERT(!gripper->state().enabled,
              "cancelled PreSelfCheck must leave motor output disabled");
  TEST_ASSERT(motor_ptr->commandCount() > 0U,
              "test must exercise command output before cancellation");
  return 0;
}

static int test_pre_self_check_accepts_low_confidence_micro_motion() {
  auto gripper = controller::createGripperController(
      std::make_unique<MicroMotionMotor>());
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path.clear();
  config.self_check.low_confidence_motion_distance = common::Mm{0.015};
  config.self_check.motion_start_distance = common::Mm{0.05};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.self_check.motion_settle_timeout = common::S{0.3};
  config.safety.self_check_current_limit = common::A{0.4};
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  const auto result = gripper->runPreSelfCheck();
  TEST_ASSERT(result.isOk(),
              "micro-motion breakaway should produce conservative PreSelfCheck");
  const auto state = gripper->state();
  TEST_ASSERT(state.pre_self_check_completed,
              "PreSelfCheck completion flag must be set");
  TEST_ASSERT(!state.enabled, "PreSelfCheck must disable output after finish");
  TEST_ASSERT(state.learned_parameters.opening.breakaway_current.value > 0.0,
              "opening bootstrap breakaway current must be learned");
  TEST_ASSERT(state.learned_parameters.closing.breakaway_current.value > 0.0,
              "closing bootstrap breakaway current must be learned");
  TEST_ASSERT(state.learned_parameters.opening.static_friction_sample_count == 0U,
              "PreA micro-motion must not be treated as opening static friction model");
  TEST_ASSERT(state.learned_parameters.closing.static_friction_sample_count == 0U,
              "PreA micro-motion must not be treated as closing static friction model");
  TEST_ASSERT(state.learned_parameters.opening.dynamic_friction_sample_count == 0U,
              "PreA/PreB boundary micro-motion must not be treated as opening dynamic friction model");
  TEST_ASSERT(state.learned_parameters.closing.dynamic_friction_sample_count == 0U,
              "PreA/PreB boundary micro-motion must not be treated as closing dynamic friction model");
  TEST_ASSERT(state.learned_parameters.opening.stable_speed_sample_count == 0U,
              "PreA/PreB boundary micro-motion must not be treated as opening stable speed model");
  TEST_ASSERT(state.learned_parameters.closing.stable_speed_sample_count == 0U,
              "PreA/PreB boundary micro-motion must not be treated as closing stable speed model");
  return 0;
}

static int test_pre_self_check_endpoint_start_prioritizes_retreat_direction() {
  auto gripper = controller::createGripperController(
      std::make_unique<EndpointStartMotor>());
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path.clear();
  config.self_check.motion_settle_timeout = common::S{0.3};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.self_check.pre_b_max_expansion_distance = common::Mm{2.0};
  config.safety.self_check_feedback_emergency_current_limit = common::A{10.0};
  std::vector<std::string> progress;
  gripper->setProgressCallback(
      [&](std::string message) { progress.push_back(std::move(message)); });

  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  const auto result = gripper->runPreSelfCheck();
  TEST_ASSERT(result.isOk(),
              "endpoint-start self-check should degrade but complete");

  bool saw_endpoint_escape = false;
  bool saw_retreat_search_order = false;
  for (const auto& line : progress) {
    if (textContains(line, "endpoint_start_escape") &&
        textContains(line, "retreat_direction=closing") &&
        textContains(line, "suspected_boundary_direction=opening")) {
      saw_endpoint_escape = true;
    }
    if (textContains(line, "phase=PreliminaryLimitSearch") &&
        textContains(line, "search_order first=closing second=opening") &&
        textContains(line, "pre_a_open_boundary_suspected=true")) {
      saw_retreat_search_order = true;
    }
  }

  TEST_ASSERT(saw_endpoint_escape,
              "opening endpoint start must be converted to endpoint_start_escape");
  TEST_ASSERT(saw_retreat_search_order,
              "PreB must first scan away from the suspected opening endpoint");
  TEST_ASSERT(!gripper->state().enabled,
              "PreSelfCheck must leave motor output disabled");
  return 0;
}

static int test_pre_self_check_release_failure_skips_false_second_boundary() {
  auto gripper = controller::createGripperController(
      std::make_unique<RemoteOpenStopReleaseFailMotor>());
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path.clear();
  config.self_check.pre_b_max_expansion_distance = common::Mm{20.0};
  config.self_check.pre_b_expansion_speed = common::MmPerS{2.0};
  config.self_check.pre_b_boundary_release_distance = common::Mm{1.0};
  config.self_check.pre_b_boundary_release_speed = common::MmPerS{2.0};
  config.self_check.motion_settle_timeout = common::S{0.3};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.safety.self_check_feedback_hard_current_limit = common::A{2.5};
  config.safety.self_check_feedback_emergency_current_limit = common::A{4.0};
  std::vector<std::string> progress;
  gripper->setProgressCallback(
      [&](std::string message) { progress.push_back(std::move(message)); });

  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  const auto result = gripper->runPreSelfCheck();
  TEST_ASSERT(result.isOk(),
              "release-failed PreB should degrade but complete PreSelfCheck");

  bool saw_opening_limit = false;
  bool saw_release_failed = false;
  bool saw_second_direction_skipped = false;
  bool saw_learning_skipped = false;
  bool saw_false_closing_limit = false;
  bool saw_closing_search_after_release_failed = false;
  bool saw_release_current_limit = false;
  bool release_failed_seen = false;
  for (const auto& line : progress) {
    if (textContains(line, "suspected mechanical limit direction=opening")) {
      saw_opening_limit = true;
    }
    if (textContains(line, "boundary_release_failed")) {
      saw_release_failed = true;
      release_failed_seen = true;
    }
    if (textContains(line, "phase=BoundaryRelease") &&
        textContains(line, "start") &&
        textContains(line, "stopped_direction=opening") &&
        textContains(line, "current_limit_a=2")) {
      saw_release_current_limit = true;
    }
    if (textContains(line, "second_direction_skipped") &&
        textContains(line, "reason=boundary_release_failed")) {
      saw_second_direction_skipped = true;
    }
    if (textContains(line, "phase=MultiRegionRoundTripLearning") &&
        textContains(line, "skipped") &&
        textContains(line, "reason=boundary_release_failed")) {
      saw_learning_skipped = true;
    }
    if (textContains(line, "suspected mechanical limit direction=closing")) {
      saw_false_closing_limit = true;
    }
    if (release_failed_seen &&
        textContains(line, "pre_b_search_start direction=closing")) {
      saw_closing_search_after_release_failed = true;
    }
  }

  TEST_ASSERT(saw_opening_limit,
              "test setup must hit the remote opening mechanical limit");
  TEST_ASSERT(saw_release_failed,
              "release failure must be reported as a PreB diagnostic");
  TEST_ASSERT(saw_release_current_limit,
              "release must use the dedicated release current limit");
  TEST_ASSERT(saw_second_direction_skipped,
              "PreB must skip the opposite boundary search after release failure");
  TEST_ASSERT(saw_learning_skipped,
              "multi-region learning must not run while still wedged at the boundary");
  TEST_ASSERT(!saw_false_closing_limit,
              "release failure must not create a false closing mechanical limit");
  TEST_ASSERT(!saw_closing_search_after_release_failed,
              "closing main scan must not start after opening release failed");
  const auto state = gripper->state();
  TEST_ASSERT(state.pre_self_check_completed,
              "PreSelfCheck completion flag must still be set after degradation");
  TEST_ASSERT(state.pre_b_mechanism_anomaly,
              "release failure should be exposed as a mechanism anomaly");
  TEST_ASSERT(state.manual_nut_stroke_range.valid,
              "degraded PreSelfCheck should expose only a conservative window");
  TEST_ASSERT(state.manual_nut_stroke_range.closed_limit.value -
                      state.manual_nut_stroke_range.open_limit.value <=
                  config.self_check.stable_short_stroke_distance.value + 0.3,
              "release failure must fall back to a small PreA window");
  TEST_ASSERT(!state.enabled, "PreSelfCheck must leave motor output disabled");
  return 0;
}

static int test_pre_self_check_single_boundary_release_allows_second_scan() {
  auto gripper = controller::createGripperController(
      std::make_unique<SingleOpenStopReleaseSucceedsMotor>());
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path.clear();
  config.self_check.pre_b_max_expansion_distance = common::Mm{20.0};
  config.self_check.pre_b_expansion_speed = common::MmPerS{2.0};
  config.self_check.pre_b_boundary_release_distance = common::Mm{1.0};
  config.self_check.pre_b_boundary_release_speed = common::MmPerS{2.0};
  config.self_check.motion_settle_timeout = common::S{0.3};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.safety.self_check_feedback_hard_current_limit = common::A{2.5};
  config.safety.self_check_feedback_emergency_current_limit = common::A{4.0};
  std::vector<std::string> progress;
  gripper->setProgressCallback(
      [&](std::string message) { progress.push_back(std::move(message)); });

  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");

  const auto result = gripper->runPreSelfCheck();
  TEST_ASSERT(result.isOk(),
              "single-boundary release should let PreB continue scanning");

  bool saw_opening_limit = false;
  bool saw_release_start = false;
  bool saw_release_current_limit = false;
  bool saw_release_failure = false;
  bool saw_closing_search_after_release = false;
  bool release_completed = false;
  for (const auto& line : progress) {
    if (textContains(line, "suspected mechanical limit direction=opening")) {
      saw_opening_limit = true;
    }
    if (textContains(line, "phase=BoundaryRelease") &&
        textContains(line, "start") &&
        textContains(line, "stopped_direction=opening")) {
      saw_release_start = true;
    }
    if (textContains(line, "phase=BoundaryRelease") &&
        textContains(line, "start") &&
        textContains(line, "stopped_direction=opening") &&
        textContains(line, "current_limit_a=2")) {
      saw_release_current_limit = true;
    }
    if (textContains(line, "phase=BoundaryRelease") &&
        textContains(line, "result code=Ok")) {
      release_completed = true;
    }
    if (textContains(line, "boundary_release_failed")) {
      saw_release_failure = true;
    }
    if (release_completed &&
        textContains(line, "pre_b_search_start direction=closing")) {
      saw_closing_search_after_release = true;
    }
  }

  TEST_ASSERT(saw_opening_limit,
              "test setup must first hit the opening mechanical limit");
  TEST_ASSERT(saw_release_start,
              "single-boundary release must be attempted without a safe zone");
  TEST_ASSERT(saw_release_current_limit,
              "single-boundary release must use the dedicated current limit");
  TEST_ASSERT(!saw_release_failure,
              "successful local release must not be reported as failed");
  TEST_ASSERT(saw_closing_search_after_release,
              "PreB must continue with the opposite direction after release");
  TEST_ASSERT(!gripper->state().enabled,
              "PreSelfCheck must leave motor output disabled");
  return 0;
}

static int test_pre_self_check_loads_seed_for_breakaway_start_current() {
  const auto seed_path =
      tempSeedPath("gripper_pre_self_check_seed_restore_test.txt");
  {
    std::ofstream seed{seed_path};
    seed << "seed_version=2\n";
    seed << "opening_static_friction_current_a=0.35\n";
    seed << "opening_static_friction_sample_count=2\n";
    seed << "closing_static_friction_current_a=0.35\n";
    seed << "closing_static_friction_sample_count=2\n";
    seed << "opening_dynamic_friction_current_max_a=0.22\n";
    seed << "opening_dynamic_friction_sample_count=2\n";
    seed << "opening_minimum_stable_nut_speed_mm_s=0.8\n";
    seed << "opening_stable_speed_sample_count=2\n";
    seed << "closing_dynamic_friction_current_max_a=0.23\n";
    seed << "closing_dynamic_friction_sample_count=2\n";
    seed << "closing_minimum_stable_nut_speed_mm_s=0.9\n";
    seed << "closing_stable_speed_sample_count=2\n";
  }

  auto gripper = controller::createGripperController(
      std::make_unique<MicroMotionMotor>());
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path = seed_path.string();
  config.self_check.low_confidence_motion_distance = common::Mm{0.015};
  config.self_check.motion_start_distance = common::Mm{0.05};
  config.self_check.motion_settle_stable_time = common::S{0.02};
  config.self_check.motion_settle_timeout = common::S{0.3};
  config.self_check.static_friction_current_start = common::A{0.1};
  config.self_check.static_friction_current_step = common::A{0.05};
  config.safety.self_check_current_limit = common::A{0.4};
  std::vector<std::string> progress;
  gripper->setProgressCallback(
      [&](std::string message) { progress.push_back(std::move(message)); });

  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");
  TEST_ASSERT(gripper->runPreSelfCheck().isOk(),
              "PreSelfCheck with a valid seed must still complete");

  bool saw_seed_start_current = false;
  for (const auto& line : progress) {
    if (textContains(line, "phase=BidirectionalMoveEnable") &&
        textContains(line, "current_start_a=0.3")) {
      saw_seed_start_current = true;
      break;
    }
  }
  TEST_ASSERT(saw_seed_start_current,
              "loaded seed should start breakaway scan near learned current");
  const auto state = gripper->state();
  TEST_ASSERT(state.learned_parameters.opening.breakaway_sample_count >= 2U,
              "loaded opening breakaway sample count must survive profile update");
  TEST_ASSERT(state.learned_parameters.closing.breakaway_sample_count >= 2U,
              "loaded closing breakaway sample count must survive profile update");
  TEST_ASSERT(state.learned_parameters.opening.breakaway_current.value >= 0.35,
              "loaded opening breakaway current must survive profile update");
  TEST_ASSERT(state.learned_parameters.closing.breakaway_current.value >= 0.35,
              "loaded closing breakaway current must survive profile update");
  TEST_ASSERT(state.learned_parameters.opening.static_friction_sample_count == 0U,
              "legacy seed static friction must migrate to bootstrap only");
  TEST_ASSERT(state.learned_parameters.closing.static_friction_sample_count == 0U,
              "legacy seed static friction must migrate to bootstrap only");
  TEST_ASSERT(state.learned_parameters.opening.dynamic_friction_sample_count == 0U,
              "legacy seed dynamic friction must not be reused as final model");
  TEST_ASSERT(state.learned_parameters.closing.dynamic_friction_sample_count == 0U,
              "legacy seed dynamic friction must not be reused as final model");
  TEST_ASSERT(state.learned_parameters.opening.stable_speed_sample_count == 0U,
              "legacy seed stable speed must not be reused as final model");
  TEST_ASSERT(state.learned_parameters.closing.stable_speed_sample_count == 0U,
              "legacy seed stable speed must not be reused as final model");
  std::filesystem::remove(seed_path);
  return 0;
}

static int test_workflow_seed_saves_travel_and_health_profile() {
  const auto seed_path =
      tempSeedPath("gripper_workflow_seed_persistence_test.txt");
  std::filesystem::remove(seed_path);

  auto gripper = makeController();
  auto config = fastWorkflowConfig();
  config.self_check.learned_profile_path = seed_path.string();
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");
  TEST_ASSERT(gripper->enable().isOk(), "enable must succeed");
  TEST_ASSERT(gripper->runPreSelfCheck().isOk(),
              "pre-self-check must succeed");
  TEST_ASSERT(gripper->homeOpenStop().isOk(), "homing must succeed");
  TEST_ASSERT(gripper->learnTravelLimits().isOk(),
              "travel learning must succeed");
  TEST_ASSERT(gripper->runMotionHealthCheck().isOk(),
              "motion health check must succeed");

  const std::string seed_text = readTextFile(seed_path);
  TEST_ASSERT(textContains(seed_text, "seed_version=3"),
              "seed file must include a version marker");
  TEST_ASSERT(textContains(seed_text, "opening_breakaway_bootstrap_sample_count="),
              "seed must persist opening bootstrap breakaway count");
  TEST_ASSERT(textContains(seed_text, "closing_breakaway_bootstrap_sample_count="),
              "seed must persist closing bootstrap breakaway count");
  TEST_ASSERT(textContains(seed_text, "opening_static_friction_sample_count="),
              "seed must persist opening static friction count");
  TEST_ASSERT(textContains(seed_text, "closing_static_friction_sample_count="),
              "seed must persist closing static friction count");
  TEST_ASSERT(textContains(seed_text, "software_closed_limit_mm="),
              "travel learning must persist software limits");
  TEST_ASSERT(textContains(seed_text, "learned_travel_mm="),
              "travel learning must persist measured travel");
  TEST_ASSERT(textContains(seed_text, "motion_health_valid_sample_count="),
              "motion health check must persist health sample count");
  TEST_ASSERT(!textContains(seed_text, "motion_health_valid_sample_count=0"),
              "successful health check must save non-zero valid sample count");
  std::filesystem::remove(seed_path);
  return 0;
}

int main() {
  std::cout << "test_gripper_workflow" << std::endl;

  RUN_TEST(full_simulated_workflow_reaches_ready_clamps_and_releases);
  RUN_TEST(clamp_requires_travel_learning);
  RUN_TEST(homing_requires_current_pre_self_check);
  RUN_TEST(manual_nut_stroke_move_requires_pre_self_check);
  RUN_TEST(manual_nut_stroke_move_uses_safe_zone_after_pre_self_check);
  RUN_TEST(default_damiao_encoder_config_uses_protocol_position_range);
  RUN_TEST(stop_cancels_running_pre_self_check);
  RUN_TEST(pre_self_check_accepts_low_confidence_micro_motion);
  RUN_TEST(pre_self_check_endpoint_start_prioritizes_retreat_direction);
  RUN_TEST(pre_self_check_release_failure_skips_false_second_boundary);
  RUN_TEST(pre_self_check_single_boundary_release_allows_second_scan);
  RUN_TEST(pre_self_check_loads_seed_for_breakaway_start_current);
  RUN_TEST(workflow_seed_saves_travel_and_health_profile);

  std::cout << "  all passed" << std::endl;
  return 0;
}
