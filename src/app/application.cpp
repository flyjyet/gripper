#include "app/application.hpp"

#include <algorithm>
#include <iostream>
#include <memory>

#include "config/config_loader.hpp"
#include "controller/gripper_controller.hpp"
#include "hardware_interface/damiao/damiao_motor.hpp"
#include "hardware_interface/damiao/dm_usb2fdcan_transport.hpp"
#include "hardware_interface/adapter_types.hpp"
#include "hardware_interface/simulated/simulated_motor.hpp"
#include "ui/main_window.hpp"
#include "ui/ui_controller.hpp"
#include "ui/web_server.hpp"

namespace gripper::app {
namespace {

[[nodiscard]] hardware_interface::damiao::EncoderUnwrapSource
toDamiaoEncoderUnwrapSource(config::EncoderUnwrapSource source) {
  switch (source) {
    case config::EncoderUnwrapSource::ProtocolPosition:
      return hardware_interface::damiao::EncoderUnwrapSource::ProtocolPosition;
    case config::EncoderUnwrapSource::RawPositionCounts:
      return hardware_interface::damiao::EncoderUnwrapSource::RawPositionCounts;
  }
  return hardware_interface::damiao::EncoderUnwrapSource::RawPositionCounts;
}

void printState(const controller::GripperStateSnapshot& state,
                std::ostream& output) {
  output << "state=" << static_cast<int>(state.top_state)
         << " connected=" << (state.connected ? "true" : "false")
         << " enabled=" << (state.enabled ? "true" : "false")
         << " stroke_mm=" << state.nut_stroke.value
         << " force_n=" << state.estimated_clamp_force.value << '\n';
}

[[nodiscard]] bool hasArg(const std::vector<std::string>& args,
                          const std::string& name) {
  return std::find(args.begin(), args.end(), name) != args.end();
}

[[nodiscard]] std::uint16_t loadWebPort(const std::vector<std::string>& args) {
  const auto iter = std::find(args.begin(), args.end(), "--web-port");
  if (iter == args.end() || iter + 1 == args.end()) {
    return 8765;
  }
  try {
    const auto value = std::stoul(*(iter + 1), nullptr, 0);
    if (value > 0U && value <= 65535U) {
      return static_cast<std::uint16_t>(value);
    }
  } catch (...) {
  }
  return 8765;
}

[[nodiscard]] config::GripperConfig loadConfig(
    const std::vector<std::string>& args, std::ostream& output) {
  const auto iter = std::find(args.begin(), args.end(), "--config");
  if (iter == args.end() || iter + 1 == args.end()) {
    return config::defaultConfig();
  }

  const auto loaded = config::loadFromFile(*(iter + 1));
  if (!loaded.isOk()) {
    output << "config warning: " << loaded.message << '\n';
  }
  return loaded.config;
}

[[nodiscard]] std::unique_ptr<hardware_interface::MotorInterface> makeMotor(
    const config::GripperConfig& config, bool force_damiao) {
  if (force_damiao || config.adapter.type == config::AdapterType::DmUsb2FdcanDual) {
    auto transport =
        std::make_unique<hardware_interface::damiao::DmUsb2FdcanTransport>();
    hardware_interface::damiao::DamiaoMotorConfig motor_config{};
    motor_config.motor_id = hardware_interface::MotorId{config.motor.motor_id};
    motor_config.host_id = hardware_interface::MotorId{config.motor.host_id};
    motor_config.adapter.adapter_type =
        hardware_interface::AdapterType::UsbCanFd;
    motor_config.adapter.device_name = config.adapter.channel;
    motor_config.adapter.driver_library_path = config.adapter.driver_library_path;
    motor_config.adapter.device_index = config.adapter.device_index;
    motor_config.adapter.channel_index = config.adapter.channel_index;
    motor_config.adapter.nominal_bitrate = config.adapter.nominal_bitrate_bps;
    motor_config.adapter.data_bitrate = config.adapter.data_bitrate_bps;
    motor_config.adapter.can_fd_enabled = config.adapter.fdcan_enabled;
    motor_config.adapter.bitrate_switch_enabled = config.adapter.brs_enabled;
    motor_config.limits.position = config.motor.max_position;
    motor_config.limits.velocity = config.motor.max_velocity;
    motor_config.limits.torque = config.motor.max_torque;
    motor_config.max_phase_current = config.motor.max_phase_current;
    motor_config.torque_per_amp = config.motor.torque_per_amp;
    motor_config.position_command_sign = config.motor.position_command_sign;
    motor_config.auto_switch_mode = config.motor.auto_switch_mode;
    motor_config.feedback_poll_period = config.motor.feedback_poll_period;
    motor_config.feedback_stale_timeout = config.motor.feedback_stale_timeout;
    motor_config.continuous_encoder_enabled =
        config.motor.continuous_encoder_enabled;
    motor_config.encoder_unwrap_source =
        toDamiaoEncoderUnwrapSource(config.motor.encoder_unwrap_source);
    motor_config.encoder_wrap_range = config.motor.encoder_wrap_range;
    motor_config.encoder_raw_count_range = config.motor.encoder_raw_count_range;
    motor_config.frame_options.motor_frames_canfd =
        config.motor.motor_frames_canfd;
    motor_config.frame_options.command_id_includes_mode_offset =
        config.motor.command_id_includes_mode_offset;
    motor_config.frame_options.bitrate_switch = config.adapter.brs_enabled;
    return std::make_unique<hardware_interface::damiao::DamiaoMotor>(
        motor_config, std::move(transport));
  }
  return std::make_unique<hardware_interface::SimulatedMotor>(
      hardware_interface::MotorId{config.motor.motor_id});
}

}  // namespace

int Application::run(const std::vector<std::string>& args, std::istream& input,
                     std::ostream& output) {
  const auto config = loadConfig(args, output);
  auto motor = makeMotor(config, hasArg(args, "--damiao"));
  ui::UiController ui_controller{
      controller::createGripperController(std::move(motor))};
  auto result = ui_controller.configure(config);
  if (result.isError()) {
    output << "configure failed: " << result.message() << '\n';
    return 1;
  }

  if (hasArg(args, "--scripted-demo")) {
    (void)ui_controller.connect();
    (void)ui_controller.enable();
    (void)ui_controller.runPreSelfCheck();
    (void)ui_controller.home();
    (void)ui_controller.learnTravelLimits();
    (void)ui_controller.runMotionHealthCheck();
    (void)ui_controller.clampForce(common::N{20.0}, common::MmPerS{1.0});
    (void)ui_controller.release();
    printState(ui_controller.viewModel().state, output);
    for (const auto& entry : ui_controller.logModel().entries()) {
      output << entry.message << '\n';
    }
    return 0;
  }

  if (hasArg(args, "--web-ui")) {
    ui::WebServer web_server{&ui_controller, loadWebPort(args)};
    return web_server.run(output);
  }

  ui::MainWindow main_window{&ui_controller};
  return main_window.run(input, output);
}

}  // namespace gripper::app
