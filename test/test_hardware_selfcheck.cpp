#include "controller/gripper_controller.hpp"

#include <iostream>
#include <memory>
#include <string>

#include "config/config_loader.hpp"
#include "hardware_interface/damiao/damiao_motor.hpp"
#include "hardware_interface/damiao/dm_usb2fdcan_transport.hpp"

namespace common = gripper::common;
namespace config = gripper::config;
namespace controller = gripper::controller;
namespace hardware = gripper::hardware_interface;
namespace damiao = gripper::hardware_interface::damiao;

namespace {

[[nodiscard]] damiao::EncoderUnwrapSource toDamiaoEncoderUnwrapSource(
    config::EncoderUnwrapSource source) {
  switch (source) {
    case config::EncoderUnwrapSource::ProtocolPosition:
      return damiao::EncoderUnwrapSource::ProtocolPosition;
    case config::EncoderUnwrapSource::RawPositionCounts:
      return damiao::EncoderUnwrapSource::RawPositionCounts;
  }
  return damiao::EncoderUnwrapSource::RawPositionCounts;
}

[[nodiscard]] config::GripperConfig loadConfig(int argc, char** argv) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string{argv[i]} == "--config") {
      const auto loaded = config::loadFromFile(argv[i + 1]);
      if (!loaded.isOk()) {
        std::cout << "config warning: " << loaded.message << '\n';
      }
      return loaded.config;
    }
  }
  return config::defaultConfig();
}

[[nodiscard]] std::unique_ptr<hardware::MotorInterface> makeDamiaoMotor(
    const config::GripperConfig& cfg) {
  auto transport = std::make_unique<damiao::DmUsb2FdcanTransport>();
  damiao::DamiaoMotorConfig motor_config{};
  motor_config.motor_id = hardware::MotorId{cfg.motor.motor_id};
  motor_config.host_id = hardware::MotorId{cfg.motor.host_id};
  motor_config.adapter.adapter_type = hardware::AdapterType::UsbCanFd;
  motor_config.adapter.device_name = cfg.adapter.channel;
  motor_config.adapter.driver_library_path = cfg.adapter.driver_library_path;
  motor_config.adapter.device_index = cfg.adapter.device_index;
  motor_config.adapter.channel_index = cfg.adapter.channel_index;
  motor_config.adapter.nominal_bitrate = cfg.adapter.nominal_bitrate_bps;
  motor_config.adapter.data_bitrate = cfg.adapter.data_bitrate_bps;
  motor_config.adapter.can_fd_enabled = cfg.adapter.fdcan_enabled;
  motor_config.adapter.bitrate_switch_enabled = cfg.adapter.brs_enabled;
  motor_config.limits.position = cfg.motor.max_position;
  motor_config.limits.velocity = cfg.motor.max_velocity;
  motor_config.limits.torque = cfg.motor.max_torque;
  motor_config.max_phase_current = cfg.motor.max_phase_current;
  motor_config.torque_per_amp = cfg.motor.torque_per_amp;
  motor_config.position_command_sign = cfg.motor.position_command_sign;
  motor_config.auto_switch_mode = cfg.motor.auto_switch_mode;
  motor_config.feedback_poll_period = cfg.motor.feedback_poll_period;
  motor_config.feedback_stale_timeout = cfg.motor.feedback_stale_timeout;
  motor_config.continuous_encoder_enabled =
      cfg.motor.continuous_encoder_enabled;
  motor_config.encoder_unwrap_source =
      toDamiaoEncoderUnwrapSource(cfg.motor.encoder_unwrap_source);
  motor_config.encoder_wrap_range = cfg.motor.encoder_wrap_range;
  motor_config.encoder_raw_count_range = cfg.motor.encoder_raw_count_range;
  motor_config.frame_options.motor_frames_canfd = cfg.motor.motor_frames_canfd;
  motor_config.frame_options.command_id_includes_mode_offset =
      cfg.motor.command_id_includes_mode_offset;
  motor_config.frame_options.bitrate_switch = cfg.adapter.brs_enabled;
  return std::make_unique<damiao::DamiaoMotor>(motor_config,
                                               std::move(transport));
}

void printResult(const char* name, const common::Result& result) {
  std::cout << name << ": " << common::toString(result.code());
  if (result.hasMessage()) {
    std::cout << " | " << result.message();
  }
  std::cout << '\n';
}

void printState(const controller::GripperStateSnapshot& state) {
  std::cout << "final_state"
            << " stroke_mm=" << state.nut_stroke.value
            << " motor_virtual_pos_rad=" << state.motor.position.value
            << " motor_wrapped_pos_rad=" << state.motor.wrapped_position.value
            << " motor_raw_pos_counts=" << state.motor.raw_position_counts
            << " motor_vel_rad_s=" << state.motor.velocity.value
            << " motor_current_a=" << state.motor.current.value
            << " motor_torque_nm=" << state.motor.torque.value
            << " motor_enabled=" << (state.motor.enabled ? "true" : "false")
            << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  const auto cfg = loadConfig(argc, argv);
  auto gripper = controller::createGripperController(makeDamiaoMotor(cfg));
  gripper->setProgressCallback(
      [](std::string message) { std::cout << message << '\n'; });

  printResult("configure", gripper->configure(cfg));
  const auto connect = gripper->connect();
  printResult("connect", connect);
  if (connect.isError()) {
    return 2;
  }
  const auto enable = gripper->enable();
  printResult("enable", enable);
  if (enable.isError()) {
    return 3;
  }
  const auto self_check = gripper->runPreSelfCheck();
  printResult("pre_self_check", self_check);
  printState(gripper->state());
  (void)gripper->disconnect();
  return self_check.isOk() ? 0 : 4;
}
