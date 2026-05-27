#include "controller/gripper_controller.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "config/config_loader.hpp"
#include "hardware_interface/simulated/simulated_motor.hpp"
#include "test_utils.hpp"

namespace controller = gripper::controller;
namespace hardware = gripper::hardware_interface;
namespace common = gripper::common;

static int test_bringup_requires_unloaded_confirmation() {
  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(gripper::config::defaultConfig()).isOk(),
              "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = false;
  const auto result = gripper->enterMotorBringupMode(request);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::InvalidArgument),
                 "bring-up must require unloaded confirmation");
  TEST_ASSERT(!gripper->state().motor_bringup_active,
              "bring-up must remain inactive after rejected confirmation");
  return 0;
}

static int test_bringup_jog_disables_after_pulse() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.default_relative_motor_position = common::Rad{0.05};
  config.motor_bringup.max_relative_motor_position = common::Rad{0.2};
  config.motor_bringup.default_motor_velocity = common::RadPerS{0.5};
  config.motor_bringup.default_motor_current = common::A{0.2};
  config.motor_bringup.max_motor_current = common::A{0.4};
  config.motor_bringup.default_pulse_duration = common::S{0.001};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupJogCommand jog{};
  jog.motor_direction_sign = 1;
  TEST_ASSERT(gripper->jogMotorBringup(jog).isOk(),
              "default positive bring-up jog must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(state.motor_bringup_active,
              "bring-up session should remain active after one jog");
  TEST_ASSERT(!state.enabled, "jog must disable motor output afterward");
  TEST_ASSERT(std::abs(state.motor.position.value - 0.5) < 1e-9,
              "velocity jog must update motor feedback position positively");

  TEST_ASSERT(gripper->exitMotorBringupMode().isOk(),
              "bring-up exit must succeed");
  TEST_ASSERT(!gripper->state().motor_bringup_active,
              "bring-up must be inactive after exit");
  return 0;
}

static int test_bringup_negative_jog_moves_negative_direction() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.default_relative_motor_position = common::Rad{0.05};
  config.motor_bringup.max_relative_motor_position = common::Rad{0.2};
  config.motor_bringup.default_motor_velocity = common::RadPerS{0.5};
  config.motor_bringup.default_motor_current = common::A{0.2};
  config.motor_bringup.max_motor_current = common::A{0.4};
  config.motor_bringup.default_pulse_duration = common::S{0.001};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupJogCommand jog{};
  jog.relative_motor_position = common::Rad{-0.05};
  TEST_ASSERT(gripper->jogMotorBringup(jog).isOk(),
              "negative bring-up jog must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(!state.enabled, "jog must disable motor output afterward");
  TEST_ASSERT(std::abs(state.motor.position.value + 0.5) < 1e-9,
              "negative velocity jog must update motor feedback position negatively");
  return 0;
}

static int test_bringup_relative_turns_move_precise_position_and_disable() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.max_relative_motor_revolutions = common::Ratio{2.0};
  config.motor_bringup.default_motor_velocity = common::RadPerS{1.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.default_motor_current = common::A{1.0};
  config.motor_bringup.max_motor_current = common::A{1.5};
  config.motor_bringup.default_position_move_timeout = common::S{1.0};
  config.motor_bringup.max_position_move_timeout = common::S{3.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupPositionMoveCommand move{};
  move.relative_motor_revolutions = common::Ratio{1.0};
  move.max_motor_velocity = common::RadPerS{1.0};
  move.feedback_current_limit = common::A{1.0};
  move.timeout = common::S{1.0};
  const auto result = gripper->moveMotorBringupRelativeTurns(move);
  TEST_ASSERT(result.isOk(), "one-turn bring-up position move must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(!state.enabled,
              "bring-up position move must disable motor output afterward");
  TEST_ASSERT(std::abs(state.motor.position.value -
                       6.28318530717958647692) < 1e-9,
              "one turn must move motor position by 2*pi rad");
  TEST_ASSERT(std::abs(state.nut_encoder.motor_delta_revolutions.value -
                       1.0) < 1e-9,
              "nut encoder diagnostic must report one motor revolution");
  TEST_ASSERT(std::abs(state.nut_encoder.nut_position.value -
                       state.nut_encoder.zero_nut_position.value - 2.0) <
                  1e-9,
              "default hardware config maps one positive motor turn to positive virtual nut travel");
  TEST_ASSERT(result.message().find("mm_per_rev_estimate=") !=
                  std::string::npos,
              "position move result must log mm/rev estimate");
  return 0;
}

static int test_bringup_relative_turns_respects_nut_encoder_direction_config() {
  auto config = gripper::config::defaultConfig();
  config.nut_position_encoder.direction_sign = 1;
  config.motor_bringup.max_relative_motor_revolutions = common::Ratio{2.0};
  config.motor_bringup.default_motor_velocity = common::RadPerS{1.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.default_motor_current = common::A{1.0};
  config.motor_bringup.max_motor_current = common::A{1.5};
  config.motor_bringup.default_position_move_timeout = common::S{1.0};
  config.motor_bringup.max_position_move_timeout = common::S{3.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupPositionMoveCommand move{};
  move.relative_motor_revolutions = common::Ratio{1.0};
  move.max_motor_velocity = common::RadPerS{1.0};
  move.feedback_current_limit = common::A{1.0};
  move.timeout = common::S{1.0};
  TEST_ASSERT(gripper->moveMotorBringupRelativeTurns(move).isOk(),
              "one-turn bring-up position move must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(std::abs(state.nut_encoder.nut_position.value -
                       state.nut_encoder.zero_nut_position.value + 2.0) <
                  1e-9,
              "nut_position_encoder.direction_sign=1 keeps the legacy negative virtual nut direction");
  return 0;
}

static int test_bringup_relative_turns_negative_direction() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.max_relative_motor_revolutions = common::Ratio{2.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.max_motor_current = common::A{1.5};
  config.motor_bringup.max_position_move_timeout = common::S{3.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupPositionMoveCommand move{};
  move.relative_motor_revolutions = common::Ratio{-1.0};
  move.max_motor_velocity = common::RadPerS{1.0};
  move.feedback_current_limit = common::A{1.0};
  move.timeout = common::S{1.0};
  TEST_ASSERT(gripper->moveMotorBringupRelativeTurns(move).isOk(),
              "negative one-turn bring-up position move must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(std::abs(state.motor.position.value +
                       6.28318530717958647692) < 1e-9,
              "negative one turn must move motor position by -2*pi rad");
  TEST_ASSERT(!state.enabled,
              "negative position move must disable motor output afterward");
  return 0;
}

static int test_bringup_relative_turns_auto_timeout_for_two_revolutions() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.max_relative_motor_revolutions = common::Ratio{2.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.max_motor_current = common::A{2.0};
  config.motor_bringup.default_position_move_timeout = common::S{0.0};
  config.motor_bringup.max_position_move_timeout = common::S{20.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupPositionMoveCommand move{};
  move.relative_motor_revolutions = common::Ratio{2.0};
  move.max_motor_velocity = common::RadPerS{1.0};
  move.feedback_current_limit = common::A{1.5};
  move.timeout = common::S{0.0};
  const auto result = gripper->moveMotorBringupRelativeTurns(move);
  TEST_ASSERT(result.isOk(), "two-turn move with auto timeout must succeed");
  TEST_ASSERT(result.message().find("timeout_s=15.708") != std::string::npos,
              "auto timeout should be based on two turns at 1 rad/s");
  TEST_ASSERT(result.message().find("timeout_s=3") == std::string::npos,
              "auto timeout must not fall back to the old 3 second default");
  return 0;
}

static int test_bringup_relative_turns_requires_active_session() {
  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(gripper::config::defaultConfig()).isOk(),
              "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupPositionMoveCommand move{};
  move.relative_motor_revolutions = common::Ratio{1.0};
  const auto result = gripper->moveMotorBringupRelativeTurns(move);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::InvalidOperation),
                 "relative turn move must require active bring-up mode");
  return 0;
}

static int test_bringup_relative_turns_rejects_target_outside_pmax_window() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.max_relative_motor_revolutions = common::Ratio{2.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.max_motor_current = common::A{1.5};
  config.motor_bringup.max_position_move_timeout = common::S{20.0};
  config.motor.max_position = common::Rad{7.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupPositionMoveCommand first_move{};
  first_move.relative_motor_revolutions = common::Ratio{1.0};
  first_move.max_motor_velocity = common::RadPerS{2.0};
  first_move.feedback_current_limit = common::A{1.0};
  TEST_ASSERT(gripper->moveMotorBringupRelativeTurns(first_move).isOk(),
              "first move should reach near the configured P_MAX window");

  controller::MotorBringupPositionMoveCommand second_move = first_move;
  const auto result = gripper->moveMotorBringupRelativeTurns(second_move);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::OutOfRange),
                 "second move should be rejected before exceeding P_MAX");
  TEST_ASSERT(result.message().find("pmax_rad=") != std::string::npos,
              "out-of-range message should report P_MAX");
  TEST_ASSERT(!gripper->state().enabled,
              "rejected bring-up position move must leave output disabled");
  return 0;
}

static int test_bringup_velocity_and_duration_can_be_configured_for_unloaded_debug() {
  auto config = gripper::config::defaultConfig();
  config.motor_bringup.max_relative_motor_position = common::Rad{1.0};
  config.motor_bringup.max_motor_velocity = common::RadPerS{5.0};
  config.motor_bringup.max_motor_current = common::A{1.0};
  config.motor_bringup.max_pulse_duration = common::S{2.0};

  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(config).isOk(), "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  controller::MotorBringupJogCommand jog{};
  jog.relative_motor_position = common::Rad{0.2};
  jog.max_motor_velocity = common::RadPerS{5.0};
  jog.max_motor_current = common::A{0.8};
  jog.pulse_duration = common::S{2.0};
  const auto result = gripper->jogMotorBringup(jog);
  TEST_ASSERT(result.isOk(), "5 rad/s, 2 s unloaded velocity jog must succeed");
  TEST_ASSERT(result.message().find("effective_vel_rad_s=5") !=
                  std::string::npos,
              "result message must show unclipped 5 rad/s effective velocity");
  TEST_ASSERT(result.message().find("effective_pulse_s=2") !=
                  std::string::npos,
              "result message must show unclipped 2 s effective duration");
  return 0;
}

static int test_bringup_blocks_normal_enable_until_exit() {
  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(gripper::config::defaultConfig()).isOk(),
              "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");

  const auto enable_result = gripper->enable();
  TEST_ASSERT_EQ(static_cast<int>(enable_result.code()),
                 static_cast<int>(common::ErrorCode::InvalidOperation),
                 "normal enable must be blocked during bring-up");

  TEST_ASSERT(gripper->exitMotorBringupMode().isOk(),
              "exit bring-up must succeed");
  TEST_ASSERT(gripper->enable().isOk(),
              "normal enable may run after bring-up exit");
  return 0;
}

static int test_bringup_disable_keeps_bringup_session_active() {
  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(gripper::config::defaultConfig()).isOk(),
              "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  controller::MotorBringupSessionRequest request{};
  request.unloaded_or_structure_removed_confirmed = true;
  TEST_ASSERT(gripper->enterMotorBringupMode(request).isOk(),
              "confirmed bring-up must start");
  TEST_ASSERT(gripper->enableMotorBringupOutput().isOk(),
              "bring-up output enable must succeed");
  TEST_ASSERT(gripper->state().enabled,
              "motor output should be enabled before bring-up disable");

  TEST_ASSERT(gripper->disableMotorBringupOutput().isOk(),
              "bring-up output disable must succeed");

  const auto state = gripper->state();
  TEST_ASSERT(state.motor_bringup_active,
              "bring-up session should remain active after output disable");
  TEST_ASSERT(!state.enabled, "bring-up output disable must disable motor");
  TEST_ASSERT_EQ(static_cast<int>(state.top_state),
                 static_cast<int>(controller::state_machine::GripperTopState::Connected),
                 "bring-up output disable must not enter ActiveStop");
  return 0;
}

static int test_bringup_probe_requires_active_session() {
  auto gripper = controller::createGripperController(
      std::make_unique<hardware::SimulatedMotor>());
  TEST_ASSERT(gripper->configure(gripper::config::defaultConfig()).isOk(),
              "configure must succeed");
  TEST_ASSERT(gripper->connect().isOk(), "connect must succeed");

  std::vector<std::string> lines{};
  const auto result = gripper->runMotorBringupCommunicationProbe(&lines);
  TEST_ASSERT_EQ(static_cast<int>(result.code()),
                 static_cast<int>(common::ErrorCode::InvalidOperation),
                 "communication probe must require active bring-up mode");
  return 0;
}

int main() {
  std::cout << "test_motor_bringup" << std::endl;

  RUN_TEST(bringup_requires_unloaded_confirmation);
  RUN_TEST(bringup_jog_disables_after_pulse);
  RUN_TEST(bringup_negative_jog_moves_negative_direction);
  RUN_TEST(bringup_relative_turns_move_precise_position_and_disable);
  RUN_TEST(bringup_relative_turns_respects_nut_encoder_direction_config);
  RUN_TEST(bringup_relative_turns_negative_direction);
  RUN_TEST(bringup_relative_turns_auto_timeout_for_two_revolutions);
  RUN_TEST(bringup_relative_turns_requires_active_session);
  RUN_TEST(bringup_relative_turns_rejects_target_outside_pmax_window);
  RUN_TEST(bringup_velocity_and_duration_can_be_configured_for_unloaded_debug);
  RUN_TEST(bringup_blocks_normal_enable_until_exit);
  RUN_TEST(bringup_disable_keeps_bringup_session_active);
  RUN_TEST(bringup_probe_requires_active_session);

  std::cout << "  all passed" << std::endl;
  return 0;
}
