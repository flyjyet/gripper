#include "controller/safety/debug_constraint_policy.hpp"

#include <iostream>
#include <string>

#include "test_utils.hpp"

namespace sf = gripper::controller::safety;
namespace common = gripper::common;

namespace {

sf::DebugConstraintPolicy safePolicy() {
  sf::DebugConstraintPolicy policy{};
  policy.global_hard_current_limit = common::A{2.0};
  policy.global_hard_temperature_limit = common::DegC{70.0};
  policy.command_duration_limit = common::S{2.0};
  return policy;
}

int test_default_policy_is_allowed_when_hard_limits_configured() {
  sf::BasicDebugConstraintPolicyEvaluator evaluator{safePolicy()};

  const auto result = evaluator.evaluate();

  TEST_ASSERT(result.command_allowed, "default constrained policy must be allowed");
  TEST_ASSERT(!result.hard_protection_violation,
              "default policy must not violate hard protection");
  TEST_ASSERT(std::string{result.reason} == "ok", "reason must be ok");
  return 0;
}

int test_global_hard_protection_cannot_be_disabled() {
  auto policy = safePolicy();
  policy.mode = sf::DebugControlMode::AdminRecovery;
  policy.risk_confirmed = true;
  policy.global_hard_protection_enabled = false;

  sf::BasicDebugConstraintPolicyEvaluator evaluator{policy};
  const auto result = evaluator.evaluate();

  TEST_ASSERT(!result.command_allowed,
              "policy with disabled hard protection must be rejected");
  TEST_ASSERT(result.hard_protection_violation,
              "disabled hard protection must be marked as hard violation");
  return 0;
}

int test_hard_limits_must_have_positive_values() {
  auto policy = safePolicy();
  policy.global_hard_current_limit = common::A{0.0};

  sf::BasicDebugConstraintPolicyEvaluator evaluator{policy};
  const auto result = evaluator.evaluate();

  TEST_ASSERT(!result.command_allowed,
              "missing hard current limit must reject policy");
  TEST_ASSERT(result.hard_protection_violation,
              "missing hard protection config must be a hard violation");
  return 0;
}

int test_normal_mode_cannot_relax_controller_constraints() {
  auto policy = safePolicy();
  policy.controller_current_limit_enabled = false;
  policy.risk_confirmed = true;

  sf::BasicDebugConstraintPolicyEvaluator evaluator{policy};
  const auto result = evaluator.evaluate();

  TEST_ASSERT(!result.command_allowed,
              "normal mode must reject relaxed controller constraints");
  TEST_ASSERT(!result.hard_protection_violation,
              "normal-mode relaxation is not a hard-protection violation");
  return 0;
}

int test_debug_mode_requires_risk_confirmation_when_relaxed() {
  auto policy = safePolicy();
  policy.mode = sf::DebugControlMode::UnloadedBringup;
  policy.controller_speed_limit_enabled = false;
  policy.risk_confirmed = false;

  sf::BasicDebugConstraintPolicyEvaluator evaluator{policy};
  const auto result = evaluator.evaluate();

  TEST_ASSERT(!result.command_allowed,
              "debug relaxation without risk confirmation must be rejected");
  TEST_ASSERT(!result.hard_protection_violation,
              "missing risk confirmation is not a hard-protection violation");
  return 0;
}

int test_admin_recovery_allows_confirmed_relaxation() {
  auto policy = safePolicy();
  policy.mode = sf::DebugControlMode::AdminRecovery;
  policy.controller_current_limit_enabled = false;
  policy.active_stop_detection_enabled = false;
  policy.risk_confirmed = true;

  sf::BasicDebugConstraintPolicyEvaluator evaluator{policy};
  const auto result = evaluator.evaluate();

  TEST_ASSERT(result.command_allowed,
              "admin recovery with risk confirmation may relax controller constraints");
  TEST_ASSERT(!result.hard_protection_violation,
              "confirmed controller relaxation must keep hard protection intact");
  return 0;
}

int test_set_policy_updates_evaluation() {
  sf::BasicDebugConstraintPolicyEvaluator evaluator{safePolicy()};
  TEST_ASSERT(evaluator.evaluate().command_allowed,
              "initial safe policy must be allowed");

  auto policy = safePolicy();
  policy.global_hard_temperature_limit = common::DegC{0.0};
  evaluator.setPolicy(policy);

  TEST_ASSERT(!evaluator.evaluate().command_allowed,
              "updated invalid hard-limit policy must be rejected");
  return 0;
}

}  // namespace

int main() {
  std::cout << "test_debug_constraint_policy" << std::endl;
  RUN_TEST(default_policy_is_allowed_when_hard_limits_configured);
  RUN_TEST(global_hard_protection_cannot_be_disabled);
  RUN_TEST(hard_limits_must_have_positive_values);
  RUN_TEST(normal_mode_cannot_relax_controller_constraints);
  RUN_TEST(debug_mode_requires_risk_confirmation_when_relaxed);
  RUN_TEST(admin_recovery_allows_confirmed_relaxation);
  RUN_TEST(set_policy_updates_evaluation);
  std::cout << "  all passed" << std::endl;
  return 0;
}
