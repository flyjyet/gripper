#include "controller/safety/debug_constraint_policy.hpp"

namespace gripper::controller::safety {

BasicDebugConstraintPolicyEvaluator::BasicDebugConstraintPolicyEvaluator(
    DebugConstraintPolicy policy)
    : policy_(policy) {}

void BasicDebugConstraintPolicyEvaluator::setPolicy(
    const DebugConstraintPolicy& policy) {
  policy_ = policy;
}

DebugConstraintPolicy BasicDebugConstraintPolicyEvaluator::policy() const {
  return policy_;
}

DebugConstraintEvaluation BasicDebugConstraintPolicyEvaluator::evaluate()
    const {
  if (!policy_.global_hard_protection_enabled) {
    return DebugConstraintEvaluation{
        false, true, "global hard protection cannot be disabled"};
  }

  if (!hardProtectionConfigured()) {
    return DebugConstraintEvaluation{
        false, true, "global hard protection limits must be configured"};
  }

  const bool relaxed = controllerConstraintDisabled();
  if (!relaxed) {
    return DebugConstraintEvaluation{true, false, "ok"};
  }

  if (policy_.mode == DebugControlMode::Normal) {
    return DebugConstraintEvaluation{
        false, false,
        "controller constraints cannot be disabled in normal mode"};
  }

  if (!policy_.risk_confirmed) {
    return DebugConstraintEvaluation{
        false, false,
        "risk confirmation is required before disabling constraints"};
  }

  return DebugConstraintEvaluation{true, false, "ok"};
}

bool BasicDebugConstraintPolicyEvaluator::controllerConstraintDisabled()
    const noexcept {
  return !policy_.controller_current_limit_enabled ||
         !policy_.controller_speed_limit_enabled ||
         !policy_.active_stop_detection_enabled ||
         !policy_.software_stroke_limit_enabled ||
         !policy_.communication_timeout_enabled;
}

bool BasicDebugConstraintPolicyEvaluator::hardProtectionConfigured()
    const noexcept {
  return policy_.global_hard_current_limit.value > 0.0 &&
         policy_.global_hard_temperature_limit.value > 0.0 &&
         policy_.command_duration_limit.value > 0.0;
}

}  // namespace gripper::controller::safety
