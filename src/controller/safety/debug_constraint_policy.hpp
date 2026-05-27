#pragma once

#include <cstdint>

#include "common/units.hpp"

namespace gripper::controller::safety {

enum class DebugControlMode : std::uint8_t {
  Normal,
  UnloadedBringup,
  AdminRecovery,
};

// Flags that describe which controller-level constraints are active. Global
// hard protection is intentionally separate and must remain enabled.
struct DebugConstraintPolicy {
  DebugControlMode mode{DebugControlMode::Normal};
  bool controller_current_limit_enabled{true};
  bool controller_speed_limit_enabled{true};
  bool active_stop_detection_enabled{true};
  bool software_stroke_limit_enabled{true};
  bool communication_timeout_enabled{true};
  bool global_hard_protection_enabled{true};
  bool risk_confirmed{false};
  common::A global_hard_current_limit{};
  common::DegC global_hard_temperature_limit{};
  common::S command_duration_limit{};
};

struct DebugConstraintEvaluation {
  bool command_allowed{true};
  bool hard_protection_violation{false};
  const char* reason{"ok"};
};

// V2 debug-constraint policy interface.
//
// The evaluator is pure controller logic. It validates whether a requested
// policy is allowed before the controller uses it to relax current, speed,
// stroke, or active-stop constraints. It does not send motor commands and does
// not access hardware.
class DebugConstraintPolicyEvaluator {
 public:
  virtual ~DebugConstraintPolicyEvaluator() = default;

  virtual void setPolicy(const DebugConstraintPolicy& policy) = 0;
  [[nodiscard]] virtual DebugConstraintPolicy policy() const = 0;
  [[nodiscard]] virtual DebugConstraintEvaluation evaluate() const = 0;
};

// Default V2 policy evaluator. It keeps global hard protection mandatory and
// requires explicit risk confirmation before any controller-level constraint is
// disabled in a debug-capable mode.
class BasicDebugConstraintPolicyEvaluator final
    : public DebugConstraintPolicyEvaluator {
 public:
  BasicDebugConstraintPolicyEvaluator() = default;
  explicit BasicDebugConstraintPolicyEvaluator(DebugConstraintPolicy policy);

  void setPolicy(const DebugConstraintPolicy& policy) override;
  [[nodiscard]] DebugConstraintPolicy policy() const override;
  [[nodiscard]] DebugConstraintEvaluation evaluate() const override;

 private:
  [[nodiscard]] bool controllerConstraintDisabled() const noexcept;
  [[nodiscard]] bool hardProtectionConfigured() const noexcept;

  DebugConstraintPolicy policy_{};
};

}  // namespace gripper::controller::safety
