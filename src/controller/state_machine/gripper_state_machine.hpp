#pragma once

#include <cstdint>

namespace gripper::controller::state_machine {

// Top-level business states for the gripper controller.
//
// These states describe controller readiness and safety posture, not raw motor
// protocol states. They are intentionally independent of Damiao SDK details.
enum class GripperTopState : std::uint8_t {
  // No hardware connection is open.
  Disconnected,
  // Hardware connection is open, but command output is not enabled.
  Connected,
  // Basic hardware feedback/availability checks are running.
  HardwareSanityCheck,
  // Motor mode switching is in progress before enable.
  ModeSwitching,
  // Motor output is enabled and setup/verification actions may run.
  Enabled,
  // Conservative structure parameters are being identified before homing.
  PreSelfCheck,
  // Low-current homing toward the open reference is running.
  HomingOpenStop,
  // Close-side travel and software limits are being learned.
  TravelLearning,
  // In-limit bidirectional motion reliability is being checked.
  MotionHealthCheck,
  // Normal work commands may be accepted.
  Ready,
  // Clamp command is actively generating closing motion/load.
  Clamping,
  // Torque is being unloaded before disable after clamp completion.
  Unloading,
  // Release command is actively opening/unloading the gripper.
  Releasing,
  // Bounded nut-position move requested by UI/manual validation.
  ManualPositioning,
  // Motor command output is disabled after normal completion or safe shutdown.
  Disabled,
  // Recoverable safety stop, usually caused by limit/contact/jam/user stop.
  ActiveStop,
  // Fault requiring clear/reinitialization or manual check.
  Fault,
};

// Events are business-level transitions. Failure events must move the state
// machine to ActiveStop or Fault according to whether recovery is safe.
enum class GripperEvent : std::uint8_t {
  ConnectRequested,
  ConnectSucceeded,
  ConnectFailed,
  DisconnectRequested,
  HardwareCheckRequested,
  HardwareCheckPassed,
  HardwareCheckFailed,
  ModeSwitchRequested,
  ModeSwitchSucceeded,
  ModeSwitchFailed,
  EnableRequested,
  EnableSucceeded,
  EnableFailed,
  PreSelfCheckRequested,
  PreSelfCheckSucceeded,
  PreSelfCheckFailed,
  HomingRequested,
  HomingSucceeded,
  HomingFailed,
  TravelLearningRequested,
  TravelLearningSucceeded,
  TravelLearningFailed,
  MotionHealthCheckRequested,
  MotionHealthCheckSucceeded,
  MotionHealthCheckFailed,
  ClampRequested,
  ClampCompleted,
  ClampFailed,
  ReleaseRequested,
  ReleaseCompleted,
  ReleaseFailed,
  MoveNutStrokeRequested,
  MoveNutStrokeCompleted,
  MoveNutStrokeFailed,
  StopRequested,
  ActiveStopTriggered,
  FaultTriggered,
  FaultCleared,
  DisableRequested,
  DisableCompleted,
};

enum class TransitionDecision : std::uint8_t {
  Accepted,
  Rejected,
  Ignored,
};

struct GripperStateSnapshot {
  GripperTopState state{GripperTopState::Disconnected};
  GripperTopState previous_state{GripperTopState::Disconnected};
  GripperEvent last_event{GripperEvent::DisconnectRequested};
  bool fault_active{false};
  bool active_stop{false};
};

class GripperStateMachine {
 public:
  GripperStateMachine() = default;

  [[nodiscard]] GripperTopState state() const;
  [[nodiscard]] GripperStateSnapshot snapshot() const;
  [[nodiscard]] bool canAccept(GripperEvent event) const;

  TransitionDecision dispatch(GripperEvent event);
  void forceFault();
  void forceActiveStop();
  void resetToDisconnected();

 private:
  GripperStateSnapshot snapshot_{};
};

}  // namespace gripper::controller::state_machine
