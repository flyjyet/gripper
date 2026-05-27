#include "controller/state_machine/gripper_state_machine.hpp"

namespace gripper::controller::state_machine {

namespace {

[[nodiscard]] bool isTerminalLike(GripperTopState state) {
  return state == GripperTopState::ActiveStop || state == GripperTopState::Fault;
}

[[nodiscard]] bool acceptsFromAny(GripperEvent event) {
  return event == GripperEvent::StopRequested ||
         event == GripperEvent::ActiveStopTriggered ||
         event == GripperEvent::FaultTriggered ||
         event == GripperEvent::DisconnectRequested;
}

[[nodiscard]] GripperTopState nextState(GripperTopState state,
                                        GripperEvent event,
                                        bool* accepted) {
  *accepted = true;

  if (event == GripperEvent::FaultTriggered) {
    return GripperTopState::Fault;
  }
  if (event == GripperEvent::ActiveStopTriggered ||
      event == GripperEvent::StopRequested) {
    return GripperTopState::ActiveStop;
  }
  if (event == GripperEvent::DisconnectRequested) {
    return GripperTopState::Disconnected;
  }

  switch (state) {
    case GripperTopState::Disconnected:
      if (event == GripperEvent::ConnectSucceeded) {
        return GripperTopState::Connected;
      }
      break;
    case GripperTopState::Connected:
      if (event == GripperEvent::HardwareCheckRequested) {
        return GripperTopState::HardwareSanityCheck;
      }
      if (event == GripperEvent::ModeSwitchRequested) {
        return GripperTopState::ModeSwitching;
      }
      break;
    case GripperTopState::HardwareSanityCheck:
      if (event == GripperEvent::HardwareCheckPassed) {
        return GripperTopState::ModeSwitching;
      }
      if (event == GripperEvent::HardwareCheckFailed) {
        return GripperTopState::Fault;
      }
      break;
    case GripperTopState::ModeSwitching:
      if (event == GripperEvent::EnableSucceeded) {
        return GripperTopState::Enabled;
      }
      if (event == GripperEvent::ModeSwitchFailed) {
        return GripperTopState::Fault;
      }
      break;
    case GripperTopState::Enabled:
      if (event == GripperEvent::PreSelfCheckRequested) {
        return GripperTopState::PreSelfCheck;
      }
      if (event == GripperEvent::MoveNutStrokeRequested) {
        return GripperTopState::ManualPositioning;
      }
      if (event == GripperEvent::HomingRequested) {
        return GripperTopState::HomingOpenStop;
      }
      if (event == GripperEvent::TravelLearningRequested) {
        return GripperTopState::TravelLearning;
      }
      if (event == GripperEvent::MotionHealthCheckRequested) {
        return GripperTopState::MotionHealthCheck;
      }
      if (event == GripperEvent::DisableRequested) {
        return GripperTopState::Disabled;
      }
      break;
    case GripperTopState::PreSelfCheck:
      if (event == GripperEvent::PreSelfCheckSucceeded) {
        return GripperTopState::Enabled;
      }
      if (event == GripperEvent::PreSelfCheckFailed) {
        return GripperTopState::ActiveStop;
      }
      break;
    case GripperTopState::HomingOpenStop:
      if (event == GripperEvent::HomingSucceeded) {
        return GripperTopState::Enabled;
      }
      if (event == GripperEvent::HomingFailed) {
        return GripperTopState::Fault;
      }
      break;
    case GripperTopState::TravelLearning:
      if (event == GripperEvent::TravelLearningSucceeded) {
        return GripperTopState::Enabled;
      }
      if (event == GripperEvent::TravelLearningFailed) {
        return GripperTopState::Fault;
      }
      break;
    case GripperTopState::MotionHealthCheck:
      if (event == GripperEvent::MotionHealthCheckSucceeded) {
        return GripperTopState::Ready;
      }
      if (event == GripperEvent::MotionHealthCheckFailed) {
        return GripperTopState::ActiveStop;
      }
      break;
    case GripperTopState::Ready:
      if (event == GripperEvent::ClampRequested) {
        return GripperTopState::Clamping;
      }
      if (event == GripperEvent::MoveNutStrokeRequested) {
        return GripperTopState::ManualPositioning;
      }
      if (event == GripperEvent::ReleaseRequested) {
        return GripperTopState::Releasing;
      }
      if (event == GripperEvent::DisableRequested) {
        return GripperTopState::Disabled;
      }
      break;
    case GripperTopState::Clamping:
      if (event == GripperEvent::ClampCompleted) {
        return GripperTopState::Unloading;
      }
      if (event == GripperEvent::ClampFailed) {
        return GripperTopState::ActiveStop;
      }
      break;
    case GripperTopState::Unloading:
      if (event == GripperEvent::DisableCompleted) {
        return GripperTopState::Disabled;
      }
      if (event == GripperEvent::ReleaseCompleted) {
        return GripperTopState::Ready;
      }
      break;
    case GripperTopState::Releasing:
      if (event == GripperEvent::ReleaseCompleted) {
        return GripperTopState::Ready;
      }
      if (event == GripperEvent::ReleaseFailed) {
        return GripperTopState::ActiveStop;
      }
      break;
    case GripperTopState::ManualPositioning:
      if (event == GripperEvent::MoveNutStrokeCompleted) {
        return GripperTopState::Unloading;
      }
      if (event == GripperEvent::MoveNutStrokeFailed) {
        return GripperTopState::ActiveStop;
      }
      break;
    case GripperTopState::Disabled:
      if (event == GripperEvent::EnableSucceeded) {
        return GripperTopState::Enabled;
      }
      break;
    case GripperTopState::ActiveStop:
      if (event == GripperEvent::FaultCleared) {
        return GripperTopState::Disabled;
      }
      break;
    case GripperTopState::Fault:
      if (event == GripperEvent::FaultCleared) {
        return GripperTopState::Disabled;
      }
      break;
  }

  *accepted = false;
  return state;
}

}  // namespace

GripperTopState GripperStateMachine::state() const {
  return snapshot_.state;
}

GripperStateSnapshot GripperStateMachine::snapshot() const {
  return snapshot_;
}

bool GripperStateMachine::canAccept(GripperEvent event) const {
  if (acceptsFromAny(event)) {
    return true;
  }
  if (isTerminalLike(snapshot_.state) && event != GripperEvent::FaultCleared) {
    return false;
  }
  bool accepted = false;
  (void)nextState(snapshot_.state, event, &accepted);
  return accepted;
}

TransitionDecision GripperStateMachine::dispatch(GripperEvent event) {
  if (!canAccept(event)) {
    return TransitionDecision::Rejected;
  }

  bool accepted = false;
  const auto next = nextState(snapshot_.state, event, &accepted);
  if (!accepted) {
    return TransitionDecision::Ignored;
  }

  snapshot_.previous_state = snapshot_.state;
  snapshot_.state = next;
  snapshot_.last_event = event;
  snapshot_.fault_active = next == GripperTopState::Fault;
  snapshot_.active_stop = next == GripperTopState::ActiveStop;
  return TransitionDecision::Accepted;
}

void GripperStateMachine::forceFault() {
  snapshot_.previous_state = snapshot_.state;
  snapshot_.state = GripperTopState::Fault;
  snapshot_.last_event = GripperEvent::FaultTriggered;
  snapshot_.fault_active = true;
  snapshot_.active_stop = false;
}

void GripperStateMachine::forceActiveStop() {
  snapshot_.previous_state = snapshot_.state;
  snapshot_.state = GripperTopState::ActiveStop;
  snapshot_.last_event = GripperEvent::ActiveStopTriggered;
  snapshot_.fault_active = false;
  snapshot_.active_stop = true;
}

void GripperStateMachine::resetToDisconnected() {
  snapshot_ = {};
}

}  // namespace gripper::controller::state_machine
