#include "controller/state_machine/gripper_state_machine.hpp"

#include <iostream>

#include "test_utils.hpp"

namespace sm = gripper::controller::state_machine;
using sm::GripperEvent;
using sm::GripperTopState;
using sm::TransitionDecision;

static int test_normal_startup_flow() {
  sm::GripperStateMachine fsm;

  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "initial state must be Disconnected");

  auto r = fsm.dispatch(GripperEvent::ConnectSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(r), static_cast<int>(TransitionDecision::Accepted),
                 "ConnectSucceeded must be accepted from Disconnected");
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Connected),
                 "after ConnectSucceeded state must be Connected");

  r = fsm.dispatch(GripperEvent::ModeSwitchRequested);
  TEST_ASSERT_EQ(static_cast<int>(r), static_cast<int>(TransitionDecision::Accepted),
                 "ModeSwitchRequested must be accepted from Connected");
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::ModeSwitching),
                 "after ModeSwitchRequested state must be ModeSwitching");

  r = fsm.dispatch(GripperEvent::EnableSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(r), static_cast<int>(TransitionDecision::Accepted),
                 "EnableSucceeded must be accepted from ModeSwitching");
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "after EnableSucceeded state must be Enabled");

  return 0;
}

static int test_full_workflow() {
  sm::GripperStateMachine fsm;

  // Enabled -> PreSelfCheck -> Enabled -> Homing -> Enabled ->
  // TravelLearning -> Enabled -> MotionHealthCheck -> Ready ->
  // Clamping -> Unloading -> Disabled
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "must be Enabled before PreSelfCheck");

  fsm.dispatch(GripperEvent::PreSelfCheckRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::PreSelfCheck),
                 "must enter PreSelfCheck");

  fsm.dispatch(GripperEvent::PreSelfCheckSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "PreSelfCheck success returns to Enabled");

  fsm.dispatch(GripperEvent::HomingRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::HomingOpenStop),
                 "must enter HomingOpenStop");

  fsm.dispatch(GripperEvent::HomingSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "Homing success returns to Enabled");

  fsm.dispatch(GripperEvent::TravelLearningRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::TravelLearning),
                 "must enter TravelLearning");

  fsm.dispatch(GripperEvent::TravelLearningSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "TravelLearning success returns to Enabled");

  fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::MotionHealthCheck),
                 "must enter MotionHealthCheck");

  fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Ready),
                 "MotionHealthCheck success -> Ready");

  fsm.dispatch(GripperEvent::ClampRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Clamping),
                 "must enter Clamping");

  fsm.dispatch(GripperEvent::ClampCompleted);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Unloading),
                 "ClampCompleted -> Unloading");

  fsm.dispatch(GripperEvent::DisableCompleted);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disabled),
                 "DisableCompleted -> Disabled");

  return 0;
}

static int test_fault_from_any_state() {
  // FaultTriggered is accepted from any state (acceptsFromAny).
  const GripperTopState states[] = {
      GripperTopState::Disconnected,
      GripperTopState::Enabled,
      GripperTopState::Ready,
      GripperTopState::Clamping,
  };

  for (auto init : states) {
    sm::GripperStateMachine fsm;
    // Manually set up to the desired state.
    if (init == GripperTopState::Enabled) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
    } else if (init == GripperTopState::Ready) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
      fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
      fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
    } else if (init == GripperTopState::Clamping) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
      fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
      fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
      fsm.dispatch(GripperEvent::ClampRequested);
    }
    TEST_ASSERT_EQ(static_cast<int>(fsm.state()), static_cast<int>(init),
                   "precondition state mismatch");

    TEST_ASSERT(fsm.canAccept(GripperEvent::FaultTriggered),
                "FaultTriggered must be accepted from any state");

    auto r = fsm.dispatch(GripperEvent::FaultTriggered);
    TEST_ASSERT_EQ(static_cast<int>(r), static_cast<int>(TransitionDecision::Accepted),
                   "FaultTriggered dispatch must be Accepted");
    TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                   static_cast<int>(GripperTopState::Fault),
                   "after FaultTriggered state must be Fault");
  }

  return 0;
}

static int test_stop_from_any_state() {
  const GripperTopState states[] = {
      GripperTopState::Enabled,
      GripperTopState::Ready,
      GripperTopState::Clamping,
  };

  for (auto init : states) {
    sm::GripperStateMachine fsm;
    if (init == GripperTopState::Enabled) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
    } else if (init == GripperTopState::Ready) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
      fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
      fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
    } else if (init == GripperTopState::Clamping) {
      fsm.dispatch(GripperEvent::ConnectSucceeded);
      fsm.dispatch(GripperEvent::ModeSwitchRequested);
      fsm.dispatch(GripperEvent::EnableSucceeded);
      fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
      fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
      fsm.dispatch(GripperEvent::ClampRequested);
    }

    TEST_ASSERT(fsm.canAccept(GripperEvent::StopRequested),
                "StopRequested must be accepted from any state");
    TEST_ASSERT(fsm.canAccept(GripperEvent::ActiveStopTriggered),
                "ActiveStopTriggered must be accepted from any state");

    fsm.dispatch(GripperEvent::StopRequested);
    TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                   static_cast<int>(GripperTopState::ActiveStop),
                   "StopRequested -> ActiveStop");
  }

  return 0;
}

static int test_disconnect_from_any_state() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
  fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Ready),
                 "precondition must be Ready");

  TEST_ASSERT(fsm.canAccept(GripperEvent::DisconnectRequested),
              "DisconnectRequested must be accepted from Ready");

  fsm.dispatch(GripperEvent::DisconnectRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "DisconnectRequested -> Disconnected");

  // Also from Fault
  sm::GripperStateMachine fsm2;
  fsm2.dispatch(GripperEvent::FaultTriggered);
  TEST_ASSERT_EQ(static_cast<int>(fsm2.state()),
                 static_cast<int>(GripperTopState::Fault), "must be Fault");
  TEST_ASSERT(fsm2.canAccept(GripperEvent::DisconnectRequested),
              "DisconnectRequested must be accepted from Fault");
  fsm2.dispatch(GripperEvent::DisconnectRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm2.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "Fault + DisconnectRequested -> Disconnected");

  return 0;
}

static int test_activestop_recovery() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::PreSelfCheckRequested);
  fsm.dispatch(GripperEvent::PreSelfCheckFailed);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::ActiveStop),
                 "PreSelfCheckFailed -> ActiveStop");

  TEST_ASSERT(fsm.canAccept(GripperEvent::FaultCleared),
              "FaultCleared must be accepted from ActiveStop");

  fsm.dispatch(GripperEvent::FaultCleared);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disabled),
                 "ActiveStop + FaultCleared -> Disabled");

  return 0;
}

static int test_fault_recovery() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::HomingRequested);
  fsm.dispatch(GripperEvent::HomingFailed);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Fault),
                 "HomingFailed -> Fault");

  TEST_ASSERT(fsm.canAccept(GripperEvent::FaultCleared),
              "FaultCleared must be accepted from Fault");

  fsm.dispatch(GripperEvent::FaultCleared);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disabled),
                 "Fault + FaultCleared -> Disabled");

  return 0;
}

static int test_illegal_clamp_from_disconnected() {
  sm::GripperStateMachine fsm;
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "initial state");

  TEST_ASSERT(!fsm.canAccept(GripperEvent::ClampRequested),
              "Disconnected must not accept ClampRequested");

  auto r = fsm.dispatch(GripperEvent::ClampRequested);
  TEST_ASSERT_EQ(static_cast<int>(r), static_cast<int>(TransitionDecision::Rejected),
                 "ClampRequested from Disconnected must be Rejected");
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "state must not change on rejection");

  return 0;
}

static int test_illegal_travel_learning_from_ready() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
  fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Ready),
                 "precondition must be Ready");

  TEST_ASSERT(!fsm.canAccept(GripperEvent::TravelLearningRequested),
              "Ready must not accept TravelLearningRequested");

  return 0;
}

static int test_illegal_homing_from_clamping() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
  fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
  fsm.dispatch(GripperEvent::ClampRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Clamping),
                 "precondition must be Clamping");

  TEST_ASSERT(!fsm.canAccept(GripperEvent::HomingRequested),
              "Clamping must not accept HomingRequested");

  return 0;
}

static int test_enabled_to_disabled() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Enabled),
                 "must be Enabled");

  TEST_ASSERT(fsm.canAccept(GripperEvent::DisableRequested),
              "Enabled must accept DisableRequested");

  fsm.dispatch(GripperEvent::DisableRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disabled),
                 "Enabled + DisableRequested -> Disabled");

  return 0;
}

static int test_manual_positioning_flow() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  TEST_ASSERT(fsm.canAccept(GripperEvent::MoveNutStrokeRequested),
              "Enabled must accept MoveNutStrokeRequested");

  fsm.dispatch(GripperEvent::MoveNutStrokeRequested);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::ManualPositioning),
                 "MoveNutStrokeRequested -> ManualPositioning");

  fsm.dispatch(GripperEvent::MoveNutStrokeCompleted);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Unloading),
                 "MoveNutStrokeCompleted -> Unloading");

  fsm.dispatch(GripperEvent::DisableCompleted);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disabled),
                 "manual positioning unload completion -> Disabled");
  return 0;
}

static int test_force_fault_and_activestop() {
  sm::GripperStateMachine fsm;
  fsm.dispatch(GripperEvent::ConnectSucceeded);
  fsm.dispatch(GripperEvent::ModeSwitchRequested);
  fsm.dispatch(GripperEvent::EnableSucceeded);
  fsm.dispatch(GripperEvent::MotionHealthCheckRequested);
  fsm.dispatch(GripperEvent::MotionHealthCheckSucceeded);
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Ready),
                 "must be Ready");

  fsm.forceFault();
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Fault),
                 "forceFault -> Fault");
  TEST_ASSERT(fsm.snapshot().fault_active, "fault_active must be true");
  TEST_ASSERT(!fsm.snapshot().active_stop, "active_stop must be false");

  fsm.resetToDisconnected();
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::Disconnected),
                 "resetToDisconnected -> Disconnected");

  fsm.forceActiveStop();
  TEST_ASSERT_EQ(static_cast<int>(fsm.state()),
                 static_cast<int>(GripperTopState::ActiveStop),
                 "forceActiveStop -> ActiveStop");
  TEST_ASSERT(!fsm.snapshot().fault_active, "fault_active must be false");
  TEST_ASSERT(fsm.snapshot().active_stop, "active_stop must be true");

  return 0;
}

int main() {
  std::cout << "test_state_machine" << std::endl;

  RUN_TEST(normal_startup_flow);
  RUN_TEST(full_workflow);
  RUN_TEST(fault_from_any_state);
  RUN_TEST(stop_from_any_state);
  RUN_TEST(disconnect_from_any_state);
  RUN_TEST(activestop_recovery);
  RUN_TEST(fault_recovery);
  RUN_TEST(illegal_clamp_from_disconnected);
  RUN_TEST(illegal_travel_learning_from_ready);
  RUN_TEST(illegal_homing_from_clamping);
  RUN_TEST(enabled_to_disabled);
  RUN_TEST(manual_positioning_flow);
  RUN_TEST(force_fault_and_activestop);

  std::cout << "  all passed" << std::endl;
  return 0;
}
