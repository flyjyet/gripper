#pragma once

#include <cstdint>

#include "controller/state_machine/gripper_state_machine.hpp"

namespace gripper::controller::state_machine {

enum class PreSelfCheckPhase : std::uint8_t {
  Idle,
  LimitedProbe,
  BidirectionalMoveEnable,
  StableShortStrokeMotion,
  PreliminaryLimitSearch,
  TheoryTravelCheck,
  SafeZoneBuild,
  MultiRegionRoundTripLearning,
  StructureProfileUpdate,
  Completed,
  Failed,
};

enum class PreSelfCheckEvent : std::uint8_t {
  Start,
  ProbePassed,
  ProbeFailed,
  BidirectionalMoveEnabled,
  BidirectionalMoveFailed,
  StableStrokePassed,
  StableStrokeFailed,
  PreliminaryLimitFound,
  PreliminaryLimitFailed,
  TheoryTravelMatched,
  TheoryTravelMismatch,
  SafeZoneBuilt,
  SafeZoneFailed,
  RegionSampleAccepted,
  RegionSampleRejected,
  RegionLearningCompleted,
  StructureProfileUpdated,
  AbortRequested,
  FaultTriggered,
};

enum class PreSelfCheckResult : std::uint8_t {
  NotStarted,
  InProgress,
  Completed,
  NeedConservativeHoming,
  NeedManualCheck,
  Failed,
};

struct PreSelfCheckSnapshot {
  PreSelfCheckPhase phase{PreSelfCheckPhase::Idle};
  PreSelfCheckPhase previous_phase{PreSelfCheckPhase::Idle};
  PreSelfCheckEvent last_event{PreSelfCheckEvent::Start};
  PreSelfCheckResult result{PreSelfCheckResult::NotStarted};
  std::uint32_t valid_sample_count{0};
  std::uint32_t rejected_sample_count{0};
};

class PreSelfCheckStateMachine {
 public:
  PreSelfCheckStateMachine() = default;

  [[nodiscard]] PreSelfCheckPhase phase() const;
  [[nodiscard]] PreSelfCheckSnapshot snapshot() const;
  [[nodiscard]] bool canAccept(PreSelfCheckEvent event) const;

  TransitionDecision dispatch(PreSelfCheckEvent event);
  void reset();

 private:
  PreSelfCheckSnapshot snapshot_{};
};

}  // namespace gripper::controller::state_machine
