#include "controller/state_machine/self_check_state.hpp"

namespace gripper::controller::state_machine {

namespace {

[[nodiscard]] PreSelfCheckPhase nextPhase(PreSelfCheckPhase phase,
                                          PreSelfCheckEvent event,
                                          bool* accepted,
                                          PreSelfCheckResult* result) {
  *accepted = true;

  if (event == PreSelfCheckEvent::AbortRequested) {
    *result = PreSelfCheckResult::NeedConservativeHoming;
    return PreSelfCheckPhase::Failed;
  }
  if (event == PreSelfCheckEvent::FaultTriggered) {
    *result = PreSelfCheckResult::Failed;
    return PreSelfCheckPhase::Failed;
  }

  switch (phase) {
    case PreSelfCheckPhase::Idle:
      if (event == PreSelfCheckEvent::Start) {
        *result = PreSelfCheckResult::InProgress;
        return PreSelfCheckPhase::LimitedProbe;
      }
      break;
    case PreSelfCheckPhase::LimitedProbe:
      if (event == PreSelfCheckEvent::ProbePassed) {
        return PreSelfCheckPhase::BidirectionalMoveEnable;
      }
      if (event == PreSelfCheckEvent::ProbeFailed) {
        *result = PreSelfCheckResult::NeedConservativeHoming;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::BidirectionalMoveEnable:
      if (event == PreSelfCheckEvent::BidirectionalMoveEnabled) {
        return PreSelfCheckPhase::StableShortStrokeMotion;
      }
      if (event == PreSelfCheckEvent::BidirectionalMoveFailed) {
        *result = PreSelfCheckResult::NeedManualCheck;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::StableShortStrokeMotion:
      if (event == PreSelfCheckEvent::StableStrokePassed) {
        return PreSelfCheckPhase::PreliminaryLimitSearch;
      }
      if (event == PreSelfCheckEvent::StableStrokeFailed) {
        *result = PreSelfCheckResult::NeedConservativeHoming;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::PreliminaryLimitSearch:
      if (event == PreSelfCheckEvent::PreliminaryLimitFound) {
        return PreSelfCheckPhase::TheoryTravelCheck;
      }
      if (event == PreSelfCheckEvent::PreliminaryLimitFailed) {
        *result = PreSelfCheckResult::NeedConservativeHoming;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::TheoryTravelCheck:
      if (event == PreSelfCheckEvent::TheoryTravelMatched) {
        return PreSelfCheckPhase::SafeZoneBuild;
      }
      if (event == PreSelfCheckEvent::TheoryTravelMismatch) {
        *result = PreSelfCheckResult::NeedManualCheck;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::SafeZoneBuild:
      if (event == PreSelfCheckEvent::SafeZoneBuilt) {
        return PreSelfCheckPhase::MultiRegionRoundTripLearning;
      }
      if (event == PreSelfCheckEvent::SafeZoneFailed) {
        *result = PreSelfCheckResult::NeedConservativeHoming;
        return PreSelfCheckPhase::Failed;
      }
      break;
    case PreSelfCheckPhase::MultiRegionRoundTripLearning:
      if (event == PreSelfCheckEvent::RegionLearningCompleted) {
        return PreSelfCheckPhase::StructureProfileUpdate;
      }
      if (event == PreSelfCheckEvent::RegionSampleAccepted ||
          event == PreSelfCheckEvent::RegionSampleRejected) {
        return phase;
      }
      break;
    case PreSelfCheckPhase::StructureProfileUpdate:
      if (event == PreSelfCheckEvent::StructureProfileUpdated) {
        *result = PreSelfCheckResult::Completed;
        return PreSelfCheckPhase::Completed;
      }
      break;
    case PreSelfCheckPhase::Completed:
    case PreSelfCheckPhase::Failed:
      break;
  }

  *accepted = false;
  return phase;
}

}  // namespace

PreSelfCheckPhase PreSelfCheckStateMachine::phase() const {
  return snapshot_.phase;
}

PreSelfCheckSnapshot PreSelfCheckStateMachine::snapshot() const {
  return snapshot_;
}

bool PreSelfCheckStateMachine::canAccept(PreSelfCheckEvent event) const {
  bool accepted = false;
  auto result = snapshot_.result;
  (void)nextPhase(snapshot_.phase, event, &accepted, &result);
  return accepted;
}

TransitionDecision PreSelfCheckStateMachine::dispatch(
    PreSelfCheckEvent event) {
  bool accepted = false;
  auto result = snapshot_.result;
  const auto next = nextPhase(snapshot_.phase, event, &accepted, &result);
  if (!accepted) {
    return TransitionDecision::Rejected;
  }

  snapshot_.previous_phase = snapshot_.phase;
  snapshot_.phase = next;
  snapshot_.last_event = event;
  snapshot_.result = result;

  if (event == PreSelfCheckEvent::RegionSampleAccepted) {
    ++snapshot_.valid_sample_count;
  } else if (event == PreSelfCheckEvent::RegionSampleRejected) {
    ++snapshot_.rejected_sample_count;
  }

  return TransitionDecision::Accepted;
}

void PreSelfCheckStateMachine::reset() {
  snapshot_ = {};
}

}  // namespace gripper::controller::state_machine
