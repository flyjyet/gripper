#include "gripper_control/types.hpp"

namespace gripper_control {

const char* toString(GripperState state) {
  switch (state) {
    case GripperState::Disabled:
      return "Disabled";
    case GripperState::PowerOnSelfCheck:
      return "PowerOnSelfCheck";
    case GripperState::HomingOpenStop:
      return "HomingOpenStop";
    case GripperState::FrictionIdentify:
      return "FrictionIdentify";
    case GripperState::Ready:
      return "Ready";
    case GripperState::GuardedClosing:
      return "GuardedClosing";
    case GripperState::ContactDetected:
      return "ContactDetected";
    case GripperState::ForceBuild:
      return "ForceBuild";
    case GripperState::UnloadBeforeDisable:
      return "UnloadBeforeDisable";
    case GripperState::ClampDoneDisabled:
      return "ClampDoneDisabled";
    case GripperState::UnlockBeforeOpen:
      return "UnlockBeforeOpen";
    case GripperState::Opening:
      return "Opening";
    case GripperState::AntiJamRelease:
      return "AntiJamRelease";
    case GripperState::FaultStop:
      return "FaultStop";
  }
  return "Unknown";
}

const char* toString(GripperFault fault) {
  switch (fault) {
    case GripperFault::None:
      return "None";
    case GripperFault::FeedbackTimeout:
      return "FeedbackTimeout";
    case GripperFault::EncoderFault:
      return "EncoderFault";
    case GripperFault::DriveFault:
      return "DriveFault";
    case GripperFault::OverTemperature:
      return "OverTemperature";
    case GripperFault::OverCurrent:
      return "OverCurrent";
    case GripperFault::PositionLimit:
      return "PositionLimit";
    case GripperFault::HomingFailed:
      return "HomingFailed";
    case GripperFault::ContactNotFound:
      return "ContactNotFound";
    case GripperFault::AbnormalJam:
      return "AbnormalJam";
    case GripperFault::AntiJamFailed:
      return "AntiJamFailed";
    case GripperFault::CalibrationInvalid:
      return "CalibrationInvalid";
  }
  return "Unknown";
}

}  // namespace gripper_control
