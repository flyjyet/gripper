#pragma once

#include <cstdint>
#include <string_view>

namespace gripper::common {

// Project-wide error categories. Numeric ranges are reserved by category so
// module-specific code can add detail without changing public interfaces.
enum class ErrorCode : std::uint32_t {
  Ok = 0,

  Unknown = 1,
  InvalidArgument = 2,
  InvalidOperation = 3,
  OutOfRange = 4,
  ResourceUnavailable = 5,
  InternalError = 6,

  ConfigMissing = 100,
  ConfigInvalid = 101,
  ConfigUnsupported = 102,

  ConnectionFailed = 200,
  ConnectionLost = 201,
  ConnectionNotOpen = 202,
  ProtocolError = 203,

  HardwareUnavailable = 300,
  HardwareFault = 301,
  HardwareRejectedCommand = 302,
  HardwareFeedbackInvalid = 303,

  StateMachineInvalidTransition = 400,
  StateMachineRejected = 401,
  StateMachineNotReady = 402,

  SelfCheckFailed = 500,
  SelfCheckInconsistentFeedback = 501,
  SelfCheckUnsafeMotion = 502,

  SafetyLimitExceeded = 600,
  SafetyActiveStop = 601,
  SafetyJamDetected = 602,
  SafetyContactDetected = 603,

  ControlNotReady = 700,
  ControlTargetInvalid = 701,
  ControlSaturation = 702,
  ControlUnstable = 703,

  Timeout = 800,
  OperationTimedOut = 801,
  FeedbackTimedOut = 802,

  NotImplemented = 900,
  Unsupported = 901,
};

[[nodiscard]] constexpr bool isOk(ErrorCode code) noexcept {
  return code == ErrorCode::Ok;
}

[[nodiscard]] constexpr std::uint32_t toValue(ErrorCode code) noexcept {
  return static_cast<std::uint32_t>(code);
}

[[nodiscard]] constexpr std::string_view toString(ErrorCode code) noexcept {
  switch (code) {
    case ErrorCode::Ok:
      return "Ok";
    case ErrorCode::Unknown:
      return "Unknown";
    case ErrorCode::InvalidArgument:
      return "InvalidArgument";
    case ErrorCode::InvalidOperation:
      return "InvalidOperation";
    case ErrorCode::OutOfRange:
      return "OutOfRange";
    case ErrorCode::ResourceUnavailable:
      return "ResourceUnavailable";
    case ErrorCode::InternalError:
      return "InternalError";
    case ErrorCode::ConfigMissing:
      return "ConfigMissing";
    case ErrorCode::ConfigInvalid:
      return "ConfigInvalid";
    case ErrorCode::ConfigUnsupported:
      return "ConfigUnsupported";
    case ErrorCode::ConnectionFailed:
      return "ConnectionFailed";
    case ErrorCode::ConnectionLost:
      return "ConnectionLost";
    case ErrorCode::ConnectionNotOpen:
      return "ConnectionNotOpen";
    case ErrorCode::ProtocolError:
      return "ProtocolError";
    case ErrorCode::HardwareUnavailable:
      return "HardwareUnavailable";
    case ErrorCode::HardwareFault:
      return "HardwareFault";
    case ErrorCode::HardwareRejectedCommand:
      return "HardwareRejectedCommand";
    case ErrorCode::HardwareFeedbackInvalid:
      return "HardwareFeedbackInvalid";
    case ErrorCode::StateMachineInvalidTransition:
      return "StateMachineInvalidTransition";
    case ErrorCode::StateMachineRejected:
      return "StateMachineRejected";
    case ErrorCode::StateMachineNotReady:
      return "StateMachineNotReady";
    case ErrorCode::SelfCheckFailed:
      return "SelfCheckFailed";
    case ErrorCode::SelfCheckInconsistentFeedback:
      return "SelfCheckInconsistentFeedback";
    case ErrorCode::SelfCheckUnsafeMotion:
      return "SelfCheckUnsafeMotion";
    case ErrorCode::SafetyLimitExceeded:
      return "SafetyLimitExceeded";
    case ErrorCode::SafetyActiveStop:
      return "SafetyActiveStop";
    case ErrorCode::SafetyJamDetected:
      return "SafetyJamDetected";
    case ErrorCode::SafetyContactDetected:
      return "SafetyContactDetected";
    case ErrorCode::ControlNotReady:
      return "ControlNotReady";
    case ErrorCode::ControlTargetInvalid:
      return "ControlTargetInvalid";
    case ErrorCode::ControlSaturation:
      return "ControlSaturation";
    case ErrorCode::ControlUnstable:
      return "ControlUnstable";
    case ErrorCode::Timeout:
      return "Timeout";
    case ErrorCode::OperationTimedOut:
      return "OperationTimedOut";
    case ErrorCode::FeedbackTimedOut:
      return "FeedbackTimedOut";
    case ErrorCode::NotImplemented:
      return "NotImplemented";
    case ErrorCode::Unsupported:
      return "Unsupported";
  }
  return "UnrecognizedErrorCode";
}

}  // namespace gripper::common
