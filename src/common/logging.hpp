#pragma once

#include <cstdint>
#include <string_view>

#include "common/error_code.hpp"
#include "common/timestamp.hpp"

namespace gripper::common {

enum class LogLevel : std::uint8_t {
  Trace,
  Debug,
  Info,
  Warning,
  Error,
  Critical,
};

// Lightweight event data for logging boundaries. This does not implement a log
// sink and does not own message/module storage.
struct LogEvent {
  Timestamp timestamp{};
  LogLevel level{LogLevel::Info};
  ErrorCode code{ErrorCode::Ok};
  std::string_view module{};
  std::string_view message{};
};

[[nodiscard]] constexpr std::string_view toString(LogLevel level) noexcept {
  switch (level) {
    case LogLevel::Trace:
      return "Trace";
    case LogLevel::Debug:
      return "Debug";
    case LogLevel::Info:
      return "Info";
    case LogLevel::Warning:
      return "Warning";
    case LogLevel::Error:
      return "Error";
    case LogLevel::Critical:
      return "Critical";
  }
  return "Unknown";
}

}  // namespace gripper::common
