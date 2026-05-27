#pragma once

#include <string>
#include <utility>

#include "common/error_code.hpp"

namespace gripper::common {

// Lightweight operation result for public module interfaces.
//
// Success:
// - code is ErrorCode::Ok.
//
// Failure:
// - code identifies the project-level failure category.
// - message may carry a human-readable diagnostic without exposing SDK details.
class Result {
 public:
  Result() = default;

  explicit Result(ErrorCode code) : code_(code) {}

  Result(ErrorCode code, std::string message)
      : code_(code), message_(std::move(message)) {}

  [[nodiscard]] static Result ok() { return Result{}; }

  [[nodiscard]] static Result error(ErrorCode code) { return Result{code}; }

  [[nodiscard]] static Result error(ErrorCode code, std::string message) {
    return Result{code, std::move(message)};
  }

  [[nodiscard]] bool isOk() const noexcept { return gripper::common::isOk(code_); }
  [[nodiscard]] bool isError() const noexcept { return !isOk(); }
  [[nodiscard]] explicit operator bool() const noexcept { return isOk(); }

  [[nodiscard]] ErrorCode code() const noexcept { return code_; }
  [[nodiscard]] const std::string& message() const noexcept { return message_; }
  [[nodiscard]] bool hasMessage() const noexcept { return !message_.empty(); }

 private:
  ErrorCode code_{ErrorCode::Ok};
  std::string message_{};
};

[[nodiscard]] inline Result Ok() { return Result::ok(); }

[[nodiscard]] inline Result Error(ErrorCode code) { return Result::error(code); }

[[nodiscard]] inline Result Error(ErrorCode code, std::string message) {
  return Result::error(code, std::move(message));
}

}  // namespace gripper::common
