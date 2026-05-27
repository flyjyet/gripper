#pragma once

#include <filesystem>
#include <string>

#include "common/error_code.hpp"
#include "config/gripper_config.hpp"

namespace gripper::config {

struct ConfigLoadResult {
  common::ErrorCode code{common::ErrorCode::Ok};
  GripperConfig config{};
  std::string message{};

  [[nodiscard]] bool isOk() const noexcept {
    return code == common::ErrorCode::Ok;
  }

  [[nodiscard]] explicit operator bool() const noexcept { return isOk(); }
};

// Returns conservative defaults. Units are documented in GripperConfig fields.
[[nodiscard]] GripperConfig defaultConfig();

// Loads a constrained YAML-like configuration file. The parser supports the
// scalar and one-line list fields used by default_gripper.yaml; unknown fields
// are ignored so future templates can remain backward compatible.
[[nodiscard]] ConfigLoadResult loadFromFile(const std::filesystem::path& path);

}  // namespace gripper::config
