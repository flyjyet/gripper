#pragma once

#include <cstdint>
#include <string_view>

namespace gripper::common {

inline constexpr std::string_view kProjectName{"TS2000 Gripper"};
inline constexpr std::string_view kProjectVersion{"0.1.0"};

inline constexpr std::uint32_t kProjectVersionMajor{0};
inline constexpr std::uint32_t kProjectVersionMinor{1};
inline constexpr std::uint32_t kProjectVersionPatch{0};

}  // namespace gripper::common
