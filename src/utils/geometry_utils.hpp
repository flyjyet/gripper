#pragma once

#include "common/units.hpp"

namespace gripper::utils {

struct Point2 {
  common::Mm x{0.0};
  common::Mm y{0.0};
};

[[nodiscard]] common::Mm distance(Point2 lhs, Point2 rhs) noexcept;
[[nodiscard]] common::MmPerS angularToLinearSpeed(common::RadPerS angular_speed,
                                                  common::Mm radius) noexcept;
[[nodiscard]] common::RadPerS linearToAngularSpeed(common::MmPerS linear_speed,
                                                   common::Mm radius) noexcept;
[[nodiscard]] common::Rad degreesToRadians(double degrees) noexcept;
[[nodiscard]] double radiansToDegrees(common::Rad radians) noexcept;

}  // namespace gripper::utils
