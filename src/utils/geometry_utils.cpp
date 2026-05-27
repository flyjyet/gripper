#include "utils/geometry_utils.hpp"

#include <cmath>

#include "utils/math_utils.hpp"

namespace gripper::utils {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

common::Mm distance(Point2 lhs, Point2 rhs) noexcept {
  const double dx = lhs.x.value - rhs.x.value;
  const double dy = lhs.y.value - rhs.y.value;
  return common::Mm{std::sqrt(square(dx) + square(dy))};
}

common::MmPerS angularToLinearSpeed(common::RadPerS angular_speed,
                                    common::Mm radius) noexcept {
  return common::MmPerS{angular_speed.value * radius.value};
}

common::RadPerS linearToAngularSpeed(common::MmPerS linear_speed,
                                     common::Mm radius) noexcept {
  if (nearZero(radius.value, 0.0)) {
    return common::RadPerS{0.0};
  }
  return common::RadPerS{linear_speed.value / radius.value};
}

common::Rad degreesToRadians(double degrees) noexcept {
  return common::Rad{degrees * kPi / 180.0};
}

double radiansToDegrees(common::Rad radians) noexcept {
  return radians.value * 180.0 / kPi;
}

}  // namespace gripper::utils
