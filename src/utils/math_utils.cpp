#include "utils/math_utils.hpp"

namespace gripper::utils {

bool near(double lhs, double rhs, double tolerance) noexcept {
  return std::abs(lhs - rhs) <= std::abs(tolerance);
}

bool nearZero(double value, double tolerance) noexcept {
  return near(value, 0.0, tolerance);
}

double square(double value) noexcept {
  return value * value;
}

}  // namespace gripper::utils
