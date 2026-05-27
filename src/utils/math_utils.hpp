#pragma once

#include <cmath>
#include <type_traits>

namespace gripper::utils {

template <typename T>
[[nodiscard]] constexpr const T& clamp(const T& value, const T& low,
                                       const T& high) {
  return value < low ? low : (high < value ? high : value);
}

[[nodiscard]] bool near(double lhs, double rhs, double tolerance) noexcept;
[[nodiscard]] bool nearZero(double value, double tolerance) noexcept;

template <typename Unit>
[[nodiscard]] bool nearUnit(Unit lhs, Unit rhs, Unit tolerance) noexcept {
  return near(lhs.value, rhs.value, tolerance.value);
}

template <typename T>
[[nodiscard]] constexpr int sign(T value) noexcept {
  return (T{} < value) - (value < T{});
}

[[nodiscard]] double square(double value) noexcept;

}  // namespace gripper::utils
