#pragma once

#include <cstdint>

namespace gripper::common {

// Monotonic duration in nanoseconds. The value is supplied by the caller; this
// header intentionally does not call any platform clock API.
struct Duration {
  std::int64_t ns{0};

  [[nodiscard]] static constexpr Duration fromNanoseconds(
      std::int64_t value) noexcept {
    return Duration{value};
  }

  [[nodiscard]] static constexpr Duration fromMicroseconds(
      std::int64_t value) noexcept {
    return Duration{value * 1000};
  }

  [[nodiscard]] static constexpr Duration fromMilliseconds(
      std::int64_t value) noexcept {
    return Duration{value * 1000 * 1000};
  }

  [[nodiscard]] static constexpr Duration fromSeconds(double value) noexcept {
    return Duration{static_cast<std::int64_t>(value * 1000000000.0)};
  }

  [[nodiscard]] constexpr double seconds() const noexcept {
    return static_cast<double>(ns) / 1000000000.0;
  }
};

// Monotonic timestamp in nanoseconds from an unspecified steady epoch.
struct Timestamp {
  std::int64_t ns{0};

  [[nodiscard]] static constexpr Timestamp fromNanoseconds(
      std::int64_t value) noexcept {
    return Timestamp{value};
  }
};

[[nodiscard]] constexpr Duration operator-(Timestamp lhs,
                                           Timestamp rhs) noexcept {
  return Duration{lhs.ns - rhs.ns};
}

[[nodiscard]] constexpr Timestamp operator+(Timestamp timestamp,
                                            Duration duration) noexcept {
  return Timestamp{timestamp.ns + duration.ns};
}

[[nodiscard]] constexpr Timestamp operator-(Timestamp timestamp,
                                            Duration duration) noexcept {
  return Timestamp{timestamp.ns - duration.ns};
}

[[nodiscard]] constexpr bool operator==(Timestamp lhs,
                                        Timestamp rhs) noexcept {
  return lhs.ns == rhs.ns;
}

[[nodiscard]] constexpr bool operator!=(Timestamp lhs,
                                        Timestamp rhs) noexcept {
  return !(lhs == rhs);
}

[[nodiscard]] constexpr bool operator<(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.ns < rhs.ns;
}

[[nodiscard]] constexpr bool operator<=(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.ns <= rhs.ns;
}

[[nodiscard]] constexpr bool operator>(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.ns > rhs.ns;
}

[[nodiscard]] constexpr bool operator>=(Timestamp lhs, Timestamp rhs) noexcept {
  return lhs.ns >= rhs.ns;
}

}  // namespace gripper::common
