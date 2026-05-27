#pragma once

#include <vector>

namespace gripper::utils {

struct LookupPoint {
  double x{0.0};
  double y{0.0};
};

[[nodiscard]] double lerp(double start, double end, double ratio) noexcept;

[[nodiscard]] double inverseLerp(double start, double end,
                                 double value) noexcept;

[[nodiscard]] double interpolateLinear(double x0, double y0, double x1,
                                       double y1, double x) noexcept;

// Piecewise linear interpolation. Points must be sorted by x ascending.
[[nodiscard]] double interpolateLinear(const std::vector<LookupPoint>& points,
                                       double x) noexcept;

}  // namespace gripper::utils
