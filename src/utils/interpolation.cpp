#include "utils/interpolation.hpp"

#include "utils/math_utils.hpp"

namespace gripper::utils {

double lerp(double start, double end, double ratio) noexcept {
  return start + (end - start) * ratio;
}

double inverseLerp(double start, double end, double value) noexcept {
  if (near(start, end, 0.0)) {
    return 0.0;
  }
  return (value - start) / (end - start);
}

double interpolateLinear(double x0, double y0, double x1, double y1,
                         double x) noexcept {
  return lerp(y0, y1, inverseLerp(x0, x1, x));
}

double interpolateLinear(const std::vector<LookupPoint>& points,
                         double x) noexcept {
  if (points.empty()) {
    return 0.0;
  }
  if (points.size() == 1 || x <= points.front().x) {
    return points.front().y;
  }
  if (x >= points.back().x) {
    return points.back().y;
  }

  for (std::size_t index = 1; index < points.size(); ++index) {
    const LookupPoint& right = points[index];
    if (x <= right.x) {
      const LookupPoint& left = points[index - 1];
      return interpolateLinear(left.x, left.y, right.x, right.y, x);
    }
  }

  return points.back().y;
}

}  // namespace gripper::utils
