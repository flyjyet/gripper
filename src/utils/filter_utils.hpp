#pragma once

#include <cstddef>
#include <deque>

namespace gripper::utils {

class MovingAverage {
 public:
  explicit MovingAverage(std::size_t window_size);

  void reset() noexcept;
  [[nodiscard]] double update(double sample);
  [[nodiscard]] double value() const noexcept;
  [[nodiscard]] std::size_t sampleCount() const noexcept;

 private:
  std::size_t window_size_{1};
  std::deque<double> samples_{};
  double sum_{0.0};
};

class FirstOrderLowPass {
 public:
  // alpha is dimensionless and must be in [0, 1].
  explicit FirstOrderLowPass(double alpha);

  void reset(double value = 0.0) noexcept;
  [[nodiscard]] double update(double sample) noexcept;
  [[nodiscard]] double value() const noexcept;
  [[nodiscard]] bool initialized() const noexcept;

 private:
  double alpha_{1.0};
  double value_{0.0};
  bool initialized_{false};
};

}  // namespace gripper::utils
