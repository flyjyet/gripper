#include "utils/filter_utils.hpp"

#include "utils/math_utils.hpp"

namespace gripper::utils {

MovingAverage::MovingAverage(std::size_t window_size)
    : window_size_(window_size == 0 ? 1 : window_size) {}

void MovingAverage::reset() noexcept {
  samples_.clear();
  sum_ = 0.0;
}

double MovingAverage::update(double sample) {
  samples_.push_back(sample);
  sum_ += sample;

  while (samples_.size() > window_size_) {
    sum_ -= samples_.front();
    samples_.pop_front();
  }

  return value();
}

double MovingAverage::value() const noexcept {
  if (samples_.empty()) {
    return 0.0;
  }
  return sum_ / static_cast<double>(samples_.size());
}

std::size_t MovingAverage::sampleCount() const noexcept {
  return samples_.size();
}

FirstOrderLowPass::FirstOrderLowPass(double alpha)
    : alpha_(clamp(alpha, 0.0, 1.0)) {}

void FirstOrderLowPass::reset(double value) noexcept {
  value_ = value;
  initialized_ = false;
}

double FirstOrderLowPass::update(double sample) noexcept {
  if (!initialized_) {
    value_ = sample;
    initialized_ = true;
    return value_;
  }

  value_ = alpha_ * sample + (1.0 - alpha_) * value_;
  return value_;
}

double FirstOrderLowPass::value() const noexcept {
  return value_;
}

bool FirstOrderLowPass::initialized() const noexcept {
  return initialized_;
}

}  // namespace gripper::utils
