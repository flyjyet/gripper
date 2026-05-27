#include "hardware_interface/virtual_encoder.hpp"

#include <cmath>

namespace gripper::hardware_interface {

MultiTurnVirtualEncoder::MultiTurnVirtualEncoder(VirtualEncoderConfig config)
    : config_(config) {}

void MultiTurnVirtualEncoder::setConfig(VirtualEncoderConfig config) noexcept {
  config_ = config;
  reset();
}

void MultiTurnVirtualEncoder::reset() noexcept {
  initialized_ = false;
  last_wrapped_position_ = {};
  continuous_position_ = {};
}

VirtualEncoderSample MultiTurnVirtualEncoder::update(
    common::Rad wrapped_position) noexcept {
  return update(wrapped_position, wrapped_position);
}

VirtualEncoderSample MultiTurnVirtualEncoder::update(
    common::Rad wrapped_position,
    common::Rad initial_continuous_position) noexcept {
  VirtualEncoderSample sample{};
  sample.wrapped_position = wrapped_position;
  sample.initialized = initialized_;

  const double wrap_range = config_.wrap_range.value;
  if (!config_.enabled || wrap_range <= 0.0) {
    initialized_ = true;
    last_wrapped_position_ = wrapped_position;
    continuous_position_ = initial_continuous_position;
    sample.continuous_position = continuous_position_;
    return sample;
  }

  if (!initialized_) {
    initialized_ = true;
    last_wrapped_position_ = wrapped_position;
    continuous_position_ = initial_continuous_position;
    sample.initialized = false;
    sample.continuous_position = continuous_position_;
    return sample;
  }

  double delta = wrapped_position.value - last_wrapped_position_.value;
  const double half_range = wrap_range * 0.5;
  if (delta > half_range) {
    delta -= wrap_range;
    sample.wrap_corrected = true;
  } else if (delta < -half_range) {
    delta += wrap_range;
    sample.wrap_corrected = true;
  }

  continuous_position_.value += delta;
  last_wrapped_position_ = wrapped_position;
  sample.incremental_delta = common::Rad{delta};
  sample.continuous_position = continuous_position_;
  sample.initialized = true;
  return sample;
}

bool MultiTurnVirtualEncoder::initialized() const noexcept {
  return initialized_;
}

common::Rad MultiTurnVirtualEncoder::continuousPosition() const noexcept {
  return continuous_position_;
}

common::Rad MultiTurnVirtualEncoder::lastWrappedPosition() const noexcept {
  return last_wrapped_position_;
}

}  // namespace gripper::hardware_interface
