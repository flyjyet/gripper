#include "gripper_control/force_command_mapper.hpp"

#include <algorithm>

namespace gripper_control {

ForceCommandMapper::ForceCommandMapper(ForceMapParams params) : params_(params) {}

void ForceCommandMapper::setParams(const ForceMapParams& params) { params_ = params; }

const ForceMapParams& ForceCommandMapper::params() const { return params_; }

double ForceCommandMapper::targetCurrentForForce(double target_force_per_side_N,
                                                 double /*contact_stroke_mm*/,
                                                 const FrictionParams& friction) const {
  const double force = std::clamp(target_force_per_side_N, 0.0, params_.max_target_force_N);
  const double friction_current = friction.valid ? friction.current_close_A : 0.10;
  return params_.baseline_current_A + friction_current + force * params_.current_per_N;
}

double ForceCommandMapper::estimateForceFromCurrent(double current_A,
                                                    double /*contact_stroke_mm*/,
                                                    const FrictionParams& friction) const {
  const double friction_current = friction.valid ? friction.current_close_A : 0.10;
  const double force_current = current_A - params_.baseline_current_A - friction_current;
  return std::max(0.0, force_current / params_.current_per_N);
}

}  // namespace gripper_control
