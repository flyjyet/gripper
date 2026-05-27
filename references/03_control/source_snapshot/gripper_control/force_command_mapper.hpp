#pragma once

#include "gripper_control/types.hpp"

namespace gripper_control {

class ForceCommandMapper {
 public:
  explicit ForceCommandMapper(ForceMapParams params = {});

  void setParams(const ForceMapParams& params);
  const ForceMapParams& params() const;

  double targetCurrentForForce(double target_force_per_side_N,
                               double contact_stroke_mm,
                               const FrictionParams& friction) const;

  double estimateForceFromCurrent(double current_A,
                                  double contact_stroke_mm,
                                  const FrictionParams& friction) const;

 private:
  ForceMapParams params_;
};

}  // namespace gripper_control
