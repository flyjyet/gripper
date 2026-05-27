#pragma once

#include <vector>

#include "common/result.hpp"
#include "controller/self_check/friction_identifier.hpp"
#include "controller/self_check/motion_health_checker.hpp"
#include "controller/self_check/structure_parameter_identifier.hpp"
#include "controller/self_check/structure_profile.hpp"
#include "controller/self_check/travel_limit_identifier.hpp"

namespace gripper::controller::self_check {

struct SelfCheckManagerConfig {
  StructureParameterIdentifierConfig structure_parameter_config{};
  FrictionIdentifierConfig friction_config{};
  TravelLimitIdentifierConfig travel_limit_config{};
  MotionHealthCheckerConfig motion_health_config{};
};

struct SelfCheckInput {
  std::vector<MotionIdentificationSample> motion_samples{};
  std::vector<FeedbackNoiseSample> noise_samples{};
  std::vector<StaticFrictionSample> static_friction_samples{};
  std::vector<DynamicFrictionSample> dynamic_friction_samples{};
  std::vector<LimitObservationSample> limit_samples{};
  std::vector<MotionHealthSample> health_samples{};
};

struct SelfCheckOutput {
  common::Result result{};
  StructureProfile profile{};
};

class SelfCheckManager {
 public:
  explicit SelfCheckManager(SelfCheckManagerConfig config = {});

  [[nodiscard]] SelfCheckOutput run(const SelfCheckInput& input) const;

 private:
  SelfCheckManagerConfig config_{};
};

}  // namespace gripper::controller::self_check
