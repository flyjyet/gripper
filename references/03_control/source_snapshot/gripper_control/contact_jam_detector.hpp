#pragma once

#include "gripper_control/types.hpp"

namespace gripper_control {

enum class StallKind {
  None,
  Contact,
  AbnormalJam,
};

struct DetectionResult {
  bool contact = false;
  bool stalled = false;
  StallKind stall_kind = StallKind::None;
};

class ContactJamDetector {
 public:
  explicit ContactJamDetector(JamDetectionParams params = {});

  void reset();
  void setParams(const JamDetectionParams& params);
  const JamDetectionParams& params() const;

  DetectionResult update(double now_s,
                         const MotorFeedback& feedback,
                         double commanded_velocity_mm_s,
                         const FrictionParams& friction,
                         bool closing,
                         bool contact_can_be_normal);

 private:
  double directionFriction(const FrictionParams& friction, bool closing) const;

  JamDetectionParams params_;
  bool have_last_position_ = false;
  double last_position_mm_ = 0.0;
  double last_window_start_s_ = 0.0;
  double window_start_position_mm_ = 0.0;
  double contact_started_s_ = -1.0;
  double stall_started_s_ = -1.0;
};

}  // namespace gripper_control
