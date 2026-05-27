#include "controller/safety/safety_limiter.hpp"

#include <cmath>
#include <iostream>

#include "test_utils.hpp"

namespace sf = gripper::controller::safety;
namespace sm = gripper::controller::state_machine;

using gripper::common::A;
using gripper::common::Mm;
using gripper::common::MmPerS;
using gripper::common::MmPerS2;
using gripper::common::S;

static bool nearDouble(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) <= eps;
}

// Helper: construct a minimal SafetyLimitCommand for current/speed testing.
static sf::SafetyLimitCommand makeCmd(double current, double speed) {
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{current};
  cmd.nut_speed = MmPerS{speed};
  cmd.target_nut_stroke = Mm{8.0};
  cmd.current_nut_stroke = Mm{8.0};
  cmd.previous_nut_speed = MmPerS{speed};
  cmd.control_period = S{0.01};
  return cmd;
}

static int test_current_in_range() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{2.0};
  config.max_nut_speed = MmPerS{10.0};
  config.active_stop_on_current_limit = true;

  sf::SafetyLimiter limiter(config);
  auto cmd = makeCmd(1.0, 3.0);
  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(!result.current_limited, "current 1.0 <= 2.0 should not be limited");
  TEST_ASSERT(!result.active_stop_required, "in-range current must not trigger stop");
  TEST_ASSERT(nearDouble(result.motor_current.value, 1.0),
              "current value must be unchanged");

  return 0;
}

static int test_current_exceed_and_stop() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{1.0};
  config.max_nut_speed = MmPerS{10.0};
  config.active_stop_on_current_limit = true;

  sf::SafetyLimiter limiter(config);
  auto cmd = makeCmd(2.0, 3.0);
  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.current_limited, "current 2.0 > 1.0 must be limited");
  TEST_ASSERT(result.active_stop_required, "active_stop must be required");
  TEST_ASSERT_EQ(static_cast<int>(result.active_stop_reason),
                 static_cast<int>(sm::ActiveStopReason::CurrentLimit),
                 "reason must be CurrentLimit");
  TEST_ASSERT(nearDouble(result.motor_current.value, 1.0),
              "current must be clipped to 1.0");

  return 0;
}

static int test_current_exceed_no_stop() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{1.0};
  config.max_nut_speed = MmPerS{10.0};
  config.active_stop_on_current_limit = false;

  sf::SafetyLimiter limiter(config);
  auto cmd = makeCmd(2.0, 3.0);
  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.current_limited, "current must still be limited");
  TEST_ASSERT(!result.active_stop_required,
              "active_stop must NOT be required when switch is off");
  TEST_ASSERT(nearDouble(result.motor_current.value, 1.0),
              "current must still be clipped");

  return 0;
}

static int test_speed_exceed_and_stop() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{10.0};
  config.max_nut_speed = MmPerS{5.0};
  config.active_stop_on_speed_limit = true;

  sf::SafetyLimiter limiter(config);
  auto cmd = makeCmd(1.0, 10.0);
  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.speed_limited, "speed 10 > 5 must be limited");
  TEST_ASSERT(result.active_stop_required, "active_stop must be required");
  TEST_ASSERT_EQ(static_cast<int>(result.active_stop_reason),
                 static_cast<int>(sm::ActiveStopReason::SpeedLimit),
                 "reason must be SpeedLimit");
  TEST_ASSERT(nearDouble(result.nut_speed.value, 5.0),
              "speed must be clipped to 5.0");

  return 0;
}

static int test_speed_exceed_no_stop() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{10.0};
  config.max_nut_speed = MmPerS{5.0};
  config.active_stop_on_speed_limit = false;

  sf::SafetyLimiter limiter(config);
  auto cmd = makeCmd(1.0, 10.0);  // current OK, speed exceeds
  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.speed_limited, "speed must still be limited");
  TEST_ASSERT(!result.active_stop_required,
              "active_stop must NOT be required when speed stop switch is off");
  TEST_ASSERT(nearDouble(result.nut_speed.value, 5.0),
              "speed must still be clipped");

  return 0;
}

static int test_stroke_limit_current_outside() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{10.0};
  config.max_nut_speed = MmPerS{10.0};
  config.stroke_limits_enabled = true;
  config.active_stop_on_stroke_limit = true;
  config.min_nut_stroke = Mm{0.0};
  config.max_nut_stroke = Mm{16.0};

  sf::SafetyLimiter limiter(config);
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{1.0};
  cmd.nut_speed = MmPerS{3.0};
  cmd.target_nut_stroke = Mm{10.0};
  cmd.current_nut_stroke = Mm{-1.0};  // outside safe range
  cmd.previous_nut_speed = MmPerS{3.0};
  cmd.control_period = S{0.01};

  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.stroke_limited, "current stroke outside range must be flagged");
  TEST_ASSERT(result.active_stop_required, "stroke violation must trigger stop");
  TEST_ASSERT_EQ(static_cast<int>(result.active_stop_reason),
                 static_cast<int>(sm::ActiveStopReason::StrokeLimit),
                 "reason must be StrokeLimit");

  return 0;
}

static int test_stroke_limit_target_outside() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{10.0};
  config.max_nut_speed = MmPerS{10.0};
  config.stroke_limits_enabled = true;
  config.active_stop_on_stroke_limit = true;
  config.min_nut_stroke = Mm{0.0};
  config.max_nut_stroke = Mm{16.0};

  sf::SafetyLimiter limiter(config);
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{1.0};
  cmd.nut_speed = MmPerS{3.0};
  cmd.target_nut_stroke = Mm{20.0};  // target outside range
  cmd.current_nut_stroke = Mm{8.0};  // current inside
  cmd.previous_nut_speed = MmPerS{3.0};
  cmd.control_period = S{0.01};

  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.stroke_limited, "target stroke outside range must be flagged");
  TEST_ASSERT(result.active_stop_required, "stroke violation must trigger stop");
  TEST_ASSERT(nearDouble(result.target_nut_stroke.value, 16.0),
              "target must be clipped to 16.0");

  return 0;
}

static int test_acceleration_limit() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{10.0};
  config.max_nut_speed = MmPerS{10.0};
  config.acceleration_limits_enabled = true;
  config.max_nut_acceleration = MmPerS2{10.0};  // 10 mm/s^2

  sf::SafetyLimiter limiter(config);
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{1.0};
  cmd.nut_speed = MmPerS{5.0};           // desired 5 mm/s
  cmd.previous_nut_speed = MmPerS{0.0};  // was 0, max delta = 10 * 0.01 = 0.1
  cmd.target_nut_stroke = Mm{8.0};
  cmd.current_nut_stroke = Mm{8.0};
  cmd.control_period = S{0.01};

  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.acceleration_limited, "speed jump 0->5 in 0.01s must be limited");
  TEST_ASSERT(nearDouble(result.nut_speed.value, 0.1),
              "speed must be clipped to max_delta = 0.1");

  return 0;
}

static int test_multiple_limits_stroke_priority() {
  // Stroke + current both exceed. Stroke has priority because it returns early.
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{1.0};
  config.max_nut_speed = MmPerS{5.0};
  config.stroke_limits_enabled = true;
  config.active_stop_on_current_limit = true;
  config.active_stop_on_stroke_limit = true;
  config.min_nut_stroke = Mm{0.0};
  config.max_nut_stroke = Mm{16.0};

  sf::SafetyLimiter limiter(config);
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{5.0};           // exceeds current
  cmd.nut_speed = MmPerS{3.0};
  cmd.target_nut_stroke = Mm{10.0};
  cmd.current_nut_stroke = Mm{-2.0};    // exceeds stroke
  cmd.previous_nut_speed = MmPerS{3.0};
  cmd.control_period = S{0.01};

  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(result.active_stop_required, "must trigger active stop");
  TEST_ASSERT_EQ(static_cast<int>(result.active_stop_reason),
                 static_cast<int>(sm::ActiveStopReason::StrokeLimit),
                 "stroke limit must take priority over current limit (early return)");

  return 0;
}

static int test_all_stop_switches_off() {
  sf::SafetyLimitConfig config{};
  config.max_motor_current = A{1.0};
  config.max_nut_speed = MmPerS{5.0};
  config.stroke_limits_enabled = true;
  config.active_stop_on_current_limit = false;
  config.active_stop_on_speed_limit = false;
  config.active_stop_on_stroke_limit = false;
  config.min_nut_stroke = Mm{0.0};
  config.max_nut_stroke = Mm{16.0};

  sf::SafetyLimiter limiter(config);
  sf::SafetyLimitCommand cmd{};
  cmd.motor_current = A{5.0};
  cmd.nut_speed = MmPerS{20.0};
  cmd.target_nut_stroke = Mm{30.0};
  cmd.current_nut_stroke = Mm{-5.0};
  cmd.previous_nut_speed = MmPerS{20.0};
  cmd.control_period = S{0.01};

  auto result = limiter.applyLimits(cmd);

  TEST_ASSERT(!result.active_stop_required,
              "no active stop when all stop switches are off");
  TEST_ASSERT(result.current_limited || result.speed_limited || result.stroke_limited,
              "values should still be clipped");

  return 0;
}

static int test_setConfig_updates_behavior() {
  sf::SafetyLimitConfig config1{};
  config1.max_motor_current = A{2.0};
  config1.max_nut_speed = MmPerS{10.0};
  config1.active_stop_on_current_limit = false;

  sf::SafetyLimiter limiter(config1);
  auto cmd = makeCmd(5.0, 1.0);
  auto r1 = limiter.applyLimits(cmd);
  TEST_ASSERT(!r1.active_stop_required, "config1: stop switch off");

  sf::SafetyLimitConfig config2{};
  config2.max_motor_current = A{2.0};
  config2.max_nut_speed = MmPerS{10.0};
  config2.active_stop_on_current_limit = true;
  limiter.setConfig(config2);

  auto r2 = limiter.applyLimits(cmd);
  TEST_ASSERT(r2.active_stop_required, "config2: stop switch on");

  return 0;
}

int main() {
  std::cout << "test_safety_limiter" << std::endl;

  RUN_TEST(current_in_range);
  RUN_TEST(current_exceed_and_stop);
  RUN_TEST(current_exceed_no_stop);
  RUN_TEST(speed_exceed_and_stop);
  RUN_TEST(speed_exceed_no_stop);
  RUN_TEST(stroke_limit_current_outside);
  RUN_TEST(stroke_limit_target_outside);
  RUN_TEST(acceleration_limit);
  RUN_TEST(multiple_limits_stroke_priority);
  RUN_TEST(all_stop_switches_off);
  RUN_TEST(setConfig_updates_behavior);

  std::cout << "  all passed" << std::endl;
  return 0;
}
