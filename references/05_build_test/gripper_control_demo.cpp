#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "gripper_control/gripper_controller.hpp"
#include "gripper_control/simulated_motor.hpp"
#include "gripper_control/types.hpp"

using gripper_control::ClampCommand;
using gripper_control::GripperController;
using gripper_control::GripperState;
using gripper_control::OpenCommand;
using gripper_control::SimulatedMotor;
using gripper_control::SimulatedObject;
using gripper_control::toString;

namespace {

double secondsNow() {
  using clock = std::chrono::steady_clock;
  static const auto start = clock::now();
  return std::chrono::duration<double>(clock::now() - start).count();
}

void runFor(GripperController& controller,
            SimulatedMotor& motor,
            double duration_s,
            const char* label) {
  const double end = secondsNow() + duration_s;
  double last_print = 0.0;
  while (secondsNow() < end) {
    const double now = secondsNow();
    motor.update(now);
    controller.update(now);
    const auto status = controller.getStatus();
    if (now - last_print > 0.25) {
      last_print = now;
      std::cout << label << " state=" << toString(status.state)
                << " stroke=" << status.stroke_mm
                << " vel=" << status.velocity_mm_s
                << " current=" << status.current_A
                << " fault=" << toString(status.fault) << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace

int main() {
  auto motor = std::make_shared<SimulatedMotor>();
  // Put the simulated object close to zero so the demo exercises contact,
  // force build, unload-before-disable, and reopen in a short run.
  motor->setObject(SimulatedObject{0.5, 5.0});

  GripperController controller(motor);
  controller.initialize(secondsNow());
  runFor(controller, *motor, 3.5, "init");

  ClampCommand clamp;
  clamp.target_force_per_side_N = 150.0;
  clamp.close_speed_mm_s = 0.3;
  clamp.max_current_A = 1.5;
  clamp.timeout_s = 8.0;
  clamp.known_cable_mode = false;
  controller.clampForce(clamp, secondsNow());
  runFor(controller, *motor, 5.0, "clamp");

  OpenCommand open;
  open.open_speed_mm_s = 0.8;
  open.max_current_A = 1.0;
  open.timeout_s = 8.0;
  controller.open(open, secondsNow());
  runFor(controller, *motor, 4.0, "open");

  const auto status = controller.getStatus();
  std::cout << "final state=" << toString(status.state)
            << " fault=" << toString(status.fault) << '\n';
  return status.fault == gripper_control::GripperFault::None ? 0 : 1;
}
