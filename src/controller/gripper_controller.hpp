#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common/result.hpp"
#include "controller/gripper_types.hpp"
#include "hardware_interface/motor_interface.hpp"

namespace gripper::config {
struct GripperConfig;
}

namespace gripper::controller {

using ProgressCallback = std::function<void(std::string)>;

class GripperController {
 public:
  GripperController() = default;
  virtual ~GripperController() = default;

  GripperController(const GripperController&) = delete;
  GripperController& operator=(const GripperController&) = delete;
  GripperController(GripperController&&) = delete;
  GripperController& operator=(GripperController&&) = delete;

  // Applies static configuration for hardware, mechanism, self-check, safety,
  // homing, clamp, and UI defaults.
  //
  // Preconditions:
  // - The controller must not be running an active motion command.
  //
  // Success:
  // - The controller becomes configured but does not connect or enable hardware.
  virtual common::Result configure(const config::GripperConfig& config) = 0;

  // Opens the configured hardware connection.
  //
  // Preconditions:
  // - configure() must have succeeded.
  //
  // Failure:
  // - Returns an error without exposing adapter-specific SDK details.
  virtual common::Result connect() = 0;

  // Closes the hardware connection after stopping active command output.
  virtual common::Result disconnect() = 0;

  // Enables the motor through the hardware abstraction.
  //
  // Preconditions:
  // - The controller must be configured and connected.
  // - The concrete motor implementation must switch to the required mode before
  //   sending the final enable command.
  virtual common::Result enable() = 0;

  // Disables the motor and prevents continuous holding torque.
  //
  // Behavior:
  // - Used after normal work completion and for safe shutdown.
  virtual common::Result disable() = 0;

  // Enters low-energy motor bring-up mode for real-hardware communication and
  // direction checks before using gripper workflows.
  //
  // Preconditions:
  // - The controller must be configured and connected.
  // - The caller must confirm that the motor is unloaded or the structure is
  //   mechanically safe for short low-current pulses.
  //
  // Behavior:
  // - Does not set zero, home, learn travel limits, or mark the controller
  //   ready for clamping.
  virtual common::Result enterMotorBringupMode(
      const MotorBringupSessionRequest& request) = 0;

  // Leaves motor bring-up mode and disables output if it is still enabled.
  virtual common::Result exitMotorBringupMode() = 0;

  // Reads a fresh motor feedback frame while in bring-up mode.
  virtual common::Result refreshMotorBringupFeedback() = 0;

  // Runs a passive raw communication probe while in bring-up mode.
  //
  // Behavior:
  // - Does not enable output, jog, home, zero, or update travel limits.
  // - Sends low-energy protocol requests and returns raw TX/RX diagnostic lines
  //   from the hardware implementation when supported.
  virtual common::Result runMotorBringupCommunicationProbe(
      std::vector<std::string>* lines) = 0;

  // Enables output only for bring-up checks. Normal gripper workflows should
  // continue to use enable().
  virtual common::Result enableMotorBringupOutput() = 0;

  // Disables output only for bring-up checks.
  //
  // Behavior:
  // - Keeps the bring-up session active so CAN probe, feedback, or another
  //   bounded jog can continue.
  // - Does not dispatch the normal workflow DisableRequested event.
  virtual common::Result disableMotorBringupOutput() = 0;

  // Sends one short, bounded motor-side jog and disables output afterward.
  //
  // Behavior:
  // - Clamps velocity/current/duration against bring-up config.
  // - Uses motor-side position, velocity, and current-limited command output.
  // - Does not update homing, travel-limit, or structure-profile validity.
  virtual common::Result jogMotorBringup(
      const MotorBringupJogCommand& command) = 0;

  // Moves a precise relative motor-side revolution count in bring-up mode.
  //
  // Behavior:
  // - Uses motor-side position mode with a velocity envelope.
  // - Monitors feedback current because Damiao position-velocity command does
  //   not carry a command-side current limit.
  // - Disables output after success, failure, timeout, or current protection.
  // - Does not set zero, update structure profile, or mark self-check results.
  virtual common::Result moveMotorBringupRelativeTurns(
      const MotorBringupPositionMoveCommand& command) = 0;

  // Runs the pre-self-check sequence before homing.
  //
  // Preconditions:
  // - The controller must be configured and connected.
  // - Homing is not required.
  //
  // Behavior:
  // - Probes both directions with limited stroke/current/speed.
  // - Finds conservative motion parameters.
  // - Builds a preliminary safe zone when possible.
  //
  // Failure:
  // - May transition to ActiveStop or Fault depending on severity.
  virtual common::Result runPreSelfCheck() = 0;

  // Performs low-current, low-speed homing toward the open mechanical stop.
  //
  // Preconditions:
  // - PreSelfCheck should have completed or conservative defaults must be used.
  //
  // Success:
  // - Establishes the open zero reference.
  virtual common::Result homeOpenStop() = 0;

  // Learns travel limits after homing and creates software limits.
  //
  // Preconditions:
  // - Homing must have succeeded.
  //
  // Behavior:
  // - Moves toward the close direction with conservative limits.
  // - Detects the close-side limit.
  // - Backs off and stores software limits inside mechanical limits.
  virtual common::Result learnTravelLimits() = 0;

  // Verifies bidirectional motion inside learned software limits.
  //
  // Preconditions:
  // - Travel limits must have been learned.
  virtual common::Result runMotionHealthCheck() = 0;

  // Clamps with a target single-gripper force in N.
  //
  // Preconditions:
  // - The controller must be ready.
  // - Travel limits should be learned.
  // - Motion health check should have passed for normal operation.
  //
  // Speed behavior:
  // - ClampSpeedMode::GripperAngularSpeed maps target gripper angular speed to
  //   nut speed through the mechanism model.
  // - ClampSpeedMode::NutLinearSpeed keeps the nut speed constant and is mainly
  //   for compatibility and controlled debugging.
  //
  // Failure:
  // - Contact, jam, speed, current, and stroke violations may trigger
  //   ActiveStop or Fault.
  virtual common::Result clampByForce(const ClampForceCommand& command) = 0;

  // Clamps at a target nut speed in mm/s until contact, stop, timeout, or fault.
  // This is a nut-linear-speed interface, not a constant gripper-angular-speed
  // interface.
  //
  // Preconditions:
  // - The controller must be ready.
  // - Travel limits should be learned.
  virtual common::Result clampBySpeed(const ClampSpeedCommand& command) = 0;

  // Releases the gripper with default release parameters.
  virtual common::Result release() = 0;

  // Releases the gripper with explicit release parameters.
  virtual common::Result release(const ReleaseCommand& command) = 0;

  // Moves to an absolute nut stroke target through the normal safety path.
  //
  // Preconditions:
  // - PreSelfCheck must have completed.
  // - Before MotionHealthCheck, the target must be inside the low-confidence
  //   window exposed in GripperStateSnapshot::manual_nut_stroke_range.
  // - After MotionHealthCheck, the target must be inside learned software
  //   limits.
  // - Motor bring-up and terminal states reject this command.
  //
  // Behavior:
  // - Uses controller-side current/speed/stroke safety limiting.
  // - Does not clamp or release by force; it is a bounded positioning aid for
  //   UI/manual validation.
  // - Disables output after success, failure, or timeout to avoid lead-screw
  //   lockup.
  virtual common::Result moveToNutStroke(
      const MoveNutStrokeCommand& command) = 0;

  // Requests active stop of current motion.
  //
  // Behavior:
  // - Stops command generation through the safety path.
  // - The resulting state can be recoverable ActiveStop or Fault.
  virtual common::Result stop() = 0;

  // Clears a recoverable fault after the caller has handled the root cause.
  //
  // Failure:
  // - Non-recoverable faults must require reinitialization or manual check.
  virtual common::Result clearFault() = 0;

  // Runs one control update tick.
  //
  // Behavior:
  // - Reads latest feedback through the hardware abstraction.
  // - Advances state machines.
  // - Applies safety limiting before motor command output.
  virtual common::Result update() = 0;

  // Installs an optional progress sink for long-running workflows.
  //
  // Behavior:
  // - The callback receives human-readable diagnostic lines.
  // - The callback must not call back into GripperController.
  // - Passing an empty callback disables progress reporting.
  virtual void setProgressCallback(ProgressCallback callback) = 0;

  // Returns the latest controller state snapshot.
  [[nodiscard]] virtual GripperStateSnapshot state() const = 0;
};

// Creates the default controller implementation. If motor is null, the
// returned controller reports ControlNotReady when hardware access is required.
[[nodiscard]] std::unique_ptr<GripperController> createGripperController(
    std::unique_ptr<hardware_interface::MotorInterface> motor);

}  // namespace gripper::controller
