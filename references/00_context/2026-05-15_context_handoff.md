# 2026-05-15 Gripper Context Handoff

## Project Goal

The project is a self-developed robot-arm gripper driven by a DC servo motor and
a trapezoidal screw. The current phase is preparing for a control-system
refactor after mechanism research, initial control design, hardware bring-up,
and a PC UI prototype.

This archive freezes the useful context before the old prototype workspace is
moved into `old/`.

## Mechanism Summary

- Actuator: DC servo motor plus trapezoidal screw.
- Screw lead: 2 mm.
- Nut usable travel from fully open to mechanical limit: about 16 mm.
- Earlier nominal total stroke: 18 mm, later corrected to actual usable stroke
  around 16 mm from fully open to mechanical limit.
- Fully open state: the drawing position shown by the user.
- Closing direction: nut moves downward; the curved gripper closes and clamps.
- Joint 2: pin joint between nut and link 2. It translates with the nut and
  swings due to linkage constraints.
- All joints use pin shafts.
- Mechanical limit constrains nut travel.
- Cable support: fixed to the base, supports the cable from below and works
  with the outer gripper jaw to clamp.
- Cable support geometry: R15 concave-up circular arc, center at `(0, 62)`.
- Gripper working inner arc: R60, contacting the outside of the cable.
- R60 center is a point on the left gripper rigid body and moves with it.
- R60 center at fully open state: approximately `(18.34, 59.78)`.
- Joint coordinates from clarified drawing:
  - Joint 1 x coordinate: `-12`.
  - Joint 2 y-related dimension: `28.5`.
  - Joint 3 y-related dimension: `27.83`.
- Cable range: nominal cable diameter 14-28 mm.
- Additional use case: clamping other objects in the 0-12 mm range, so full
  stroke protection is required, not only cable-contact protection.
- User-corrected contact strokes:
  - 14 mm cable contact stroke: about 15.66 mm.
  - 28 mm cable contact stroke: about 12.98 mm.

## Mechanical Analysis Already Completed

The previous analysis covered:

- Kinematic model between nut stroke and gripper angle.
- Nonlinear angle/stroke, angular velocity/nut velocity, and angular
  acceleration/nut acceleration relationships.
- Curves and MATLAB/Python plotting scripts.
- Single-side target clamping force: 200 N.
- Nut thrust calculation based on target clamping force.
- Pin shaft shear check.
- Pin/bore bearing pressure check.
- Wear and surface pressure concerns.
- Screw efficiency and self-locking discussion.
- Dynamic impact and peak-load validation concept.
- Prototype test-validation plan.

Key conclusion: the gripper motion to nut motion relationship is nonlinear.
As closing stroke increases, the slope grows rapidly, especially in the
acceleration mapping. Poor speed/current/force control can cause impact at
contact and near mechanical limits.

## Motor And Adapter

- Motor: Damiao `DM-J4310P-2EC`.
- Motor output rated torque: 3.5 Nm.
- Motor output peak torque: 12.5 Nm.
- Motor feedback available from motor side:
  - absolute encoder/dual encoder information,
  - current,
  - torque,
  - velocity,
  - position,
  - temperature,
  - enable/fault state.
- No extra gripper sensors are currently installed.
- Adapter currently used: `DM-USB2FDCAN_Dual`.

## Control Requirements

Required control behavior:

- Hard fallback limiting of current and speed to avoid structural damage and
  screw-nut lock-up.
- Low-speed, low-current stall homing to establish an open reference.
- Target gripper clamping force.
- Target constant clamping speed.
- Impact prevention at contact and near mechanical limits.
- Active stall/contact detection and active stop.
- After work completion, disable motor instead of continuously holding torque,
  to avoid screw/nut binding and inability to release.
- Initialization self-check:
  - start from small torque/current,
  - identify internal friction torque,
  - use measured friction to configure later current/force thresholds.
- Software architecture should clearly separate modules.
- Hardware should be abstracted behind callbacks/interfaces/subclasses.

Additional correction:

- Even though cable diameter is 14-28 mm, the gripper may clamp other objects in
  the 0-12 mm region. Protection must cover the whole travel.
- Current system failure mode: poor limit speed/current behavior often causes
  screw/nut jamming and inability to move.

## Verified Hardware Configuration

The UI prototype eventually connected to the device and reached basic motor
feedback/control bring-up using:

- Adapter: `DM-USB2FDCAN_Dual`.
- Device type: `1` for Dual.
- CAN channel: `0` or `1`, depending on physical connection.
- Motor ID: `0x08`.
- Master/feedback ID: `0x18`.
- Working mode: FDCAN.
- Arbitration bitrate: 1 Mbps.
- Data bitrate: 5 Mbps.
- Adapter FDCAN enabled.
- BRS enabled.
- Motor frames sent as CANFD.
- Command ID plus mode enabled.
- Auto mode switch enabled.
- Official upper computer software can open/control the adapter, so if the
  prototype fails while the official software succeeds, focus on the access
  layer, DeviceSDK calls, frame format, or mode-switch command sequence.

Earlier serial/CAN assumptions that were not sufficient:

- Generic `python-can` backends such as `slcan` and `pcan` failed because the
  corresponding modules/interfaces were not installed or were not the actual
  adapter access path.
- The relevant path is the Damiao DeviceSDK / USB2FDCAN_Dual access layer, not
  a generic serial-CAN path.

## Verified UI Prototype State

Current verified UI source:

- `src/gripper_control/ui/gripper_control_ui.py`
- Archived copy:
  `references/03_control/verified_ui/gripper_control_ui.py`

Current packaged executable path before archiving:

- `build/ui/dist/gripper_control_ui/gripper_control_ui.exe`

Important fixes already made in the UI prototype:

- First command forces a write of `CTRL_MODE=4`.
- Initial `current_mode` is `None`; the code no longer assumes POS_FORCE mode.
- Enable command switches to POS_FORCE before sending the special enable frame.
- No automatic zeroing on first feedback; zero is only assigned after homing.
- Before homing, velocity command uses a small relative target around current
  position instead of commanding the current point or a full-stroke endpoint.
- Default USB2FDCAN motor frames use CANFD/BRS.
- Default DM device type is set to `1`.
- UI log has auto-scroll control and detail logging so a specific log line can
  be inspected.
- UI button bindings call the current controller/backend instance.

Observed behavior during bring-up:

- Connection success alone did not mean commands were reaching the motor.
- A key diagnostic symptom was that clicking `Enable` produced no TX/RX detail
  logs beyond routine status prints.
- After fixes, basic functions were reported as running through.

## Current Software Shape

The prototype contains two layers of work:

- A C++ skeleton under `src/gripper_control/`:
  - controller,
  - safety limiter,
  - contact/jam detector,
  - force command mapper,
  - friction identifier,
  - simulated motor,
  - Damiao protocol/motor placeholders,
  - tests and demo.
- A Python UI under `src/gripper_control/ui/`:
  - currently the most useful verified hardware reference,
  - monolithic: UI, DeviceSDK access, protocol, backend, controller, and logging
    are in one file,
  - should be split during refactor.

## Refactor Priorities

1. Treat the verified Python UI behavior as the integration reference.
2. Split hardware access from control logic:
   - transport/DeviceSDK adapter,
   - CAN frame/protocol encoding,
   - motor state model,
   - gripper kinematics and force mapping,
   - safety limiter,
   - homing state machine,
   - clamp state machine,
   - UI.
3. Make speed/current limiting the bottom-most safety layer.
4. Make command generation stateful and explicit:
   - disconnected,
   - connected,
   - mode switching,
   - enabled,
   - homing,
   - ready,
   - clamping,
   - unloading,
   - disabled/fault.
5. Prevent screw/nut binding:
   - do not hold continuous torque after clamp completion,
   - unload slightly or reduce current before disable if needed,
   - validate release behavior in repeated tests.
6. Build friction identification into initialization and use it to adapt current
   thresholds.
7. Replace placeholder force mapping with calibrated mapping from motor torque,
   screw thrust, mechanism geometry, and measured clamp force.
8. Keep logs pauseable/searchable/exportable in the new UI.

## Known Remaining Risks

- Target-force mapping has not been physically calibrated.
- Friction and contact thresholds need repeated sample testing.
- Screw/nut lock-up prevention needs explicit endurance testing.
- The C++ skeleton does not yet fully match the verified Python UI behavior.
- Concrete USB2FDCAN DeviceSDK transport should be implemented cleanly, not kept
  as UI-local code.
- Mechanical impact peak load still needs prototype instrumentation or
  controlled current/velocity sweep testing.

## Archive Decision

The old workspace is preserved under `old/`. The curated reference set in
`references/` is the preferred starting point for the next refactor.
