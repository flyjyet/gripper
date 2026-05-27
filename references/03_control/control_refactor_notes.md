# Control Refactor Notes

## Current Baseline

The verified behavior is in:

- `verified_ui/gripper_control_ui.py`

The cleaner but less hardware-verified C++ skeleton is in:

- `source_snapshot/gripper_control/`

Use the Python UI as the integration reference and the C++ skeleton as a module
boundary reference. Do not assume the skeleton already matches the latest
bring-up fixes.

## Suggested Module Split

- `hardware/`
  - DeviceSDK adapter.
  - Channel configuration.
  - CAN/CANFD frame send and receive.
  - DLL loading diagnostics.
- `motor/`
  - Damiao motor protocol.
  - Mode switching.
  - Enable/disable command sequencing.
  - Feedback decoding.
- `mechanism/`
  - Stroke-to-angle kinematics.
  - Force conversion.
  - Stroke limit and contact geometry.
- `control/`
  - Homing state machine.
  - Clamp state machine.
  - Unload/release state machine.
  - Friction identification.
- `safety/`
  - Current limit.
  - Speed limit.
  - Stall detection.
  - Contact detection.
  - Mechanical-limit protection.
- `ui/`
  - Test controls.
  - Runtime status.
  - Pauseable/searchable/exportable logs.

## Safety Rules To Preserve

- Limit current and speed at the lowest possible command-generation layer.
- Homing must use low speed and low current.
- Clamp must approach with controlled speed to avoid impact.
- Active stall/contact detection should stop command output.
- Do not hold continuous torque after operation; disable after controlled stop
  or small unload as validated by testing.
- Self-check should identify friction from small-current motion and use it to
  adjust contact/stall thresholds.

## Calibration Work Still Required

- Calibrate motor current/torque to nut thrust.
- Calibrate nut thrust to single-side jaw force across stroke.
- Validate friction over temperature and repeated cycles.
- Determine reliable contact thresholds for:
  - 0-12 mm miscellaneous objects,
  - 14-28 mm cables,
  - hard mechanical limit.
- Validate release/unload distance so the screw nut does not bind.
