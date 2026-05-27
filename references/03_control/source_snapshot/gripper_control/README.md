# Gripper Control Source Layout

This folder contains the first C++ implementation skeleton for the gripper
control system.

## Modules

- `MotorInterface`: hardware abstraction for servo drivers.
- `SafetyLimiter`: centralized stroke, speed, current, and temperature limits.
- `ContactJamDetector`: full-stroke contact and jam detection.
- `FrictionIdentifier`: low-current friction baseline identification.
- `ForceCommandMapper`: target force to current/torque mapping placeholder.
- `GripperController`: state machine and high-level command handling.
- `SimulatedMotor`: PC simulation backend for early testing.
- `damiao/CanBus`: raw CAN send/receive abstraction.
- `damiao/DmJ4310Protocol`: DM-J4310P-2EC CAN frame packing and feedback parsing.
- `damiao/DmJ4310Motor`: `MotorInterface` implementation for the DM-J4310P-2EC.

## Current Status

The control state machine and DM-J4310P-2EC protocol are implemented. Real
hardware still needs a concrete `CanBus` subclass for the selected USB-CAN
adapter or vendor SDK. The PC UI includes a `python-can` backend for adapters
supported by python-can.

As of 2026-05-15, the PC UI hardware path has been brought up with a
DM-USB2FDCAN_Dual adapter through Damiao `DM_DeviceSDK v1.1`. The verified
hardware implementation currently lives in
`src/gripper_control/ui/gripper_control_ui.py`; the C++ production-oriented
modules should be updated from that verified behavior during the next refactor.
See the control-system bring-up archive and refactor checklist Markdown files
under the requirements-discussion folder.

DM-J4310P defaults used by this project:

- CAN standard frame, 1 Mbps.
- The PC UI defaults to the gripper IDs used by `assets/openarm_can` examples:
  motor send ID `0x08`, feedback receive ID `0x18`. Override these in the UI
  if the actual motor was configured differently.
- Screw lead is `2 mm`, so `1 mm` nut stroke equals `pi rad` output shaft
  rotation.
- Velocity guarding uses PVT mode with current limit.
- Force/current build uses MIT torque command and unloads before disable.
- `setStrokeZero()` is a software gripper zero; it does not send the motor's
  persistent "save zero" frame.

## Build

The local development environment can use the Python virtual environment tools:

```powershell
.\.venv\Scripts\python -m pip install cmake ninja ziglang
```

Zig needs a writable cache directory on this machine:

```powershell
$env:ZIG_GLOBAL_CACHE_DIR=(Resolve-Path ".").Path + "\build\zig-cache-global"
```

From the repository root:

```powershell
.\.venv\Scripts\cmake -S . -B build -G Ninja `
  -DCMAKE_MAKE_PROGRAM=".\.venv\Scripts\ninja.exe" `
  -DCMAKE_CXX_COMPILER=".\.venv\Lib\site-packages\ziglang\zig.exe" `
  -DCMAKE_CXX_COMPILER_ARG1="c++" `
  -DCMAKE_BUILD_TYPE=Debug

.\.venv\Scripts\cmake --build build
.\build\gripper_control_unit_tests.exe
.\build\gripper_control_demo.exe
```

The current verified setup uses CMake 4.3.2, Ninja 1.13.0, and Zig/Clang 21.1.0
installed inside `.venv`.
