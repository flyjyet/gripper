# Hardware Reference Notes

## Verified Device Path

The current working hardware path is based on:

- Motor: Damiao `DM-J4310P-2EC`.
- Adapter: `DM-USB2FDCAN_Dual`.
- Access layer: Damiao `DM_DeviceSDK`, not generic `python-can` `slcan` or
  `pcan` interfaces.
- Runtime DLLs: current refactor keeps them in
  `src/third_party/damiao/bin/`.

Known working configuration from bring-up:

- Device type: `1` for Dual adapter.
- Work mode: FDCAN.
- Arbitration bitrate: 1 Mbps.
- Data bitrate: 5 Mbps.
- Motor ID: `0x01`.
- Master/feedback ID: `0x11`.
- CANFD motor frames enabled.
- BRS enabled.
- Command ID plus mode enabled.
- Auto mode switch enabled.

## Files To Read

Motor:

- `damiao_motor/DM-J4310P-2EC减速电机说明书V1.0.pdf`

DeviceSDK:

- `protocol_refs/dmcan.h`
- `protocol_refs/DM_DeviceSDK_README.md`
- `protocol_refs/DM_DeviceSDK_Python_README.md`
- `protocol_refs/DM_DeviceSDK_python_demo.py`
- `protocol_refs/dmcan_sdk-1.0.4-py3-none-any.whl`

USB2FDCAN:

- `dm_usb2fdcan/DM-USB2CANFD_Dual 模块使用说明书 V1.0.pdf`
- `dm_usb2fdcan/USB2CANFD_Dual_README.md`
- `dm_usb2fdcan/USB2CANFD_SETUP.md`
- `dm_usb2fdcan/USB2CANFD_SDK_README.md`

Protocol/control examples:

- `protocol_refs/legacy_usb2canfd_damiao.py`
- `protocol_refs/legacy_usb2canfd_cpp_damiao.cpp`
- `protocol_refs/legacy_usb2canfd_cpp_damiao.h`
- `openarm_can_ref/src/openarm/damiao_motor/dm_motor_control.cpp`
- `openarm_can_ref/include/openarm/damiao_motor/dm_motor_constants.hpp`
- `openarm_can_ref/python/examples/test_gripper_posforce.py`

## Runtime DLL Manifest

The following DLLs are required for the current Windows prototype. In the
current C++ refactor they are stored under `src/third_party/damiao/bin/`:

- `libdm_device.dll`
- `libgcc_s_seh-1.dll`
- `libstdc++-6.dll`
- `libusb-1.0.dll`
- `libwinpthread-1.dll`

During refactor, do not hard-code one absolute DLL path. The current C++ code
loads `libdm_device.dll` from the configurable adapter field
`driver_library_path`; the default is
`src/third_party/damiao/bin/libdm_device.dll`.

Prefer one of:

- load from an application-local runtime directory,
- add an explicit configurable SDK/DLL path,
- package the DLLs beside the final executable.

## Implementation Notes For Refactor

- Keep DeviceSDK transport outside the UI.
- Expose adapter open/close, channel config, frame send, and frame receive
  through a small interface.
- Log every mode switch, enable, disable, and command frame at the transport
  boundary.
- Keep the first `CTRL_MODE=4` write explicit before position-force commands.
- Treat connection success and command success separately; a device can open
  while motor command frames are still malformed or routed to the wrong channel.
- Avoid any automatic position zeroing on first feedback. Zero only after a
  completed homing procedure.

## Current C++ Refactor Status

As of 2026-05-20, the C++ hardware path has a first real DeviceSDK injection:

- `DmUsb2FdcanTransport` dynamically loads `libdm_device.dll`.
- Runtime DLL has been checked against
  `references/04_hardware/DM_DeviceSDK/C&C++/lib/v1.1.0/windows/mingw/libdm_device.dll`;
  both hashes are `DB7FC43D0D60B847479DCB58CFBE13E524B0CA43C5DFA847D14E760BC0E2A8A7`.
- `bringup_can_probe` now logs SDK and device versions. Current observed values:
  `sdk_version_raw=0x1010000`, `device_version=dual_app v1.0.0.7`.
- It opens `DM-USB2FDCAN_Dual`, configures FDCAN bitrates, enables the selected
  channel, sends project `CanFrame` values, and receives frames through SDK
  callbacks.
- `DamiaoProtocol` packs enable, disable, clear fault, refresh, `CTRL_MODE`,
  velocity, position-velocity, position-force, and MIT torque frames.
- `DamiaoProtocol` parses DM4310-style motor feedback frames using configured
  position, velocity, and torque ranges.
- `DamiaoMotor` maps `MotorInterface` commands to Damiao frames and keeps SDK
  types hidden from controller/UI code.
- The console UI now exposes a low-energy motor bring-up path for real hardware
  step testing before homing, travel learning, or clamp workflows are used.

The current implementation has passed compile and unit tests, but it still
needs real hardware confirmation for:

- DLL and dependent DLL loading on the target PC. Verified on 2026-05-20 after
  adding `VCRUNTIME140.dll` and `VCRUNTIME140_1.dll` beside
  `libdm_device.dll`.
- Device enumeration and selected channel index. `connect` verified on
  2026-05-20 for both channel 0 and channel 1.
- `CTRL_MODE=4` write acceptance.
- Enable/disable frame ID mode offset.
- Feedback frame host ID and motor ID matching.
- Real motor motion direction and current/torque scaling.

## Real Hardware Step Test Procedure

Use this sequence for the first real DM-J4310P-2EC test. The first test should
be unloaded, or the gripper mechanism must be mechanically made safe so a wrong
direction cannot tighten the screw/nut or clamp the structure.

Recommended command:

```powershell
.\build\dev-zig\gripper_app.exe --damiao --config src\config\default_gripper.yaml
```

Console sequence:

1. `connect`
   - Opens `DM-USB2FDCAN_Dual` through `libdm_device.dll`.
   - Does not enable the motor.
2. `bringup_enter_confirm_unloaded`
   - Required confirmation gate for motor-side jog testing.
   - Does not home, zero, or set travel limits.
3. `bringup_can_probe`
   - Passive raw CAN/CAN-FD diagnosis ported from the verified Python UI.
   - Sends refresh and `MASTER_ID` register-read requests only.
   - Does not enable output, jog, home, zero, or learn limits.
   - Expected log shape when the bus responds:

```text
bringup_can_probe | TX refresh FD id=0x7FF ...
bringup_can_probe | RX FD id=0x11 ...
bringup_can_probe | TX query_master_id FD id=0x7FF ...
bringup_can_probe | RX FD id=...
```

   - If both requests report `RX none`, do not continue to enable or jog. Debug
     power, CANH/CANL, termination, channel index, FDCAN/BRS, motor ID,
     firmware protocol, or adapter ownership first.
4. `bringup_feedback`
   - Reads one fresh motor feedback frame.
   - Check `motor_pos_rad`, `motor_vel_rad_s`, `motor_current_a`,
     `motor_torque_nm`, and `temp_c`.
5. `bringup_enable`
   - Enables output for bring-up only.
   - If this fails, debug mode switch, enable frame ID, motor ID, host ID, and
     feedback parsing before trying motion.
6. `bringup_disable`
   - Confirms disable works and feedback/logs show output disabled.
7. `bringup_jog_pos`
   - Sends one short positive motor-side position pulse using config defaults.
   - The controller disables output after the pulse.
8. `bringup_jog_neg`
   - Sends one short negative motor-side position pulse using config defaults.
   - Compare feedback position change to establish actual motor/mechanism
     direction.
9. `bringup_feedback`
   - Confirm feedback is still live after jog/disable.
10. `bringup_exit`
   - Leaves bring-up mode and disables output if needed.
11. `clear_fault`
    - Sends a generic clear-fault request through the hardware abstraction.
    - Use only after recording the previous fault symptom.

Optional explicit jog parameters:

```text
bringup_jog_pos [relative_rad] [velocity_rad_s] [current_a] [duration_s]
bringup_jog_neg [relative_rad] [velocity_rad_s] [current_a] [duration_s]
```

The controller clamps these values against `motor_bringup` config. Current
bring-up defaults are intentionally below self-check current:

- `default_relative_motor_position_rad: 0.05`
- `max_relative_motor_position_rad: 0.2`
- `default_motor_velocity_rad_s: 0.5`
- `max_motor_velocity_rad_s: 1.0`
- `default_motor_current_a: 0.2`
- `max_motor_current_a: 0.4`
- `default_pulse_duration_s: 0.1`
- `max_pulse_duration_s: 0.3`

Bring-up mode must not be used as administrator recovery. It is only for low
energy communication, enable/disable, feedback, and direction checks. Higher
authority recovery remains a separate design and must keep bounded current,
speed, duration, and direction-confidence gates.

## 2026-05-20 Real Hardware Test Result

Detailed issue-by-issue log:

- `real_hardware_test_issue_log_2026-05-20.md`

Verified:

- `gripper_app.exe --damiao --config src\config\default_gripper.yaml` starts.
- `connect` succeeds with `DM-USB2FDCAN_Dual`.
- Runtime DLL dependency issue was fixed by adding:
  - `VCRUNTIME140.dll`
  - `VCRUNTIME140_1.dll`
- The C++ transport now preloads local SDK dependencies before loading
  `libdm_device.dll`.
- `disconnect` / `quit` no longer blocks the console. The current C++ prototype
  intentionally avoids SDK `device_close` / `device_disable_channel` because the
  Windows SDK can hang there after USB transfer cancellation on this PC.

Resolved during this test pass:

- Earlier `RX none` was caused by stale ID settings in the project.
- The official DMTool screenshot showed `CAN ID=0x01`, `Master ID=0x11`.
- After updating `src/config/default_gripper.yaml`, passive probe succeeded:

```text
bringup_can_probe | TX refresh FD id=0x7FF dlc=8 brs=1 data=01 00 cc 00 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 79 b2 7f f7 ea 21 1f
bringup_can_probe | TX query_master_id FD id=0x7FF dlc=8 brs=1 data=01 00 33 08 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 00 33 08 01 00 00 00
bringup_can_probe: Ok
```

- `bringup_feedback` also succeeded:

```text
stroke_mm=-0.195923 motor_pos_rad=-0.615511 motor_vel_rad_s=-0.00732601 motor_current_a=-0.019536 motor_torque_nm=-0.01221 temp_c=33.000 enabled=false
```

Before any enable or jog test, still check these on the bench:

- Gripper mechanism is unloaded or mechanically safe for direction testing.
- Motor ID is `0x01` and master/feedback ID is `0x11`.
- Adapter remains configured as FDCAN, arbitration 1 Mbps, data 5 Mbps, BRS
  enabled.
- No DMTool or other program is using the same adapter.

## 2026-05-20 Official DMTool Device Settings

User-provided DMTool screenshot:

- USB2CAN baudrate: `1000 Kbps`.
- Motor communication parameters: `CAN ID=0x01`, `Master ID=0x11`,
  `CAN Timeout=0`, `CAN Baud=5M`.
- USB2FDCAN device port: `device:1,3`.
- Work mode: `FDCAN`.
- Arbitration baudrate: custom `1M`.
- Data baudrate: custom `5M`.
- Sample point: `75.0`.

Current C++ `bringup_can_probe` reads the configured channel back from
DeviceSDK v1.1.0:

```text
channel_readback=ok rb_ch=0 rb_canfd=1 rb_nominal_bps=1000000 rb_data_bps=5000000 rb_can_sp=0.75 rb_canfd_sp=0.75 rb_fd_details=11/4/4/1
```

This confirms the current program is setting and reading back the same CAN/FDCAN
timing as the official tool screenshot. The earlier `RX none` issue was then
resolved by changing the project motor IDs from the stale `0x08/0x18` pair to
the currently configured `0x01/0x11` pair.
