# Build And Test Notes

## Python UI Prototype

The useful verified prototype is:

- `references/03_control/verified_ui/gripper_control_ui.py`

The previous packaged executable, before the workspace was moved, was:

- `old/build/ui/dist/gripper_control_ui/gripper_control_ui.exe`

Runtime DLLs kept at root:

- `dlls/libdm_device.dll`
- `dlls/libgcc_s_seh-1.dll`
- `dlls/libstdc++-6.dll`
- `dlls/libusb-1.0.dll`
- `dlls/libwinpthread-1.dll`

Python dependencies from the previous prototype are archived as:

- `references/05_build_test/requirements.txt`

## C++ Prototype

The previous C++ prototype build files are archived as:

- `references/05_build_test/CMakeLists.txt`
- `references/05_build_test/gripper_control_demo.cpp`
- `references/05_build_test/gripper_control_unit_tests.cpp`
- `references/03_control/source_snapshot/`

The previous build output is preserved under:

- `old/build/`

## Environment Notes

- The existing `.venv/` is kept at the root.
- For a clean refactor, prefer recreating dependency installation steps instead
  of relying blindly on the old environment.
- The current archive deliberately keeps `dlls/` at root. In a new source layout,
  decide whether runtime libraries belong under `src/libs/`,
  `third_party/damiao/bin/`, or packaged beside the final executable.

## Hardware Bring-Up Notes

Known working configuration:

- Adapter: DM-USB2FDCAN_Dual.
- Device type: `1`.
- Mode: FDCAN.
- Arbitration bitrate: 1 Mbps.
- Data bitrate: 5 Mbps.
- Motor ID: `0x08`.
- Master ID: `0x18`.
- CANFD/BRS enabled for motor frames.
- Auto mode switch enabled.

If official Damiao software works but this program does not, check:

- DeviceSDK library loading and DLL path.
- Device count and open return codes.
- Correct channel index.
- FDCAN/BRS frame settings.
- `CTRL_MODE=4` write before enable/position-force commands.
- Whether TX/RX detail logging is actually emitted.
