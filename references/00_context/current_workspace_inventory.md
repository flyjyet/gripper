# Current Workspace Inventory Before Move

Root before archiving contained:

- `.venv/`: Python virtual environment.
- `assets/`: vendor SDKs, Damiao tools, USB2FDCAN documents, openarm CAN package.
- `build/`: generated build/package outputs, including the UI executable.
- `build_tools/`: helper compiler wrapper scripts.
- `demo/`: C++ demo program.
- `dlls/`: runtime DLLs copied beside the prototype UI.
- `old/`: initially empty archive destination.
- `references/`: curated refactor reference destination.
- `src/`: current gripper-control source tree.
- `test/`: C++ unit tests.
- `需求讨论/`: original requirement discussion, diagrams, mechanical documents,
  motor manual, scripts, and generated curves.
- `CMakeLists.txt`: current C++ prototype build configuration.
- `requirements.txt`: current Python UI/package dependencies.

## Curated Files Copied Into references

Requirement and discussion documents:

- `夹爪PC测试UI说明.md`
- `夹爪控制系统设计.md`
- `夹爪控制系统联调归档.md`
- `夹爪控制系统重构准备清单.md`
- `夹爪机构计算校核.md`
- `夹爪样机测试验证方案.md`
- `夹爪结构研究归档.md`

Mechanical assets:

- Original and redrawn mechanism diagrams.
- Mechanical calculation script `gripper_check.py`.
- Motion-curve outputs: CSV, MATLAB script, PNG, SVG.
- Motion-curve Python generation scripts.

Control/source assets:

- Verified Python UI/control prototype.
- Full current `src/gripper_control/` snapshot.
- Current C++ demo and unit test files.
- Current `CMakeLists.txt` and `requirements.txt`.

Hardware references:

- DM-J4310P-2EC motor manual from `需求讨论/`.
- Damiao DeviceSDK README files.
- DeviceSDK `dmcan.h`.
- DeviceSDK Python demo.
- DeviceSDK Python wheel.
- USB2CANFD and USB2CANFD_Dual manuals/README/setup files.
- OpenArm CAN Damiao motor and CAN reference source files.
- Current runtime DLL list is documented; actual DLLs remain in root `dlls/`.

## Full Workspace Copy

Everything not kept at the root is moved, not deleted, into `old/`:

- `assets/`
- `build/`
- `build_tools/`
- `demo/`
- `src/`
- `test/`
- `需求讨论/`
- `CMakeLists.txt`
- `requirements.txt`

This means the root is no longer directly buildable as the old prototype.
Reconstruction or refactor should start from `references/`, with full fallback
available from `old/`.
