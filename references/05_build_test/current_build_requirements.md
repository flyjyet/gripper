# 当前构建要求与注意事项

本文档记录当前 C++ 重构工程的推荐构建方式。旧原型、历史 CMake 和 Python UI 的构建信息仍保留在 `build_notes.md` 中；当前开发应优先以本文档为准。

## 当前结论

当前阶段推荐使用：

- `CMakePresets.json`
- `scripts/build.ps1`
- `scripts/test.ps1`
- 项目本地 `.venv` 中的 `cmake.exe`、`ctest.exe`、`ninja.exe`、`python-zig.exe`

日常开发优先执行：

```powershell
.\scripts\build.ps1
.\scripts\test.ps1
```

这比手动输入长命令更稳定，也避免混用系统全局 CMake、Ninja 或不同 C++ 编译器。

## 推荐命令

一键构建：

```powershell
.\scripts\build.ps1
```

一键构建并测试：

```powershell
.\scripts\test.ps1
```

如果需要直接调用 preset：

```powershell
.venv\Scripts\cmake.exe --preset dev-zig
.venv\Scripts\cmake.exe --build --preset dev-zig
.venv\Scripts\ctest.exe --preset dev-zig
```

## 构建目录

当前规范构建目录为：

```text
build/dev-zig
```

历史上手动构建使用过：

```text
build
```

注意事项：

- 不建议在同一个构建目录里混用不同工具链。
- `build/dev-zig` 是当前 preset 管理的目录。
- 旧的 `build` 目录可以作为历史构建产物保留，但不应作为当前开发的主构建目录。
- 如果未来增加 MSVC、Release、硬件 SDK 专用构建，应使用新的 preset 和新的二进制目录，例如 `build/msvc-debug`。

## 工具链要求

当前 `dev-zig` preset 依赖以下文件存在：

```text
.venv/Scripts/cmake.exe
.venv/Scripts/ctest.exe
.venv/Scripts/ninja.exe
.venv/Scripts/python-zig.exe
```

`CMakePresets.json` 已设置：

```text
CMAKE_MAKE_PROGRAM = .venv/Scripts/ninja.exe
CMAKE_CXX_COMPILER = .venv/Scripts/python-zig.exe;c++
ZIG_GLOBAL_CACHE_DIR = build/dev-zig/zig-cache
```

因此日常构建不需要手动设置 `ZIG_GLOBAL_CACHE_DIR`。

## 为什么不直接使用系统全局工具

可以使用全局 CMake 或 MSVC，但当前不作为默认方式。原因：

- 当前工程已经验证 `.venv + Ninja + python-zig` 可以稳定构建。
- 使用项目本地工具链可以降低不同电脑全局环境差异。
- CMake 构建目录会记录编译器、生成器和工具路径，混用工具链容易导致缓存不一致。
- 普通 PowerShell 即可构建，不依赖 Visual Studio Developer Command Prompt。

如果需要使用全局 MSVC 或其他工具链，应新建独立 preset，不要复用 `build/dev-zig`。

## 测试要求

当前测试由 CTest 管理。推荐统一通过：

```powershell
.\scripts\test.ps1
```

测试数量以当前 `CMakeLists.txt` 注册的测试为准。当前已注册的基础脚本测试包括：

```text
gripper_scripted_demo
gripper_damiao_placeholder
```

如果后续重新添加 `test/` 目录中的单元测试，应在 CMake 中使用 `add_test()` 注册，使其自动进入 `ctest --preset dev-zig`。

## PowerShell 执行策略

如果系统阻止执行脚本，可以临时使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\test.ps1
```

不建议为了项目构建长期放宽整机执行策略。

## 后续建议

后续进入真实硬件和达妙 SDK 深度接入阶段时，建议增加：

- `msvc-debug` preset：用于 Windows SDK、DLL、真实硬件联调。
- `msvc-release` preset：用于发布测试程序。
- 明确的 DLL 拷贝或运行时路径配置。
- 硬件测试与纯软件测试分离，避免无设备时阻塞普通回归测试。

当前阶段不建议切换默认工具链。先保持 `dev-zig` 稳定，用它覆盖架构、控制器、状态机、安全逻辑和模拟测试。
