# 2026-05-16 夹紧速度模式接口更新记录

## 1. 修改目的

此前 `clampByForce()` 使用 `target_nut_speed` 或命令中的 `max_nut_speed`，实际含义是螺母匀速。由于夹爪角速度与螺母速度之间存在非线性关系，螺母匀速会导致闭合末端夹爪角速度升高，增加接触冲击风险。

本次修改让目标力夹紧接口显式区分速度定义：

- 螺母线速度恒定。
- 夹爪角速度恒定。

## 2. 接口变化

新增：

- `ClampSpeedMode::NutLinearSpeed`
- `ClampSpeedMode::GripperAngularSpeed`

`ClampForceCommand` 新增字段：

- `speed_mode`
- `target_gripper_angular_speed`

`max_nut_speed` 的语义调整为：

- 在 `NutLinearSpeed` 模式下，它是目标螺母速度。
- 在 `GripperAngularSpeed` 模式下，它是螺母速度上限。

## 3. 默认行为

`ClampForceCommand` 默认使用：

```text
ClampSpeedMode::GripperAngularSpeed
```

默认目标夹爪角速度来自：

```text
config.clamp.target_gripper_angular_speed
```

默认螺母速度上限仍来自：

```text
config.clamp.target_nut_speed
```

当前默认配置：

```yaml
target_nut_speed_mm_s: 1.0
target_gripper_angular_speed_rad_s: 0.03
```

## 4. 当前实现边界

控制器现在通过：

```text
v_nut = omega_target / J(s)
```

把目标夹爪角速度转换为螺母速度。

其中：

```text
J(s) = dphi/ds
```

但 `src/controller/mechanism/gripper_kinematics.*` 当前仍使用一阶常数导数作为运行时占位。完整非线性模型和曲线数据已经归档在：

```text
references/02_mechanics/
```

后续应将闭链几何模型或查表插值模型正式迁入 `src/controller/mechanism`，让 `J(s)` 随螺母行程变化。

## 5. UI 兼容性

当前控制台 UI 的 `clampForce(target_force, speed)` 仍按旧语义解释 `speed`，即螺母速度，并显式设置：

```text
ClampSpeedMode::NutLinearSpeed
```

这样可以避免 UI 参数含义突然变化。后续图形 UI 应增加速度模式选择，并分别显示：

- 目标螺母速度 mm/s。
- 目标夹爪角速度 rad/s 或 deg/s。

## 6. 验证

已通过：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:\jyt_ws_ai\TS2000_human_robot\gripper\build\zig-cache'
.venv\Scripts\cmake.exe --build build --target gripper_app
.venv\Scripts\ctest.exe --test-dir build --output-on-failure
```

结果：

- `gripper_scripted_demo` 通过。
- `gripper_damiao_placeholder` 通过。
