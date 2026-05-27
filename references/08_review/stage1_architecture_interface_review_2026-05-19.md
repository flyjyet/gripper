# 2026-05-19 第一阶段架构与接口审查归档

## 1. 审查定位

当前阶段定义为 **第一阶段架构与接口审查**，不是最终需求验收。

本阶段只确认：

- 目录和模块边界是否合理。
- 控制器公开接口是否能表达后续真实需求。
- 状态机框架是否覆盖主要控制阶段。
- 硬件抽象是否隔离具体达妙 SDK、CAN 帧和 UI。
- 未实现项是否明确标记为占位、低置信或 `NotImplemented`。

本阶段不验收：

- 真实 `PreSelfCheck` 逐步试探。
- 真实摩擦学习。
- 真实行程限位发现。
- 非线性 `J(s)` 运行时模型。
- 达妙 C++ DeviceSDK/FDCAN 真实接入。
- 目标夹紧力标定。
- 夹紧后卸载预紧和打开前防抱死解锁。

## 2. 当前框架自检结论

已确认：

- `controller` 未直接依赖 UI、commander、`hardware_interface/damiao` 或 `hardware_interface/simulated`。
- app 组合层负责选择模拟电机或达妙占位电机。
- 达妙 C++ 硬件路径仍明确返回 `NotImplemented`，没有伪装为可真实连接或可真实使能。
- `ClampSpeedMode` 已能区分螺母匀速和夹爪角速度模式，但运行时非线性模型仍是一阶占位。
- `runPreSelfCheck()` 当前仍是 first-pass proxy，只能生成 `LowConfidence` 结构参数。
- 已确认真实硬件模式下 `LowConfidence` 后续应禁止夹紧。

## 3. 是否需要新增测试

当前阶段 **不建议新增大规模测试**，原因：

- 现阶段主要验证架构、接口和占位边界，不是验证最终控制算法。
- 真实 PreSelfCheck、真实硬件接入、非线性运动学、夹紧防抱死等关键算法还没有完整实现。
- 如果现在围绕占位实现补大量测试，后续实现真实算法时测试会大面积重写，性价比低。

当前阶段只需要保留并通过：

- 构建测试：确认 `gripper_app` 可构建。
- 脚本化模拟冒烟测试：确认模拟流程仍可跑通。
- 达妙占位路径测试：确认真实硬件路径不会被误判为可用。

后续进入 `PreSelfCheck` 真实实现后，应新增针对该功能的单元测试和仿真测试。

## 4. 本次验证记录

执行日期：2026-05-19

命令：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:\jyt_ws_ai\TS2000_human_robot\gripper\build\zig-cache'
.venv\Scripts\cmake.exe --build build --target gripper_app
.venv\Scripts\ctest.exe --test-dir build --output-on-failure
```

结果：

- `gripper_app` 构建通过。
- `gripper_scripted_demo` 通过。
- `gripper_damiao_placeholder` 通过。

## 5. 阶段结论

结论：**有条件通过**。

通过条件：

- 当前代码只作为架构与接口骨架。
- 不作为真实硬件安全控制闭环交付。
- 不作为最终需求完成度验收。
- 下一阶段优先实现真实 `PreSelfCheck`。

## 6. 下一阶段测试建议

下一阶段主线已确认为 `PreSelfCheck`，应补充以下测试：

- 速度/电流阶梯试探测试：从保守小速度、小电流逐步提升，直到机构双向稳定运动。
- 最低稳定速度识别测试：确认开向和闭向分别记录最低稳定速度。
- 静摩擦识别测试：确认 breakaway current/torque 能被记录，且限位附近不会被误判为摩擦。
- 动摩擦识别测试：确认不同安全速度下记录平均值和最大值，并优先采用保守最大值。
- 限位排除测试：确认学习过程中能识别机械限位风险，不把限位堵转当作正常摩擦。
- 安全区建立测试：确认初步机械限位和理论行程差异不大时才能建立安全区。
- `LowConfidence` 阻断测试：真实硬件模式下结构参数未达到可信质量时禁止夹紧。

