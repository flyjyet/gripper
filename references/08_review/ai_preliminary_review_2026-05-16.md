# 2026-05-16 AI 初步设计与代码审查记录

## 1. 审查范围

本次审查覆盖第一轮 `src` 重构落盘后的关键控制路径：

- `src/controller/gripper_controller.cpp`
- `src/controller/state_machine/gripper_state_machine.hpp`
- `src/controller/safety/safety_limiter.hpp`
- `src/controller/safety/contact_jam_detector.hpp`
- `src/hardware_interface/damiao/*`
- 架构依赖边界：`controller` 是否直接依赖 UI、commander、达妙具体实现或仿真电机。

本次审查目标不是确认真实硬件可用，而是确认当前骨架是否能支撑后续人工设计审查和真实硬件联调。

## 2. 已确认事项

- `controller` 未直接引用 `hardware_interface/damiao`、`hardware_interface/simulated`、UI 或 commander，具体电机仍由 app 组合层注入。
- 达妙硬件路径仍明确保持 `NotImplemented` 占位，没有伪装成可真实连接或可真实使能。
- P0 级源码注释已经补入关键控制器、安全、状态机和达妙占位接口。
- 构建和现有 CTest 均通过。

验证命令：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:\jyt_ws_ai\TS2000_human_robot\gripper\build\zig-cache'
.venv\Scripts\cmake.exe --build build --target gripper_app
.venv\Scripts\ctest.exe --test-dir build --output-on-failure
```

验证结果：

- `gripper_scripted_demo` 通过。
- `gripper_damiao_placeholder` 通过。

## 3. 审查发现与处理

### 3.1 PreSelfCheck 首版采样质量容易被误判为已验证

发现：

当前 `runPreSelfCheck()`、`learnTravelLimits()`、`runMotionHealthCheck()` 主要使用单次命令后的电机反馈投影样本，属于第一版流程打通手段，不是完整的多速度、多电流、多区域真实结构参数学习。

风险：

如果质量等级被标为 `Verified`，后续控制逻辑或人工审查可能误以为已经完成真实机构识别。

处理：

- 增加 `capFirstPassProjectionQuality()`，将首版反馈投影样本产生的 `Verified` 降级为 `LowConfidence`。
- 对理论限位占位样本补充注释，明确真实硬件必须用低电流接近、确认和回退替换。
- 对噪声、静摩擦、动摩擦、运动健康样本的首版代理性质补充注释。

剩余人工确认：

- 真实硬件接入后，是否必须禁止 `LowConfidence` 结构参数进入夹紧流程。
- 是否需要增加配置项，区分“模拟流程验证模式”和“真实硬件严格安全模式”。

### 3.2 接触检测接口存在误导性参数

发现：

`evaluateContact()` 传入了 `motor_current` 参数，但函数内部实际使用 `last_feedback_.current`，该参数未使用。

风险：

人工审查时容易误认为接触检测使用的是命令电流，而不是反馈电流。

处理：

- 移除未使用的 `motor_current` 参数。
- 保持接触/堵转检测使用反馈电流和反馈速度。

### 3.3 运动识别样本的布尔标记过于乐观

发现：

`probeMotion()` 早期版本将 `direction_matches`、`position_monotonic`、`velocity_stable`、`current_stable` 直接置为 `true`。

风险：

这会让单帧反馈样本比实际更容易通过识别器校验。

处理：

- 改为根据实测行程方向、实测距离、反馈速度和反馈电流生成这些布尔标记。
- 仍保留“首版代理采样”的注释，避免把它当成完整识别流程。

## 4. 当前残余风险

- 真实 DM-USB2FDCAN_Dual 与 DM-J4310P-2EC SDK 尚未接入，不能进行真实使能、反馈解析、FDCAN 发送接收验证。
- 目标力到电机电流/力矩映射仍需样机标定，当前只是控制架构占位。
- 接触/堵转检测没有外部力传感器或夹爪侧位移传感器，只能依赖电机电流、力矩、速度和位置反馈代理判断，必须通过样机测试确定阈值。
- 当前模拟路径会进入 `Ready`，用于验证软件流程；这不代表真实硬件已经完成充分学习。
- 动作完成后失能可以降低丝杆螺母抱死风险，但是否影响实际夹持保持力，需要结合梯形丝杆自锁、线缆弹性和振动工况测试确认。

## 5. 建议人工重点审查项

1. 真实硬件严格模式下，`StructureProfile.quality != Verified` 时是否禁止夹紧。
2. `ActiveStop` 与 `Fault` 的边界是否符合现场恢复策略。
3. 限流、限速、限加速度触发时，是裁剪命令还是主动停止，是否符合保护结构的目标。
4. 目标力夹紧后立即失能是否满足夹持可靠性。
5. 达妙 SDK 接入后，通信失败是否一定不会继续输出上一条运动命令。
6. PreSelfCheck 的真实采样流程是否覆盖不同结构行程区域，并且参数取值是否采用最大保守值。

## 6. 结论

当前代码可以作为继续重构和人工设计审查的基础，但不能作为真实硬件安全控制闭环交付。

建议结论：有条件通过。

条件：

- 保持达妙真实硬件路径 `NotImplemented`，直到 SDK 接入和台架验证完成。
- 真实硬件联调前，必须补足严格模式、真实 PreSelfCheck 采样、真实行程学习、接触/堵转阈值标定和目标力标定。
