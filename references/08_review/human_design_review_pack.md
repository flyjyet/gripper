# 人工设计审查包

## 1. 审查目的

本审查包用于帮助人工审查当前夹爪控制系统重构状态，重点不是逐行检查代码格式，而是确认控制架构、安全策略、状态机、硬件边界和后续真实联调风险是否清晰。

当前阶段仍属于第一轮重构落盘：

- 模拟路径已可构建和运行。
- 达妙真实硬件路径已完成第一阶段 SDK 动态加载和协议封装，但尚未完成真实电机联调验收。
- 控制器已经避免直接依赖具体硬件实现。
- 后续仍需要在 DM-USB2FDCAN_Dual 和 DM-J4310P-2EC 上确认命令、反馈和方向。

## 2. 建议审查角色

建议至少包含：

- 控制软件负责人。
- 机械结构负责人。
- 电机/驱动/通信负责人。
- 测试或现场联调负责人。

如果只能一人审查，应优先审控制安全和真实硬件接入风险。

## 3. 必读材料

按顺序阅读：

1. `references/01_requirements/夹爪控制系统设计.md`
2. `references/03_control/control_architecture_design.md`
3. `references/06_implementation/full_implementation_pass_2026-05-16.md`
4. `references/07_standards/project_coding_standards.md`
5. `references/08_review/ai_preliminary_review_2026-05-16.md`
6. `references/04_hardware/hardware_reference_notes.md`
7. `src/controller/gripper_controller.hpp`
8. `src/controller/gripper_types.hpp`
9. `src/controller/gripper_controller.cpp`
10. `src/controller/state_machine/gripper_state_machine.hpp`
11. `src/controller/state_machine/gripper_state_machine.cpp`
12. `src/hardware_interface/motor_interface.hpp`
13. `src/hardware_interface/damiao/*`
14. `test/test_damiao_protocol.cpp`

## 4. 审查重点

### 4.1 架构边界

确认：

- `controller` 不依赖 UI。
- `controller` 不依赖 commander。
- `controller` 不依赖 `hardware_interface/damiao`。
- `controller` 不依赖 `hardware_interface/simulated`。
- UI 和 commander 不绕过 controller 直接访问硬件。
- app 组合层负责选择真实硬件或模拟硬件。

### 4.2 控制需求覆盖

确认是否覆盖以下需求：

- 限流。
- 限速。
- 限加速度。
- 低速低电流回零。
- PreSelfCheck 学习最低稳定速度。
- PreSelfCheck 学习静摩擦和动摩擦。
- 初步限位发现。
- 安全区和软件限位建立。
- 运动健康检查。
- 目标力夹紧。
- 恒速夹紧。
- 接触/堵转检测。
- 动作完成后失能，避免丝杆螺母抱死。

### 4.3 当前未完成项

必须明确：

- 达妙 SDK 已完成第一阶段代码接入，但尚未真实硬件验收。
- `DmUsb2FdcanTransport` 的 DLL 加载、设备枚举、通道配置需要在现场 PC 上验证。
- `DamiaoMotor` 的使能、失能、模式切换、反馈解析需要在真实电机上验证。
- 当前模拟电机不能代表真实摩擦、机械限位、线缆接触和冲击。
- 当前控制台 UI 只是测试壳，不是最终 UI。
- `PreSelfCheck` 已改为逐步试探流程，但仍需要真实硬件采样闭环验证和参数整定。

### 4.4 安全策略

重点审查：

- 限流、限速、限加速度是否足够保守。
- 触发限幅时是裁剪命令还是主动停止。
- 接触/堵转检测是否存在误判或漏判风险。
- 目标力夹紧后失能是否会影响夹持可靠性。
- 回零失败时是否会进入安全状态。
- 行程学习失败时是否禁止进入夹紧。
- 真实硬件通信失败时是否不会继续发运动命令。

### 4.5 达妙硬件路径审查

重点审查：

- `driver_library_path` 是否指向目标 PC 上实际可加载的 `libdm_device.dll`。
- `device_index` 和 `channel_index` 是否与现场设备连接一致。
- `motor_id=0x08`、`host_id=0x18` 是否与当前电机参数一致。
- `command_id_includes_mode_offset=true` 是否符合当前固件配置。
- 位置力控命令中的相电流归一化是否与 DM-J4310P-2EC 参数一致。
- `readFeedback()` 对非本电机反馈帧的处理是否满足多电机或总线噪声场景。

## 5. 状态机审查表

需要人工确认：

| 状态 | 业务含义 | 允许进入条件 | 成功后 | 失败后 |
|---|---|---|---|---|
| `Disconnected` | 未连接硬件 | 初始或断开 | `Connected` | 保持或 `Fault` |
| `Connected` | 硬件连接成功但未使能 | connect 成功 | `HardwareSanityCheck` 或 `ModeSwitching` | `Fault` |
| `Enabled` | 电机已使能，可执行控制流程 | enable 成功 | 自检/回零/学习/健康检查 | `Disabled` 或 `Fault` |
| `PreSelfCheck` | 学习保守结构参数 | 已连接且可使能 | `Enabled` | `ActiveStop` 或 `Fault` |
| `HomingOpenStop` | 低电流开侧靠零 | 已自检或使用保守参数 | `Enabled` | `Fault` |
| `TravelLearning` | 学习闭合侧行程限位 | 已回零 | `Enabled` | `Fault` |
| `MotionHealthCheck` | 安全区往返运动验证 | 已学习行程 | `Ready` | `ActiveStop` |
| `Ready` | 可接受夹紧/释放 | 健康检查通过 | `Clamping`/`Releasing` | `Fault` |
| `Clamping` | 夹紧中 | Ready 且命令有效 | `Unloading`/`Disabled` | `ActiveStop` |
| `Releasing` | 释放中 | Ready 或已连接 | `Disabled` | `ActiveStop` |
| `Disabled` | 电机失能 | 正常完成或手动失能 | `Enabled` | `Fault` |
| `ActiveStop` | 可恢复主动停止 | 安全触发 | `Enabled` 或重新初始化 | `Fault` |
| `Fault` | 故障 | 硬件/通信/严重安全失败 | 人工处理后恢复 | 保持 |

## 6. 人工审查问题清单

审查时逐项回答：

- 当前状态机是否允许不经过回零和行程学习就夹紧？
- 任何夹紧动作完成后是否一定失能？
- 触发接触/堵转后是否会继续输出夹紧方向速度？
- 通信失败后是否会继续执行上一次命令？
- 真实硬件路径是否可能在未完成现场联调前被误认为已经验收？
- 配置中的电流、速度、加速度默认值是否足够保守？
- `StructureProfile` 中的参数是否能追溯到自检或保守配置？
- 是否存在裸 `double` 进入公开接口表达物理量？
- 是否存在关键函数缺少审查所需注释？
- `PreSelfCheck` 的逐步试探速度、电流、行程窗口是否足够保守？
- `MotorBringup` 是否只用于低能量真实硬件通信/方向测试，并且不会绕过
  归零、行程学习或夹紧安全门？
- `MotorBringup` 点动后的主动失能策略是否满足现场调试安全要求？

## 7. 审查结论模板

建议审查后记录：

```text
审查日期：
审查人：
审查范围：

结论：
- 通过 / 有条件通过 / 不通过

必须修改：
1.
2.
3.

建议修改：
1.
2.
3.

真实硬件联调前阻塞项：
1.
2.
3.
```
