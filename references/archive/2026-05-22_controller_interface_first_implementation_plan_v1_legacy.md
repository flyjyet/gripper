# 控制器接口优先的逐步落盘计划

## 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v1.0 | 2026-05-16 | 建立控制器接口优先的逐步落盘计划。 |
| v1.1 | 2026-05-20 | 增加管理员维护恢复模式落地步骤；补充权限、方向识别、硬保护和恢复后重新自检要求。 |
| v1.2 | 2026-05-20 | 将维护模式拆分为 `MotorBringupMode` 和 `AdminRecoveryMode`；增加 UI 原型展示要求。 |

## 1. 核心策略

本阶段采用“两头定，中间实现”的策略：

- 先定义 `controller` 的公开接口，作为功能需求锚点。
- 再从底层模块开始逐步实现依赖，避免控制器接口在实现过程中丢失目标或被硬件细节污染。

也就是说，先写：

- `src/controller/gripper_controller.hpp`
- `src/controller/gripper_types.hpp`

但第一步只写接口、结构体、枚举和注释，不写控制算法实现。

## 2. 为什么先写控制器接口

夹爪控制系统的最终目标不是“把某个电机转起来”，而是完成一组明确的夹爪功能：

- 连接和诊断硬件。
- 限流、限速。
- 初始化/自检。
- 低速低电流堵转靠零。
- 行程学习和软件限位。
- 安全范围运动健康检查。
- 指定夹紧力夹紧。
- 指定恒定速度夹紧。
- 主动堵转检测和主动停止。
- 工作完成后失能，避免丝杆螺母抱死。

如果直接从底层写起，容易在硬件适配、状态机细节、参数结构中丢失这些功能目标。因此应先用控制器公开接口把目标固定下来。

## 3. 总体落盘顺序

推荐顺序如下：

```text
1. controller public API
   gripper_controller.hpp
   gripper_types.hpp

2. common
   Result / ErrorCode / units / timestamp / logging

3. hardware_interface
   MotorInterface / MotorFeedback / MotorCommand / TransportInterface

4. config
   GripperConfig / ConfigLoader / default_gripper.yaml

5. controller/self_check
   StructureProfile / self-check identifiers

6. controller/state_machine
   GripperStateMachine / PreSelfCheckStateMachine / phase snapshots

7. controller/safety
   SafetyLimiter / ContactJamDetector

8. controller/mechanism
   kinematics and stroke mapping

9. controller/calibration
   clamp force mapping

10. controller implementation
    gripper_controller.cpp

11. controller/maintenance
    motor bring-up, administrator recovery state, safety supervisor,
    direction probe, audit data

12. ui / commander
    call controller API only

13. hardware_interface/damiao
    concrete Damiao SDK and USB2FDCAN implementation
```

## 4. 第一批控制器公开接口

`GripperController` 应按业务功能组织，而不是按电机命令组织。

建议接口如下：

```cpp
class GripperController {
 public:
  Result configure(const GripperConfig& config);

  Result connect();
  Result disconnect();

  Result enable();
  Result disable();

  Result runPreSelfCheck();
  Result homeOpenStop();
  Result learnTravelLimits();
  Result runMotionHealthCheck();

  Result clampByForce(const ClampForceCommand& command);
  Result clampBySpeed(const ClampSpeedCommand& command);
  Result release();

  Result stop();
  Result clearFault();

  Result enterMotorBringupMode(const MotorBringupRequest& request);
  Result motorBringupJog(const MotorBringupJogCommand& command);
  Result exitMotorBringupMode();

  Result enterAdminRecovery(const AdminRecoveryRequest& request);
  Result adminProbeOpenDirection(const AdminDirectionProbeCommand& command);
  Result adminJog(const AdminJogCommand& command);
  Result adminPulseRelease(const AdminPulseReleaseCommand& command);
  Result adminClearMotorFault();
  Result exitAdminRecovery();

  Result update();
  GripperStateSnapshot state() const;
};
```

注意：

- 该接口不暴露达妙 SDK。
- 该接口不暴露 CAN 帧。
- 该接口不暴露电机私有模式细节。
- 该接口表达夹爪功能需求。
- 管理员恢复接口必须独立于普通 `release()`，不能作为普通释放命令的 bypass 参数。
- 管理员恢复接口必须保留硬保护、方向识别、命令超时和审计要求。
- 电机空载调试接口属于维护权限，但不属于管理员恢复。它用于结构未安装时验证通信、正反转和反馈，不更新零位、软件限位或 `StructureProfile`。

## 5. gripper_types.hpp 需要承载的内容

`gripper_types.hpp` 用于定义控制器公开接口需要的业务数据类型。

建议至少包括：

- `GripperMode`
  - 未连接。
  - 已连接。
  - 已使能。
  - 自检中。
  - 回零中。
  - 行程学习中。
  - 健康检查中。
  - 就绪。
  - 夹紧中。
  - 释放中。
  - 主动停止。
  - 故障。
- `ClampForceCommand`
  - 目标夹紧力。
  - 最大夹紧速度。
  - 最大允许电流或力矩。
  - 超时时间。
- `ClampSpeedCommand`
  - 目标夹紧速度。
  - 最大允许电流或力矩。
  - 接触检测策略。
  - 超时时间。
- `ReleaseCommand`
  - 释放速度。
  - 释放目标或释放行程。
  - 超时时间。
- `AdminRecoveryPrivilege`
  - 无权限。
  - 维护权限。
  - 工程权限。
  - 工厂权限。
- `MotorBringupRequest`
  - 权限等级。
  - 结构未安装或空载确认。
- `MotorBringupJogCommand`
  - 电机侧目标速度。
  - 最大电流。
  - 点动持续时间。
- `MotionDirectionConfidence`
  - 未知。
  - 仅来自配置。
  - 方向试探确认。
  - 自检确认。
  - 回零确认。
  - 行程学习确认。
- `AdminRecoveryBlockReason`
  - 无。
  - 打开方向未知。
  - 实际运动方向与命令相反。
  - 电流上升但无释放位移。
  - 疑似越卡越紧。
  - 温度限制。
  - 命令超时。
  - 连续使能超时。
  - 硬件故障。
  - 用户停止。
- `AdminRecoveryRequest`
  - 权限等级。
  - 用户是否确认风险。
  - 是否强制要求方向试探。
- `AdminDirectionProbeCommand`
  - 方向试探电流。
  - 方向试探速度。
  - 方向试探时间。
  - 最小有效位移。
- `AdminJogCommand`
  - 目标方向。
  - 最大电流。
  - 最大速度。
  - 点动持续时间。
  - 是否允许绕过普通软件行程限制。
- `AdminRecoveryAuditRecord`
  - 权限等级。
  - 请求方向。
  - 命令前方向可信度。
  - 命令电流、速度、持续时间。
  - 命令前后行程。
  - 峰值电流、力矩和温度。
  - 阻止原因。
- `GripperStateSnapshot`
  - 顶层状态。
  - 自检状态。
  - 是否已连接。
  - 是否已使能。
  - 是否已完成自检。
  - 是否已回零。
  - 是否已学习行程。
  - 当前螺母行程。
  - 当前夹爪角度。
  - 估算夹紧力。
  - 电机反馈摘要。
  - 故障摘要。
  - 最近一次命令结果。

`StructureProfile` 不建议直接放在 `gripper_types.hpp`，应保留在：

- `src/controller/self_check/structure_profile.hpp`

`gripper_types.hpp` 可以引用或摘要 `StructureProfile` 的有效等级。

## 6. 控制器接口注释要求

每个公开接口必须写明：

- 功能目的。
- 前置条件。
- 是否要求已连接。
- 是否要求已使能。
- 是否要求已完成 `PreSelfCheck`。
- 是否要求已回零。
- 是否要求已学习软件限位。
- 成功后进入什么状态。
- 失败后是否进入 `ActiveStop` 或 `Fault`。
- 关键单位。
- 管理员恢复接口还必须写明不可绕过的硬保护、是否要求方向可信、是否会在结束后失能、是否会清除已回零/已学习等可信状态。

示例：

```cpp
// Runs the pre-self-check sequence before homing.
//
// Preconditions:
// - The controller must be configured and connected.
// - The motor should be enabled or enable-capable.
// - Homing is not required.
//
// Behavior:
// - Finds conservative motion parameters.
// - Probes both directions with limited stroke/current/speed.
// - Builds a preliminary safe zone when possible.
//
// Failure:
// - Returns an error if motion is unsafe or inconsistent.
// - May transition to ActiveStop or Fault depending on severity.
Result runPreSelfCheck();
```

注释语言建议使用英文写在代码中，便于后续工具、IDE、文档生成和团队协作；需求说明继续使用中文文档维护。

## 7. 依赖边界

第一批控制器头文件允许依赖：

- `common/result.hpp`
- `common/timestamp.hpp`
- `common/units.hpp`
- `config/gripper_config.hpp`
- `hardware_interface/motor_types.hpp`
- `controller/state_machine/gripper_state_machine.hpp`

第一批控制器头文件不允许依赖：

- 达妙 SDK 头文件。
- `hardware_interface/damiao/*`
- UI 头文件。
- commander 头文件。

这样可以确保控制器接口是业务接口，不被具体硬件和界面污染。

## 8. 分阶段目标

### 阶段 1：控制器 API 锚点

目标：

- 完成 `gripper_controller.hpp`。
- 完成 `gripper_types.hpp`。
- 明确所有公开功能入口。
- 明确状态快照和命令数据结构。

验收：

- UI 和 commander 可以只看控制器头文件就知道能调用什么功能。
- 接口覆盖当前所有需求。
- 接口不暴露硬件私有细节。

### 阶段 2：基础类型和硬件抽象

目标：

- 完成 `common` 基础类型。
- 完成 `hardware_interface` 抽象接口。

验收：

- 控制器头文件中引用的基础类型都存在。
- controller 不需要包含任何达妙 SDK。

### 阶段 3：配置和结构参数

目标：

- 完成 `GripperConfig`。
- 完成 `StructureProfile`。

验收：

- 自检、回零、安全、夹紧都能从配置和结构参数中读取必要阈值。

### 阶段 4：状态机和安全模块

目标：

- 完成顶层状态机。
- 完成 `PreSelfCheck` 子状态机。
- 完成安全限制接口。

验收：

- 非法状态调用会被拒绝。
- 任意状态均可进入 `ActiveStop` 或 `Fault`。
- 管理员恢复状态只能由授权入口进入。
- 打开方向未知或低可信时禁止进入高电流恢复。
- `AdminRecoveryBlocked` 不能直接回到 `Ready`。

### 阶段 5：维护模式框架

目标：

- 将维护模式拆分为 `MotorBringupMode` 和 `AdminRecoveryMode`。
- `MotorBringupMode` 用于结构未安装时验证电机通信、模式切换、使能、正反转点动和反馈方向。
- `AdminRecoveryMode` 用于结构已安装后的抱死、卡滞和普通释放失败恢复。
- 两个模式都必须保留硬保护、命令超时、温度限制和命令后失能。

验收：

- `MotorBringupMode` 要求结构未安装或空载确认。
- `MotorBringupMode` 不更新 `StructureProfile`、零位或软件限位。
- `MotorBringupMode` 不能进入 `Ready`。
- `MotorBringupMode` 只能确认电机命令方向和编码器反馈方向，不能确认夹爪打开/闭合方向。
- `AdminRecoveryMode` 仍必须执行打开方向可信度检查和越卡越紧识别。

### 阶段 6：管理员维护恢复模式

目标：

- 扩展 `gripper_types.hpp` 中的管理员恢复数据结构。
- 扩展 `GripperConfig` 中的 `admin_recovery` 配置。
- 扩展顶层状态机，增加 `AdminRecoveryArmed`、`AdminDirectionProbe`、`AdminRecoveryJogging`、`AdminRecoveryBlocked`。
- 实现管理员恢复安全策略，区分 `HardSafetyPolicy`、`AdminRecoverySafetyPolicy` 和 `NormalSafetyPolicy`。
- 实现低能量打开方向试探。
- 实现短时点动。
- 实现越卡越紧识别。
- 实现审计记录。

验收：

- 管理员模式可以从 `ActiveStop/Fault/Disabled/Enabled` 进入。
- 未授权或未确认风险时拒绝进入。
- 方向未知时只能低能量试探，不能高电流打开。
- 如果期望打开但实际向闭合方向运动，立即停止、失能并进入 `AdminRecoveryBlocked`。
- 如果电流或力矩上升但没有释放位移，立即停止、失能并进入 `AdminRecoveryBlocked`。
- 管理员命令不能无限持续，必须有超时。
- 管理员模式不能绕过温度、通信超时、命令看门狗、工程最大电流、工程最大速度、硬件故障和用户停止。
- 管理员恢复退出后清除 `homed`、`travel_limits_learned`、`motion_health_checked` 等可信标志，不能直接进入 `Ready`。

### 阶段 7：控制器实现

目标：

- 实现 `gripper_controller.cpp`。
- 编排 self_check、state_machine、safety、mechanism、hardware_interface。

验收：

- 使用 simulated motor 能跑通基本流程。
- 不连接真实硬件也能验证状态流转。

### 阶段 8：UI/Commander 接入

目标：

- 提供 UI 原型展示，先用于确认布局和交互，不接硬件。
- 维护页面拆分为电机空载调试和管理员恢复。
- 管理员功能放在独立维护页面或独立命令组。
- 需要二次确认。
- 显示方向可信度。
- 显示是否允许升流。
- 显示阻止原因。
- 显示实时电流、速度、力矩、温度。
- 支持日志暂停查看。

验收：

- 普通页面无法误触管理员点动。
- 方向未确认时 UI 不允许高电流打开。
- `AdminRecoveryBlocked` 时必须显示明确原因。

### 阶段 9：达妙 SDK 接入

目标：

- 连接 DM-USB2FDCAN_Dual。
- 设置 FDCAN、BRS、仲裁域 1 Mbps、数据域 5 Mbps。
- 模式切换。
- 使能。
- 短时命令发送。
- 反馈读取。
- 失能。

验收：

- 真实硬件先验证低能量方向试探。
- 不允许跳过方向确认直接做高电流恢复。
- 管理员模式和 `PreSelfCheck` 共用经过验证的低能量双向试探和反馈一致性检测策略。

## 9. 测试策略

第一轮测试只做接口级和编译级检查：

- 所有新增头文件可以独立 include。
- `controller` 不包含达妙 SDK。
- `hardware_interface` 不依赖 `controller`。
- `common` 不依赖业务模块。
- `gripper_controller.hpp` 能表达所有功能需求。

第二轮测试使用仿真电机：

- connect。
- enable。
- runPreSelfCheck。
- homeOpenStop。
- learnTravelLimits。
- runMotionHealthCheck。
- clampByForce。
- release。
- disable。
- enterMotorBringupMode。
- motorBringupJog。
- exitMotorBringupMode。
- enterAdminRecovery。
- adminProbeOpenDirection。
- adminJog。
- exitAdminRecovery。
- 方向未知时禁止高电流恢复。
- 错误方向时进入 AdminRecoveryBlocked。
- 电流上升无位移时进入 AdminRecoveryBlocked。
- 管理员恢复退出后禁止直接进入 Ready。

第三轮再接入达妙硬件：

- 只替换 `hardware_interface` 实现。
- controller API 不变化。
- UI 和 commander 不直接调用硬件实现。
- 硬件管理员恢复测试从低能量方向试探开始，不直接做高电流恢复。

## 10. 默认决策

- 先写控制器公开接口，作为功能目标锚点。
- 具体实现仍从底层依赖逐步向上。
- 第一阶段只写头文件接口和注释，不写控制算法。
- 控制器接口不暴露达妙 SDK、CAN 帧和电机私有模式。
- 源码注释使用英文；设计和需求文档使用中文。
- 管理员维护恢复模式不能实现无限制电流、无限制速度或无限持续使能。
- 管理员高电流恢复前必须完成打开方向确认。
- 管理员恢复后必须重新执行自检、回零、行程学习和运动健康检查。
