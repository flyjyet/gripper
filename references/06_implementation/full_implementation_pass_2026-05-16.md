# 2026-05-16 全盘实现落盘记录

## 1. 本轮目标

本轮目标是在新的 `src` 目录框架下完成一版可构建、可运行、边界清晰的夹爪控制系统骨架，并把核心控制链路跑通：

- `common`、`config`、`utils`、`hardware_interface`、`controller`、`ui`、`app` 均有最小实现。
- 控制器公开接口保持业务语义，不暴露 CAN 帧、达妙 SDK、UI 或 commander。
- 物理量接口优先使用 `src/common/units.hpp` 中的单位类型。
- 可通过模拟电机执行脚本化流程。
- 真实达妙硬件路径第一阶段已接入 DeviceSDK 动态加载和协议封装，但仍需真实硬件联调确认。

## 2. 已落盘模块

### common

- `ErrorCode`
- `Result`
- `Timestamp`
- `units`
- 基础日志类型

`common` 只放基础类型，不包含机构参数、控制参数或硬件配置。

### config

- `GripperConfig` 覆盖 adapter、motor、mechanism、self_check、safety、homing、clamp、ui。
- `default_gripper.yaml` 为人工可读模板。
- `config_loader` 实现受限 YAML-like 字段解析，支持当前模板中的标量和一行列表。

### hardware_interface

- `MotorInterface`
- `MotorCommand`
- `MotorFeedback`
- `TransportInterface`
- `CanFrame`
- `SimulatedMotor`
- `hardware_interface/damiao` 第一阶段真实实现

注意：达妙真实硬件实现当前已能编译并通过协议单元测试，但尚未在本轮通过真实电机现场联调验收。

### controller

已实现：

- `GripperController` 公共接口
- 默认控制器实现
- 顶层状态机
- `PreSelfCheck` 子状态机
- 自检参数识别器
- 行程限位识别器
- 运动健康检查器
- 安全限幅器
- 接触/堵转检测器
- 机构一阶运动学映射
- 目标力到电机电流/力矩映射

当前控制器约束：

- 不依赖 UI。
- 不依赖 commander。
- 不依赖 `hardware_interface/damiao`。
- 不依赖 `hardware_interface/simulated`。
- 具体电机由 app 组合层注入。

### ui/app

- 当前 UI 是控制台测试壳，不是最终图形界面。
- `UiController` 只调用 `GripperController` 公开接口。
- `Application` 负责选择模拟电机或达妙真实硬件电机。
- `--scripted-demo` 可跑通模拟流程。
- `--damiao --scripted-demo` 会走达妙硬件路径；真实设备不在线时应返回硬件连接错误。

## 3. 已验证命令

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:\jyt_ws_ai\TS2000_human_robot\gripper\build\zig-cache'
.venv\Scripts\cmake.exe -S . -B build -G Ninja -DCMAKE_MAKE_PROGRAM='D:/jyt_ws_ai/TS2000_human_robot/gripper/.venv/Scripts/ninja.exe' -DCMAKE_CXX_COMPILER='D:/jyt_ws_ai/TS2000_human_robot/gripper/.venv/Scripts/python-zig.exe;c++'
.venv\Scripts\cmake.exe --build build --target gripper_app
.venv\Scripts\ctest.exe --test-dir build --output-on-failure
```

验证结果：

- `gripper_scripted_demo` 通过。
- `gripper_damiao_placeholder` 通过。
- `test_damiao_protocol` 通过。
- `test_safety_limiter` 通过。
- `test_state_machine` 通过。

## 4. 当前行为边界

### 模拟路径

模拟路径用于验证：

- 配置
- 连接
- 使能
- PreSelfCheck
- 回零
- 行程学习
- 运动健康检查
- 目标力夹紧
- 释放
- 动作完成后失能

模拟电机仍是轻量模型，不能代表真实丝杆摩擦、机械限位、线缆接触和冲击。

### 真实硬件路径

真实硬件路径当前完成第一阶段接入：

- `DmUsb2FdcanTransport`
- `DamiaoProtocol`
- `DamiaoMotor`

当前能力边界：

- `DmUsb2FdcanTransport` 动态加载 `libdm_device.dll`，打开
  `DM-USB2FDCAN_Dual`，配置 FDCAN，发送/接收 `CanFrame`。
- `DamiaoProtocol` 已封装使能、失能、清错、刷新、`CTRL_MODE`、速度、
  位置速度、位置力控和 MIT 力矩命令。
- `DamiaoMotor` 已实现 `connect/enable/disable/sendCommand/readFeedback` 的
  最小闭环。
- `default_gripper.yaml` 已增加达妙硬件联调必要参数，例如 DLL 路径、设备
  序号、通道序号、`0x08` 电机 ID、`0x18` 反馈 ID、相电流和力矩常数。

仍需真实硬件确认：

- DLL 及依赖 DLL 是否能在目标运行目录被加载。
- 设备序号和通道序号是否与现场连接一致。
- `CTRL_MODE=4`、使能、失能是否被电机接受。
- 位置力控命令的速度、电流归一化是否与 DM-J4310P-2EC 实机一致。
- 反馈帧 ID、反馈数据和错误码解释是否与当前电机固件一致。

## 5. 本轮审查修正

只读审查指出了第一版实现中的风险，已修正：

- 控制器不再直接创建或包含 `SimulatedMotor`。
- 自检、行程学习、健康检查、夹紧不再无条件写入 `Verified` 或 `Healthy`。
- 接触/堵转检测改为使用反馈速度和反馈电流。
- `SafetyLimiter` 增加加速度限制字段和逻辑。
- 状态机允许 `Enabled -> Disabled`。
- app 支持 `--damiao --scripted-demo` 的无序参数组合。
- CTest 增加达妙占位路径测试。
- 达妙协议新增独立单元测试，锁定关键帧格式和反馈解析。
- `PreSelfCheck` 从固定单次探测改为低能量逐步试探：
  先逐步增加速度/电流寻找可动，再验证反向可动，并在安全小窗口内做
  往返采样。

## 6. 后续真实实现任务

后续应优先实现：

1. 达妙 SDK 真实联调确认
   - 在插入 `DM-USB2FDCAN_Dual` 和 DM-J4310P-2EC 后验证设备打开、FDCAN
     配置、发送、接收、关闭。
   - 确认 DM-J4310P-2EC 的使能、失能、模式切换、反馈解析。
   - 确认真实运动方向、反馈方向、电流/力矩换算和 `direction_sign`。

2. 真实 PreSelfCheck 采样编排
   - 按不同区域逐步试探最低稳定速度。
   - 学习静摩擦、动摩擦、噪声底。
   - 初步发现机械限位并建立安全区。
   - 多区域往返运动，记录最大保守参数。

3. 真实回零和行程学习
   - 低速低电流向开侧机械止挡靠零。
   - jam confirm。
   - backoff。
   - 向闭合方向学习另一侧限位。
   - 软件限位必须在机械限位内侧。

4. 真实夹紧闭环
   - 恒速接近。
   - 接触/堵转检测。
   - 目标力映射和反馈确认。
   - 达到目标后卸载或失能，避免丝杆螺母抱死。

5. 图形 UI
   - 当前控制台 UI 只是测试壳。
   - 后续图形 UI 应继续只依赖 `UiController` 和 `GripperController`。

## 7. 2026-05-20 真实硬件低能量 MotorBringup 入口

为支持真实 DM-J4310P-2EC 硬件逐步测试，当前 C++ 重构已新增独立
`MotorBringup` 路径。该路径只用于通信、反馈、使能/失能和正反方向低能量
点动验证，不进入归零、行程学习、夹紧或管理员恢复流程。

已落盘内容：

- `GripperController` 新增 `enterMotorBringupMode`、`exitMotorBringupMode`、
  `refreshMotorBringupFeedback`、`enableMotorBringupOutput`、`jogMotorBringup`。
- `MotorBringupSessionRequest` 要求调用方确认电机空载或结构已机械安全处理。
- `MotorBringupJogCommand` 使用电机侧相对位置、速度、电流、持续时间物理量。
- 参数统一来自 `motor_bringup` 配置节，不在控制逻辑中写死。
- 控制台 UI 新增命令：
  - `bringup_enter_confirm_unloaded`
  - `bringup_feedback`
  - `bringup_enable`
  - `bringup_disable`
  - `bringup_jog_pos [rad] [rad_s] [amp] [sec]`
  - `bringup_jog_neg [rad] [rad_s] [amp] [sec]`
  - `bringup_exit`
  - `clear_fault`
- 每次 `bringup_jog_*` 发送短脉冲后主动失能，避免持续输出力矩。
- Bring-up 会话期间普通 `enable/selfcheck/home/learn/health/clamp/release`
  工作流会被阻止，必须先 `bringup_exit`。
- 新增 `test_motor_bringup`，覆盖未确认不能进入、点动后自动失能、bring-up
  期间阻止普通使能。

已验证：

```powershell
.\scripts\test.ps1
```

结果：6/6 通过，包括 `test_motor_bringup`。

真实硬件仍需现场验证：

- `connect` 是否能打开当前 PC 上的 `DM-USB2FDCAN_Dual`。
- `bringup_feedback` 是否能读到 `motor_pos_rad`、`motor_vel_rad_s`、
  `motor_current_a`、`motor_torque_nm` 和温度。
- `bringup_enable/bringup_disable` 是否被电机固件接受。
- `bringup_jog_pos/bringup_jog_neg` 的实际方向和反馈方向是否一致。
- 电流、力矩缩放是否与 DM-J4310P-2EC 当前固件一致。

2026-05-20 现场测试更新：

- `connect` 已通过，说明 DLL 加载、设备枚举、通道配置已能走通。
- `libdm_device.dll` 依赖 `VCRUNTIME140.dll` 和 `VCRUNTIME140_1.dll`，已复制到
  `src/third_party/damiao/bin/`。
- C++ transport 改为预加载 SDK 同目录依赖 DLL，再加载 `libdm_device.dll`。
- `disconnect/quit` 采用调试期非阻塞关闭策略，暂不调用会卡住的 SDK
  `device_close/device_disable_channel`。
- `bringup_feedback` 未通过：诊断显示 `tx_count=2 last_tx_id=0x7ff`，但
  `rx_count=0`，通道 0 和通道 1 都没有任何 RX 帧。
- 下一步现场排查重点：电机上电、CANH/CANL、终端电阻、选中通道、电机 ID
  `0x08`、主机反馈 ID `0x18`、FDCAN 1M/5M/BRS 配置以及刷新帧格式。
