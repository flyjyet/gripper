# 两端硬限位线性轴通用控制库架构设计

## 1. 文档目的

本文定义将当前夹爪控制项目封装为通用控制库的架构方案。目标不是简单把现有代码打包成库，而是把已经在实机调试中验证过的控制思想抽象为可复用的“两端硬限位线性轴控制库”。

本文重点说明：

- 控制库的适用边界和非适用边界。
- 通用库核心、硬件适配层、当前夹爪应用之间的模块关系。
- 每个模块的职责、输入输出和依赖边界。
- 核心数据结构和状态语义。
- 后续应用场景如何接入和使用此库。
- 从当前项目迁移到通用库的工程路线。

本文件用于后续“库化”设计、接口拆分和代码重构。当前夹爪项目的具体控制行为仍以 `references/03_control/gripper_control_architecture_v2.md` 和 `references/06_implementation/pre_self_check_final_archive_2026-05-26.md` 为准。库化过程中不得削弱这些文件中已经确认的安全约束。

## 2. 库定位

推荐将通用库定位为：

```text
dual_hard_stop_linear_axis_control
```

即“两端硬限位、无外部限位/力传感器、仅依赖电机反馈的丝杆或线性轴控制库”。

当前夹爪只是该库的第一个应用实例。库核心应面向更通用的线性执行机构：

- 梯形丝杆夹爪。
- 小型线性滑台。
- 电机加丝杆推杆。
- 无独立限位开关的单轴压紧/释放机构。
- 仅有电机位置、速度、电流、扭矩、温度反馈的低成本执行器。

库不直接解决：

- 多轴轨迹规划。
- 高精度 CNC 插补。
- 机器人整机动力学。
- 有外部力传感器或限位开关时的完整传感器融合。
- 夹爪连杆几何、夹紧力映射、UI 和上位机交互。

这些能力可以作为应用层扩展，不进入库核心。

## 3. 设计原则

### 3.1 线性轴优先

库核心只理解“电机驱动一个线性轴”。它使用的业务单位是：

- 电机侧：rad、rad/s、A、Nm、deg C。
- 线性轴侧：mm、mm/s、mm/s2。

夹爪角度、夹爪力、物体接触语义属于应用层。

### 3.2 硬件协议隔离

控制库核心不依赖达妙 SDK、CAN 帧、USB2FDCAN DLL 或 UI。硬件通过 `MotorInterface` 抽象接入。达妙电机、仿真电机、CiA402 伺服、RS485 电机都应作为 adapter 实现。

### 3.3 状态置信度分层

库必须显式区分不同置信度：

```text
Unknown
ConservativeDefaults
PreSelfCheckCompleted
Homed
TravelLimitsLearned
MotionHealthChecked
Ready
```

其中 `PreSelfCheckCompleted` 不等于 `Homed`，历史 seed 不能替代本次启动周期的自检、回零、行程学习和健康检查。

### 3.4 安全 envelope 不可绕过

应用层可以设置目标和策略，但不能绕过库核心的硬保护：

- 用户停止。
- 通信严重异常。
- 电机故障。
- 全局最大反馈电流。
- 温度硬保护。
- 命令持续时间上限。
- 端点停机后的卸载/失能。

### 3.5 学习结果可解释

库输出的 profile、seed、anomaly map 和 trace 必须是结构化数据。日志字符串只作为展示层格式化结果，不能成为上层解析控制结论的唯一来源。

## 4. 总体架构

```text
                     +-------------------------------+
                     |        application layer       |
                     | gripper / slider / actuator    |
                     +---------------+---------------+
                                     |
                     +---------------v---------------+
                     |       application binding      |
                     | force mapping / kinematics     |
                     +---------------+---------------+
                                     |
                     +---------------v---------------+
                     |      workflow_controller       |
                     | state machine and workflows    |
                     +---+-----------+-----------+---+
                         |                       |
              +----------v---------+   +---------v----------+
              |  self_learning     |   |  safety_envelope   |
              | PreA/PreB/profile  |   | clamp/stop/limits  |
              +----------+---------+   +---------+----------+
                         |                       |
                         +-----------+-----------+
                                     |
                     +---------------v---------------+
                     |      linear_axis_encoder       |
                     | motor rad <-> axis stroke mm   |
                     +---------------+---------------+
                                     |
                     +---------------v---------------+
                     |         motor_adapter          |
                     | generic MotorInterface         |
                     +---+-----------+-----------+---+
                         |                       |
              +----------v---------+   +---------v----------+
              | simulated adapter  |   | hardware adapters  |
              | tests and CI       |   | Damiao/CiA402/etc. |
              +--------------------+   +--------------------+
```

辅助模块：

```text
axis_core      : 单位、错误码、时间戳、方向、状态快照。
persistence    : seed、anomaly map、版本迁移。
diagnostics    : event、trace、ring buffer、结构化日志。
```

## 5. 模块职责

### 5.1 axis_core

职责：

- 定义通用单位类型和基础结果类型。
- 定义线性轴方向、状态、置信度、错误码。
- 定义通用 snapshot、event、trace 数据结构。

允许依赖：

- C++ 标准库。

禁止依赖：

- 硬件 adapter。
- 控制流程。
- UI。
- 当前夹爪机构参数。

典型数据：

```cpp
enum class AxisDirection {
  Unknown,
  Opening,
  Closing,
};

enum class AxisConfidence {
  Unknown,
  ConservativeDefaults,
  LowConfidence,
  Homed,
  TravelLimitsLearned,
  Verified,
};

struct AxisRange {
  common::Mm open_limit;
  common::Mm closed_limit;
  AxisConfidence confidence;
  bool valid;
};
```

### 5.2 motor_adapter

职责：

- 提供电机连接、使能、失能、命令发送、反馈读取接口。
- 以通用电机单位表达命令和反馈。
- 隔离厂商 SDK、CAN 帧、通信线程和 DLL 加载。
- 透传运行时电机窗口，例如位置、速度、扭矩限制。

核心接口：

```cpp
class MotorInterface {
 public:
  virtual Result connect() = 0;
  virtual Result disconnect() = 0;
  virtual Result enable() = 0;
  virtual Result disable() = 0;
  virtual Result sendCommand(const MotorCommand& command) = 0;
  virtual Result readFeedback(MotorFeedback* feedback) = 0;
};
```

数据结构：

```cpp
struct MotorCommand {
  MotorControlMode control_mode;
  common::Rad target_position;
  common::RadPerS target_velocity;
  common::A target_current;
  common::Nm target_torque;
  common::Timestamp timestamp;
};

struct MotorFeedback {
  common::Rad position;
  common::RadPerS velocity;
  common::A current;
  common::Nm torque;
  common::DegC temperature;
  common::Rad runtime_position_limit;
  common::RadPerS runtime_velocity_limit;
  common::Nm runtime_torque_limit;
  bool runtime_limits_valid;
  bool enabled;
  bool fault;
  common::Timestamp timestamp;
};
```

适配器分层建议：

```text
axis_control_motor_interface
axis_control_simulated_adapter
axis_control_damiao_adapter
axis_control_cia402_adapter
```

其中 `axis_control_damiao_adapter` 可以使用达妙 SDK 和 USB2FDCAN 细节，但这些细节不能进入核心库头文件。

### 5.3 linear_axis_encoder

职责：

- 将电机多圈位置映射为线性轴位置。
- 支持启动临时零点。
- 支持回零后真实零点重基准。
- 维护反馈新鲜度和换算诊断。
- 根据运行时 `P_MAX` 或同类电机窗口裁剪目标搜索距离。

核心输入：

- `MotorFeedback.position`
- 丝杆导程 `lead_screw_pitch_mm_per_rev`
- 电机方向和轴方向配置
- 零点配置

核心输出：

```cpp
struct AxisEncoderSnapshot {
  common::Mm axis_position;
  common::MmPerS axis_velocity;
  common::Rad motor_position;
  common::Rad zero_motor_position;
  common::Mm zero_axis_position;
  common::Ratio motor_delta_revolutions;
  common::Ratio millimeters_per_revolution_estimate;
  bool fresh;
};
```

约束：

- 回零前的坐标是临时坐标。
- 临时坐标不能被当成真实软件限位。
- `0-16mm` 这类应用历史理论值只能作为参考，不能在通用库中硬编码。

### 5.4 safety_envelope

职责：

- 对输出命令做限流、限速、限位置窗口和超时保护。
- 根据反馈电流、速度、温度、推进量判断是否需要停机。
- 区分命令电流上限和反馈硬电流上限。
- 区分可降级诊断和不可降级硬故障。
- 管理调试约束策略。

安全 envelope 至少包含：

```cpp
struct AxisSafetyConfig {
  common::A max_command_current;
  common::A feedback_hard_current_limit;
  common::A feedback_emergency_current_limit;
  common::MmPerS max_axis_speed;
  common::MmPerS2 max_axis_acceleration;
  common::DegC max_motor_temperature;
  common::S command_timeout;
};
```

必须保留的判据：

- 卡点不能只靠电流判断。
- 真实结构硬限位需要持续高电流、速度塌陷和无有效推进等组合条件。
- 有持续单调推进且电流峰值回落时，优先记录为摩擦异常候选。
- 端点停机后必须先卸载或失能，不能发送当前位置 hold 持续顶住端点。
- BoundaryRelease 失败后不得继续构造假双边界。

### 5.5 self_learning

职责：

- 执行 PreA 低能量可控性验证。
- 执行 PreB 近全行程低置信预探索。
- 生成低置信结构 profile。
- 执行多区域、多速度结构参数学习。
- 维护局部摩擦异常 map。
- 加载和保存 seed。

#### PreA

PreA 只证明当前启动周期至少存在可控运动方向，并得到 bootstrap 起动电流。它不能输出最终静摩擦、动摩擦或最低稳定速度。

输出：

```cpp
struct PreABootstrapResult {
  common::A opening_bootstrap_breakaway_current;
  common::A closing_bootstrap_breakaway_current;
  bool opening_movable;
  bool closing_movable;
  bool endpoint_start_escape_detected;
  AxisDirection escape_direction;
};
```

#### PreB

PreB 做低置信近全行程预探索。它从当前临时位置按相对距离扫描，不假设初始点在中位，不被旧理论行程夹断。

输出：

```cpp
struct PreBExplorationResult {
  AxisRange low_confidence_range;
  bool opening_boundary_candidate;
  bool closing_boundary_candidate;
  bool boundary_release_failed;
  bool mechanism_anomaly;
  std::vector<AxisTracePoint> current_trace;
};
```

#### StructureProfile

`StructureProfile` 是库的核心学习结果。

```cpp
struct DirectionalAxisProfile {
  common::A bootstrap_breakaway_current;
  common::Nm bootstrap_breakaway_torque;

  common::MmPerS minimum_stable_axis_speed;
  common::A static_friction_current;
  common::A dynamic_friction_current_average;
  common::A dynamic_friction_current_max;

  std::uint32_t bootstrap_sample_count;
  std::uint32_t static_friction_sample_count;
  std::uint32_t dynamic_friction_sample_count;
  AxisConfidence confidence;
};

struct StructureProfile {
  AxisConfidence confidence;
  DirectionalAxisProfile opening;
  DirectionalAxisProfile closing;
  FeedbackNoiseFloor noise_floor;
  TravelLimitProfile travel_limits;
  MotionHealthProfile motion_health;
  std::uint32_t valid_sample_count;
  std::uint32_t rejected_sample_count;
};
```

关键语义：

- `bootstrap_breakaway_current` 来自 PreA，可作为下一次起扫参考。
- `static_friction_current` 和 `dynamic_friction_current_*` 只能来自干净多区域样本。
- seed v3 以前的历史静摩擦字段不能自动恢复为最终模型，只能迁移到 bootstrap 参考。

#### FrictionAnomalyMap

职责：

- 表达位置相关的局部摩擦异常。
- 区分局部卡点和全局结构摩擦基值。
- 为 PreB、TravelLearning、MotionHealthCheck 和 Clamp 提供误判抑制输入。

数据结构：

```cpp
struct FrictionAnomalyRecord {
  common::Mm start_position;
  common::Mm end_position;
  common::Mm center_position;
  common::Rad peak_motor_position;
  AxisDirection direction;
  common::A baseline_current;
  common::A peak_current;
  common::Ratio current_excess_ratio;
  common::Mm width;
  std::uint32_t occurrence_count;
  AnomalySeverity severity;
  AnomalyConfirmationState confirmation_state;
};
```

约束：

- anomaly map 不直接放宽实时硬保护。
- 未闭合的持续高电流段不写入 anomaly map，应交给硬限位或堵转逻辑处理。
- 回零后必须按电机位置重基准到真实打开零点。

### 5.6 workflow_controller

职责：

- 提供通用线性轴业务流程。
- 编排 encoder、self_learning、safety_envelope 和 motor_adapter。
- 维护状态机。
- 对应用层提供稳定 API。

建议状态机：

```text
Disconnected
  -> Connected
  -> Enabled
  -> PreSelfChecking
  -> PreSelfCheckCompleted
  -> HomingOpenStop
  -> Homed
  -> TravelLearning
  -> TravelLimitsLearned
  -> MotionHealthChecking
  -> Ready
  -> Working

Any active state
  -> ActiveStop
  -> Fault
```

核心接口：

```cpp
class LinearAxisController {
 public:
  Result configure(const LinearAxisConfig& config);
  Result attachMotor(std::unique_ptr<MotorInterface> motor);

  Result connect();
  Result disconnect();
  Result enable();
  Result disable();

  Result runPreSelfCheck();
  Result homeOpenStop();
  Result learnTravelLimits();
  Result runMotionHealthCheck();

  Result moveToAxisPosition(const MoveAxisCommand& command);
  Result releaseFromStop(const ReleaseAxisCommand& command);
  Result stop();
  Result clearFault();

  AxisStateSnapshot state() const;
  void setEventSink(AxisEventSink* sink);
};
```

流程边界：

- `runPreSelfCheck()` 成功后只进入 `PreSelfCheckCompleted`。
- `homeOpenStop()` 必须依赖本次启动周期 PreSelfCheck。
- `learnTravelLimits()` 必须在 Homed 后执行。
- `runMotionHealthCheck()` 必须在软件限位建立后执行。
- `moveToAxisPosition()` 在 Ready 前只能使用低置信手动窗口，Ready 后使用软件限位。

### 5.7 persistence

职责：

- 保存和加载 `StructureProfileSeed`。
- 保存和加载 `FrictionAnomalyMap`。
- 执行版本迁移。
- 记录 seed 来源、时间、样机标识和配置摘要。

建议数据：

```cpp
struct StructureProfileSeed {
  std::uint32_t version;
  std::string axis_model;
  std::string hardware_fingerprint;
  StructureProfile profile;
  std::uint64_t saved_unix_time_ms;
};

struct AnomalyMapSeed {
  std::uint32_t version;
  std::vector<FrictionAnomalyRecord> records;
  std::uint64_t saved_unix_time_ms;
};
```

规则：

- seed 只作为下一次低能量起扫、诊断显示和搜索距离参考。
- seed 不能让系统跳过本次 PreSelfCheck、回零、行程学习或健康检查。
- 版本迁移必须保守，不能把旧污染字段自动升级成高置信控制参数。

### 5.8 diagnostics

职责：

- 记录结构化事件。
- 记录 PreB 电流-行程 trace。
- 提供 ring buffer 给 UI、测试、日志导出共用。
- 保留人工诊断所需的原始量和判定原因。

典型事件：

```cpp
struct AxisEvent {
  AxisEventType type;
  AxisState state;
  ErrorCode code;
  common::Timestamp timestamp;
  std::string message;
};

struct AxisTracePoint {
  common::Mm axis_position;
  common::A motor_current;
  common::MmPerS axis_speed;
  AxisDirection direction;
  std::uint32_t segment_id;
  TracePhase phase;
  common::Timestamp timestamp;
};
```

约束：

- UI 不应解析自由文本日志来获得控制结论。
- trace 分段必须保留 `segment_id`，避免不同方向或不同阶段的端点被连成误导性曲线。
- 大电流峰值不应在后端裁剪，显示层可缩放但不能隐藏原始数据。

### 5.9 application_binding

职责：

- 将通用线性轴能力转换为具体业务能力。
- 当前夹爪应用层负责夹爪角度、夹紧力、物体接触和 UI。

当前夹爪建议结构：

```text
gripper_application
  GripperController
    LinearAxisController axis
    GripperKinematics kinematics
    ForceMapper force_mapper
    ClampPolicy clamp_policy
```

夹爪层 API：

```cpp
class GripperController {
 public:
  Result runStartupWorkflow();
  Result clampByForce(const ClampForceCommand& command);
  Result clampBySpeed(const ClampSpeedCommand& command);
  Result release();
  GripperStateSnapshot state() const;
};
```

夹爪层不应直接：

- 解析 CAN 帧。
- 调达妙 SDK。
- 修改线性轴零点。
- 绕过线性轴安全 envelope 发送电机命令。

## 6. 配置模型

通用库配置建议：

```cpp
struct LinearAxisConfig {
  AxisMechanismConfig mechanism;
  AxisEncoderConfig encoder;
  AxisMotorLimitsConfig motor_limits;
  AxisSafetyConfig safety;
  AxisSelfCheckConfig self_check;
  AxisHomingConfig homing;
  AxisTravelLearningConfig travel_learning;
  AxisHealthCheckConfig health_check;
  AxisPersistenceConfig persistence;
};
```

### 6.1 mechanism

```cpp
struct AxisMechanismConfig {
  common::Mm lead_screw_pitch;
  common::Mm expected_travel;
  AxisDirection positive_axis_direction;
  common::Mm software_limit_margin;
};
```

### 6.2 self_check

```cpp
struct AxisSelfCheckConfig {
  common::A bootstrap_current_start;
  common::A bootstrap_current_stop;
  common::A bootstrap_current_step;
  common::Mm low_confidence_motion_distance;

  common::Mm pre_b_max_expansion_distance;
  common::MmPerS pre_b_expansion_speed;
  common::S pre_b_hard_current_confirm_time;

  common::Mm boundary_release_distance;
  common::MmPerS boundary_release_speed;
  common::A boundary_release_current_limit;

  std::uint32_t min_learning_regions;
  std::uint32_t learning_anchor_count;
  std::vector<common::MmPerS> dynamic_learning_speeds;
};
```

### 6.3 persistence

```cpp
struct AxisPersistenceConfig {
  std::string structure_profile_seed_path;
  std::string friction_anomaly_map_path;
  bool enable_profile_seed;
  bool enable_anomaly_map_seed;
};
```

当前 `GripperConfig` 可以逐步变为：

```cpp
struct GripperConfig {
  LinearAxisConfig axis;
  GripperKinematicsConfig mechanism;
  ClampConfig clamp;
  UiConfig ui;
};
```

## 7. 典型数据流

### 7.1 启动和连接

```text
Application
  -> LinearAxisController.configure()
  -> MotorAdapter.connect()
  -> MotorAdapter.readFeedback()
  -> LinearAxisEncoder.update()
  -> state = Connected
```

### 7.2 PreSelfCheck

```text
Application.runPreSelfCheck()
  -> workflow_controller checks preconditions
  -> self_learning loads seed
  -> PreA samples noise and bootstrap breakaway
  -> PreB scans relative expansion range
  -> safety_envelope monitors current/speed/progress
  -> friction_anomaly_detector records candidate peaks
  -> StructureProfile low-confidence result
  -> diagnostics emits trace/events
  -> state = PreSelfCheckCompleted
```

### 7.3 回零和行程学习

```text
homeOpenStop()
  -> relative opening search
  -> detect open hard stop
  -> disable/unload output
  -> encoder.resetZero(open_stop_motor_position, 0mm)
  -> anomaly_map.rebaseByMotorPosition()
  -> backoff from stop
  -> state = Homed

learnTravelLimits()
  -> closing relative search
  -> tolerate pass-through anomaly peaks
  -> detect closed hard stop
  -> compute measured travel and software limits
  -> save profile seed
  -> state = TravelLimitsLearned
```

### 7.4 健康检查和 Ready

```text
runMotionHealthCheck()
  -> move within software limits at multiple speeds
  -> collect velocity tracking, current ripple, torque ripple, temperature
  -> update anomaly detector
  -> save health summary
  -> state = Ready
```

### 7.5 应用层夹紧

```text
GripperController.clampByForce()
  -> ForceMapper maps target force to motor current/torque envelope
  -> GripperKinematics maps gripper speed to axis speed
  -> LinearAxisController.move/close with contact policy
  -> anomaly map suppresses known local peak misclassification
  -> safety envelope keeps hard protections active
```

## 8. 后续应用如何使用此库

### 8.1 新夹爪项目

接入步骤：

1. 实现或选择一个 `MotorInterface` adapter。
2. 配置丝杆导程、方向、预期行程、安全电流和速度。
3. 调用通用启动流程：

```text
connect
enable
runPreSelfCheck
homeOpenStop
learnTravelLimits
runMotionHealthCheck
```

4. 在应用层实现夹爪机构学和力映射。
5. 夹紧动作通过 `LinearAxisController` 的受控闭合接口执行。

### 8.2 小型线性滑台

滑台应用可以只使用通用库，不需要夹爪层：

```text
connect
runPreSelfCheck
homeOpenStop
learnTravelLimits
runMotionHealthCheck
moveToAxisPosition(target_mm)
```

此时 anomaly map 用于识别丝杆局部摩擦峰值，避免把局部卡点误判为端点。

### 8.3 电动推杆或压紧机构

应用层需要定义“压紧完成”的业务条件，但不能直接把单次电流峰值作为压紧完成。推荐使用：

- 通用库提供的位置、速度、电流、推进量组合判据。
- 应用层额外定义目标压力、保持时间或释放策略。

### 8.4 ROS 2 或上位机系统

ROS 2 适配层应只绑定 `LinearAxisController` 的公开 API：

```text
ROS node
  -> LinearAxisController
  -> MotorInterface adapter
```

ROS 层负责 topic/service/action，不负责直接发电机命令或改零点。

## 9. 当前项目迁移路线

### 阶段 1：CMake 目标拆分

新增库目标：

```text
axis_control_core
axis_control_simulated_adapter
axis_control_damiao_adapter
gripper_application
```

验收：

- `axis_control_core` 不链接达妙 SDK、不编译 UI、不依赖 app。
- 当前测试继续通过。

### 阶段 2：类型命名泛化

将可复用类型从 gripper 语义迁移到 axis 语义：

```text
NutPositionEncoder       -> LinearAxisEncoder
GripperStateSnapshot     -> AxisStateSnapshot + GripperStateSnapshot
StructureProfile         -> AxisStructureProfile
MoveNutStrokeCommand     -> MoveAxisPositionCommand
```

验收：

- 夹爪层可以通过别名或适配结构保持现有 UI/API 不立即破坏。

### 阶段 3：抽出 LinearAxisController

把当前 `GripperController` 中的通用流程抽到 `LinearAxisController`：

- PreSelfCheck。
- HomingOpenStop。
- TravelLearning。
- MotionHealthCheck。
- 手动线性定位。
- 安全 envelope。
- seed 和 anomaly map。

`GripperController` 保留：

- 夹爪力控。
- 夹爪速度模式。
- 夹爪角度估算。
- UI 视图模型适配。

### 阶段 4：持久化和诊断标准化

将 seed、anomaly map 和 trace 做成库级格式：

- 明确版本号。
- 明确迁移策略。
- 支持仿真测试读取固定 seed。
- UI 和测试共用结构化 ring buffer。

### 阶段 5：发布和示例

输出：

- 通用库 API 文档。
- 仿真 adapter 示例。
- 达妙 adapter 示例。
- 当前夹爪应用示例。
- 一个非夹爪线性轴示例。
- 安全参数调试指南。

## 10. 测试策略

### 10.1 纯算法单元测试

覆盖：

- 电机 rad 到 mm 映射。
- 零点重基准。
- P_MAX 裁剪。
- FrictionAnomalyDetector。
- StructureProfile seed 版本迁移。
- safety envelope 命令限幅。

### 10.2 仿真流程测试

覆盖：

- 中位启动。
- opening 端点启动逃逸。
- closing 端点启动逃逸。
- 局部卡点可穿越。
- BoundaryRelease 成功。
- BoundaryRelease 失败降级。
- P_MAX 窗口不足。
- anomaly map 重基准。

### 10.3 应用层回归测试

当前夹爪项目继续覆盖：

- PreSelfCheck。
- HomingOpenStop。
- TravelLearning。
- MotionHealthCheck。
- ClampByForce。
- ClampBySpeed。
- Release。
- UI/API 状态字段。

### 10.4 真实硬件测试

真实硬件测试必须继续分阶段执行：

1. 连接和反馈刷新。
2. 空载 bring-up。
3. 编码器映射验证。
4. PreSelfCheck。
5. 回零。
6. 行程学习。
7. 健康检查。
8. 应用业务动作。

硬件问题、参数调整和剩余风险归档到 `references/04_hardware/` 或对应实施文档，不写入本架构文件。

## 11. 不可丢失的安全约束

库化过程中必须保留以下约束：

- 命令电流上限和反馈硬电流阈值分离。
- PreA 只输出 bootstrap 起动参考。
- PreB 只输出低置信边界和结构学习候选，不直接建立最终软件限位。
- 历史 seed 不能让流程跳过本次启动周期自检。
- 回零前坐标是临时坐标。
- 端点命中后先卸载/失能，再采样和释放。
- BoundaryRelease 可以在单边界已知时执行，不要求双边 safe zone 已形成。
- BoundaryRelease 失败后强制降级，不继续第二方向假探边。
- 多区域学习必须至少覆盖配置要求的位置分离干净区域。
- anomaly map 不能单独授权忽略实时堵转。
- UI 颜色、显示缩放、曲线断线策略不能影响控制判据。

## 12. 第一版完成标准

第一版通用控制库可以认为完成，当满足：

- `axis_control_core` 可独立编译，不依赖达妙、UI、app。
- 仿真 adapter 能跑通完整启动流程。
- 当前夹爪应用通过 `LinearAxisController` 完成原有流程。
- 当前 `scripts/build.ps1` 和 `scripts/test.ps1` 通过。
- 至少有一个非夹爪线性轴示例。
- 文档说明应用层如何实现自己的硬件 adapter、配置、启动流程和业务扩展。

## 13. 与现有文件的关系

- 当前夹爪控制行为基线：`references/03_control/gripper_control_architecture_v2.md`
- PreSelfCheck 和丝杆自学习最终归档：`references/06_implementation/pre_self_check_final_archive_2026-05-26.md`
- 增量实施计划：`references/06_implementation/v2_incremental_implementation_plan.md`
- 编码和依赖规范：`references/07_standards/project_coding_standards.md`

当通用库架构与当前夹爪业务行为发生冲突时：

1. 先确认是否是库抽象边界描述不完整。
2. 不得直接削弱当前 V2 设计中的安全规则。
3. 如果确实需要改变控制流程、状态机或安全策略，必须先更新对应设计文件，再修改代码。
