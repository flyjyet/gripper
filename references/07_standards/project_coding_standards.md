# 项目编码与命名规范

## 1. 目标

本规范用于约束后续控制系统重构过程中的命名、单位、配置来源和模块依赖，避免代码实现过程中丢失物理含义或把参数硬编码到业务逻辑中。

核心原则：

- 数据命名必须表达物理含义。
- 物理量优先使用 `src/common/units.hpp` 中定义的单位类型。
- 参数默认值和可调参数来自 `config`，不在控制逻辑中直接写死。
- `common` 只放基础类型，不放机构参数、控制阈值和硬件配置。
- 控制器接口表达夹爪功能，不暴露达妙 SDK、CAN 帧或硬件私有模式。

## 2. 命名规则

### 2.1 类型命名

类型使用 PascalCase：

```cpp
GripperController
ClampForceCommand
StructureProfile
MotorFeedbackSummary
```

### 2.2 变量和字段命名

变量和结构体字段使用 snake_case。

字段名称应表达物理意义，而不是只表达数据类型：

```cpp
target_force
max_nut_speed
max_motor_current
release_distance
estimated_clamp_force
```

避免：

```cpp
value
data
param1
speed
force
```

如果上下文中存在多个速度，应明确是哪一个速度：

- `nut_speed`
- `motor_velocity`
- `gripper_angular_velocity`
- `release_nut_speed`

### 2.3 枚举命名

枚举类型使用 PascalCase，枚举值使用 PascalCase：

```cpp
enum class GripperTopState {
  Disconnected,
  PreSelfCheck,
  HomingOpenStop,
};
```

### 2.4 函数命名

函数使用 lowerCamelCase：

```cpp
runPreSelfCheck()
homeOpenStop()
learnTravelLimits()
clampByForce()
```

函数名应表达业务动作，而不是底层电机动作。

## 3. 单位规则

### 3.1 物理量必须使用单位类型

控制器公开接口、配置结构、状态快照、机构模型和安全阈值中的物理量，应优先使用 `src/common/units.hpp` 中的单位类型。

示例：

```cpp
common::N target_force;
common::MmPerS max_nut_speed;
common::A max_motor_current;
common::Nm max_motor_torque;
common::S timeout;
```

避免在接口层使用裸 `double`：

```cpp
double target_force_n;
double max_nut_speed_mm_s;
```

### 3.2 units.hpp 的职责

`src/common/units.hpp` 只定义轻量物理单位类型和别名，例如：

- `Mm`
- `MmPerS`
- `Rad`
- `RadPerS`
- `A`
- `Nm`
- `N`
- `S`
- `DegC`

`units.hpp` 不放配置默认值，不放机构参数，不放控制阈值。

### 3.3 新增单位

如果出现新的物理量，应先判断是否已有单位类型。

如果没有，应在 `units.hpp` 中新增明确单位，例如：

- 压力。
- 频率。
- 比例。
- 电压。

新增单位必须包含英文注释说明。

## 4. 参数来源规则

### 4.1 禁止在控制逻辑中硬编码参数

以下参数不得直接写在控制算法或状态机逻辑中：

- 机构行程。
- 丝杆导程。
- 电机 ID。
- 主机 ID。
- CAN/FDCAN 波特率。
- 电流限制。
- 速度限制。
- 加速度限制。
- 堵转阈值。
- 接触阈值。
- 回零速度。
- 回零电流。
- 自检扫描范围。
- 软件限位裕量。
- 夹紧目标力。
- 夹紧速度。
- 卸载距离。

这些参数应来自：

- `src/config/default_gripper.yaml`
- `src/config/gripper_config.hpp`
- `StructureProfile`

### 4.2 common 不放项目参数

`common` 只放全局基础类型和项目级元信息。

可以放：

- 项目名。
- 版本号。
- 错误码。
- Result。
- 单位类型。
- 时间戳。
- 日志事件基础类型。

不应放：

- 默认控制周期。
- 默认行程。
- 默认丝杆导程。
- 电机 ID。
- 电流阈值。
- 速度阈值。

### 4.3 StructureProfile 的职责

`StructureProfile` 保存自检和行程学习得到的结构参数，包括：

- 最低稳定运行速度。
- 静摩擦。
- 动摩擦。
- 噪声底。
- 初步限位。
- 安全区。
- 软件限位。
- 健康检查结果。

运行时控制应优先使用 `StructureProfile` 中已验证的参数；如果参数无效，必须使用配置中的保守默认值并降低状态有效等级。

## 5. 依赖规则

### 5.1 common

`common` 不依赖任何业务模块。

允许依赖：

- C++ 标准库。
- `common` 内部头文件。

禁止依赖：

- `controller`
- `hardware_interface`
- `ui`
- `commander`
- 达妙 SDK

### 5.2 controller

`controller` 可以依赖：

- `common`
- `config`
- `hardware_interface` 抽象接口。
- `controller` 内部模块。

`controller` 禁止依赖：

- `hardware_interface/damiao`
- 达妙 SDK。
- UI。
- commander。

### 5.3 ui 和 commander

UI 和命令行工具只能调用 `GripperController` 公开接口。

禁止：

- 直接编码 CAN 帧。
- 直接调用达妙 SDK。
- 绕过 controller 调硬件。

## 6. 注释规则

源码注释使用英文。

设计文档和需求文档使用中文。

公开接口注释必须说明：

- 功能目的。
- 前置条件。
- 成功后的状态变化。
- 失败后的状态变化。
- 是否可能进入 `ActiveStop` 或 `Fault`。
- 单位。

### 6.1 人工审查友好的注释要求

本项目存在机械安全、硬件通信、状态机和控制策略风险。代码不仅要能编译，也要便于人工审查者快速判断“这段代码想保证什么、什么情况下会失败、失败后是否安全”。

因此，以下位置必须有面向人工理解的英文注释：

- 模块级头文件顶部：
  - 说明本模块职责。
  - 说明本模块不负责什么。
  - 说明允许依赖和禁止依赖的边界。
- 公开类或核心类：
  - 说明类在系统中的角色。
  - 说明它是否直接访问硬件。
  - 说明它是否保存运行时状态。
- 状态机：
  - 每个顶层状态的业务含义必须能从注释或文档中看懂。
  - 关键事件必须说明触发条件。
  - 失败事件必须说明进入 `ActiveStop`、`Fault` 还是保持可恢复状态。
- 控制流程函数：
  - 例如 `runPreSelfCheck()`、`homeOpenStop()`、`learnTravelLimits()`、`runMotionHealthCheck()`、`clampByForce()`、`clampBySpeed()`、`release()`。
  - 必须说明流程阶段。
  - 必须说明使用哪些反馈量判断成功。
  - 必须说明失败时如何保证安全。
- 安全相关逻辑：
  - 限流、限速、限加速度、行程限制、接触检测、堵转检测必须说明阈值来源。
  - 必须说明是“裁剪命令”还是“触发主动停止”。
- 硬件占位或未实现代码：
  - 必须明确写出 `NotImplemented` 的原因。
  - 必须明确说明不能用于真实硬件联调。
  - 禁止写出看起来已经完成硬件控制但实际只是空实现的代码。
- 单位换算：
  - 电机侧角度/速度与螺母行程/速度换算处必须说明公式来源或配置来源。
  - 如果只是临时一阶近似，必须标明 approximation。

### 6.2 注释风格

注释应解释工程意图、风险边界和失败处理，而不是复述代码。

推荐：

```cpp
// Uses motor-side current and velocity drop as a contact proxy because the
// gripper has no external force or jaw-position sensor. A positive detection
// must stop motion and unload/disable torque to avoid lead-screw lockup.
```

避免：

```cpp
// Set current.
// Call function.
// If true, return true.
```

### 6.3 注释与文档的关系

源码注释负责帮助审查者读懂局部代码。

中文设计文档负责说明完整方案、流程图、状态机、测试计划和人工审查结论。

当控制流程、状态机或安全策略发生变化时，必须同步检查：

- 源码注释是否仍然准确。
- `references/03_control/control_architecture_design.md` 是否需要更新。
- `references/08_review/` 下的审查文档是否需要更新。

## 7. 配置和代码的边界

代码负责定义结构和行为。

配置负责提供可调参数。

自检负责提供运行时学习参数。

不要把这三类信息混在一起：

- 如果参数需要按样机、环境、线缆或机构版本调整，放配置。
- 如果参数需要运行时学习，放 `StructureProfile`。
- 如果是物理量的单位表达，放 `units.hpp`。
- 如果是控制流程，放 controller/state_machine。

## 8. 审查清单

每次代码落盘后检查：

- 是否出现裸 `double` 表达物理量。
- 字段名是否表达物理含义。
- 是否把参数硬编码进算法。
- 是否有参数应该来自 config 或 `StructureProfile`。
- `common` 是否被污染为参数仓库。
- `controller` 是否依赖达妙 SDK 或 UI。
- UI/commander 是否绕过 controller。
- 注释是否说明单位和前置条件。
- 关键控制流程是否有足够注释支撑人工审查。
- 未实现硬件路径是否明确标注，不会误导为可用于真实联调。
