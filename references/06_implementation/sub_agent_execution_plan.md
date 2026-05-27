# Sub Agent 协作落盘计划

## 1. 协作原则

重构落盘可以使用 sub agent 并行执行，但必须由主线程保留架构裁决权和集成权。

核心原则：

- 主线程负责控制器公开接口、总体架构、依赖方向和最终集成。
- 每个 sub agent 只负责一个边界明确的模块。
- 不允许多个 sub agent 同时修改同一批文件。
- 不允许 sub agent 自行调整架构目录、状态机边界或控制器公开接口。
- 每个 sub agent 必须拿到完整任务包，避免上下文缺失导致需求丢失。
- 代码审查使用独立只读 sub agent，不参与实现。

## 2. 推荐执行阶段

### 阶段 0：主线程建立接口锚点

主线程先完成：

- `src/controller/gripper_controller.hpp`
- `src/controller/gripper_types.hpp`

目的：

- 固定系统对外功能目标。
- 避免后续底层实现偏离需求。
- 给 UI、commander、self_check、safety、hardware_interface 提供统一目标。

### 阶段 1：底层模块并行落盘

可并行分配给 sub agent：

- Agent A：`common`
- Agent B：`hardware_interface`
- Agent C：`config`
- Agent D：`controller/self_check`
- Agent E：`controller/safety`、`controller/mechanism`、`controller/calibration`

每个 agent 只写自己负责的目录。

### 阶段 2：主线程集成

主线程负责：

- 统一命名。
- 统一 include 路径。
- 检查接口依赖方向。
- 修正跨模块类型冲突。
- 保证 controller 不依赖达妙 SDK。

### 阶段 3：只读代码审查

使用独立 code review sub agent，只读审查：

- 是否丢失功能需求。
- 是否违反架构依赖方向。
- 是否存在重复或冲突的数据结构。
- 状态机是否覆盖 PreSelfCheck、回零、行程学习、健康检查、夹紧、释放、主动停止、故障。
- 是否存在硬件实现污染控制器接口的问题。

## 3. Sub Agent 任务包模板

每个任务包应包含以下内容：

```text
任务名称：

背景：
- 本项目是直流伺服电机 + 梯形丝杆 + 连杆夹爪。
- 当前没有外部传感器，仅有电机反馈。
- 控制重点是限流、限速、自检、低电流回零、行程学习、目标力夹紧、恒速夹紧、主动停止和防丝杆螺母抱死。

必须阅读：
- references/03_control/control_architecture_design.md
- references/06_implementation/controller_interface_first_implementation_plan.md
- 与本模块相关的已有头文件。

允许修改：
- 明确列出目录或文件。

禁止修改：
- src/controller/gripper_controller.hpp
- src/controller/gripper_types.hpp
- references/*
- 非本任务负责目录。

实现要求：
- 只写本模块接口、数据结构、注释或最小实现。
- 不引入达妙 SDK，除非任务明确属于 hardware_interface/damiao。
- 不改变目录结构。
- 不引入 UI 依赖。

验收标准：
- 头文件可独立 include。
- 依赖方向符合架构文档。
- 注释说明单位、前置条件、失败条件。
- 不与其他模块重复定义核心类型。
```

## 4. Agent 分工建议

### Agent A：common

负责范围：

- `src/common/result.hpp`
- `src/common/error_code.hpp`
- `src/common/timestamp.hpp`
- `src/common/units.hpp`
- `src/common/logging.hpp`
- `src/common/project_defs.hpp`

禁止：

- 依赖 controller。
- 依赖 hardware_interface。
- 依赖 UI。

### Agent B：hardware_interface

负责范围：

- `src/hardware_interface/motor_types.hpp`
- `src/hardware_interface/motor_interface.hpp`
- `src/hardware_interface/can_frame.hpp`
- `src/hardware_interface/adapter_types.hpp`
- `src/hardware_interface/transport_interface.hpp`
- `src/hardware_interface/simulated/*`

禁止：

- 依赖 controller。
- 直接写达妙 SDK 逻辑。

### Agent C：config

负责范围：

- `src/config/gripper_config.hpp`
- `src/config/config_loader.hpp`
- `src/config/default_gripper.yaml`

要求：

- 配置字段覆盖 adapter、motor、mechanism、self_check、safety、homing、clamp、ui。
- 配置单位清晰。

### Agent D：controller/self_check

负责范围：

- `src/controller/self_check/*`

要求：

- `StructureProfile` 覆盖最低稳定速度、摩擦、噪声底、初步限位、安全区、软件限位、健康检查等参数。
- `PreSelfCheck` 流程按 LimitedProbe -> BidirectionalMoveEnable -> StableShortStrokeMotion -> PreliminaryLimitSearch -> TheoryTravelCheck -> SafeZoneBuild -> MultiRegionRoundTripLearning -> StructureProfileUpdate。

禁止：

- 自行修改顶层控制器公开接口。

### Agent E：controller/safety + mechanism + calibration

负责范围：

- `src/controller/safety/*`
- `src/controller/mechanism/*`
- `src/controller/calibration/*`

要求：

- safety 使用 `StructureProfile` 和配置做限流、限速、行程限制、接触/堵转判断。
- mechanism 只做机构运动学和行程映射。
- calibration 只做夹紧力映射，不做初始化自检。

### Agent F：只读代码审查

负责范围：

- 全仓库只读。

输出：

- 按严重程度列出问题。
- 标注文件和行号。
- 优先审查需求丢失、依赖方向错误、接口冲突、状态机缺口。

禁止：

- 修改文件。
- 重构代码。

## 5. 信息保真要求

每个 sub agent 必须获得以下不可省略信息：

- 机构无外部传感器，只有电机反馈。
- 真实硬件路径是 DM-J4310P-2EC + DM-USB2FDCAN_Dual。
- 控制器接口不能暴露达妙 SDK、CAN 帧和硬件私有模式。
- `PreSelfCheck` 先建立保守结构参数，再允许回零和行程学习。
- 软件限位不能与机械限位重合。
- 夹紧完成后不能持续保持力矩，应受控卸载或失能，避免丝杆螺母抱死。
- 方向切换反向间隙无法可靠学习，不作为 `StructureProfile` 核心参数。

## 6. 技能和工具

当前阶段不需要额外安装技能。

后续如果进入编译、静态检查和格式化阶段，可考虑：

- CMake。
- clang-format。
- clang-tidy。
- cppcheck。

这些工具应在需要实际检查时再安装或启用，避免当前阶段引入额外环境复杂度。

## 7. 主线程集成检查清单

每轮 sub agent 返回后，主线程检查：

- 是否只修改了授权文件。
- 是否违反依赖方向。
- 是否重复定义已有类型。
- 是否遗漏单位注释。
- 是否引入达妙 SDK 到 controller。
- 是否改变了公开控制器接口。
- 是否满足本模块验收标准。
- 是否需要同步更新 `references` 文档。
