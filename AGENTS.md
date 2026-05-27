# 项目级 AGENTS 规则

本文件是项目级工作入口，只维护“先读什么、按谁为准、变更如何同步”的规则。具体需求、设计、安全、编码、硬件和测试要求不在这里重复描述，统一以 `references/` 下对应文件为准。

## 1. 单一事实源

同一项要求只维护在一个权威文件中。其他文件如需涉及该要求，只引用权威文件路径，不复制细节。

如果某个权威文件体量过大、又不适合每次完整注入上下文，应优先在同一 `references/` 子目录下新增轻量索引或阅读指南，并让本文件引用该索引。轻量文件只做导航、章节定位和阅读顺序，不复制具体规则。

## 2. 工作前必读

开始任何代码、文档、测试、重构或硬件联调前，先按任务类型阅读对应文件：

- 总体阅读入口：`references/README.md`
- 控制架构 V2、状态机、虚拟螺母位置编码器、调试约束策略：`references/03_control/gripper_control_architecture_v2.md`
- 控制系统 V2 增量实施计划：`references/06_implementation/v2_incremental_implementation_plan.md`
- 项目编码、命名、单位、配置来源、依赖和注释规范：`references/07_standards/project_coding_standards.md`
- 子任务拆分和 sub-agent 执行要求：`references/06_implementation/sub_agent_execution_plan.md`
- 构建和测试要求：`references/05_build_test/current_build_requirements.md`
- UI 测试和操作说明：`references/07_ui/web_ui_test_guide_2026-05-20.md`
- 硬件、达妙 SDK、USB2FDCAN 和真实硬件问题记录：`references/04_hardware/`
- 人工审查范围和阅读路径：`references/08_review/human_design_review_pack.md`

用户明确指出某个文件、章节或已有方案时，必须先阅读该位置，再执行。

## 3. 设计先行

实现必须服从当前设计文件。若代码实现与设计方案冲突，或需要改变控制流程、状态机、安全策略、硬件模式、预自检、回零、行程学习、夹紧控制、管理员恢复等内容，必须先更新对应设计/计划文件，再修改代码。

主要设计文件：

- 控制设计：`references/03_control/gripper_control_architecture_v2.md`
- 实施计划：`references/06_implementation/v2_incremental_implementation_plan.md`
- 项目规范：`references/07_standards/project_coding_standards.md`

V1 控制设计和 V1 实施计划已归档到 `references/archive/`，仅作为历史参考，不作为后续新增实现的默认依据。

## 4. Bring-up 边界

`MotorBringupMode` 的边界以后续 V2 设计文件中的维护/调试章节为准。它只用于早期电机通信链路和空载调试，不作为 `PreSelfCheck`、回零、行程学习、夹紧控制或结构参数学习的实现依据。

## 5. PreSelfCheck 边界

`PreSelfCheck` 的后续方案以 `references/03_control/gripper_control_architecture_v2.md` 和对应阶段设计文件为准。实现、测试和问题分析前必须先阅读 V2 设计与实施计划。

## 6. 代码修改规则

代码修改必须遵守：

- `references/07_standards/project_coding_standards.md`
- 当前模块所在目录的既有风格和接口边界
- 用户当前轮次提出的最新约束

源码注释、配置来源、单位类型、模块依赖和人工审查友好性要求不在本文件重复维护。

## 7. 测试与归档

构建、测试和归档要求以以下文件和目录为准：

- `references/05_build_test/current_build_requirements.md`
- `references/06_implementation/`
- `references/04_hardware/`
- `references/07_ui/`
- `references/08_review/`

硬件相关问题、测试现象、修复结论和剩余风险必须归档到对应 `references/` 子目录，不写入本文件。

## 8. 中断和纠偏

如果用户指出实现方向与设计边界不一致，应立即停止当前实现路径，回到对应 `references/` 文件重新确认，再继续执行。
