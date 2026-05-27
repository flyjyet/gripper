# 夹爪控制系统 V2 设计方案

## 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v2.0 | 2026-05-22 | 建立完整 V2 控制系统设计方案。本文件是后续控制系统设计、实现和测试的唯一出口。 |
| v2.1 | 2026-05-22 | 补充 `MotorBringupMode` 下按电机圈数执行相对位置到位的空载调试能力，用于验证电机多圈反馈与虚拟螺母行程映射。 |
| v2.2 | 2026-05-22 | 补充达妙运行时 `P_MAX/VMAX/TMAX` UI 诊断显示和 Bring-up 圈数目标越界拒绝规则。 |
| v2.3 | 2026-05-22 | 修正 Bring-up 圈数到位的默认超时链路和反馈电流中止策略：缺省 `timeout=0` 统一表示由控制器自动估算；反馈电流普通超限采用持续超限判据，全局硬保护仍立即停机。 |
| v2.4 | 2026-05-22 | 补充 Web UI 的只读配置参数分页，以及 Bring-up 操作区贴近显示达妙运行时 `P_MAX/VMAX/TMAX` 和当前多圈位置。 |
| v2.5 | 2026-05-22 | 补充 PreSelfCheck V2 第一版实施边界：只执行低能量可控性和短行程低置信验证，暂不主动执行完整理论边界搜索和多区域往复学习。 |
| v2.6 | 2026-05-22 | 补充虚拟螺母位置编码器独立方向配置 `nut_position_encoder.direction_sign`，用于根据 Bring-up 实测单独反转螺母位置反馈方向。 |
| v2.7 | 2026-05-22 | 补充梯形丝杆局部摩擦异常映射设计：先实现独立 `FrictionAnomalyMap/Detector`，为后续 PreB、行程学习和健康检查区分局部卡点与真实结构限位提供输入。 |
| v2.8 | 2026-05-22 | 补充 PreSelfCheck V2 当前最终版边界：PreA 低能量可控性验证后执行 PreB 分段扩展、安全区往复和摩擦异常候选采样，但仍不把结果等同于最终软件限位。 |
| v2.9 | 2026-05-22 | 明确 PreB 不能反向否定已通过的 PreA：PreB 扩展或安全区往复无法取得足够区域时降级为 PreA 保守窗口；反馈矛盾、主动停止、硬安全事件、硬件/配置错误仍失败。 |
| v2.10 | 2026-05-22 | 将 PreB 调整为理论护栏内的近全行程低置信探测：允许从当前位置连续扫描到理论开/闭边界附近，保留 partial bounds，并在低置信安全区内执行多区域、多速度参数采样。 |
| v2.11 | 2026-05-22 | 补充 PreB 局部卡点复核策略：疑似软堵转先受控小步穿越确认，能继续推进则视为摩擦异常候选，不能推进或触发自检硬反馈上限才收为低置信边界。 |
| v2.12 | 2026-05-25 | 将 PreB 扩展/回位速度从正式行程学习速度中解耦，默认提高到可穿越局部卡点的 `3.0mm/s`；PreA 低能量起动扫描仍保持从低速开始。 |
| v2.13 | 2026-05-25 | 补充 PreB 实机扩展修正：普通超时但已产生有效 partial progress 时继续扩展；主扩展段把软 jam/contact 作为诊断信号，只有硬反馈电流、无有效推进、方向矛盾、理论护栏或达妙 `P_MAX` 窗口不足才收边界，并要求日志输出分段决策和电机侧余量。 |
| v2.14 | 2026-05-25 | 修正 PreB 对反馈硬电流的判定：主扩展和安全区学习对短时反馈电流尖峰只记录诊断，只有超过 `pre_b_hard_current_confirm_time_s` 的持续硬电流才中止并收边界，避免把启动瞬态误判为结构限位。 |
| v2.15 | 2026-05-25 | 明确 PreB 物理边界与局部卡点的区分：探边必须按 `pre_b_boundary_step_mm` 小步推进；短时电流尖峰只能记录为卡点/摩擦候选，物理边界必须满足速度塌陷、无有效推进和持续电流超限等时间判据；PreSelfCheck 的初步摩擦 map 以电机位置锚定，回零后重算到当前零点坐标供后续行程学习参考。 |
| v2.16 | 2026-05-25 | 调整 PreB 实机默认档位：探边步长默认提高到 `1.0mm`，主扩展/回位/软卡点复核速度默认提高到 `5.0mm/s`，软卡点复核距离默认提高到 `1.0mm`，用于穿越更大的局部卡点。 |
| v2.17 | 2026-05-25 | 修正 PreSelfCheck 后低置信窗口暴露策略：PreB 已验证并写入 profile 的 safe zone 应作为完整低置信手动定位窗口暴露，不再额外缩成当前位置附近的小窗口；最终软件限位仍等待后续回零和行程学习。 |
| v2.18 | 2026-05-25 | 将 PreB 主探测从逐步小步扩展调整为 bring-up-like 连续受控扫描：每个方向优先一次连续扫描到理论护栏/配置护栏附近，过程中持续记录摩擦异常候选；只有持续硬电流、速度塌陷、无有效推进、方向矛盾、理论护栏或达妙 `P_MAX` 限制才收为低置信边界，边界候选再用小步/软卡点复核确认。 |
| v2.19 | 2026-05-25 | 根据连续扫描实机日志下调 PreB 默认速度：`5.0mm/s` 在当前 `2.0mm/rev` 丝杆上约 `15.7rad/s`，会在卡点处触发持续硬电流；连续扫描默认改为 `1.2mm/s`，约 `3.77rad/s`，动态采样最高档降到 `1.5mm/s`。 |
| v2.20 | 2026-05-25 | 现场继续下调 PreB 连续扫描/软卡点复核默认速度到 `0.6mm/s`，约 `1.88rad/s`，用于验证低速连续扫描是否能穿越卡点并扩大低置信范围。 |
| v2.21 | 2026-05-25 | 补充 PreB 探边后的学习入口约束：若连续探边命中物理边界，必须先按配置主动反向回退释放边界抱死，再进入多区域参数学习；多速度学习不得在已知或本轮新发现的摩擦异常附近取样，避免把局部卡点电流写入结构基值。 |
| v2.22 | 2026-05-25 | 修正 PreB partial bounds 规则：连续扫描即使最终因持续硬电流/疑似物理边界停止，只要停止前已经产生停稳、单调、方向正确的有效推进，也必须把推进后的停稳位置写入对应低置信边界；摩擦异常避让默认只阻断达到配置严重度阈值的卡点，避免轻微候选把全部学习锚点过滤掉。 |
| v2.23 | 2026-05-25 | 修正 PreB 回零前坐标语义：预自检初始 `7.98mm` 只是虚拟螺母临时参考，不代表物理行程中位；PreB 主探边必须按起点相对距离扩展到物理边界/P_MAX/安全停止，不能被临时 `0-16mm` 理论坐标提前截断。 |
| v2.24 | 2026-05-25 | 同步修正回零入口：PreSelfCheck 允许临时坐标越过 `0/16mm` 后，`HomingOpenStop` 也必须按 opening 相对搜索真实开端，不能把临时 `0mm` 当成回零目标。 |
| v2.25 | 2026-05-25 | 补充回零端部命中后的释放策略：`HomingOpenStop` 检测 opening 物理端部后必须立即卸载输出并采样端部反馈，设置真实打开零点后按 `homing.backoff_distance_mm` 主动向 closing 方向回退释放抱死，不能在端部继续使用普通运动停稳等待。 |
| v2.26 | 2026-05-25 | 修正局部卡点与结构限位判据：PreB、行程学习和夹紧不得把有持续单调位移的高电流卡点直接当作结构限位或夹紧接触；结构限位必须满足高电流/速度塌陷叠加持续无有效推进。PreB 一侧疑似限位或卡点停机后必须释放并继续另一侧探测。 |
| v2.27 | 2026-05-25 | 落实 PreB/TravelLearning/Clamp 的卡点容忍实现要求：自检硬电流确认绑定噪声级无推进；卡点停机但有有效推进时按 partial progress 处理而非物理边界；行程学习和夹紧接触判定必须加入持续无推进确认。 |
| v2.28 | 2026-05-25 | 修正 PreB 连续扫描执行语义：单次远目标 probe 因卡点/普通超时中断但已有有效推进时，不代表该方向扫描结束，必须保留 partial bound 后继续扫描剩余相对距离；若一侧疑似物理边界或卡点抱死，切换方向或学习前必须主动释放。 |
| v2.29 | 2026-05-26 | 修正初始停在结构限位时的 PreA/PreB 交接：PreA 只需证明存在可控退开方向，不得在端点附近反复要求双向短行程；进入 PreB 后必须先沿可退开方向长距离探测另一侧边界，再回头确认起始侧边界。 |
| v2.30 | 2026-05-26 | 根据实机卡点日志提高现场默认档位：PreB 连续扫描/边界释放/软卡点复核默认提高到 `1.0mm/s`，TravelLearning 默认提高到 `1.2mm/s` / `1.2A`，并要求行程学习输出详细运动诊断。 |
| v2.31 | 2026-05-26 | 修正实际行程假设：`0-16mm` 只作为旧理论参考和日志对照，不再作为 PreB/TravelLearning 的硬搜索终点；新增 `self_check.travel_learning_search_distance_mm`，默认按 `20mm` 闭合搜索，PreB 单向扩展默认同步到 `20mm`，达妙运行时 `P_MAX` 仍是最终电机位置窗口约束。 |
| v2.32 | 2026-05-26 | 根据现场确认，样机真实丝杆行程可能为 `18-20mm`，上位机已将达妙 `P_MAX` 调到约 `65rad`；TravelLearning 闭合目标必须按配置搜索距离和 P_MAX 动态裁剪，不能再由 `theoretical_close_limit_mm=16` 或旧软件守卫提前截断，默认行程学习电流提高到 `1.5A` 以通过已知卡点。 |
| v2.33 | 2026-05-26 | 完善 PreSelfCheck 后续三段闭环：`HomingOpenStop` 明确要求 PreSelfCheck 已完成；`TravelLearning` 和 `MotionHealthCheck` 成功后必须刷新运行时学习 seed；seed 保存静/动摩擦、低置信/软件限位和健康摘要，并恢复样本计数，使下一次 PreSelfCheck 可从已学习电流附近起扫。 |
| v2.34 | 2026-05-26 | 落实运行时 seed 合并语义：PreSelfCheck 加载 seed 后，最终 profile 更新不得被 `SelfCheckManager` 输出覆盖掉历史样本计数和已学电流；`TravelLearning` 与 `MotionHealthCheck` 的往复运动继续向 `FrictionAnomalyDetector` 送入反馈样本，用于延续卡点候选 map。 |
| v2.35 | 2026-05-26 | 明确 PreA 起动扫描只生成 bootstrap breakaway seed，不得作为最终静摩擦/最低稳定速度模型；最终结构模型由避开端点和卡点的多区域、多速度样本统计得到，安全电流上限继续由 `safety` 配置提供。 |
| v2.36 | 2026-05-26 | 明确 PreB 主探边、回到学习锚点和边界释放动作只更新低置信边界、bootstrap/anomaly 诊断，不写入最终静摩擦、动摩擦或最低稳定速度统计；seed v3 以前的最终摩擦/速度字段全部视为历史污染字段，只迁移起动电流到 bootstrap。 |
| v2.37 | 2026-05-26 | 明确 PreB 多区域学习验收：必须至少取得 `self_check.pre_b_min_learning_regions` 个位置分离的有效干净区域，默认 `3` 个；候选锚点数量由 `pre_b_learning_anchor_count` 提供，未达到最小区域数时该阶段降级且回滚本阶段样本，不把局部少量样本写成最终结构基值。 |
| v2.38 | 2026-05-26 | 修正 PreB 多区域学习与诊断：每个必需区域只能在本区域内寻找干净锚点，禁止跨区迁移导致学习集中；若严重卡点覆盖导致无法满足区域数，必须上报 `mechanism_anomaly` 诊断。PreB 预探索反馈应在 UI 中显示电流-行程曲线，摩擦异常默认只记录明显电流尖峰。回零、行程学习和健康检查必须优先使用本轮/seed 学习出的结构参数并在日志中说明电流来源。 |
| v2.39 | 2026-05-26 | 修正 PreB 探测与学习采样边界：从 PreB 入口当前位置先向 opening 结构限位探测的路径只更新低置信边界和安全诊断，不写入摩擦异常 map 或 UI 电流-行程 trace；从 closing 结构限位回 opening 软限位准备多区域学习时必须使用连续受控运动，禁用 detector/trace，避免分段启停尖峰污染后续选点。 |
| v2.40 | 2026-05-26 | 修正 PreB 电流-行程 trace 的 UI 表达：曲线必须按连续采样段绘制，不得把不同方向、不同阶段或位置跳变后的端点直接连线；坐标轴应显示网格和刻度，异常反馈峰值保持可见用于人工判断安全停机是否及时。 |
| v2.41 | 2026-05-26 | 细化 PreB trace 数据边界：后端为每个连续 probe 分配 `segment_id`，UI 必须优先按该字段断线；从 closing 限位连续回 opening 软限位的准备动作改为仅记录 UI trace，不进入 anomaly map 或结构模型学习样本。 |
| v2.42 | 2026-05-26 | 增加 PreSelfCheck 专用极端反馈电流急停：普通自检硬反馈阈值仍按持续时间过滤卡点，超过 `safety.self_check_feedback_emergency_current_limit_a` 则立即停止，避免物理端点顶死时出现十安培级峰值。 |
| v2.43 | 2026-05-26 | 修正 PreSelfCheck 端点停机语义：PreB 内部因持续硬反馈电流或 emergency 电流中止时，必须立即卸载/失能采样，不得再发送当前位置 hold；该事件是自检探边结果，不直接升级为全局 `ActiveStop`，释放失败也应降级继续保留低置信边界。 |
| v2.44 | 2026-05-26 | 修正 opening 端点启动识别：PreA/StableShortStroke 中的双向低置信微动不能证明双向可用；一侧小幅可控退开、反向持续硬电流/无推进时必须标记反向为起始结构限位，让 PreB 先沿退开方向长距离探测。 |
| v2.45 | 2026-05-26 | 修正 PreB 边界释放失败保护：从远处撞到结构端点后，若反向 `BoundaryRelease` 未成功解除抱死状态，不得继续把紧邻端点的反向小位移高电流 stop 收为另一侧结构限位，也不得基于该假双边界进入多区域学习；本轮 PreB 应降级为单侧边界诊断和 PreA 保守窗口。 |
| v2.46 | 2026-05-26 | 修正单边界启动/撞端后的释放前置条件：`BoundaryRelease` 不得要求已有双边 low-confidence safe zone；只知道一个端点时，应围绕当前停留点构造本地反向释放窗口，先退出端点抱死，再继续另一方向 PreB 主扫描。若本地释放仍失败，最终 profile 必须强制覆盖为 PreA 小窗口，不能被 SelfCheckManager 理论边界或历史 seed 放大。 |
| v2.47 | 2026-05-26 | 补充 PreB 电流-行程 trace 可视化要求：UI 应允许操作者在曲线下方设置打开、闭合和未知/诊断曲线颜色，颜色仅影响本地显示和人工判读，不改变 trace 数据、控制判据或安全策略。 |
| v2.48 | 2026-05-26 | 分离 PreB 普通探边和边界释放的电流 envelope：普通 PreSelfCheck emergency 反馈急停默认下调到 `3.0A`，`BoundaryRelease` 新增 `self_check.pre_b_boundary_release_current_limit_a`，默认 `2.0A` 并仍受 `safety.max_motor_current_a` 裁剪。 |

## 1. 设计目标

本夹爪机构采用直流伺服电机、梯形丝杆、螺母和连杆机构实现夹爪开合。系统除电机自身反馈外，没有外部力传感器、夹爪角度传感器、螺母独立位置传感器或接触传感器。控制系统必须基于电机反馈、丝杆导程、机构模型、学习参数和安全策略完成可调试、可验证、可逐步推进的控制。

V2 设计目标如下：

- 建立以“虚拟螺母位置编码器”为基础的控制架构，统一后续自检、回零、行程学习、健康检查和夹紧控制的位置来源。
- 保留状态机驱动的控制方式，使每个流程阶段有明确状态、入口、退出条件和失败原因。
- 支持默认安全控制：限流、限速、软件行程限制、主动停止、通信超时和硬保护默认开启。
- 支持调试推进：用户在调试阶段可以主动关闭部分控制器约束，但全局硬保护不可关闭。
- 支持真实硬件分阶段调试：通信链路、反馈链路、虚拟螺母位置、预自检、回零、行程学习、健康检查和夹紧控制逐步实现、逐步测试。
- 工作完成后电机默认失能，不持续输出力矩，避免丝杆螺母抱死。
- UI、命令行、控制器、硬件抽象和达妙实现解耦，UI 和命令行只能调用控制器公开接口。

## 2. 总体架构

```text
                         +----------------------+
                         |        app           |
                         | 主程序 / 对象装配       |
                         +----------+-----------+
                                    |
                   +----------------+----------------+
                   |                                 |
          +--------v---------+              +--------v---------+
          |        ui        |              |     commander    |
          | PC 测试界面        |              | 命令行调试入口      |
          +--------+---------+              +--------+---------+
                   |                                 |
                   +----------------+----------------+
                                    |
                         +----------v-----------+
                         |     controller       |
                         | 夹爪业务控制核心        |
                         +----------+-----------+
                                    |
       +----------------------------+-----------------------------+
       |                            |                             |
+------v------+          +----------v---------+        +----------v---------+
| state_machine|          | safety/debug       |        | self_check        |
| 流程状态       |          | 安全与调试约束策略     |        | 自检与参数学习       |
+------+------+          +----------+---------+        +----------+---------+
       |                            |                             |
       +----------------------------+-----------------------------+
                                    |
                         +----------v-----------+
                         | nut_position_encoder |
                         | 虚拟螺母位置编码器      |
                         +----------+-----------+
                                    |
                         +----------v-----------+
                         | hardware_interface  |
                         | 电机与通信抽象          |
                         +----------+-----------+
                                    |
             +----------------------+----------------------+
             |                                             |
   +---------v----------+                       +----------v---------+
   | Damiao motor       |                       | simulated motor    |
   | 达妙电机真实硬件       |                       | 仿真电机            |
   +--------------------+                       +--------------------+
```

核心原则：

- 控制器使用螺母位置和夹爪业务单位，不直接使用 CAN 帧或达妙私有协议。
- 达妙原始反馈用于诊断、日志和 UI 显示；控制用位置来自虚拟螺母位置编码器。
- 状态机控制流程边界；安全与调试约束策略控制命令是否允许、是否限幅、是否主动停止。
- 每个长流程必须可取消，取消后不继续刷新新命令。

## 3. 源码目录规划

```text
src/
  app/
    main.cpp
    application.hpp
    application.cpp

  common/
    error_code.hpp
    result.hpp
    timestamp.hpp
    units.hpp
    logging.hpp

  config/
    gripper_config.hpp
    config_loader.hpp
    config_loader.cpp
    default_gripper.yaml

  utils/
    math_utils.hpp
    geometry_utils.hpp
    filter_utils.hpp
    interpolation.hpp

  hardware_interface/
    motor_interface.hpp
    motor_types.hpp
    transport_interface.hpp
    can_frame.hpp
    adapter_types.hpp
    damiao/
    simulated/

  controller/
    gripper_controller.hpp
    gripper_types.hpp
    nut_position_encoder/
    state_machine/
    safety/
    self_check/
    mechanism/
    calibration/

  ui/
  commander/
  third_party/
```

说明：

- 第三方库、达妙 SDK 和 DLL 放在 `third_party/` 或明确的硬件实现目录下。
- 项目自有工具放在 `utils/`。
- 单位类型、结果类型、错误码和通用定义放在 `common/`。
- 业务数据结构优先放在对应模块目录；跨模块公共业务类型放在 `controller/gripper_types.hpp`。

## 4. 模块职责

### 4.1 app

负责加载配置、创建具体硬件实现、创建控制器、装配 UI 和 commander。`app` 不包含控制算法。

### 4.2 common

提供错误码、结果类型、时间戳、物理单位和日志基础类型。所有带物理意义的数值必须使用单位类型或带单位字段名。

### 4.3 config

保存系统配置，包括电机、机构、安全、调试约束、自检、UI 和硬件适配器配置。控制参数不得散落在业务代码中硬编码。

### 4.4 hardware_interface

定义电机和通信抽象。硬件层负责：

- 连接/断开设备。
- 使能/失能电机。
- 发送电机命令。
- 读取最新电机反馈。
- 暴露原始诊断字段。

硬件层不负责夹爪状态机、螺母零点、夹紧力控制或结构参数学习。

### 4.5 controller

控制器是业务核心，负责：

- 状态机调度。
- 调用虚拟螺母位置编码器。
- 执行自检、回零、行程学习、健康检查、夹紧和释放。
- 应用安全与调试约束策略。
- 对 UI/commander 提供稳定公开接口。

### 4.6 nut_position_encoder

将电机多圈位置映射为螺母位置，是 V2 的运动位置基础设施。详见第 6 章。

### 4.7 state_machine

维护顶层流程状态和允许的状态转换。状态机不直接访问硬件。

### 4.8 safety

实现限流、限速、软件限位、主动停止、堵转/接触检测、通信超时和调试约束策略。

### 4.9 self_check

实现预自检、摩擦/速度/噪声/初步限位等低置信参数识别。自检结果写入结构参数 profile，但低置信结果不能直接等同于最终软件限位。

局部摩擦异常检测属于 `self_check` 的独立学习输入。检测器只消费恒速或近似恒速运动中的螺母位置、电机位置、电机电流、速度、温度和方向样本，不直接访问硬件，不发送运动命令，不写 `StructureProfile` 软件限位。后续由 PreB、`TravelLearning` 和 `MotionHealthCheck` 决定采样覆盖范围和确认策略。

### 4.10 mechanism / calibration

`mechanism` 负责螺母位置与夹爪角度、夹爪速度之间的运动学映射。`calibration` 负责电流、力矩和目标夹紧力之间的标定映射。

### 4.11 ui / commander

UI 和命令行只调用控制器接口，不直接发送电机命令和 CAN 帧。UI 必须显示状态、约束、反馈、日志和失败原因。

## 5. 关键数据结构

### 5.1 MotorFeedback

电机反馈至少包含：

- 电机多圈位置 `motor_position_rad`。
- 电机速度 `motor_velocity_rad_s`。
- 电机电流 `motor_current_a`。
- 电机力矩 `motor_torque_nm`。
- 电机温度 `motor_temperature_deg_c`。
- 原始协议位置、原始计数、帧时间戳和故障标志。

### 5.2 NutPositionFeedback

虚拟螺母位置编码器输出：

- `nut_position_mm`
- `nut_velocity_mm_s`
- `zero_motor_position_rad`
- `zero_nut_position_mm`
- `motor_position_rad`
- `motor_delta_rad`
- `motor_delta_rev`
- `mm_per_rev_estimate`
- `fresh`
- `last_update_timestamp`

### 5.3 StructureProfile

结构参数 profile 保存当前启动周期或学习周期得到的参数：

- 预自检质量等级。
- PreA bootstrap 起动电流和样本数，只用于本次或下次自检起扫。
- 最低稳定螺母速度，必须带有效运动样本计数。
- 打开/闭合方向静摩擦电流。
- 打开/闭合方向动摩擦电流。
- 反馈噪声底。
- 初步低置信安全区。
- 回零后的打开零点。
- 行程学习后的软件开/闭限位。
- 运动健康检查结果。
- 参数有效时间和温度。

运行时学习 seed 是 `StructureProfile` 的低置信启动参考。控制器可在 PreSelfCheck 开始前从 `self_check.learned_profile_path` 加载上一轮成功学习的 bootstrap 起动电流、静摩擦、动摩擦、低置信边界、软件限位和运动健康摘要，用于调高下一轮低能量起扫电流、提供搜索距离参考和显示历史基线；加载 seed 不得直接把系统置为 `Homed`、`TravelLimitsLearned` 或 `MotionHealthChecked`，所有安全状态仍必须由本次启动周期重新完成。PreSelfCheck 最终生成本轮 profile 时必须把加载的 seed 样本计数、电流/力矩基值与本轮 runtime bootstrap/dynamic 样本合并回来，避免被纯识别输出覆盖，导致下一次又从过低电流起扫。

seed 版本 `3` 起，`opening_breakaway_bootstrap_*` / `closing_breakaway_bootstrap_*` 与 `static_friction_*` 分开保存。旧版本 seed 中的 `static_friction_*` 只作为历史起动电流迁移到 bootstrap 字段，旧版本 seed 中的 `dynamic_friction_*` 和 `minimum_stable_nut_speed_*` 不再自动作为最终模型恢复。任何没有有效样本计数的静摩擦、动摩擦或最低稳定速度字段都只能显示为未学习，不能用安全电流上限或配置 fallback 伪造学习值。

### 5.4 FrictionAnomalyMap

`FrictionAnomalyMap` 是独立于 `StructureProfile` 的位置-摩擦异常记录。它描述丝杆局部毛刺、磨损、润滑不均或螺纹表面缺陷导致的电流局部峰值，不描述机构几何参数和软件限位。

单条记录至少包含：

- 异常区起止螺母位置、中心位置和宽度，单位 `mm`。
- 峰值处电机位置，单位 `rad`。
- 通过方向：打开方向和闭合方向独立记录。
- 当前速度下的局部基线电流、峰值电流和峰值/基线比例。
- 出现次数、确认状态和严重度：`minor`、`moderate`、`severe`。
- 采样速度、采样温度和更新时间戳。

边界：

- 第一版只做内存候选记录，不持久化，不做前馈补偿；候选记录必须保留峰值电机位置作为跨零点重基准锚点。
- PreSelfCheck 期间生成的初步 map 使用当前临时零点坐标展示；`HomingOpenStop` 设置真实打开零点后，控制器应按记录中的 `motor_position_rad` 重算 `position_mm/start/end/center`，使这些候选能作为后续 `TravelLearning` 和 `MotionHealthCheck` 的参考输入。
- 查到已知异常不能单独授权继续推进，必须同时满足“位置仍可继续变化、峰值通过后电流回落、速度未持续塌陷”等判据。
- 未闭合的电流持续升高段不能记录为摩擦异常候选，应继续按真实限位或堵转风险处理。

### 5.5 DebugConstraintPolicy

调试约束策略包含：

- 控制器限流是否启用。
- 控制器限速是否启用。
- 主动停止是否启用。
- 软件行程限制是否启用。
- 通信超时保护是否启用。
- 全局硬保护是否启用。
- 风险确认状态。
- 当前模式：普通、空载调试、管理员恢复。

### 5.6 GripperStateSnapshot

控制器快照供 UI 和 commander 读取，至少包含：

- 顶层状态。
- 当前流程阶段。
- 电机反馈摘要。
- 虚拟螺母位置反馈。
- 夹爪角度估计。
- 学习参数摘要。
- 调试约束状态。
- 是否允许执行各类按钮动作。
- 最近一次错误和故障信息。

## 6. 虚拟螺母位置编码器

V2 的核心位置抽象是虚拟螺母位置编码器，不是电机单圈到多圈编码器。达妙电机反馈提供电机侧多圈位置；控制系统需要将该位置映射为螺母位置。

### 6.1 输入与输出

输入：

- 电机多圈位置，单位 `rad`。
- 电机速度、电流、力矩、温度和时间戳。
- 配置参数：丝杆导程、方向符号、反馈过期时间。

方向配置：

- `motor.direction_sign` 表示机构侧螺母行程与电机侧位置之间的基础换算方向。
- `nut_position_encoder.direction_sign` 是虚拟螺母位置编码器的独立修正方向，用于 Bring-up 实测发现虚拟螺母计数方向与实际相反时单独反转。
- 控制器使用的有效螺母方向为 `motor.direction_sign * nut_position_encoder.direction_sign`。
- `position_command_sign` 只用于达妙位置命令下发极性，不用于修正虚拟螺母反馈方向。

输出：

- 螺母位置，单位 `mm`。
- 螺母速度，单位 `mm/s`。
- 原始电机反馈快照。
- 数据新鲜度和更新时间。

### 6.2 零点策略

启动时：

- 读取当前电机多圈位置。
- 将当前电机位置作为临时零点。
- 将当前螺母位置设为配置的启动参考值，默认 `0mm`。

回零后：

- 使用回零停稳后的电机多圈位置重新设置零点。
- 将螺母位置设为理论打开零点或回零定义的零点。

零点设置必须写日志，包含旧零点、新零点、来源流程和时间戳。

### 6.3 映射公式

```text
motor_delta_rad = motor_position_rad - zero_motor_position_rad
motor_delta_rev = motor_delta_rad / (2*pi)
nut_position_mm = zero_nut_position_mm
                + direction_sign * motor_delta_rev
                * lead_screw_pitch_mm_per_rev
nut_velocity_mm_s = direction_sign * motor_velocity_rad_s / (2*pi)
                  * lead_screw_pitch_mm_per_rev
```

默认导程：

```text
lead_screw_pitch_mm_per_rev = 2.0
```

### 6.4 更新频率与实时性

默认更新频率先采用 `50Hz`。如果硬件反馈稳定、队列无积压且测试确认收益明显，可以调整为 `100Hz`。

更新要求：

- 使用最新电机反馈，不使用旧队列积压帧。
- 反馈时间戳超过配置过期时间时，编码器输出 `fresh=false`。
- 任何使用螺母位置的控制动作必须检查编码器新鲜度。

### 6.5 验收标准

空载调试时：

- 电机位置变化约 `2*pi rad`，螺母位置变化应接近 `2mm`。
- 日志输出 `motor_delta_rad`、`motor_delta_rev`、`nut_delta_mm`、`mm_per_rev_estimate`。
- UI 同时显示电机多圈位置、虚拟螺母位置和原始反馈字段。

## 7. 控制器公开接口

控制器接口按夹爪业务组织，不按电机协议组织。

建议公开能力：

```cpp
configure(config)
connect()
disconnect()
enable()
disable()
runHardwareSanityCheck()
initializeNutPositionEncoder()
runPreSelfCheck()
homeOpenStop()
learnTravelLimits()
runMotionHealthCheck()
clampByForce(command)
clampBySpeed(command)
release()
stop()
clearFault()
setDebugConstraintPolicy(policy)
enterMotorBringupMode(request)
exitMotorBringupMode()
enterAdminRecovery(request)
exitAdminRecovery()
state()
```

约束：

- 控制器接口不暴露 CAN 帧。
- 控制器接口不暴露达妙 SDK 对象。
- 维护和管理员接口必须与普通夹紧/释放接口分离。
- 调试约束修改必须经控制器接口完成并写日志。

## 8. 状态机设计

### 8.1 顶层状态

```text
Disconnected
Connected
HardwareSanityCheck
NutEncoderReady
PreSelfCheck
HomingOpenStop
TravelLearning
MotionHealthCheck
Ready
Clamping
Releasing
MotorBringupMode
AdminRecoveryMode
ActiveStop
Fault
Disabled
```

### 8.2 启动主线

```text
Disconnected
  -> Connected
  -> HardwareSanityCheck
  -> NutEncoderReady
  -> PreSelfCheck
  -> HomingOpenStop
  -> TravelLearning
  -> MotionHealthCheck
  -> Ready
```

### 8.3 长流程取消

所有长流程必须支持取消。取消来源：

- 用户点击停止。
- `stop()`。
- `disable()`。
- 断开连接。
- 通信严重异常。
- 硬件故障。

取消后：

- 停止刷新当前命令。
- 尽可能发送零输出或失能。
- 不再为了继续流程重新使能。
- 进入 `ActiveStop` 或 `Fault`。
- 日志记录取消上下文。

### 8.4 ActiveStop 恢复

`ActiveStop` 是主动停止状态，不是反馈停止状态。进入 `ActiveStop` 后，UI 和控制器仍应继续读取反馈并更新显示。

恢复要求：

- 用户明确清除主动停止。
- 电机输出已失能或零输出。
- 最近反馈无硬件故障。
- 恢复后不得直接假定 `Ready`，必须根据已完成阶段回到合适状态。

## 9. 启动与自检流程

### 9.1 HardwareSanityCheck

目的：

- 确认设备连接。
- 确认反馈持续刷新。
- 确认使能/失能链路。
- 确认电机多圈位置字段可用。

该阶段不学习结构参数，不设置夹爪零点。

### 9.2 NutEncoderReady

目的：

- 建立虚拟螺母位置编码器临时零点。
- 验证螺母位置能随电机多圈位置更新。
- 将虚拟螺母位置作为后续流程输入。

### 9.3 PreSelfCheck

目的：

- 低能量试探电机与机构是否可控。
- 识别 PreA bootstrap 起动电流、反馈噪声和初步可控性。
- 在后续干净运动段中逐步识别最低稳定速度、静摩擦、动摩擦和摩擦异常。
- 初步发现低置信安全区。

PreSelfCheck 输出低置信 `StructureProfile`，不能直接作为最终软件限位和业务夹紧依据。

V2 当前最终版实施边界：

- 使用虚拟螺母位置编码器作为唯一位置反馈输入。
- PreA 执行静止噪声采样、双向低能量起动扫描和稳定短行程低置信验证。
- PreA 的低能量起动扫描只生成 bootstrap breakaway seed，用于让当前流程和下次自检从更合理的电流附近开始。若启动点就在结构限位或局部卡点，PreA 电流会混入端部顶死或卡点峰值，因此不得写入最终静摩擦、动摩擦或最低稳定速度模型，也不得用于业务夹紧接触判断。
- 每段 probe 必须在停止或低电流保持后读取新鲜反馈并确认停稳，未停稳样本只用于诊断，不进入 profile 统计。
- PreB 在回零前使用的是临时虚拟螺母坐标。初始 `provisionalReferenceStroke()` 的中点值只提供可读参考，不代表丝杆物理行程中位；因此某一方向的剩余物理行程可能大于 `8mm`。主探边必须按当前位置的相对距离连续扩展，最大单方向扩展距离来自 `self_check.pre_b_max_expansion_distance_mm`，直到物理边界候选、达妙 `P_MAX`、安全停止或配置距离耗尽；不能因为临时坐标到达理论 `0/16mm` 就提前停止。
- 如果 PreSelfCheck 从真实结构端点附近启动，PreA 的低能量/短行程阶段不能把“双向都能完成短行程”作为进入 PreB 的硬前置条件。只要一侧小范围 probe 能产生停稳、单调、方向正确的可控退开，另一侧表现为低位移/持续电流/速度塌陷的疑似端点，PreA 应记录起始侧疑似边界并立即交给 PreB 连续探边；不得在端点附近反复开闭短行程直到超时。端点逃逸判定的“可控退开”阈值应使用低置信起动/噪声级有效位移，而不是 PreB 主扩展的有效推进阈值；端点附近 `0.1mm` 量级退开伴随硬电流停机也可作为退开候选，但只能用于起始边界调度和 bootstrap，不能写入最终摩擦/最低稳定速度模型。
- PreB 主探边期间可使用临时自检行程窗口 `current +/- max(pre_b_max_expansion_distance_mm, theoretical_travel)` 作为 safety limiter 的 stroke guard，同时保留全局电流、速度、加速度、用户停止和达妙位置窗口保护。低置信 safe zone/profile 允许暂时出现小于理论 open 或大于理论 close 的临时坐标；后续 `HomingOpenStop` 和 `TravelLearning` 才把真实打开零点和实测闭合端重基准。旧 `0-16mm` 只能作为理论参考，不得作为真实行程硬边界。
- 若相对扫描目标超过达妙运行时 `P_MAX`，PreB 应把单段目标裁到当前方向上 `P_MAX` 内的最大可达距离并记录 `pmax_scan_distance_limited`，只有连一个有效安全距离都没有时才把当前位置收为 P_MAX 边界；不能因为旧 `16mm` 理论参考行程或默认 P_MAX fallback 不足就完全不扫描。当前现场样机可把达妙上位机 `P_MAX` 调到约 `65rad`，真实硬件连接后仍必须以寄存器读取到的运行时 `P_MAX` 为准。
- PreB 控制循环必须持续刷新目标命令、反馈电流、速度、位置和摩擦异常检测。默认目标是覆盖接近当前预期实际行程的低置信区域；当实测显示结构行程可能达到 `18-20mm` 时，`pre_b_max_expansion_distance_mm` 应配置到相同量级，而不是要求回零前坐标值必须落在 `0.3-15.8mm`。
- PreB 不要求从丝杆中位开始。第一方向探测结束后，第二方向可以直接从当前已到达位置继续穿越已验证路径并向另一侧扩展，避免因当前位置靠近端部而必须先回到原始起点。
- 若 PreA 已标记某一侧为起始疑似边界，PreB 的第一主扫描方向应优先选择相反方向，即先离开起始端点并寻找另一侧结构边界；另一侧边界确认后再释放并沿反方向回到起始端，确认起始侧边界。一个 PreSelfCheck 周期必须尝试 opening 和 closing 两个方向的边界探测；除用户主动停止、硬件故障、反馈矛盾、达妙位置窗口无可用空间或不可降级安全事件外，单侧疑似边界不得终止整个 PreB。
- PreB 入口若第一方向为 opening，则从当前停留点到 opening 结构限位这一段只做探测：允许更新 opening 低置信边界、限位样本和安全诊断，但不得向 `FrictionAnomalyDetector`、初步 anomaly map 或 UI 电流-行程 trace 写入样本，也不得进入最终摩擦/速度统计。该段包含未知起点、端点释放和可能的启动瞬态，不适合作为卡点 map 或学习选点依据。
- PreB 探测过程中，若单个 probe 因普通超时但已经产生停稳、单调、方向正确的有效位移，应保留该 partial motion 作为低置信样本和边界进展，不应直接丢弃已走过区域。
- PreB 主扩展段在理论护栏和自检硬反馈电流上限内运行。单帧软 jam/contact 判据只作为诊断和摩擦异常候选输入，不应单独终止本段；若本段最终产生有效 partial progress，可更新扩展边界并继续下一段。短时电流尖峰不能单独收为物理边界；只有持续硬反馈电流确认、方向矛盾、无有效推进并伴随速度持续塌陷/电流不回落、理论护栏或电机运行时位置窗口不足时，才收为低置信边界或降级。
- PreB 遇到非理论边界附近的 `jam/contact` 软判定时，不能第一次就把当前位置判为结构限位；应先执行受控小步穿越复核。复核能继续产生单调位移并在硬反馈电流以下完成或部分完成时，按“局部卡点/摩擦异常候选”继续扩展；复核无有效位移、方向矛盾、反馈电流超过自检硬上限或电流持续升高不回落时，才把上一稳定位置收为低置信边界。
- PreB 的扩展搜索、回到采样锚点和软卡点穿越复核使用独立的 `self_check.pre_b_expansion_speed_mm_s` / `self_check.pre_b_soft_jam_retry_speed_mm_s`，不再借用正式 `TravelLearning` 速度。连续扫描默认 `1.0mm/s`，在 `2.0mm/rev` 丝杆上约等于 `3.14rad/s` 电机速度，用于在不接近早期 `5.0mm/s` 高速冲击档的前提下提高穿越局部卡点的动量；安全区多速度采样允许使用不超过全局螺母限速的速度档位，默认最高 `1.5mm/s`。
- PreB 的命令电流上限和反馈硬停上限分离：命令电流仍由 `safety.self_check_current_limit_a` 限制；反馈硬停可使用仅限 PreSelfCheck 的 `safety.self_check_feedback_hard_current_limit_a`，用于容忍达妙反馈电流的短峰值。该自检反馈硬停不能放宽其他流程的全局硬保护。
- PreB 主扩展、回到采样锚点和多区域采样对反馈硬停采用两级判据：单帧或短时超过 `safety.self_check_feedback_hard_current_limit_a` 只记录 `hard_current_transient`、峰值和持续时间；只有连续超过 `self_check.pre_b_hard_current_confirm_time_s` 且高电流期间无噪声级有效推进，才判为 `SafetyJamDetected` 并收为低置信边界。该持续时间是卡点和物理边界区分的关键配置，必须短于真正顶死后的风险时间、长于现场常见局部卡点尖峰时间；默认值按现场卡点尖峰调整为约 `0.25s` 量级。若反馈电流超过 `safety.self_check_feedback_emergency_current_limit_a`，必须立即停止当前 probe 并记录 `emergency_current_limit`，不再等待持续确认；该阈值仅用于限制物理端点顶死时的极端峰值，不能用于放行更高命令电流。普通 PreB 探边默认 emergency 反馈急停应低于早期 `4A` 档，现场默认约 `3.0A`，以减少端点顶死后的高峰值。触发持续硬电流或 emergency 电流后的停机收尾必须优先卸载/失能并等待新反馈停稳，不得再用当前位置 hold 命令继续压住端点。PreA 低能量起动和短行程验证不使用该放宽策略，仍保持更严格的硬电流响应。
- PreB 的持续硬电流确认必须与位置推进状态绑定。若反馈电流超过 `safety.self_check_feedback_hard_current_limit_a`，但螺母仍按命令方向产生连续单调位移，当前段应记录为局部摩擦/卡点候选并继续扫描；只有高电流持续期间位置没有超过噪声级有效推进，或同时出现速度塌陷/方向矛盾/无进度超时，才可停止并收为结构限位候选。低速扫描时用于判断“有推进”的阈值应接近反馈噪声和 `motion_start_distance`，不能直接使用较大的 `max_distance_error`，否则 `0.6mm/s` 等低速会被误判为无推进。
- PreB 分段日志必须包含方向、步号、当前/目标螺母位置、目标电机位置、单段电机增量、实际电机增量、实际螺母位移、mm/rev 估算、判定结果、边界收缩原因，以及达妙运行时 `P_MAX` 可用余量。若 `P_MAX` 窗口不足以覆盖理论 `0-16mm`，日志应明确提示这是电机位置窗口限制，不是结构限位学习结果。
- PreB 构建低置信安全区后执行多区域、多速度往返采样，用于补充静摩擦、动摩擦、最低稳定速度和局部摩擦异常候选；统计聚合结果描述结构基值，摩擦异常 map 描述离散卡点。静/动摩擦样本必须来自已停稳、方向正确、未触发限位/堵转、且未与严重 anomaly 重叠的干净运动段。PreB 主探边、软卡点复核、回到学习锚点和边界释放动作只允许更新 `expansion_bounds`、limit/anomaly 诊断或 bootstrap，不得进入 `motion_samples`、`static_friction_samples` 或 `dynamic_friction_samples`。
- PreB 若在开向或闭向探边时命中物理边界候选，或因局部卡点高电流中止但已经保留有效 partial progress，切换方向或进入多区域、多速度采样前必须先执行主动释放：从当前停留点沿反方向回退 `self_check.pre_b_boundary_release_distance_mm`，速度使用 `self_check.pre_b_boundary_release_speed_mm_s`，释放命令电流使用独立的 `self_check.pre_b_boundary_release_current_limit_a`，并仍受 `safety.max_motor_current_a`、反馈硬电流持续确认、方向一致性和临时护栏约束。释放电流允许略高于普通 PreB 探边命令电流，用于解除端点/卡点抱死，但不能提高普通探边的反馈急停阈值，也不能绕过全局硬保护。释放动作只用于解除端部或卡点抱死状态，不作为结构参数样本，也不写入 anomaly map 或 PreB trace。释放不要求已经形成双边 low-confidence safe zone；当只探到单侧端点时，控制器应围绕当前停留点和反向释放目标构造本地临时护栏，避免因 safe zone 尚未有效而直接跳过另一方向主扫描。若释放 probe 只退开了部分距离但已经达到噪声级有效推进，应允许在剩余 release 距离内继续分段释放，直到达到配置释放距离或再次出现无有效推进的端点抱死。若释放因同类 PreB 端点反馈保护失败且没有足够有效退开，应记录 `boundary_release_failed`/`stuck_at_boundary` 诊断并降级：保留已探测的单侧边界和 PreA 保守窗口，但不得继续把紧邻该端点的反向小位移高电流 stop 收为另一侧结构限位，不得把单侧边界扩展成有效 low-confidence safe zone，也不得进入多区域、多速度学习。该降级 profile 必须显式覆盖 preliminary/safe/software limits 到 PreA 小窗口，不能被 SelfCheckManager 理论 fallback 或历史 seed 恢复出的旧安全区放大。PreB 内部的结构端点命中、持续硬反馈电流和 emergency 电流属于探边诊断结果，不等同于用户停止、通信故障或全局硬安全错误；控制器不得因此直接把顶层状态置为 `ActiveStop`。
- 达妙 `POS_FORCE` 的电流字段是相对 `motor.max_phase_current_a` 的 per-unit 限幅，反馈电流又由反馈扭矩和 `motor.torque_per_amp_nm_per_a` 换算得到。若 `max_phase_current_a`、运行时 `TMAX` 或扭矩/电流换算与实机不匹配，日志中的反馈电流可能显著高于命令限流；此类现象必须通过日志同时对照 `current_limit_a`、`max_current_a`、`last_torque_nm`、`runtime_tmax_nm` 和电机上位机参数确认，不能只在控制层提高反馈急停阈值掩盖。
- 从 closing 结构限位回到 opening 软限位以准备多区域学习时，必须采用与主探边一致的单次连续受控运动到 `safe_zone.open_limit`，不得按 `pre_b_boundary_step_mm` 分解成多段启停 probe。该回位只用于进入学习起点，`FrictionAnomalyDetector` 和结构模型样本入口必须关闭；但允许以 `ui_trace_only` 方式记录 PreB 电流-行程曲线，供人工确认从闭合侧回打开侧的连续路径，不得把该回位样本写入 anomaly map、`motion_samples`、`static_friction_samples` 或 `dynamic_friction_samples`。若连续回位未到达目标或出现持续无进展堵转，应降级/失败，不能把中途位置当作干净学习锚点。
- PreB 的“连续扫描”是按方向累计完成配置的相对扩展距离，而不是只发送一次远目标命令。若单次 probe 因普通超时或非物理边界卡点停下，但停下前已经产生方向正确、单调、停稳的有效推进，应更新低置信边界、记录摩擦候选，然后从当前停稳位置继续扫描该方向剩余距离。只有 P_MAX/配置距离耗尽、真正物理边界候选、方向矛盾、主动停止或无有效推进降级，才结束该方向。
- 多区域、多速度采样必须避开局部摩擦异常候选。锚点和计划采样段如果与达到 `self_check.friction_anomaly_learning_avoid_ratio` 严重度阈值的 `FrictionAnomalyDetector` 已闭合记录重叠，或距离小于 `self_check.friction_anomaly_avoid_margin_mm`，应跳过或选择附近干净候选；probe 完成后如果检测器新增的严重 anomaly 与本段重叠，该段不得进入 `motion_samples` / `dynamic_friction_samples`。轻微单次候选先保留为诊断，不阻断所有结构基值学习。这样结构基值由干净区域的多速度统计得到，离散卡点由 `FrictionAnomalyMap` 表达。
- 多区域学习不是“尝试 3 个锚点”即可通过，而是必须至少获得 `self_check.pre_b_min_learning_regions` 个位置分离的有效学习区域，默认 `3` 个。实现应使用 `self_check.pre_b_learning_anchor_count` 个候选锚点在 safe zone 内分布搜索，记录每个锚点的接受/跳过原因；只有某个区域至少接受一个干净 motion/static/dynamic 样本，才计入有效区域。若有效区域数不足，`MultiRegionRoundTripLearning` 降级并回滚本阶段已暂存样本，PreSelfCheck 可继续保留低置信边界，但最终静摩擦、动摩擦和最低稳定速度不得由不足区域的局部样本生成。
- 多区域学习的锚点搜索必须是区域局部搜索：先把 low-confidence safe zone 均匀划分为不少于 `self_check.pre_b_min_learning_regions` 个区域，每个必需区域只允许在该区域内部按 margin/sample distance 寻找干净 anchor。候选锚点可多于区域数，但不能把中部或闭合侧区域的锚点迁移到打开侧干净区域；否则会把学习结果集中在局部，错误表达全行程结构基值。
- 若某个必需区域没有足够 room，或被达到 `friction_anomaly_learning_avoid_ratio` 的严重 anomaly 覆盖到无法放置完整往返采样段，应记录 `region_skipped reason=friction_anomaly_coverage` 或 `reason=insufficient_region_room`。若最终有效区域数小于要求，必须输出 `mechanism_anomaly=true` 的诊断，提示疑似丝杆局部摩擦异常过多或机构状态异常；低置信边界仍可保留用于后续人工判断，但该阶段样本必须回滚。
- 摩擦异常 map 默认应偏向“明显电流尖峰”，避免把 1.5x 左右的普通波动填满 `max_records` 并过度影响学习区域选择。`friction_anomaly_current_ratio_threshold` 和 `friction_anomaly_minor_ratio` 默认不应低于约 `2.0x`；`friction_anomaly_learning_avoid_ratio` 继续只让较严重 anomaly 阻断结构基值学习。
- UI/API 必须暴露 PreB 预探索期间的电流-行程 trace，至少包含临时行程 `stroke_mm`、反馈电流绝对值 `current_a`、速度方向、阶段标签和后端生成的连续采样段 `segment_id`。曲线用于人工判断卡点，不能替代控制器的安全判据；trace 需要限长采样，防止长时间自检导致 UI 响应变差。UI 绘制时必须优先按 `segment_id` 拆分连续段，并保留方向变化、阶段变化和位置大跳变作为兼容旧数据的断线兜底；不同采样段的点只能重新起笔，不能直接连成跨越左右极限的斜线。坐标轴需要显示 mm/A 网格和刻度，超过硬反馈阈值的峰值不得被显示层裁剪隐藏。
- PreB 电流-行程 trace 的颜色属于本地显示偏好。UI 可在曲线下方提供打开方向、闭合方向和未知/诊断曲线的颜色选择，并允许浏览器本地持久化；该设置不得写入控制配置，不得影响后端 trace、摩擦异常 map 或任何安全判据。
- PreB 成功取得最终结构样本后，后续 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck` 的电流选择应优先参考已学静/动摩擦和最低稳定速度，再受 `safety` 中的流程电流上限裁剪。回零属于碰撞找开端，默认仍使用小电流或由学习摩擦加小裕量得到的小电流，不得退回 PreA/PreB 的 `self_check_current_limit=1.9A`。日志必须输出 `current_source`，区分 `learned_structure_profile`、`homing_config` 和 `travel_config_cap`。
- PreSelfCheck 期间启用 `FrictionAnomalyMap/Detector` 采样，记录局部电流峰值候选。PreB 可使用实时“位置继续推进、电流未持续越过硬上限、峰值后可停稳”的复核结果避免把局部卡点直接收为边界；内存 anomaly map 本身仍只作为诊断和后续流程参考，不单独授权继续推进。候选 map 在回零后必须按电机位置重基准到新的打开零点坐标，避免临时零点导致后续行程学习误用位置。
- 理论开闭边界在回零前不代表当前物理绝对位置，只能作为行程长度、日志参考和后续重基准目标；PreB 扩展边界和疑似限位只能生成临时坐标下的低置信 profile。
- 如果 PreB 因当前位置靠近端部、可扩展宽度不足、低置信微动或普通超时无法完成扩展/往复采样，应优先保留已停稳、单调、方向正确的 partial bounds；只有没有足够 partial bounds 时才降级为 PreA 保守窗口并继续完成 `PreSelfCheckCompleted`。此降级不能扩大安全区，只能保持或缩小当前低置信范围。
- PreB 成功建立低置信 safe zone 后，PreSelfCheck 后允许的手动螺母定位可使用该完整低置信 safe zone；若只完成 PreA 或 PreB 降级为保守小窗口，则手动定位仍只允许围绕当前虚拟螺母位置的小窗口。完整软件限位范围仍由 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck` 后续确认。

PreB 安全边界：

- PreB 的目标不是要求机构从丝杆行程中位开始，而是在 PreA 已建立的低置信安全窗口内逐步扩展覆盖区域。
- 当目标是近全行程预建模时，`pre_b_max_expansion_distance_mm` 可配置为接近当前预期实际行程长度；这表示“从当前停留点向某一侧最多尝试扩展这么多距离”，不是“目标夹到临时 0/16mm 坐标”。不能因为 PreB 得到接近预期行程长度的低置信宽度就把 profile 升级为最终软件限位。
- PreA 的 `min_speed_scan_start/stop` 和历史命名的 `static_friction_current_start/stop/step` 只用于低能量 bootstrap 起动扫描；最终最低稳定速度和静/动摩擦必须来自 PreB/TravelLearning/MotionHealthCheck 中避开端点和卡点的有效样本。PreB 近全行程扩展应优先使用独立的 `pre_b_expansion_speed_mm_s`。如果现场发现 `0.6mm/s` 仍不能稳定穿越已知卡点，应先结合电流、温度和机械状态评估后再逐步微调，不能通过关闭硬保护绕过风险。
- 扩展必须单方向连续扫描、可停止、可回退；`pre_b_boundary_step_mm` 保留为边界候选后的 refine/软卡点确认步长，不再限制主扫描覆盖距离。达到相对扩展上限、达妙 `P_MAX`、无有效推进或触发持续硬反馈保护时立即停止扩展，不假定另一方向还有对称空间。
- PreB 使用 `FrictionAnomalyMap/Detector` 收集局部电流峰值候选，用于后续区分“局部卡点”和“真实结构限位”。局部卡点的实时穿越复核只允许在自检硬反馈电流上限内进行，且不能绕过方向矛盾、理论护栏和用户主动停止。
- 若反馈电流只短时超过自检硬反馈上限，但位置仍单调推进并在确认时间内回落，PreB 可保留该段 partial progress 继续扩展，并把该位置段交给摩擦异常检测器作为候选；若电流持续超过确认时间、速度持续塌陷、位置不再推进或方向矛盾，仍按限位/堵转风险处理。
- 若连续扫描最终因持续硬电流或疑似物理边界停止，但停止前已经产生停稳、单调、方向正确的有效推进，应先把该停稳位置写入对应 `expansion_bounds`，再把该位置作为低置信边界候选；不能因为最终结果是 `SafetyJamDetected` 就把边界回退到扫描起点。
- 若局部电流峰值后位置仍继续变化且电流回落，可记录为候选异常；若电流持续升高、位置停止或速度持续塌陷，仍按限位/堵转风险处理。
- 多速度学习采用成熟摩擦辨识中的速度分层思路：低速档覆盖库仑/静摩擦附近，中速档观察粘性摩擦斜率，异常点用位置相关 map 从全局结构基值统计中剔除。第一版采用稳健的样本筛选和分档统计，不在 PreSelfCheck 内拟合高阶 Stribeck/LuGre 模型；后续可在 `TravelLearning` 全行程数据足够后再做更完整模型。
- 反馈方向矛盾、理论行程外、用户主动停止、全局硬保护、硬件故障、通信/反馈错误和配置错误不可降级，必须进入失败或主动停止路径。

### 9.4 HomingOpenStop

目的：

- 使用预自检得到的保守速度和电流执行低速低电流靠零。
- 在打开方向堵转/限位停稳后设置真实零点。
- 调用虚拟螺母位置编码器重新设零。
- `HomingOpenStop` 只能在本次启动周期 `PreSelfCheck` 成功后执行；历史 seed 只能影响自检起扫和诊断参考，不能替代本次预自检。
- PreSelfCheck 之后的临时螺母坐标可能小于理论 open 或大于理论 close；回零不能以临时 `0mm` 作为绝对目标。`HomingOpenStop` 应从当前位置沿 opening 方向执行相对搜索，搜索距离至少覆盖理论行程长度或本轮 PreB 低置信宽度，受达妙 `P_MAX`、电流、速度、用户停止和持续堵转判据约束；在 opening 端停稳后再将该电机位置设为真实打开零点。
- 回零端部检测不能只依赖通用 `ContactJamDetector` 的“相对动态摩擦电流上升”判据。低电流靠零时，物理端部可能表现为反馈电流贴近 `homing.homing_current_a`、速度塌陷且位置不再有效推进，但电流上升量未超过通用 jam 阈值；该状态持续超过 `homing.jam_confirm_time_s` 时应作为 opening stop 候选。
- opening stop 候选成立后，控制器必须立即停止/失能输出并用最近端部反馈作为零点采样，避免继续向物理端部施加保持力。此处不使用普通运动的长时间停稳等待，因为丝杆端部顶死时停稳判据可能导致用户看到“卡死后无响应”。
- 设置真实打开零点并重基准 `FrictionAnomalyMap` 后，若 `homing.backoff_distance_mm > 0`，必须主动向 closing 方向回退该距离以释放端部抱死。回退动作不更新行程学习结果；若回退失败，回零不得标记为完成。

### 9.5 TravelLearning

目的：

- 从零点向闭合方向运行。
- 寻找闭合方向极限。
- 设置软件开/闭限位。

软件限位必须比机械限位保守，不能与机械限位完全重合。

`TravelLearning` 是摩擦异常 map 的主要确认阶段。全行程学习过程中应在恒速或近似恒速段持续采样，更新打开/闭合方向的异常候选；只有多次通过同一位置区间并满足电流回落和位置继续推进条件后，才能将候选提升为 confirmed。

行程学习不得使用普通业务运动中的单次 jam/contact 作为唯一终止条件。闭合方向学习必须采用与 PreB 一致的卡点容忍扫描：局部高电流、速度下降但位置仍持续推进时，记录为摩擦异常候选并继续；只有持续无有效推进、速度塌陷和电流不回落同时成立时，才把当前位置作为闭合结构限位候选。已知 anomaly 附近的电流峰值不能直接覆盖真实结构限位判断，必须叠加“通过后是否还能继续运动、峰值后电流是否回落”的判据。默认行程学习档位为 `1.2mm/s` / `1.5A`，低于 PreSelfCheck 的命令电流上限，但高于早期 `0.6A` 保守档，以降低正式学习在已知卡点处提前停止的概率；日志必须输出目标、搜索距离、理论参考行程、实际位移、速度、电流上限、contact/jam/force 判据和是否目标到达。

回零后，真实打开端被定义为 `0mm`；闭合端搜索目标不得再直接使用 `mechanism.theoretical_close_limit_mm`。控制器应使用 `self_check.travel_learning_search_distance_mm`、PreB 已知低置信宽度、旧理论参考行程和最小行程要求中的最大值作为闭合方向相对搜索距离，默认按 `20mm` 搜索。若目标先到达搜索距离而没有检测到结构端部，日志必须明确 `target_reached=true`，该结果只能说明配置搜索距离仍可能偏短或真实结构未碰端，不应误写为“理论 16mm 行程已验证”。最终软件闭合限位只来自实测闭合端位置减 `software_limit_margin_mm`；`theoretical_close_limit_mm=16` 仅用于旧模型误差、配置页和日志参考。

`TravelLearning` 成功更新软件限位和实测行程后必须刷新运行时学习 seed。下一次 PreSelfCheck 可以把已学 bootstrap 起动电流和有效静/动摩擦样本计数作为参考，避免每次都从明显过低的电流开始；但最终静摩擦、动摩擦和最低稳定速度仍以有效样本计数区分来源，seed 中的软件限位只作为低置信参考，不能在未回零前作为绝对软件限位使用。

### 9.6 MotionHealthCheck

目的：

- 在软件安全范围内用多个安全速度往复运动。
- 验证螺母位置、速度、电流、方向和行程一致性。
- 通过后系统进入 `Ready`。

`MotionHealthCheck` 可用于复核历史摩擦异常：新异常出现、严重度升高或异常消失都应作为健康趋势输入；但健康检查同样不能仅凭历史 map 忽略实时堵转风险。

健康检查样本必须来自运动过程中的真实反馈统计，包括速度跟踪误差、电流/力矩纹波和最高温度。第一版可按每段往返运动的多帧最值与均值生成样本，成功后把健康摘要写入运行时学习 seed，用于下次启动时显示和趋势对比；健康 seed 不得绕过本次 `MotionHealthCheck`。

`MotionHealthCheck` 的软件限位内往返段也应继续向摩擦异常检测器送入实时反馈样本。该阶段发现的新候选只更新诊断 map 和后续趋势输入，不单独放宽实时 jam/contact 判据。

### 9.7 Clamp / Release

夹紧控制必须在 `Ready` 后执行。

夹紧目标：

- 支持目标夹紧力。
- 支持恒定夹爪速度或螺母速度的夹紧过程。
- 接触前降低冲击。
- 达到目标或检测异常后停止并失能。

夹紧过程使用 `FrictionAnomalyMap` 作为误判抑制输入。经过已知卡点附近时，短时电流峰值或速度下降不能单独视为夹紧接触；只有离开异常窗口后电流仍持续升高，或位置持续无有效推进并达到接触/堵转确认时间，才可判定为夹紧接触或异常停止。严重 anomaly 可提高日志告警和维护提示等级，但不得绕过全局硬保护。

释放目标：

- 在软件安全范围内打开夹爪。
- 完成后失能。

## 10. 安全与调试约束策略

### 10.1 默认约束

默认开启：

- 控制器限流。
- 控制器限速。
- 软件行程限制。
- 主动停止。
- 通信超时保护。
- 温度保护。
- 全局硬保护。

### 10.2 可关闭的调试约束

在调试模式下，用户可主动关闭：

- 控制器级限流。
- 控制器级限速。
- 主动停止判据。
- 软件行程限制。

关闭前必须：

- UI 明确显示风险。
- 用户确认当前为空载或已采取机械安全措施。
- 日志记录开关变化。

### 10.3 不可关闭的硬保护

任何模式下不可关闭：

- 用户停止/急停。
- 电机硬件故障处理。
- 通信严重异常后的停止输出。
- 全局最大电流硬保护。
- 温度硬保护。
- 命令持续时间上限。

### 10.4 调试约束与学习结果

关闭控制器约束后的动作只用于调试，不得产生高置信结构学习结果。若需要写入 `StructureProfile`，必须标记为低置信或诊断数据。

## 11. 达妙硬件规则

达妙硬件实现必须遵守：

- 连接时读取或确认电机运行参数，如 `P_MAX`、`V_MAX`、`T_MAX`。
- 使用电机反馈中的多圈位置作为控制位置来源。
- 保留协议位置、原始计数和原始帧摘要作为诊断字段。
- 将连接阶段读取到的运行时 `P_MAX`、`VMAX`、`TMAX` 透传到控制器快照、API 和 UI，用于判断位置命令是否接近达妙位置窗口边界。
- `readFeedback()` 返回最新快照，不应在控制器调用时临时抢旧帧。
- `disable()` 不得伪造零速度、零电流或零力矩反馈；只能改变输出状态。
- 位置/速度/力控命令必须通过 `MotorInterface` 抽象，不允许 UI 直接调用达妙 SDK。

## 12. 管理员恢复与电机空载调试

### 12.1 MotorBringupMode

用途：

- 结构未安装或空载时验证通信链路。
- 验证电机正反方向。
- 验证反馈刷新。
- 使用电机侧相对圈数位置到位命令验证多圈位置反馈和虚拟螺母行程映射，例如 `+1rev/-1rev/+2rev/-2rev`。该能力用于确认 `2*pi rad -> 2mm` 的丝杆导程映射，不属于结构自检或行程学习。

限制：

- 不设置螺母零点。
- 不更新 `StructureProfile`。
- 不作为 PreSelfCheck 的实现路径。
- 圈数位置到位命令必须要求空载或结构安全确认，必须限制最大圈数、最大速度、反馈电流中止阈值和超时时间，并在完成、失败、超时或触发电流保护后失能。
- 圈数位置到位命令采用电机侧多圈位置作为起点，目标位置为 `start_motor_position_rad + relative_revolutions * 2*pi`。日志必须记录起点、目标、终点、电机增量、圈数增量、虚拟螺母增量和 `mm_per_rev_estimate`。
- 圈数位置到位命令下发前必须检查目标位置是否位于达妙运行时位置窗口 `[-P_MAX, P_MAX]` 内。若当前位置接近边界导致目标越界，应拒绝命令并在 UI/日志中提示先反向移动或调大电机 `P_MAX`。
- 若用户未指定合理超时，控制器应按 `abs(relative_revolutions * 2*pi) / max_motor_velocity` 自动估算到位超时，并叠加保守裕量；避免 2 圈、1rad/s 但只给 3s 这类必然超时的默认参数。
- UI、API 和 commander 的圈数到位入口在用户未指定超时时必须传递 `0`，由控制器统一执行自动估算，不允许入口层重新默认成固定 `3s`。
- 圈数到位的反馈电流中止阈值用于空载调试保护，默认建议为 `1.5A`，上限不得超过全局不可绕过硬保护 `safety.max_motor_current_a`。普通超过反馈电流中止阈值时，应持续超过 `safety.contact_detection_time_s` 才触发 `ActiveStop`；超过全局硬保护阈值时仍必须立即停机。

### 12.2 AdminRecoveryMode

用途：

- 机构抱死或异常后，由管理员以更高权限尝试恢复。

要求：

- 必须区分普通释放和管理员恢复。
- 必须识别打开方向，防止方向错误导致越卡越紧。
- 高电流或高速度动作必须有持续时间上限。
- 全局硬保护不可关闭。
- 恢复后必须重新自检或回到必要流程确认状态。

## 13. UI 要求

UI 至少包含：

- 控制器状态。
- 启动流程状态。
- 电机反馈：多圈位置、速度、电流、力矩、温度。
- 虚拟螺母位置：位置、速度、零点、数据新鲜度。
- 达妙运行时限制：`P_MAX/VMAX/TMAX`。该信息应在右侧状态区和 `MotorBringupMode` 操作区同时显示，便于判断 `+1rev/+2rev` 圈数到位目标是否接近或越过 `[-P_MAX, P_MAX]`。
- 学习参数：PreA bootstrap 起动电流、最低稳定速度、静/动摩擦、低置信安全区、软件限位。UI 必须区分 bootstrap 起动电流与最终静摩擦模型，并显示或使用样本计数避免把未学习字段误读为学习结果。
- 调试约束状态和开关。
- 普通控制、自检流程、维护模式、配置参数、日志诊断分页。
- 配置参数分页只读显示当前已加载配置快照，按 `adapter/motor/mechanism/self_check/safety/motor_bringup/homing/clamp/ui` 分组展示；`common` 目录只定义单位和基础类型，不作为运行参数组展示。

UI 规则：

- 未连接时禁用依赖硬件的动作。
- 编码器未就绪时禁用依赖螺母位置的动作。
- 未完成必要自检时禁用业务夹紧动作。
- `ActiveStop` 下仍持续显示反馈。
- 所有按钮动作必须写日志。

## 14. 配置策略

配置按模块分组：

- `adapter`：设备类型、端口、波特率、FDCAN/BRS。
- `motor`：电机 ID、主机 ID、位置方向、反馈周期、达妙参数。
- `mechanism`：丝杆导程、理论行程、夹爪运动学参数。
- `nut_position_encoder`：虚拟螺母位置编码器方向修正、启动参考、更新频率、过期时间。
- `safety`：硬保护电流、温度、速度和超时。
- `debug_constraints`：默认约束开关和调试权限。
- `self_check`：预自检速度、电流、距离、停稳、噪声阈值和局部摩擦异常检测阈值。
- `clamp`：目标力、夹紧速度、力矩/电流映射。
- `ui`：刷新周期和日志容量。

所有配置项必须有单位或明确枚举含义。

## 15. 测试策略

### 15.1 单元测试

必须覆盖：

- 虚拟螺母位置编码器 `2*pi rad -> 2mm`。
- 零点重设。
- 方向符号。
- 反馈过期。
- 调试约束策略开关和硬保护不可关闭。
- 状态机合法/非法转换。
- 摩擦异常检测：平稳电流不误报、局部峰值且回落时记录、窄尖峰过滤、方向独立记录、未回落峰值不作为异常候选。

### 15.2 仿真测试

必须覆盖：

- 启动到 `NutEncoderReady`。
- PreSelfCheck 低置信通过。
- 回零后重新设零。
- 行程学习和软件限位。
- 夹紧/释放后失能。
- 用户停止取消长流程。

### 15.3 真实硬件测试

推荐顺序：

1. 连接和反馈刷新。
2. 空载电机正反点动。
3. 电机多圈位置连续性。
4. 虚拟螺母位置映射：`2*pi rad -> 2mm`。
5. 调试约束开关显示和日志。
6. PreSelfCheck。
7. 回零。
8. 行程学习。
9. 运动健康检查。
10. 夹紧和释放。

每次真实硬件问题必须归档到 `references/04_hardware/`。

## 16. 当前实施边界

本文件建立 V2 完整设计方案。当前源码仍保留 V1/V1.5 期间的可运行实现，用于硬件对照和回归参考。后续实现应按 `references/06_implementation/v2_incremental_implementation_plan.md` 分阶段推进，不应一次性重写所有控制逻辑。

## 17. V1 归档说明

V1 设计和实施计划已归档：

- `references/archive/2026-05-22_control_architecture_design_v1_legacy.md`
- `references/archive/2026-05-22_controller_interface_first_implementation_plan_v1_legacy.md`

V1 文件只作为历史参考。后续控制系统设计、实现、测试和问题纠偏均以本 V2 设计方案为唯一设计出口。
