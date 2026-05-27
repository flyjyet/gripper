# PreSelfCheck 与丝杆自学习控制归档（2026-05-26）

## 1. 归档目的

本文归档 2026-05-22 至 2026-05-26 多轮实机调试后形成的两硬限位梯形丝杆夹爪控制方案。重点记录 `PreSelfCheck` 的最终实现、实机问题修正过程、结构参数与摩擦异常自学习边界、后续回零/行程学习/健康检查/夹紧流程如何使用这些结果，以及后续封装为通用控制库的建议。

当前实现仍遵循 V2 控制架构：

- 以虚拟螺母位置编码器为统一位置源。
- `PreSelfCheck` 只输出低置信结构 profile，不直接宣称完成真实回零和最终软件限位。
- `HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 负责把低置信 profile 逐步升级到真实零点、实测行程、健康确认。
- 局部卡点由独立的 `FrictionAnomalyMap/Detector` 表达，不混入全局结构基值。

主要代码入口：

- 控制器主流程：`src/controller/gripper_controller.cpp`
- 自检算法模块：`src/controller/self_check/`
- 配置结构和默认值：`src/config/gripper_config.hpp`、`src/config/default_gripper.yaml`
- UI/API：`src/ui/web_server.cpp`、`src/ui/prototype/admin_recovery_ui_preview.html`
- 设计基线：`references/03_control/gripper_control_architecture_v2.md`
- 实施计划：`references/06_implementation/v2_incremental_implementation_plan.md`

## 2. 功能分工和实现摘要

### 2.1 虚拟螺母位置编码器

虚拟螺母位置编码器把达妙电机多圈位置换算为螺母行程。换算使用丝杆导程 `mechanism.lead_screw_pitch_mm_per_rev`，并通过 `nut_position_encoder.direction_sign` 独立配置螺母计数方向。这个方向配置是实机 bring-up 验证后加入的：它只反转控制器侧螺母反馈方向，不改变达妙位置命令极性。

回零前的螺母坐标是临时坐标。`0-16mm` 只保留为旧理论参考，不再作为 `PreB` 或 `TravelLearning` 的硬终点；真实样机行程可达 `18-20mm`，达妙运行时 `P_MAX` 约 `65rad` 是电机侧最终位置窗口约束。

### 2.2 StructureProfile 与 seed

`StructureProfile` 保存当前启动周期学习到的结构信息：

- PreA bootstrap 起动电流和样本数。
- 最低稳定螺母速度。
- 打开/闭合方向静摩擦、动摩擦。
- 低置信边界、安全区和后续软件限位。
- 运动健康摘要。

`self_check.learned_profile_path` 指向运行时 seed 文件。控制器在 `PreSelfCheck` 开始前加载 seed，只把它作为下一次低能量起扫、诊断显示和搜索距离参考；它不能让系统跳过本次 `PreSelfCheck`、回零、行程学习或健康检查。

seed 版本 3 之后把 `opening_breakaway_bootstrap_*` / `closing_breakaway_bootstrap_*` 与最终静摩擦字段分离。旧版本 seed 中的静摩擦字段只迁移为 bootstrap 起动参考，旧动态摩擦和最低稳定速度不再自动恢复为最终控制模型，避免历史污染。

### 2.3 安全电流 envelope

当前默认安全电流分层为：

- `safety.max_motor_current_a = 2.0`：全局命令电流硬上限。
- `safety.self_check_current_limit_a = 1.9`：PreSelfCheck 普通探边命令上限。
- `safety.self_check_feedback_hard_current_limit_a = 2.5`：PreB 反馈硬电流持续确认阈值，短时尖峰只记录诊断。
- `safety.self_check_feedback_emergency_current_limit_a = 3.0`：PreB 极端反馈电流立即停当前 probe，避免端点顶死后出现十安培级峰值持续。
- `self_check.pre_b_boundary_release_current_limit_a = 2.0`：BoundaryRelease 专用释放命令电流，仍受全局命令电流上限裁剪。
- `safety.travel_learning_current_limit_a = 1.5`：正式行程学习电流上限。

达妙 `POS_FORCE` 的命令电流、反馈扭矩、电机 `TMAX`、`max_phase_current_a` 和 `torque_per_amp_nm_per_a` 之间存在换算链路。日志中的反馈电流可能高于命令限流，因此控制器同时记录 `current_limit_a`、`max_current_a`、`last_torque_nm`、运行时 `TMAX` 等诊断字段；不能只靠提高反馈急停阈值掩盖端点顶死风险。

## 3. PreSelfCheck 最终流程

### 3.1 总体语义

`PreSelfCheck` 由 PreA 和 PreB 两层组成。

PreA 是低能量 bootstrap，目的是证明当前系统至少存在可控运动方向，并得到让流程继续推进的起动电流参考。PreA 结果可能受结构端点、局部卡点或初始姿态污染，所以只写 bootstrap breakaway seed，不能写成最终静摩擦、动摩擦或最低稳定速度。

PreB 是低置信近全行程预探索和多区域参数学习。它从当前临时位置按相对距离扫描，不假设初始点在丝杆中位，也不被旧 `0-16mm` 坐标夹断。PreB 的边界仍是低置信边界，后续必须由回零和行程学习重基准。

### 3.2 PreA：低能量可控性验证

PreA 实现步骤：

1. 静止反馈噪声采样，建立位置、速度、电流、力矩和螺母行程噪声底。
2. 按 `static_friction_current_start/stop/step` 执行低电流起动扫描。历史键名保留为兼容 YAML，但语义已经改为 bootstrap 起动扫描。
3. 以 `motion_start_distance_mm` 和 `low_confidence_motion_distance_mm` 判定是否出现方向正确、单调、已停稳的小位移。
4. 做稳定短行程验证，若检测到一侧像结构端点、另一侧可控退开，则记录 `endpoint_start_escape`，不再在端点附近强行做双向短行程。

端点启动是多轮实机调试后修正的重点：当机构初始停在 opening 结构限位时，PreA 不能在端点附近来回要求成对微动；只要 closing 方向能退开，就应把 opening 标记为起始疑似边界，让 PreB 先向 closing 长距离探测另一端。

### 3.3 PreB：连续受控探边

PreB 主探边使用 bring-up-like 连续相对扫描。`pre_b_boundary_step_mm` 仍保留给边界候选 refine 和软卡点复核，不再限制主扫描覆盖距离。

核心规则：

- 默认单向扩展距离 `pre_b_max_expansion_distance_mm = 20.0`，表示从当前位置向某一侧最多尝试这么多相对距离。
- 扫描速度 `pre_b_expansion_speed_mm_s = 1.0`，在 `2mm/rev` 丝杆上约为 `3.14rad/s`。
- 单次 probe 因普通超时或非物理边界卡点中断，但已有方向正确、单调、已停稳的有效推进时，保留 partial bound，并从当前点继续消耗该方向剩余扫描距离。
- 达妙 `P_MAX` 裁剪目标距离，日志输出请求距离、裁剪距离和余量；只有裁剪后没有有效扫描距离，才以 P_MAX 窗口不足结束该方向。
- 默认第一方向若为 opening，则“当前位置到 opening 结构限位”的首段只探测，不写 anomaly map 和 UI trace，避免初始段污染后续卡点和选点判断。
- 如果 PreA 标记了起始端点，则 PreB 先沿可退开方向去找远端，再释放并回头确认起始端。

### 3.4 卡点与物理硬限位区分

多轮实机日志表明，局部卡点会表现为速度降低、电流尖峰，但位置仍能继续推进，且峰值后电流回落。真实结构硬限位则表现为速度塌陷、持续无有效推进、反馈电流持续超阈值或 emergency 超限。

当前判据：

- 短时超过 `self_check_feedback_hard_current_limit_a` 只记录 transient，不立即收边界。
- 超过 `pre_b_hard_current_confirm_time_s = 0.25s` 且高电流期间无噪声级有效推进，才按物理边界或堵转风险处理。
- 超过 `self_check_feedback_emergency_current_limit_a = 3.0A` 立即停止当前 probe，但该结果属于 PreB 探边诊断，不直接升级为全局 `ActiveStop`。
- 有持续单调位移的高电流段优先按卡点/partial progress 处理，不直接收为结构限位。

触发端点停机后，probe 收尾必须卸载/失能并采样，不再发送当前位置 hold，避免电机继续压在结构端点上。

### 3.5 BoundaryRelease

PreB 在探到疑似物理端点或卡点抱死后，必须先反向释放，再换向或进入参数学习。

实现规则：

- 释放距离 `pre_b_boundary_release_distance_mm = 1.0`。
- 释放速度 `pre_b_boundary_release_speed_mm_s = 1.0`。
- 释放命令电流 `pre_b_boundary_release_current_limit_a = 2.0`，独立于普通探边电流 envelope。
- 只知道单侧端点时，不要求已经形成双边 safe zone；控制器围绕当前停止点和反向释放目标构造本地临时护栏。
- 若释放 probe 已产生有效退开但未达到总释放距离，允许继续释放剩余距离。
- 若最终释放失败，标记 `boundary_release_failed` / `mechanism_anomaly`，跳过第二方向主扫描和多区域学习，并强制 profile 回到 PreA 保守窗口，不能把紧邻端点的小位移高电流误收为另一侧边界。

这条规则解决了“从远处撞到 opening 边界后，反向释放失败却继续在端点附近来回动，甚至误判双边界”的实机问题。

### 3.6 MultiRegionRoundTripLearning

只有 PreB 形成有效低置信 safe zone 且边界释放未失败时，才进入多区域、多速度学习。

学习目标：

- 从多个干净位置区域学习最终静摩擦、动摩擦和最低稳定速度。
- 避开端点、BoundaryRelease、回到锚点路径、PreA 微动和严重卡点。
- 将全局结构基值与离散卡点 map 分开表达。

实现规则：

- `pre_b_min_learning_regions = 3`，至少需要 3 个位置分离的有效干净区域。
- `pre_b_learning_anchor_count = 7`，候选锚点多于最低区域数，以便绕开严重卡点。
- low-confidence safe zone 被区域化，每个必需区域只能在本区域内寻找干净锚点，禁止把中部或闭合侧区域迁移到打开侧干净位置。
- 学习速度使用 `dynamic_friction_speeds_mm_s = [0.6, 1.0, 1.2, 1.5]`，按多速度档统计动摩擦和稳定速度。
- 采样前用当前 anomaly 记录过滤锚点和计划路径；采样后若新增严重 anomaly 与本段重叠，则该段不进入最终模型。
- 若有效区域数不足，回滚本阶段 `motion/static/dynamic` 样本，保留低置信边界，并上报 `mechanism_anomaly=true`，提示机构卡点过多或状态异常。

从 closing 结构限位回到 opening 软限位以准备学习时，使用连续受控运动，并只写 UI trace，不写 anomaly map 或最终结构模型，避免分段启停尖峰污染选点。

### 3.7 PreB 电流-行程 trace

UI/API 已增加 PreB 电流-行程曲线：

- 后端记录 `stroke_mm`、反馈电流、速度方向、phase、`segment_id`。
- 每个连续 probe 开新 `segment_id`。
- 前端优先按 `segment_id` 断线，兼容方向变化、phase 变化和位置大跳变断线，避免把左右极限点直接连成斜线。
- 横纵坐标显示 mm/A 网格和刻度。
- 大电流峰值不在 UI 层裁剪，保留给人工判断端点停机是否及时。
- 曲线下方提供 opening、closing、unknown/diagnostic 颜色设置，保存在浏览器 `localStorage`，只影响显示，不改变控制判据。

## 4. FrictionAnomalyMap / Detector

`FrictionAnomalyMap` 独立于 `StructureProfile`。它描述梯形丝杆局部毛刺、磨损、润滑不均或螺纹表面缺陷导致的位置相关电流峰值，不描述机构几何、最终行程或软件限位。

检测器输入来自恒速或近似恒速段的反馈样本：

- 螺母位置和电机位置。
- 电机反馈电流、速度、温度。
- 运动方向。

算法使用滑动距离窗口：

1. 按 `friction_anomaly_sliding_window_distance_mm = 1.0` 维护局部电流基线。
2. 当前电流相对基线超过 `friction_anomaly_current_ratio_threshold = 2.0` 时进入疑似异常。
3. 电流回落后关闭异常，记录位置起止、中心、宽度、峰值、baseline、ratio、方向、速度和温度。
4. 未闭合的持续升高段在 `finish()` 中丢弃，不写为卡点；控制流程继续按端点或堵转风险处理。

当前实现只保留内存候选记录，不做文件持久化和前馈补偿。PreSelfCheck 生成的候选以临时零点展示；`HomingOpenStop` 设置真实打开零点后，应按峰值电机位置重基准到真实 0 点坐标，供 `TravelLearning` 和 `MotionHealthCheck` 继续确认。

默认阈值已从早期 1.5x 调整为 2.0x，学习避让阈值 `friction_anomaly_learning_avoid_ratio = 3.0`，避免把普通波动填满 20 条候选并过度影响区域选择。

## 5. 后续流程如何使用自学习结果

### 5.1 HomingOpenStop

`HomingOpenStop` 必须在本次启动周期 `PreSelfCheck` 成功后执行，历史 seed 不能替代本次预自检。

实现要点：

- 从当前位置沿 opening 方向做相对搜索，不把回零前临时 `0mm` 当成绝对目标。
- 搜索距离取 PreB 低置信宽度、旧理论行程、`pre_b_max_expansion_distance_mm` 和最小行程要求中的较大值。
- 电流选择使用 `homingCurrentLimitFromProfile()`，优先学习模型加小裕量，再受 `safety.homing_current_limit_a` 裁剪；日志输出 `current_source`。
- opening 端部命中后立即卸载/失能并采样零点，不在端点继续 hold。
- 设置真实打开零点后，按 `homing.backoff_distance_mm` 向 closing 回退释放端点抱死。
- 回零后重基准 anomaly map。

### 5.2 TravelLearning

`TravelLearning` 从真实打开零点向 closing 搜索闭合端。

实现要点：

- 闭合搜索距离使用 `self_check.travel_learning_search_distance_mm = 20.0`、PreB 低置信宽度、旧理论行程和最小行程要求中的最大值。
- `mechanism.theoretical_close_limit_mm = 16.0` 只作为日志和旧理论参考，不再截断搜索。
- 默认速度 `1.2mm/s`、电流上限 `1.5A`，用于通过已知卡点但仍低于全局硬保护。
- 复用 PreB 卡点容忍逻辑：局部电流峰值但位置继续推进时记录 anomaly 并继续；只有持续无有效推进、速度塌陷和电流不回落同时成立，才收为闭合结构限位。
- 成功后写实测行程、软件开/闭限位，并刷新 seed。

### 5.3 MotionHealthCheck

`MotionHealthCheck` 在软件限位内执行多速度往返：

- 使用学习电流加裕量并受安全上限裁剪，日志输出 `current_source`。
- 统计速度跟踪误差、电流纹波、力矩纹波和最高温度。
- 继续向 anomaly detector 输入反馈样本，用于更新健康趋势和卡点候选。
- 成功后把健康摘要保存到 seed；健康 seed 只作为趋势参考，不能跳过下一次健康检查。

### 5.4 ClampByForce / ClampBySpeed

夹紧必须在 `Ready` 后执行。已知 anomaly 只用于抑制误判：

- 经过 anomaly 窗口时，短时电流峰值不能单独作为夹紧接触或结构限位。
- 接触/夹紧仍需要持续电流、速度塌陷和无有效推进判据。
- 如果 anomaly 严重度持续升高，后续应提示检查丝杆，而不是简单提高夹紧电流。

## 6. 多轮实机修正记录

本功能经历了多轮实机日志驱动调整，关键结论如下：

1. 最初 PreSelfCheck 只在很小范围内完成 PreA，这对安全起动足够，但不能表达结构模型；因此加入 PreB 做近全行程低置信探测和多区域学习。
2. 初版 PreB 假设起点接近中位，实际机构经常停在端点或卡点附近；因此改成相对当前位置扫描，且端点启动时先沿可退开方向探另一侧。
3. 旧 `0-16mm` 行程假设过早截断扫描；现场确认真实行程可能 `18-20mm`，PreB 和 TravelLearning 改为 `20mm` 搜索预算，并由达妙 `P_MAX` 动态裁剪。
4. 一步步探测导致启停尖峰太多，后改为 bring-up-like 连续受控扫描；`pre_b_boundary_step_mm` 只保留给边界候选复核。
5. 高速 `5.0mm/s` 在卡点处冲击太大，`0.6mm/s` 又容易过不了卡点；现场默认收敛到 `1.0mm/s`，动态学习档位保留 `[0.6, 1.0, 1.2, 1.5]mm/s`。
6. 短时电流尖峰不能等同于结构限位；加入 `pre_b_hard_current_confirm_time_s` 和“高电流期间是否仍推进”的判据。
7. 端点顶死后反馈电流可能出现很高峰值；加入 `self_check_feedback_emergency_current_limit_a`，并把普通探边默认值下调到 `3.0A`。
8. 端点停机后不能继续当前位置 hold；改为失能/卸载采样，防止流程卡死在结构限位。
9. BoundaryRelease 不能依赖已有双边 safe zone；单侧边界也要用本地释放窗口退开。
10. 释放失败时不能继续第二方向收假边界；必须降级到 PreA 保守窗口，跳过多区域学习。
11. 多区域学习不能集中在打开端局部；锚点改为区域内搜索，至少 3 个有效区域，不足时报 `mechanism_anomaly` 并回滚样本。
12. 卡点阈值 1.5x 太敏感，容易把普通波动写满 map；默认改为明显尖峰 2.0x，学习避让仍使用更高严重度门槛。
13. UI 曲线原先把左右极限点直接连线，造成斜线误读；后端增加 `segment_id`，前端按段断线并增加网格刻度。
14. PreB 首段 opening 探边和 BoundaryRelease 不写 anomaly/trace；closing 回 opening 软限位改成连续回位并仅写 UI trace，减少启停尖峰污染。
15. PreA 起动电流不等于最终静摩擦；最终控制模型必须来自避开端点和卡点的多区域多速度样本。

## 7. 相关研究和开源项目参考

### 7.1 研究文献

本项目参考的是成熟摩擦建模和伺服辨识思想，但没有在第一版直接实现完整高阶模型。

- Armstrong-Helouvry、Dupont、Canudas de Wit 的摩擦控制综述系统总结了静摩擦、库仑摩擦、粘性摩擦、Stribeck、Dahl 等模型和补偿方法。参考点：把摩擦分成结构基值和非线性/位置相关扰动，不把单次电流峰值当成全局摩擦。
  - DOI: `10.1016/0005-1098(94)90209-7`
  - https://cir.nii.ac.jp/crid/1361699995769695232

- Canudas de Wit、Olsson、Astrom、Lischinsky 的 LuGre 模型是动态摩擦建模经典工作。参考点：摩擦具有预滑移、滞回和 Stribeck 等动态特性；但当前 PreSelfCheck 只做稳健分档统计，不拟合 LuGre 参数。
  - DOI: `10.1109/9.376053`
  - https://ieeexplore.ieee.org/document/376053
  - https://portal.research.lu.se/en/publications/a-new-model-for-control-of-systems-with-friction

- Olsson 等人的 “Friction Models and Friction Compensation” 进一步总结了摩擦模型与补偿。参考点：低速和换向附近的数据不能简单外推到全行程高速控制。
  - https://portal.research.lu.se/en/publications/friction-models-and-friction-compensation/

- Elhami、Brookfield 的机器人驱动摩擦辨识实验比较指出，可用速度与电枢电流关系离线辨识库仑/粘性摩擦，也可用递推最小二乘做在线估计。参考点：本项目采用多速度档和电流统计获取结构基值，但先用稳健统计而不是在线最小二乘。
  - https://journals.sagepub.com/doi/pdf/10.1243/pime_proc_1996_210_228_02

- Kim、Chung 的球丝杆伺服摩擦辨识通过极限环分析识别静摩擦、库仑、粘性和 Stribeck 元素。参考点：丝杆伺服摩擦辨识应覆盖多速度/多状态，并可用嵌入式位置反馈验证；但该方法需要较强实验条件，本项目先采用安全优先的低置信扫描。
  - DOI: `10.1016/j.mechatronics.2005.09.006`
  - https://www.sciencedirect.com/science/article/abs/pii/S0957415805001182

- ball screw sensorless friction estimation 研究使用伺服电机转矩电流和工作台位置估计摩擦。参考点：电机电流可作为力/摩擦代理，但需要注意时间同步、换算和噪声。
  - DOI: `10.3390/app10093122`
  - https://www.mdpi.com/2076-3417/10/9/3122

### 7.2 开源工程参考

- ODrive anticogging 使用一次标定生成位置相关 torque map，再作为位置相关前馈项。参考点：位置索引补偿 map 是成熟工程做法；本项目的 `FrictionAnomalyMap` 类似“位置相关异常表”，但当前只做诊断/避让，不做前馈补偿。
  - https://docs.odriverobotics.com/v/latest/guides/anticogging.html

- LinuxCNC homing 文档区分 machine origin、HOME_OFFSET、HOME 和 soft limits。参考点：硬件/传感器命中后要建立机器坐标，再回到安全 HOME；本项目回零后把 opening 端设为真实零点，并回退释放。
  - https://linuxcnc.org/docs/2.8/html/config/ini-homing.html

- Klipper / Marlin 的 TMC sensorless homing 使用驱动器 stall/load 指标替代传统限位开关，需要调灵敏度，并存在误触发/不触发风险。参考点：无独立限位传感器时，负载/电流类信号可用于回零，但必须调参并叠加速度和位移判据。
  - https://www.klipper3d.org/TMC_Drivers.html
  - https://marlinfw.org/docs/gcode/M914.html
  - https://marlinfw.org/docs/hardware/tmc_drivers.html

- SimpleFOC 展示了 torque/current control 分层，电流模式下目标电流近似对应目标扭矩。参考点：本项目把达妙电流命令看作 torque/current envelope，但反馈电流还受驱动器和换算参数影响。
  - https://docs.simplefoc.com/torque_control
  - https://docs.simplefoc.com/torque_loop

- ROS 2 `ros2_control` 的 hardware interface 把硬件组件、命令接口和状态接口分层。参考点：后续封装成通用库时，应把硬件适配层、轴状态、控制状态机和算法模块分离。
  - https://docs.ros.org/en/ros2_packages/rolling/api/hardware_interface/doc/hardware_interface_types_userdoc.html

### 7.3 本项目参考了什么

本项目直接参考的成熟思想包括：

- 分阶段 homing 和 soft limit 建立。
- 电机电流/转矩作为摩擦和接触代理。
- 多速度档摩擦辨识。
- 位置相关摩擦/齿槽/扰动 map。
- 传感器缺失时的 sensorless stall/homing 调参与风险控制。
- 控制库分层：硬件接口、状态机、算法、UI/诊断分离。

### 7.4 本项目的原创性

本项目原创点主要是面向“两侧都是结构硬限位、无独立限位/力传感器、初始位置未知、梯形丝杆有局部卡点”的组合工程方案：

- PreA/PreB 两层预自检：PreA 只证明低能量可控性，PreB 再做近全行程低置信预探索。
- 回零前临时坐标模型：允许临时坐标越过旧 `0/16mm`，不把理论坐标当物理绝对位置。
- 端点启动逃逸：一侧疑似硬限位、另一侧可退开时，先沿可退开方向长距离探另一端。
- 连续扫描 + partial progress 累积：卡点/普通超时但有有效推进时继续扫描剩余距离。
- 卡点与结构硬限位的组合判据：电流、速度、持续时间和位移推进必须联合判断。
- BoundaryRelease 本地释放窗口：单边界已知时也能先退开端点抱死，释放失败则降级而不是收假边界。
- 最终结构基值和离散卡点 map 分离：多区域干净样本输出静/动摩擦和最低稳定速度，局部峰值写 anomaly map。
- 多区域学习区域内锚点搜索：防止学习样本集中在某一段。
- PreB 后端分段 trace + UI 人工判读：把控制判据和人工诊断曲线分离。
- seed v3 语义：bootstrap 起动电流与最终静/动摩擦模型严格分离。

## 8. 后续封装为可复用控制库

目标是把“两端硬限位丝杆 + 电机反馈 + 无外部力/限位传感器”的控制方法封装为通用库，供同类夹爪、线性推杆、小型滑台复用。

### 8.1 推荐库边界

控制库应拆成 5 层：

1. `axis_core`：单位类型、方向、错误码、时间戳、状态快照。
2. `hardware_adapter`：电机抽象接口，只暴露 enable/disable、位置-速度-电流命令、反馈读取、运行时限位；不包含具体达妙 SDK。
3. `virtual_axis_encoder`：电机位置到丝杆线位移的映射、方向配置、零点重基准、P_MAX/多圈处理。
4. `self_learning`：PreA/PreB、摩擦异常检测、多区域学习、seed 持久化。
5. `workflow_controller`：回零、行程学习、健康检查、夹紧/释放、状态机和安全 envelope。

UI、日志和上位机命令不进入控制库核心，只通过接口订阅状态、trace 和事件。

### 8.2 通用配置模型

可复用配置至少包含：

- 机构：导程、预期行程、方向、软限位 margin。
- 电机：位置/速度/扭矩窗口、反馈周期、扭矩电流换算。
- 安全：命令电流上限、反馈硬电流阈值、emergency 阈值、速度/加速度限制。
- PreSelfCheck：PreA 起动扫描、PreB 扫描距离/速度、硬电流确认时间、释放距离/速度/电流。
- 学习：多速度档、最小学习区域数、锚点数量、卡点避让距离和严重度阈值。
- 持久化：结构 profile seed 路径、anomaly map 路径、版本迁移策略。

配置应保持物理单位命名，例如 `_mm`、`_mm_s`、`_a`、`_rad`，避免业务层裸 `double`。

### 8.3 通用状态机

建议把状态机抽象为：

`Disconnected -> Connected -> Enabled -> PreSelfCheckCompleted -> Homed -> TravelLimitsLearned -> MotionHealthChecked/Ready -> Working`

库需要明确：

- `PreSelfCheckCompleted` 不是 `Homed`。
- 历史 seed 不能替代本次启动周期状态。
- 端点探边失败和释放失败是可降级诊断，不必直接进入全局故障。
- 用户停止、通信失败、反馈矛盾和全局硬保护不可降级。

### 8.4 通用算法接口

建议提供以下算法接口：

- `run_pre_self_check(config, seed, hardware)`：返回低置信 profile、trace、anomaly candidates、degrade reason。
- `home_open_stop(profile)`：返回真实零点和重基准 anomaly map。
- `learn_travel_limits(profile)`：返回实测行程和软件限位。
- `run_motion_health_check(profile)`：返回健康摘要和 anomaly 更新。
- `clamp_by_force(profile, anomaly_map, force_command)`：执行接触/夹紧控制。

所有算法都返回结构化诊断，不只返回字符串。日志字符串可以由上层格式化。

### 8.5 复用时必须保留的安全约束

同类项目复用时不能省略以下约束：

- 命令电流上限和反馈硬电流上限分离。
- 卡点不能只靠电流判断，必须叠加速度和位移推进。
- 端点停机后先卸载/失能，再释放，不在端点 hold。
- 释放失败时不能继续构造假双边界。
- 最终静/动摩擦必须来自多个干净区域，不能来自 PreA 或端点附近样本。
- anomaly map 只能抑制误判，不能单独授权忽略实时堵转。

### 8.6 后续工程化路线

1. 把 `FrictionAnomalyMap` 持久化为独立文件，支持按电机位置重基准、过期时间、确认次数和双向确认。
2. 把 PreB trace 和自检事件改为结构化 ring buffer，UI/测试共用。
3. 将达妙硬件适配器和控制库核心分离，提供 simulated adapter、Damiao adapter、通用 CiA402/RS485 adapter。
4. 对 `PreSelfCheck` 做硬件在环测试脚本，覆盖端点启动、卡点穿越、释放失败、P_MAX 裁剪、异常过多等场景。
5. 在全行程数据足够后评估是否增加 Stribeck/LuGre 参数拟合；第一版库仍以稳健统计和 map 表达为主。
6. 输出库级 API 文档和安全案例，明确哪些参数必须由实机标定确认。

## 9. 当前验证状态和剩余风险

当前软件回归在最近一轮修改后通过 `scripts/build.ps1` 和 `scripts/test.ps1` 11/11。本文档落盘为归档说明，除同步修正 YAML emergency 默认值外未改变控制流程。

剩余风险：

- 实机反馈电流峰值仍依赖达妙运行时 `TMAX`、`max_phase_current_a` 和扭矩/电流换算准确性，需要继续用日志对照上位机参数。
- 当前 anomaly map 仍以内存候选为主，持久化和多次双向确认尚未完成。
- 多区域学习是稳健统计，不是完整 LuGre/Stribeck 模型；高精度轨迹控制仍需后续模型升级。
- 真实机构若出现严重卡点覆盖多区域，PreB 会报 `mechanism_anomaly` 并回滚最终结构模型样本，需要人工检查丝杆润滑、毛刺和装配状态。
