# V2 控制系统增量实施计划

## 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v2.0 | 2026-05-22 | 建立 V2 增量实施计划，配合 `references/03_control/gripper_control_architecture_v2.md` 使用。 |
| v2.1 | 2026-05-22 | 确认阶段 2 通信/反馈链路已具备继续推进条件；落盘阶段 3 虚拟螺母位置编码器基础实现和单元测试。 |
| v2.2 | 2026-05-22 | 落盘阶段 4 调试约束策略基础实现和单元测试。 |
| v2.3 | 2026-05-22 | 将 V2 虚拟螺母位置编码器等价接入控制器反馈路径；完整 CTest 10/10 通过。 |
| v2.4 | 2026-05-22 | 将虚拟螺母位置编码器诊断字段和达妙原始反馈帧透传到 UI/API；完整 CTest 10/10 通过。 |
| v2.5 | 2026-05-22 | 在 `MotorBringupMode` 中新增按电机圈数执行相对位置到位的空载调试入口，用于现场验证多圈位置反馈与 `2mm/rev` 虚拟螺母映射。 |
| v2.6 | 2026-05-22 | 计划补充达妙运行时 `P_MAX/VMAX/TMAX` UI 诊断显示、Bring-up 圈数目标越界拒绝和自动超时估算。 |
| v2.7 | 2026-05-22 | 落盘修复 Bring-up 圈数到位默认参数链路：UI/API/commander 缺省超时统一传 `0` 触发控制器自动估算；空载圈数到位反馈电流默认值提高到 `1.5A`、配置上限提高到 `2.0A`，并使用持续超限判据。 |
| v2.8 | 2026-05-22 | 落盘 Web UI 只读配置参数分页；在 Bring-up 操作区贴近显示达妙运行时 `P_MAX/VMAX/TMAX` 和当前多圈位置。 |
| v2.9 | 2026-05-22 | 落盘 PreSelfCheck V2 第一版收敛实现：使用虚拟螺母位置编码器执行噪声采样、双向低能量起动和短行程低置信验证；暂不执行完整理论边界搜索和多区域往复学习，只输出低置信临时 profile。 |
| v2.10 | 2026-05-22 | 新增 `nut_position_encoder.direction_sign`，允许根据 Bring-up 实测单独反转虚拟螺母位置编码器方向，不改变达妙位置命令极性。 |
| v2.11 | 2026-05-22 | 先于 PreB 落盘局部摩擦异常 map 的纯算法模块和配置项；第一版只产生内存候选记录，不接入限位/堵转控制决策。 |
| v2.12 | 2026-05-22 | 落盘 PreSelfCheck V2 当前最终版：启用 PreB 分段扩展、安全区往复和摩擦异常候选采样；输出仍为低置信 profile，最终软件限位仍待回零/行程学习确认。 |
| v2.13 | 2026-05-22 | 补充 PreB 降级策略：PreB 普通扩展失败或安全区不足时保持 PreA 保守结果；不可降级的安全/硬件/配置错误仍失败。完整回归 `scripts/test.ps1` 通过 11/11。 |
| v2.14 | 2026-05-22 | 将 PreB 实施目标调整为近全行程低置信预建模：延长自检停稳/运动等待，默认允许全理论行程扩展，保留 partial bounds，并在安全区内执行多区域、多速度采样。 |
| v2.15 | 2026-05-22 | 调整 PreB 实机策略：自检命令限流提高到现场可穿越卡点的量级，并新增软堵转受控小步复核；只有复核无有效推进或触发自检硬反馈保护才收为低置信边界。 |
| v2.16 | 2026-05-25 | 调整 PreB 实机默认速度：新增独立 `pre_b_expansion_speed_mm_s`，默认扩展/回位/软卡点复核为 `3.0mm/s`，并把安全区动态采样扩展到 `1.0/2.0/3.0mm/s`。 |
| v2.17 | 2026-05-25 | 修正 PreB 分段扩展实机策略：普通超时但已有有效 partial progress 时继续扩展；主扩展段将软 jam/contact 作为诊断而非立即停止；新增分段决策、电机圈数、mm/rev 和达妙 `P_MAX` 余量日志，用于区分位置窗口限制、参数缩放错误和真实结构边界。 |
| v2.18 | 2026-05-25 | 落盘 PreB 反馈硬电流持续确认：主扩展/回位/安全区学习使用 `pre_b_hard_current_confirm_time_s` 过滤启动瞬态反馈电流峰值，日志输出硬电流上限、确认时间、超限持续时间和是否确认。 |
| v2.19 | 2026-05-25 | 落盘 PreB 小步探边和卡点/物理边界区分：新增 `pre_b_boundary_step_mm`，移除短时峰值电流作为单独收边界依据；PreSelfCheck 初步摩擦异常 map 以电机位置锚定并在回零后重基准到当前零点。 |
| v2.20 | 2026-05-25 | 根据实机大卡点反馈提高 PreB 默认测试档位：`pre_b_boundary_step_mm = 1.0`，`pre_b_expansion_speed_mm_s = 5.0`，`pre_b_soft_jam_retry_distance_mm = 1.0`，`pre_b_soft_jam_retry_speed_mm_s = 5.0`。 |
| v2.21 | 2026-05-25 | 修正 PreSelfCheck 后手动定位窗口：PreB 已建立的完整低置信 safe zone 直接暴露给普通手动定位，不再额外限制到当前停留点附近 `max_probe_window/2`。 |
| v2.22 | 2026-05-25 | 将 PreB 主扩展实施策略改为连续受控扫描：主扫描一次覆盖到理论/配置护栏，20ms 反馈循环中记录摩擦异常候选并用持续硬电流/速度塌陷/无有效推进区分物理边界；`pre_b_boundary_step_mm` 保留给边界候选 refine 和软卡点复核。 |
| v2.23 | 2026-05-25 | 根据连续扫描实机日志下调 PreB 默认速度：`5.0mm/s` 约等于 `15.7rad/s`，在卡点处触发持续硬电流；默认连续扫描/软卡点复核改为 `1.2mm/s`，约 `3.77rad/s`，动态摩擦采样档位调整为 `[0.6, 1.0, 1.2, 1.5]mm/s`。 |
| v2.24 | 2026-05-25 | 现场继续下调 PreB 连续扫描/软卡点复核默认速度到 `0.6mm/s`，约 `1.88rad/s`，保留动态摩擦采样 `[0.6, 1.0, 1.2, 1.5]mm/s` 供安全区内统计。 |
| v2.25 | 2026-05-25 | 计划落盘 PreB 边界释放与卡点避让学习：探边命中物理边界后先主动反向回退释放抱死，再做多区域多速度学习；学习锚点和样本段避开 `FrictionAnomalyMap` 记录附近区域，防止局部卡点污染结构基值。 |
| v2.26 | 2026-05-25 | 计划修正实机日志暴露的问题：PreB 连续扫描在持续硬电流停机前已有大段有效推进时，必须把最终停稳位置作为 partial low-confidence bound；摩擦异常避让只阻断达到严重度阈值的卡点，防止轻微候选清空所有阶梯速度学习锚点。 |
| v2.27 | 2026-05-25 | 计划修正 PreB 回零前虚拟坐标误用：初始 `7.98mm` 不是物理中位，主扫描改为相对起点扩展并允许临时坐标越过理论 `0/16mm`，由物理边界/P_MAX/安全判据停止。 |
| v2.28 | 2026-05-25 | 计划同步修正 `HomingOpenStop`：PreB 可产生越过理论 `0/16mm` 的临时坐标后，回零必须沿 opening 相对搜索真实开端，不能再把临时 `0mm` 当成回零目标。 |
| v2.29 | 2026-05-25 | 计划修正 `HomingOpenStop` 端部抱死处理：回零低电流端部命中后立即卸载并采样零点，随后按 `homing.backoff_distance_mm` 向 closing 方向释放，避免物理边界顶死后停在普通停稳等待或保持输出。 |
| v2.30 | 2026-05-25 | 计划修正局部卡点误收边界：PreB 持续硬电流必须叠加无有效推进才收结构限位；有单调位移时保留 partial progress、记录卡点候选、释放后继续另一方向。`TravelLearning` 和夹紧同步引入卡点容忍/误判抑制路径。 |
| v2.31 | 2026-05-25 | 实施 PreB/TravelLearning/Clamp 卡点容忍第一版：PreB probe 新增硬电流期间推进诊断并只在无噪声级推进时硬停；TravelLearning/Clamp 的 contact/jam/force 停止加入持续无推进确认；默认 YAML 同步到 `0.6mm/s` PreB 现场档位。 |
| v2.32 | 2026-05-25 | 计划修正 PreB 执行覆盖：主扩展由单次远目标 probe 改为按剩余距离循环连续扫描，非物理边界卡点/普通超时有有效推进时继续同方向剩余扫描；卡点停机或物理边界候选后在换向/学习前执行主动释放。 |
| v2.33 | 2026-05-26 | 计划修正端点启动场景：PreA 检测到一侧疑似结构边界且另一侧可控退开时，跳过成对短行程要求并进入 PreB；PreB 优先向可退开方向长距离探另一端，再回头确认起始端，确保两侧边界都被尝试探测。 |
| v2.34 | 2026-05-26 | 根据实机日志调高现场默认档位并增强诊断：PreB 连续扫描/释放/复核改为 `1.0mm/s`，TravelLearning 改为 `1.2mm/s` / `1.2A`，输出 travel motion 结果用于区分卡点、电流不足和真实限位。 |
| v2.35 | 2026-05-26 | 计划修正实际行程超过旧理论值的问题：新增 `travel_learning_search_distance_mm=20.0`，PreB 单向扩展默认同步到 `20.0mm`，`motor.max_position_rad` fallback 同步现场 `65rad`；`TravelLearning` 闭合搜索使用实测搜索距离/P_MAX 窗口，而不是在 `theoretical_close_limit_mm=16` 提前结束。 |
| v2.36 | 2026-05-26 | 实施 TravelLearning 搜索距离/P_MAX 动态裁剪：真实行程按 `18-20mm` 现场假设搜索，旧 `0-16mm` 只保留为理论参考；闭合搜索、回退和摩擦异常重基准不再用 `16mm` 夹断，默认行程学习电流提高到 `1.5A`。 |
| v2.37 | 2026-05-26 | 计划完善 `HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 三段闭环和运行时 seed：回零显式依赖本次 PreSelfCheck；行程学习/健康检查成功后保存结构参数；seed 恢复样本计数，使下一次 PreSelfCheck 起扫电流使用历史学习值。 |
| v2.38 | 2026-05-26 | 实施三段闭环收尾：PreSelfCheck 最终 profile 合并已加载 seed 与本轮 runtime breakaway/dynamic 样本；TravelLearning/MotionHealthCheck 继续采样摩擦异常候选；新增测试覆盖回零前置条件、seed 起扫电流恢复、行程/健康 seed 持久化。 |
| v2.39 | 2026-05-26 | 实施 PreA bootstrap 与最终摩擦模型分离：PreA 起动扫描只更新 breakaway seed；静摩擦、动摩擦和最低稳定速度只由干净多区域样本计数支撑，未学习字段不再用安全上限回填。 |
| v2.40 | 2026-05-26 | 收紧最终模型采样入口：PreB 探边、回位和边界释放样本不再进入 `motion_samples/static_friction_samples/dynamic_friction_samples`；安全区多区域学习入口改为只依赖有效 low-confidence safe zone；旧 seed 的动态摩擦和最低稳定速度同样不再恢复为最终学习值。 |
| v2.41 | 2026-05-26 | 计划实施 PreB 多区域学习最低覆盖验收：默认至少 `3` 个有效干净位置区域；增加候选锚点网格、区域级接受日志，不足时回滚本阶段样本并将 `MultiRegionRoundTripLearning` 降级，避免打开端局部样本污染最终结构模型。 |
| v2.42 | 2026-05-26 | 实施 PreB 学习集中问题修正：候选锚点按区域局部搜索，不允许跨区迁移；严重 anomaly 导致区域不足时上报 `mechanism_anomaly`。同步提高默认 anomaly 触发阈值到明显电流尖峰，UI/API 增加 PreB 电流-行程曲线，并让回零、行程学习、健康检查记录并使用学习参数裁剪后的电流。 |
| v2.43 | 2026-05-26 | 实施 PreB 探测/记录边界和回位方式修正：入口到 opening 结构限位的首段只探测不记录 anomaly/trace；closing 限位回 opening 软限位的学习准备动作改为连续受控运动，禁用 detector/trace，避免分段启停尖峰干扰区域选点。 |
| v2.44 | 2026-05-26 | 计划修正 PreB 电流-行程曲线显示：前端按连续采样段断线绘制，不再把左右极限或不同扫描段直接连线；增加横纵网格和刻度，保留异常大峰值用于安全诊断。 |
| v2.45 | 2026-05-26 | 实施 PreB trace 后端分段和回程可视化：每个连续 probe 输出 `segment_id`，前端优先按段断线；closing 限位回 opening 软限位的连续回位仅记录 UI trace，不进入 anomaly map 或最终结构模型样本。 |
| v2.46 | 2026-05-26 | 实施 PreSelfCheck 极端反馈电流急停：新增 `safety.self_check_feedback_emergency_current_limit_a`，超过该阈值立即停止当前 probe，避免结构端点顶死后等待持续确认导致十安培级峰值。 |
| v2.47 | 2026-05-26 | 修正 PreSelfCheck 端点停机收尾：PreB 因持续硬反馈电流、emergency 电流或持续无推进停止后必须立即卸载/失能并采样，不再发送当前位置 hold；该类结果作为 PreB 探边诊断降级处理，不直接升级全局 `ActiveStop`。 |
| v2.48 | 2026-05-26 | 修正端点启动逃逸实现：StableShortStroke 中一侧低置信退开、反向硬反馈/无推进时记录 `endpoint_start_escape` 并设置起始疑似边界；退开阈值使用低置信起动有效位移，不再要求达到 PreB 主扩展有效推进距离。 |
| v2.49 | 2026-05-26 | 计划修正 PreB 远距离撞端点后的释放失败保护：`BoundaryRelease` 可按有效退开进展继续分段释放；若最终仍释放失败，则标记端点抱死降级，禁止继续第二方向主扫描收假边界，禁止进入多区域学习，最终只暴露 PreA 保守窗口。 |
| v2.50 | 2026-05-26 | 实施 PreB 单边界本地释放：方向切换前的 `BoundaryRelease` 使用当前停止点附近的本地反向释放窗口，不再依赖已成形双边 safe zone；释放成功后继续第二方向主扫描，释放失败时强制 profile 回到 PreA 小窗口，避免理论/历史边界放大。 |
| v2.51 | 2026-05-26 | 实施 PreB 电流-行程曲线本地颜色设置：曲线下方增加打开、闭合和未知/诊断颜色选择，浏览器本地持久化并立即重绘，不改变后端 trace 或控制策略。 |
| v2.52 | 2026-05-26 | 实施 PreB 探边与边界释放电流分离：普通 emergency 反馈急停默认下调到 `3.0A`，新增 `self_check.pre_b_boundary_release_current_limit_a=2.0A`，BoundaryRelease 日志输出专用释放电流并仍受全局硬保护裁剪。 |
| v2.53 | 2026-05-27 | 计划修正实机 `MotionHealthCheck` 失败诊断：健康检查应先选择软件限位内部且避开严重摩擦异常的往返窗口，失败时输出每段真实反馈统计和有效阈值；电流/力矩纹波判据允许基于已学习摩擦与噪声底给出下限，但不放宽实时堵转、接触、硬电流和软件限位保护。 |

## 1. 总体策略

V2 采用“设计、实现、测试交叉推进”的方式。任何功能进入代码实现前，必须先有本阶段设计说明、接口边界、日志要求和验收标准。

旧 `src/` 暂不归档、不删除、不重构。当前源码保持可构建状态，用于硬件对照和行为回归。V2 代码替换应逐步进行，避免一次性破坏当前可运行程序。

## 2. 阶段 0：文档基线切换

目标：

- 归档 V1 控制设计和 V1 实施计划。
- 建立 V2 控制设计和 V2 实施计划。
- 更新 `references/README.md` 和 `AGENTS.md` 的读取入口。

验收：

- `references/archive/` 中存在 V1 归档快照。
- 新增 V2 设计和计划文件。
- 后续控制系统工作默认指向 V2 文件。

## 3. 阶段 1：空文件和接口占位

目标：

- 在不改变现有可运行逻辑的前提下，为 V2 基础模块建立文件占位。
- 占位文件只包含命名空间、类名、接口注释和 TODO，不接入主程序。

建议模块：

- `hardware_interface`：继续保留电机抽象。
- `controller`：保留业务控制器接口。
- `controller/encoder` 或 `hardware_interface`：放置虚拟螺母位置编码器。
- `controller/safety`：放置调试约束策略。

验收：

- 项目仍可构建。
- 占位文件不改变运行行为。

当前落盘状态（2026-05-22）：

- 已新增 `src/controller/nut_position_encoder/nut_position_encoder.hpp/.cpp`。
- 已新增 `src/controller/safety/debug_constraint_policy.hpp/.cpp`。
- 这些文件只定义 V2 接口边界和 TODO，不接入现有控制流程。
- 已纳入 CMake 编译，用于持续检查接口占位不破坏构建。

## 4. 阶段 2：通信和反馈链路确认

目标：

- 确认达妙电机反馈中的多圈位置来源。
- 确认反馈刷新频率和旧帧清理策略。
- 明确 UI 显示原始反馈字段。

验收：

- 连接真实硬件后能持续获得电机位置、速度、电流、力矩和时间戳。
- 日志能证明反馈不是旧队列数据。
- 该阶段不更新螺母零点、不学习结构参数。

当前确认状态（2026-05-22）：

- 已确认现有达妙通信和反馈链路可以作为 V2 后续实现基础，不在本阶段重写。
- `DamiaoMotor::connect()` 在打开设备后读取运行时 `P_MAX/VMAX/TMAX`，并用运行时限制参数解析后续反馈。
- 达妙硬件层已有后台反馈线程，默认 `motor.feedback_poll_period_s = 0.02s`，即约 `50Hz`。
- `readFeedback()` 返回最新缓存反馈快照，并通过 `motor.feedback_stale_timeout_s` 判断反馈是否过期。
- 硬件记录 `references/04_hardware/real_hardware_test_issue_log_2026-05-20.md` 已记录 `bringup_feedback` 成功、后台反馈线程、运行时 `P_MAX=50` 等关键结论。
- UI/API/日志已暴露电机连续位置、协议位置和原始计数字段，可用于继续验证 `2*pi rad -> 2mm` 螺母行程映射。

剩余注意事项：

- 阶段 3 的虚拟螺母位置编码器只消费 `MotorFeedback.position` 等硬件层输出，不直接解析达妙帧。
- 若后续实测再次出现电机实际转动与 `MotorFeedback.position` 不一致，应先回到 `references/04_hardware/` 记录并排查硬件反馈链路，不应在螺母位置编码器里补偿未知误差。

## 5. 阶段 3：虚拟螺母位置编码器

目标：

- 实现电机多圈位置到螺母位置的映射。
- 支持启动临时零点和重新设零。
- 保存最近原始反馈用于 UI 和调试。

最小接口能力：

- `resetZero(motor_position_rad, nut_position_mm)`
- `update(motor_feedback)`
- `nutPosition()`
- `lastMotorFeedback()`
- `isFresh()`

验收：

- 单元测试覆盖 `2*pi rad -> 2mm` 的映射。
- 方向符号可配置并可测试。
- 空载实测时，滑块或点动运动日志能输出 `mm_per_rev_estimate`。

当前落盘状态（2026-05-22）：

- 已在 `src/controller/nut_position_encoder/nut_position_encoder.hpp/.cpp` 中实现 `LeadScrewNutPositionEncoder`。
- 编码器使用 `MotorFeedback.position` 作为电机侧多圈位置输入，按丝杆导程和方向符号映射为虚拟螺母位置。
- 编码器方向现在由 `motor.direction_sign * nut_position_encoder.direction_sign` 决定；其中 `nut_position_encoder.direction_sign` 只用于虚拟螺母位置方向修正，`position_command_sign` 仍只用于达妙命令极性。
- 编码器支持启动临时零点、显式重设零点、反馈新鲜度判断、最近电机反馈快照保留。
- 已新增 `test/test_nut_position_encoder.cpp`，覆盖 `2*pi rad -> 2mm`、方向符号、零点重设、启动参考和反馈过期。
- 已等价接入现有 `GripperController::updateFeedback()` 路径：控制器读取 `MotorFeedback` 后，统一由 `LeadScrewNutPositionEncoder` 更新 `current_nut_stroke_`。
- 现阶段仍保留旧的目标位置换算函数，用于向硬件发送目标电机位置；接入目的是统一反馈侧螺母位置来源，不改变现有运动控制行为。
- `NutPositionFeedback` 中的零点、螺母速度、电机增量、增量圈数、`millimeters_per_revolution_estimate` 和新鲜度已进入 `GripperStateSnapshot` 与 `/api/view`。
- Web UI 的“夹爪状态”区域已显示编码器零点、电机增量圈数、导程估计和编码器新鲜度，用于现场验证 `2*pi rad -> 2mm`。
- 达妙原始反馈帧已从硬件层 `MotorFeedback` 透传到控制器快照与 `/api/view`，包含 CAN ID、DLC 和十六进制 data bytes。
- 完整回归 `ctest --preset dev-zig` 已通过 `10/10`。
- 新增 `MotorBringupMode` 相对圈数位置到位入口。该入口只用于空载调试，不设置零点，不更新 `StructureProfile`，不作为 PreSelfCheck 实现路径。UI 应提供 `+1rev/-1rev/+2rev/-2rev` 和自定义圈数按钮，日志应输出起点、目标、终点、电机增量、虚拟螺母增量和 `mm_per_rev_estimate`。
- 已补充：达妙运行时 `P_MAX/VMAX/TMAX` 显示在 UI 的夹爪/电机状态区域；`MotorBringupMode` 相对圈数位置到位命令在目标超过 `[-P_MAX, P_MAX]` 时拒绝，并根据圈数和速度自动估算默认超时。
- 已修正：UI、Web API 和 commander 的圈数到位缺省超时均传 `0`，避免入口层重新引入固定 `3s` 默认值；控制器日志会输出实际生效 `timeout_s`，`2rev`、`1rad/s` 时应约为 `15.7s`。
- 已修正：空载圈数到位的反馈电流中止默认值为 `1.5A`，配置上限为 `2.0A`，普通反馈电流超限采用持续超限判据，全局硬保护仍立即触发停机。

剩余接入任务：

- 后续 PreSelfCheck V2 应直接使用该编码器输出作为反馈侧螺母位置，不再新增局部换算逻辑。
- 目标电机位置换算后续可逐步收敛到同一编码器/转换模块，但必须先保持现有硬件行为可回归。

## 6. 阶段 4：调试约束策略

目标：

- 将限流、限速、主动停止、软件限位等控制器约束做成明确策略对象。
- 默认全部开启。
- 调试模式可关闭部分控制器约束。
- 全局硬保护不可关闭。

验收：

- UI/API 能读取当前约束状态。
- 关闭约束会写入日志。
- 关闭约束后仍不能绕过硬保护。

当前落盘状态（2026-05-22）：

- 已在 `src/controller/safety/debug_constraint_policy.hpp/.cpp` 中实现 `BasicDebugConstraintPolicyEvaluator`。
- 策略对象当前只负责纯逻辑判断，不直接发送电机命令，不访问硬件，不修改现有控制器动作。
- 已锁定以下基础不变量：
  - 普通模式不允许关闭控制器级限流、限速、主动停止、软件限位或通信超时约束。
  - `UnloadedBringup` 和 `AdminRecovery` 模式下关闭控制器级约束必须有风险确认。
  - 全局硬保护不可关闭。
  - 全局硬保护电流、温度和命令持续时间上限必须配置为正值。
- 已新增 `test/test_debug_constraint_policy.cpp`，覆盖默认策略、硬保护不可关闭、硬保护配置缺失、普通模式拒绝放宽、调试/管理员模式风险确认等场景。
- 完整回归 `ctest --preset dev-zig` 已通过 `10/10`。

剩余接入任务：

- 将策略状态接入 `GripperController` 快照，供 UI/API 显示。
- 将策略评估结果用于后续 V2 运动命令入口。
- 策略切换写入审计日志。
- 硬保护阈值继续由 `config` 或经过验证的 `StructureProfile` 提供，不能在控制逻辑中硬编码。

## 7. 阶段 5：PreSelfCheck V2

目标：

- 在虚拟螺母位置编码器稳定后重新设计 PreSelfCheck。
- PreSelfCheck 只建立低置信初步结论，不直接等同于最终软件限位。
- 每一步必须有清晰状态、进度日志、失败原因和取消路径。

验收：

- 空载时能识别理论安全区并允许编码器映射验证。
- 有结构时能低能量试探，不因单帧反馈尖峰误判。
- 用户停止后立即停止继续发新命令。

当前第一版落盘策略（2026-05-22）：

- `PreSelfCheck` 直接使用已接入控制器反馈路径的虚拟螺母位置编码器，不再新增局部电机位置到螺母行程换算。
- 执行范围收敛为：静止反馈噪声采样、双向低能量起动扫描、稳定短行程低置信验证、低置信 `StructureProfile` 更新。
- 暂不主动执行完整 `PreliminaryLimitSearch` 和 `MultiRegionRoundTripLearning`。理论开闭边界只作为低置信 profile 的保守配置边界，用于 UI 显示和空载/结构移除确认后的调试移动，不作为实测机械限位。
- 未确认空载时，PreSelfCheck 后的手动螺母定位仍只允许围绕当前虚拟螺母位置的小窗口移动；完整软件限位范围仍必须等待 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck` 后确认。
- 每个 probe 仍在停止/保持后读取新鲜反馈并确认停稳，未停稳样本只作为诊断，不进入低置信 profile。
- 用户停止、`disable()`、断开连接、硬件故障和反馈超时仍通过控制器取消标志中止后续命令刷新，并进入 `ActiveStop` 或错误路径。

当前最终版落盘策略（2026-05-22）：

- PreA 保持第一版低能量可控性路径：噪声采样、双向低能量起动扫描、稳定短行程低置信验证。PreA 起动扫描只更新 bootstrap breakaway seed，用于让当前流程和下一次自检从更合理电流附近开始；启动点若在结构限位或卡点，PreA 电流可能被端部/峰值污染，不能写成最终静摩擦、动摩擦或最低稳定速度模型。
- PreB 主扩展启用 bring-up-like 连续受控扫描，不要求机构从丝杆中位开始；回零前虚拟螺母坐标只是临时参考，初始 `7.98mm` 不能解释为物理中位。主扫描从当前位置按相对距离扩展，第一方向探测结束后可直接从当前位置穿越已验证路径并向另一方向扩展，主扫描距离受 `pre_b_max_expansion_distance_mm` 限制，`pre_b_boundary_step_mm` 只用于边界候选 refine 和软卡点复核。
- PreB 若默认第一方向为 opening，则从当前位置到 opening 结构限位的首段属于探测路径，只更新 low-confidence bound、limit 样本和安全诊断，不向 `FrictionAnomalyDetector`、UI trace 或最终结构模型样本写入数据。若 PreA 已提示 opening 起始端点并使第一方向改为 closing，则仍按“先离开端点、再回头确认”的调度执行，记录边界以实际 detector 启用路径为准。
- PreA 若在低能量双向 probe 中发现一侧疑似端点、另一侧已经有可控退开样本，`StableShortStrokeMotion` 不再继续做成对短行程扫描；实现应记录 `endpoint_start_escape` 降级日志并直接进入 `PreliminaryLimitSearch`。该路径只用于端点启动，不能把端点附近的微小往复样本写成高置信结构参数。端点退开判定必须允许 `motion_start_distance_mm` 量级的方向正确、单调、已停稳小位移；若该小位移伴随当前 probe 的持续硬反馈停机，只要不是反馈矛盾、用户停止、硬件/通信错误，也可作为退开候选，用于让 PreB 第一方向切到远离端点的一侧。
- PreB 方向调度需要感知 PreA 的起始疑似边界。若 opening 被标记为起始疑似边界，则先执行 closing 连续扫描到另一侧边界，再释放并执行 opening 扫描确认起始侧；反之亦然。若没有起始边界标记，保持默认 opening 后 closing 的顺序。无论第一侧是否命中边界，除不可降级错误外都必须尝试第二侧。
- 默认配置将 `pre_b_max_expansion_distance_mm` 调整为接近当前预期实际行程，使 PreSelfCheck 可尝试得到接近实际行程长度的低置信覆盖宽度。低置信边界可在临时坐标中小于 `0mm` 或大于 `16mm`；该坐标不会作为最终软件限位，后续回零和行程学习负责重基准到真实打开零点和实测闭合端。旧 `0-16mm` 只能作为参考行程，不再作为硬边界。
- PreB 相对扫描目标若超过达妙 `P_MAX`，实现应按当前方向把扫描距离裁到 P_MAX 内最大可达值并继续扫描，日志输出 `pmax_scan_distance_limited`、请求距离和裁剪后距离。只有裁剪后没有有效扫描距离时，才以 P_MAX 窗口不足结束该方向。
- 如果某方向达到相对扩展上限、无有效推进、触发自检硬反馈上限、反馈方向矛盾或达妙运行时 `P_MAX` 窗口不足，该方向停止扩展并记录低置信边界；普通超时但已经产生停稳、单调、方向正确的有效位移时，保留 partial bounds 和摩擦异常候选，并继续后续分段，不回退丢弃已探测区域。该探边样本不进入最终 `motion_samples/static_friction_samples/dynamic_friction_samples`。
- 如果某方向出现非硬保护的 `SafetyJamDetected`，PreB 先以受控小步执行卡点复核；复核能继续推进则记录为低置信 motion/anomaly 候选并继续扩展，复核仍无有效推进才收为疑似结构限位。
- 默认配置将 `safety.self_check_current_limit_a` 提高到接近全局硬保护但仍低于硬保护的 `1.9A`；新增 `safety.self_check_feedback_hard_current_limit_a` 作为仅限 PreSelfCheck 的反馈硬停上限，避免为了穿越局部卡点而放大全局 `safety.max_motor_current_a`。
- 默认配置将 `self_check.pre_b_expansion_speed_mm_s`、`pre_b_boundary_release_speed_mm_s` 和 `pre_b_soft_jam_retry_speed_mm_s` 调整为 `1.0mm/s`。在当前 `2.0mm/rev` 丝杆下约为 `3.14rad/s` 电机速度，目标是在早期 `5.0mm/s` 高冲击档和 `0.6mm/s` 易卡保守档之间取得更适合穿越局部卡点的现场默认值。
- `dynamic_friction_speeds_mm_s` 默认调整为 `[0.6, 1.0, 1.2, 1.5]`，用于 PreB 安全区内多速度学习；动态采样速度过滤使用 `max(min_speed_scan_stop_mm_s, pre_b_expansion_speed_mm_s)` 作为 PreB 上限，再受全局螺母限速约束。PreA 低能量起动仍从 `min_speed_scan_start_mm_s = 0.2` 开始，不因 PreB 连续扫描调速而取消低速可控性确认。
- PreB 主扫描对软 jam/contact 判据采用“诊断优先”：本段可继续尝试到连续扫描目标，软判据用于 anomaly 候选和后续判断；硬反馈电流、方向矛盾、无有效推进、理论护栏和达妙运行时 `P_MAX` 窗口仍是停止/收边界条件。
- PreB 主扩展、回位和安全区多速度采样对反馈硬电流使用持续确认，不再因单帧或短时超过 `safety.self_check_feedback_hard_current_limit_a` 立即收边界；默认 `self_check.pre_b_hard_current_confirm_time_s` 调整到约 `0.25s`，用于过滤现场局部卡点的短持续峰值。持续硬电流必须叠加无噪声级有效推进才中止；若高电流期间位置仍按命令方向单调推进，该段作为 pass-through partial progress 更新低置信边界并进入摩擦异常候选。若反馈电流超过 `safety.self_check_feedback_emergency_current_limit_a`，立即停止当前 probe 并保留诊断，不等待持续确认；该阈值用于限制端点顶死极端峰值，不改变命令电流上限。现场默认从 `4.0A` 下调到 `3.0A`，减少普通探边撞端时的反馈峰值。持续硬电流、emergency 电流或持续无推进触发停止后，probe 收尾必须先失能/卸载并等待反馈停稳，不得用当前位置 hold 继续压住端点。若持续超限、速度持续塌陷且无有效推进，则仍以 `SafetyJamDetected` 中止并保留诊断。
- PreB 物理边界判定不能只看本段最大电流。短时尖峰电流用于卡点/摩擦候选；低置信结构边界需要持续硬电流确认，或无有效位移叠加持续速度塌陷/软 jam 复核失败/理论护栏/P_MAX 窗口限制。连续扫描到达边界候选后，应停止并用小步/软卡点复核确认是否仍能穿越。
- PreB 日志必须输出每段目标与实际电机增量、实际螺母位移、mm/rev 估算、判定结果和 `P_MAX` 余量。若无机构测试仍只转一圈，应能从日志直接判断是达妙绝对位置窗口不足、虚拟螺母比例/方向异常，还是自检软判据收边界。
- 构建低置信安全区后执行安全区内多区域、多速度往复采样，补充静摩擦、动摩擦和最低稳定速度样本并继续更新局部摩擦异常候选。进入结构模型统计的样本必须是停稳、方向正确、未触发限位/堵转、且不与严重 anomaly 重叠的干净段；边界释放、探边扫描、回到学习锚点、端点启动退开和 PreA 微动只作为 bootstrap/低置信边界/诊断，不写入最终摩擦或最低稳定速度模型。
- `FrictionAnomalyDetector` 接入每个自检 probe 的运动中反馈采样，记录候选异常数量和最大严重度到控制器快照/API；当前不持久化、不补偿。PreSelfCheck 期间的候选记录保留峰值电机位置，`HomingOpenStop` 设置真实打开零点后按电机位置重算候选的螺母位置，使初步 map 能作为 `TravelLearning` 和 `MotionHealthCheck` 的参考输入。PreB 只使用实时复核结果避免第一次软堵转就收边界，不凭历史/内存 map 单独放行。
- PreB 是 PreA 之后的覆盖增强，不是 PreA 成败条件。若 PreB 由于普通超时、低置信微动、可扩展宽度不足或安全区太窄而无法完成，应优先使用 partial bounds；只有 partial bounds 不足以形成安全区时才降级为 PreA 保守窗口并继续完成预自检；降级结果不能放大安全区。
- PreB 探边结束后，如果开向或闭向被标记为疑似物理边界，或某方向因卡点高电流停机但已保留有效 partial progress，进入另一方向探测或 `MultiRegionRoundTripLearning` 前先执行 `BoundaryRelease`：按 `pre_b_boundary_release_distance_mm` 从当前卡住点反向退回，速度由 `pre_b_boundary_release_speed_mm_s` 控制，释放命令电流由 `pre_b_boundary_release_current_limit_a` 控制，仍受 `safety.max_motor_current_a`、持续反馈硬电流确认和停稳逻辑约束。该释放 probe 只解除端部/卡点顶死状态，不写入结构参数统计，也不写入 anomaly map 或 PreB trace。该释放阶段不得依赖双边 safe zone 已经有效；如果只知道单侧边界，应按当前停留点、释放目标和反馈噪声裕量构造本地临时护栏，使释放成功后能继续第二方向主扫描。若释放 probe 自身因持续硬反馈电流、emergency 电流或持续无推进失败，但已经在释放方向产生有效退开距离，应继续尝试剩余 release 距离；若最终仍无足够退开，应记录为释放失败/端点抱死诊断并降级保留已有单侧低置信边界。释放失败后不得继续第二方向主扫描，也不得把紧邻已撞端点的反向小位移高电流 stop 记录为另一侧结构限位；最终 profile 必须强制回到 PreA 保守窗口，覆盖 preliminary/safe/software limits，`MultiRegionRoundTripLearning` 必须跳过。除反馈矛盾、用户停止、硬件/通信/配置错误外，不得直接中断整个 `PreSelfCheck` 并进入全局 `ActiveStop`。
- `MultiRegionRoundTripLearning` 开始前若当前位置在 closing 侧，回到 `safe_zone.open_limit` 必须使用单次连续受控运动，不能复用按 `pre_b_boundary_step_mm` 分段的回位 probe。该连续回位关闭 detector 和最终模型样本入口，只作为进入学习起点的准备动作；同时以 `ui_trace_only` 写入 PreB 电流-行程 trace，帮助人工确认闭合侧回打开侧的连续路径。后续真正的多速度学习 probe 才写入 anomaly/trace 和最终样本。
- PreB 主扫描必须循环消耗该方向剩余相对扩展距离。每个 probe 结束后用实际推进量更新 `expanded_mm` 和低置信边界；若返回 `OperationTimedOut` 或非物理边界的 `SafetyJamDetected` 但已经有有效推进，则从当前停稳点继续同方向扫描，直到达到配置距离/P_MAX、命中真正物理边界或连续没有有效推进。这样局部卡点不会把该方向误认为扫描完成。
- `MultiRegionRoundTripLearning` 以 `dynamic_friction_speeds_mm_s` 做分档采样。采样前用当前 `FrictionAnomalyDetector::records()` 过滤锚点和计划运动段；采样后如果本段与新出现的 anomaly 记录重叠，则跳过该 motion/dynamic friction 样本。避让距离由 `friction_anomaly_avoid_margin_mm` 配置。第一版使用稳健筛选和分档统计输出低置信结构基值，不在 PreSelfCheck 中拟合完整 Stribeck/LuGre 摩擦模型。
- `MultiRegionRoundTripLearning` 必须区分候选锚点数量和有效学习区域数量。候选锚点按 `self_check.pre_b_learning_anchor_count` 在 safe zone 内分布搜索，默认多于最低要求以便绕开卡点；只有某个锚点附近至少成功写入一组干净 motion/static/dynamic 样本，才计入有效区域。有效区域数必须不小于 `self_check.pre_b_min_learning_regions`，默认 `3`；若不足，回滚本阶段样本并把该阶段标记为降级，PreB low-confidence bounds 仍可保留。
- 历史配置键 `static_friction_current_start_a/stop_a/step_a` 暂时保留以兼容 YAML，但语义改为 PreA bootstrap 起动扫描电流范围。最终静摩擦字段只在 `static_friction_sample_count > 0` 时有效；未学习时 UI/API 应显示未学习，不得用 `safety.self_check_current_limit_a` 或其它安全上限填充。
- 连续扫描停止前已有有效推进时，即使 probe 结果是持续硬电流 `SafetyJamDetected`，也应先把最终停稳位置写入 opening/closing partial bound，再按疑似边界处理。否则会出现实际从 `0.16mm` 走到 `12.07mm`，但闭合低置信边界仍停留在扫描起点的错误。
- `MultiRegionRoundTripLearning` 的 anomaly 避让应以 `friction_anomaly_learning_avoid_ratio` 为严重度门槛；低于门槛的轻微候选继续记录到 map，但不阻止整段阶梯速度学习。若所有锚点仍被严重 anomaly 覆盖，该阶段可降级，但不能影响已经形成的 PreB low-confidence bounds。

- 已实施：multi-region anchor 搜索改为 region-local。至少 3 个必需区域分别在各自区域内寻找可执行完整往返段的干净锚点，不能把中/闭合侧 preferred anchor 移到打开侧干净位置。区域找不到锚点时输出 `region_skipped`；若有效区域数不足，输出 `mechanism_anomaly=true`、回滚本阶段最终模型样本并保留 low-confidence bounds。

- 已实施：摩擦异常默认阈值调整为只记录明显尖峰，避免 1.5x 左右波动填满 `friction_anomaly_max_records` 并过度影响学习区域选择。默认 `friction_anomaly_current_ratio_threshold` 和 `friction_anomaly_minor_ratio` 已提高到 `2.0`，`friction_anomaly_learning_avoid_ratio` 保持更高门槛用于阻断学习。

- 已实施：Web UI/API 增加 PreB 预探索电流-行程 trace。控制器在 PreB 探边、安全区学习反馈循环和学习前连续回位中限长采样，UI 使用 canvas 绘制行程横轴、电流纵轴，并显示样本数、段数、最大电流和覆盖范围，辅助人工判断卡点。后端为每个连续 probe 分配 `segment_id`；前端优先按 `segment_id` 断线，并以方向变化、阶段变化或位置大跳变作为旧数据兜底，避免把左右极限点直接连接成斜线。坐标轴显示 mm/A 网格和刻度，反馈大峰值只作为原始诊断显示，不在 UI 层裁剪隐藏。

- 已实施：PreB opening 首段探测和 BoundaryRelease 不写入 anomaly/trace；学习前从 closing 侧回 opening 软限位改为连续受控运动，并仅写入 UI trace，不写入 anomaly map 或最终结构模型样本。

- 已实施：PreB 电流-行程曲线下方增加本地颜色设置。打开、闭合和未知/诊断曲线颜色保存在浏览器 `localStorage`，调整后立即重绘现有 trace；该设置只影响 UI 判读，不写入运行配置、不影响控制器采样和安全判据。

- 已实施：PreB 普通探边和 BoundaryRelease 的电流 envelope 分离。普通 emergency 反馈急停默认下调到 `3.0A`；BoundaryRelease 使用 `self_check.pre_b_boundary_release_current_limit_a`，默认 `2.0A`，并在释放 start/result 日志中输出 `current_limit_a`，确认释放动作没有继续复用普通探边电流。

- 已实施：`HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 的日志显示 `current_source`。PreB 学到静/动摩擦后，流程电流由学习摩擦加裕量并受 `safety.*_current_limit` 裁剪；回零保持小电流找端点，不得使用 PreA/PreB 的 `self_check_current_limit=1.9A` 作为默认碰撞电流。
- PreB 已形成有效 safe zone 时，`manual_nut_stroke_range` 应暴露完整低置信 safe zone，供用户验证已探测范围。只有 PreB 未形成有效 safe zone、降级为 PreA 保守窗口时，才保留当前位置附近的小范围手动窗口。
- 以下情况不可降级：反馈方向矛盾、理论行程外、用户主动停止、全局硬保护、硬件故障、通信/反馈错误和配置错误。
- PreSelfCheck 成功后仍只设置 `PreSelfCheckCompleted`，不设置 `Homed`、`TravelLimitsLearned` 或 `MotionHealthChecked`。
- `HomingOpenStop` 在本次启动周期 PreSelfCheck 后必须按 opening 相对搜索真实开端。历史 seed 不允许替代本次 PreSelfCheck。搜索距离至少覆盖理论行程或 PreB 已知低置信宽度，并使用临时 stroke window 避免被回零前 `0/16mm` 虚拟坐标夹断；在 opening 端命中后先立即卸载输出并采样端部反馈，再调用虚拟螺母位置编码器重新设零。
- `HomingOpenStop` 的 opening stop 判据除通用 jam/contact 外，还应包含“反馈电流贴近回零命令电流、速度塌陷、位置持续无有效推进并超过 `homing.jam_confirm_time_s`”。该低电流端部判据用于回零，不放宽其他普通运动的 jam 策略。
- `HomingOpenStop` 设置零点并重基准 `FrictionAnomalyMap` 后，应按 `homing.backoff_distance_mm` 向 closing 方向主动回退释放端部抱死。回退成功后回零才算完成；回退失败则进入 `HomingFailed`，并在日志中输出零点、检测原因、回退距离和失败原因。
- `TravelLearning` 的闭合方向行程学习应复用 PreB 卡点容忍扫描：已知或实时发现的局部卡点只记录异常候选，不直接终止行程学习；只有持续无有效推进、速度塌陷和电流不回落同时满足时，才把当前位置收为闭合结构限位。
- 默认 `TravelLearning` 档位调整为 `travel_learning_speed_mm_s = 1.2`、`safety.travel_learning_current_limit_a = 1.5`。这仍低于 PreSelfCheck 的 `1.9A` 命令电流上限和全局 `2.0A` 硬保护，但避免 `0.6A/1.2A` 在已知卡点处过早停下。实现应在行程学习开始和完成/失败时输出 target、search distance、reference theoretical travel、speed、current limit、start/end/measured、target_reached、contact/jam/force 判据，便于下一轮实机日志判断是否仍需调档或改判据。
- 行程学习闭合搜索新增 `self_check.travel_learning_search_distance_mm`，默认 `20.0mm`。搜索距离应取该配置、PreB 低置信宽度、旧理论参考行程和最小行程要求中的最大值；`mechanism.theoretical_close_limit_mm=16.0` 不再作为闭合搜索终点。若运动到搜索目标而未触发端部，结果应在日志中标记为 `target_reached=true` 并保持低置信，提示搜索距离可能仍需调大。
- `TravelLearning` 成功后必须调用运行时 seed 保存，将软件限位、实测行程、bootstrap 起动电流以及静/动摩擦样本计数同步到 `self_check.learned_profile_path`。下一次 PreSelfCheck 加载 seed 时只作为低置信起扫参考，不设置 `homed_` 或 `travel_limits_learned_`；seed v3 以前的静摩擦字段只迁移为 bootstrap，旧动态摩擦和最低稳定速度字段清空，不再自动当作最终结构模型。
- `MotionHealthCheck` 应由每段软件限位内往返运动的真实反馈生成样本，至少统计速度跟踪误差、电流纹波、力矩纹波和最高温度；成功后刷新 seed 中的健康摘要。健康 seed 只能做显示/趋势参考，不能替代本次健康检查。
- 当前现场样机达妙上位机已把 `P_MAX` 调到约 `65rad`，默认 `motor.max_position_rad` fallback 同步为 `65.0`。真实硬件连接后仍以 `DamiaoMotor::connect()` 读取的运行时 `P_MAX` 覆盖 fallback；所有 PreB/TravelLearning 扩展仍必须受 `[-P_MAX, P_MAX]` 电机位置窗口约束。
- `ClampByForce` / `ClampBySpeed` 应把已知 anomaly 作为接触误判抑制输入。经过 anomaly 窗口时短时电流峰值不能单独作为夹紧接触或结构限位，必须等待离开异常区域后的持续电流/无进度判据确认。

当前验证：

- `scripts/build.ps1` 通过。
- `scripts/test.ps1` 通过 11/11。

## 8. 阶段 5A：局部摩擦异常映射

目标：

- 先于 PreB 建立 `FrictionAnomalyMap/Detector` 纯算法能力。
- 在恒速或近似恒速运动样本中识别“电流局部升高、位置继续推进、电流随后回落”的异常候选。
- 为后续 PreB、`TravelLearning` 和 `MotionHealthCheck` 提供位置-摩擦异常候选输入，避免把已知局部卡点直接当成结构限位。

第一版实施边界：

- 模块放在 `controller/self_check`，只依赖 `common`、配置单位类型和 self_check 业务类型。
- 配置项来自 `self_check.friction_anomaly_*`，默认启用检测但不改变控制决策。
- 只维护内存记录，不做文件持久化；后续持久化应独立于 `StructureProfile`。
- 检测器只接受外部采样输入，不直接访问硬件、不发送命令、不触发主动停止。
- 未回落的电流持续升高段在 `finish()` 时丢弃，不作为摩擦异常候选；控制流程仍需按限位/堵转风险处理。
- 第一版记录 occurrence_count=1、confirmation_state=candidate；多次确认和双向确认留给 PreB/TravelLearning/MotionHealthCheck 接入阶段。

验收：

- 单元测试覆盖平稳电流不误报、局部峰值记录、窄峰过滤、严重度分级、方向独立记录、禁用检测、未闭合峰值丢弃。
- 完整 `scripts/test.ps1` 回归通过。

## 9. 后续阶段

后续功能必须逐个设计和验收：

- `FrictionAnomalyMap` 接入 PreB、TravelLearning、MotionHealthCheck
- `HomingOpenStop`
- `TravelLearning`
- `MotionHealthCheck`
- 运行时学习 seed 的持久化字段扩展和版本兼容
- 目标力夹紧
- 恒速夹紧
- 管理员恢复
- UI 操作流

每个阶段完成后，应在 `references/04_hardware/` 或 `references/06_implementation/` 中记录测试现象、问题、修复和剩余风险。
