# PreSelfCheck 按设计方案实现记录

## 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v1.0 | 2026-05-20 | 按 `references/03_control/control_architecture_design.md` 第 10 章补足预自检阶段化实现和过程日志。 |
| v1.1 | 2026-05-20 | 根据真实硬件日志，将 `BidirectionalMoveEnable` 改为同一速度/电流参数点下闭合-打开成对试探，并拆分起动识别与稳定短行程识别。 |
| v1.2 | 2026-05-20 | 修正保守默认 profile 抬高扫描起点的问题；自检探测期间周期刷新受限位置力控命令；失败路径增加失能。 |
| v1.3 | 2026-05-20 | 将 `StableShortStrokeMotion` 改为短行程、低速起步、闭合/打开成对扫描，并补充失败摘要日志。 |
| v1.4 | 2026-05-20 | 修正短行程目标到达容差过大导致几微米位移被判 `Ok` 的问题，并去除电流上限处的重复扫描。 |
| v1.5 | 2026-05-20 | 允许稳定短行程低置信样本继续生成保守 profile；初步限位可使用理论边界；自检成功后主动失能。 |
| v1.6 | 2026-05-21 | 明确预自检安全区为低置信临时安全区；增加电机原始编码器位置日志；方向反馈与试探方向相反时立即停止本轮自检，避免持续推向同一侧。 |
| v1.7 | 2026-05-21 | 明确 PreSelfCheck 使用硬件层输出的连续多圈 `MotorFeedback.position` 计算行程；达妙单帧位置和原始计数只用于诊断，避免反馈回绕导致行程估算丢圈。 |
| v1.8 | 2026-05-21 | 明确 PreSelfCheck 读取的是硬件后台反馈服务维护的最新连续多圈快照；控制器不直接解析 CAN 帧，也不维护单独回绕状态。 |
| v1.9 | 2026-05-21 | 修正未停稳样本被接受的问题：`runSelfCheckMotionProbe()` 只有在停止输出并确认停稳后才允许输出可被统计的样本；未停稳时记录 `settled=false` 和停稳错误，不参与起动、低置信边界或运动样本统计。 |
| v1.10 | 2026-05-21 | 修正达妙硬件层停机反馈语义：`disable()` 不再把本地缓存速度、电流、力矩清零；控制器停稳等待必须读取停机后的新硬件反馈帧，并在自检日志中输出目标电机位置用于方向核对。 |
| v1.11 | 2026-05-21 | 将 `PreSelfCheck` probe 间收尾从直接失能改为低电流主动保持停稳；避免失能后机构反推或继续朝上一目标漂移，整个自检完成或失败时再统一失能。 |
| v1.12 | 2026-05-21 | 修正主动保持实现细节：保持阶段固定进入保持时的位置目标，等待期间周期性刷新同一 PositionForce 保持命令；保持速度限幅使用配置值而非 0，保持电流仍受自检电流安全包络限制。 |
| v1.13 | 2026-05-21 | 分离试探电流和主动保持电流：`motion_hold_current_a` 用于 probe 间停稳制动，并由 `safety.self_check_current_limit_a` 截断。 |
| v1.14 | 2026-05-21 | 将 `BidirectionalMoveEnable` probe 改为起动即停：达到 `motion_start_distance_mm` 后立即主动保持，稳定短行程验证仍由后续阶段负责。 |
| v1.15 | 2026-05-21 | 根据真实硬件日志将默认 `motor.position_command_sign` 修正为 `1`，使 PositionForce 目标电机位置方向与多圈反馈位置方向一致。 |
| v1.16 | 2026-05-21 | 新增 `self_check.motion_settle_speed_threshold_mm_s`，自检停稳判断不再复用 `safety.jam_speed_threshold_mm_s`，避免残余运动导致下一段 probe 方向判断失真。 |
| v1.17 | 2026-05-21 | 主动保持停稳失败时增加零输出/失能停稳 fallback，避免 PositionForce 保持目标在低电流下继续拖动机构。 |
| v1.18 | 2026-05-21 | 停稳判断新增位置差分速度 `settled_position_delta_speed_mm_s`，避免达妙反馈速度字段滞后导致真实静止后仍超时。 |
| v1.19 | 2026-05-21 | `BidirectionalMoveEnable` 新增低能量反向漂移降级：一个方向已起动确认后，反向小幅漂移不再触发致命方向错误，而是标记低置信疑似边界并停止继续升流。 |
| v1.20 | 2026-05-21 | `BidirectionalMoveEnable` 新增早期低能量反向漂移降级：尚未确认任一方向时，小幅反向漂移先作为疑似边界/低置信自检结果处理，并立即停止继续升流；probe 循环内发现超过反向阈值后立即结束当前命令，避免继续向风险方向施力。 |
| v1.21 | 2026-05-21 | 跟随控制设计 v1.24，明确 PreSelfCheck 使用 `MotorFeedback.position` 的虚拟多圈连续位置；该位置由硬件层按配置的数据源和回绕范围生成，当前样机默认使用原始 `q_uint` 单圈计数和 `2*pi rad` 回绕累计。 |
| v1.22 | 2026-05-21 | 跟随控制设计 v1.26，新增 `self_check.low_confidence_motion_distance_mm`，允许 `BidirectionalMoveEnable` 在停稳后同向微动达到低置信阈值时记录微动起动样本并停止继续扫参；反方向微动仍进入疑似边界/漂移降级。取消或停稳失败时保留最后真实反馈位置，避免日志终点误显示为零。 |
| v1.23 | 2026-05-21 | 跟随控制设计 v1.27，修正达妙虚拟编码器默认数据源：状态帧 `q_uint` 是协议位置量程编码，默认改用 `protocol_position` 和 `2*motor.max_position_rad` 回绕；`q_uint` 仅作为诊断字段。 |
| v1.24 | 2026-05-21 | 跟随控制设计 v1.30，达妙状态帧 `q_uint` 必须按连接时读取的运行时 `P_MAX/VMAX/TMAX` 解码；当前样机 `P_MAX=50 rad`，`encoder_wrap_range_rad=0` 表示由运行时 `2*P_MAX` 推导，`p_m/xout` 寄存器仅用于诊断校验。 |
| v1.25 | 2026-05-21 | 跟随控制设计 v1.31，修正第一阶段反馈电流判据：`BidirectionalMoveEnable` 接受同向微动样本时不再要求达妙反馈电流峰值低于 `self_check_current_limit + ripple`，该配置只约束命令限流；反馈电流/力矩峰值保留为诊断和全局硬保护依据。 |

## 1. 设计依据

本次实现以 `references/03_control/control_architecture_design.md` 第 10 章为准：

- `LimitedProbe`
- `BidirectionalMoveEnable`
- `StableShortStrokeMotion`
- `PreliminaryLimitSearch`
- `TheoryTravelCheck`
- `SafeZoneBuild`
- `MultiRegionRoundTripLearning`
- `StructureProfileUpdate`

`MotorBringupMode` 仍只用于电机通信链路和空载调试，不作为 `PreSelfCheck` 的控制路径依据。

## 2. 本次实现内容

控制器 `runPreSelfCheck()` 已改为阶段化执行：

- 静止反馈噪声采样。
- 按配置扫描速度和电流，分别寻找打开/闭合方向的最低可动参数。
- 使用 `PositionForce` 控制路径发送“目标行程 + 螺母速度上限 + 电流上限”命令。
- 每次试探采样反馈，记录位移、平均/最大螺母速度、最大电流、最大力矩、方向一致性、单调性、速度稳定性、电流稳定性和疑似限位/堵转。
- 执行稳定短行程双向验证。
- 执行低速限流的初步限位搜索，并和理论行程比较。
- 回到安全区中部后执行多区域往复采样。
- 根据有效样本生成 `StructureProfile`，当前仍标记为低置信/保守输出。

## 2.1 v1.1 调整

真实硬件日志显示，连续向同一方向试探会把机构逐步推向一侧，后续即使提高电流也可能出现位移变小或不动。为避免该问题，`BidirectionalMoveEnable` 调整为：

- 外层按 `self_check.min_speed_scan_start_mm_s` 到 `self_check.min_speed_scan_stop_mm_s` 扫描速度。
- 内层按 `self_check.static_friction_current_start_a` 到 `self_check.static_friction_current_stop_a` 扫描电流，且受 `safety.self_check_current_limit_a` 截断。
- 每一个速度/电流参数点都执行闭合一步、打开一步的成对试探。
- 起动识别使用 `self_check.motion_start_distance_mm`，只要求方向一致、位移超过阈值且没有限位/堵转。
- 稳定短行程仍在 `StableShortStrokeMotion` 阶段按更严格条件判断。
- 起动成功后将对应方向的起动电流、起动力矩和最低起动速度写入运行期 `StructureProfile`。
- 当前实现会把轻量自检种子写入 `self_check.learned_profile_path`，供下一次 `PreSelfCheck` 调整扫描起点使用。该文件只用于缩短下一次学习路径，不代表完整标定结果。

## 2.2 v1.2 调整

第二轮真实硬件日志显示，`BidirectionalMoveEnable` 已经双向通过，但 `StableShortStrokeMotion` 在 `0.4 A` 限流下只运动约 `0.16 mm`，没有完成目标小行程。日志还显示扫描起点为 `0.35 A`，这来自保守默认 `StructureProfile`，不是配置的 `0.1 A`。

处理如下：

- 只有真实起动样本或持久化种子中的静摩擦样本才允许抬高下一次扫描起点；保守默认值不再参与起点提升。
- `runSelfCheckMotionProbe()` 在等待目标到达期间周期性重发受限位置力控命令，避免单帧命令表现为短脉冲。
- `PreSelfCheck` 各失败出口先调用失能，降低失败后持续输出力矩或 UI 显示 `motor_enabled=true` 的风险。
- `StableShortStrokeMotion` 的电流上限单独通过已学习起动电流加裕量计算，但仍受 `safety.self_check_current_limit_a` 截断。

## 2.3 v1.3 调整

第三轮真实硬件日志显示，双向起动通过，但稳定短行程阶段在 `1 mm/s`、`0.5 mm` 目标下仍只运动约 `0.10 mm`。因此稳定短行程不再固定使用 `travel_learning_speed` 和 `0.5 mm` 目标，而是：

- 新增 `self_check.stable_short_stroke_distance_mm`，默认 `0.3 mm`。
- 稳定短行程按速度从 `self_check.min_speed_scan_start_mm_s` 扫到 `travel_learning_speed_mm_s`，并按当前自检电流上限成对验证闭合/打开。
- `monotonic` 日志字段只表达实际单调性，不再混入“位移是否达到最小距离”的判断。
- 失败结果增加摘要：目标距离、闭合/打开实际距离、平均速度、最大电流和电流稳定性。

## 2.4 v1.4 调整

第四轮真实硬件日志显示，`StableShortStrokeMotion` 中目标为 `0.2 mm`，但单次实际位移只有 `0.006~0.009 mm` 时仍被 `runSelfCheckMotionProbe()` 标记为 `code=Ok`。原因是自检 probe 复用了全局 `max_distance_error=0.2 mm` 作为目标到达容差，目标距离本身也是 `0.2 mm`，导致几乎刚开始就满足“到达”条件。

处理如下：

- 自检 probe 使用独立目标到达判断：容差取 `max_distance_error` 与目标距离 25% 中的较小值，并至少覆盖噪声底。
- 稳定短行程样本判定使用 `max_distance_error` 与目标距离 50% 中的较小值。
- 稳定短行程默认距离提高到 `0.3 mm`，避免目标距离与全局误差阈值相同。
- 电流扫描循环不再使用 `max_current_ripple` 扩展终止条件，避免 `current_start=current_stop=0.4 A` 时重复执行多组相同上限参数。

## 2.5 v1.5 调整

第五轮真实硬件日志显示，`BidirectionalMoveEnable` 已经能双向起动，但 `StableShortStrokeMotion` 在 `0.4 A` 自检限流下仍只能得到约 `0.06~0.10 mm` 的短距离样本，达不到 `0.3 mm` 稳定短行程目标。该现象没有伴随限位或堵转特征，适合作为第一阶段硬件联调的低置信样本，而不应阻断整个 `PreSelfCheck`。

处理如下：

- 如果闭合/打开两个方向都满足方向正确、位移单调、无堵转、无疑似限位，并且位移超过 `self_check.motion_start_distance_mm`，则记录为低置信运动样本。
- 稳定短行程未完全达标时输出 `low_confidence_samples_accepted` 和 `degraded` 日志，然后继续建立保守 `StructureProfile`。
- 第一阶段硬件联调中，`PreliminaryLimitSearch` 可使用理论开闭边界建立低置信安全区；真实机械限位仍需在后续 `HomingOpenStop` 和 `TravelLearning` 中验证。
- `MultiRegionRoundTripLearning` 如果因低置信样本不足或短距运动能力不足失败，不再阻断第一阶段 `PreSelfCheck`，而是降级继续生成保守 profile。
- `StructureProfileUpdate` 完成后立即失能电机，最终日志应显示 `controller_state=Disabled`、`motor_enabled=false`，避免自检完成后持续输出力矩造成丝杆螺母抱死风险。

## 2.6 v1.6 调整

2026-05-21 真实硬件日志显示，`opening` 试探虽然下发负向运动，但估算行程仍持续增大，实物表现为电机朝同一方向累积运动。该现象说明预自检阶段必须同时看电机原始反馈和低置信行程估算，不能只看换算后的 `stroke_mm`。

处理如下：

- `PreSelfCheck` 的初步限位和安全区定义为“低置信临时安全区”，用于让自检、回零和后续流程安全继续，不等价于最终软件限位。
- 达妙反馈中的 16 位原始位置计数 `q_uint` 贯通到 `MotorFeedback`、控制器快照、Web `/api/view` 和 UI 操作日志。
- 自检 `probe attempt/result` 日志新增：
  - `start_motor_pos_rad`
  - `end_motor_pos_rad`
  - `motor_delta_rad`
  - `start_motor_raw_pos_counts`
  - `end_motor_raw_pos_counts`
- `direction_ok` 判定不再允许“反向位移小于全局误差”直接通过。实际位移只要朝相反方向超过起动阈值或收敛后的方向阈值，就返回 `SelfCheckInconsistentFeedback`。
- `BidirectionalMoveEnable` 中某方向已确认起动后，不再重复试探该方向，避免继续把机构推向一侧。
- `StableShortStrokeMotion` 和动态摩擦往复采样统一传入正的速度幅值，由 `MotionDirection` 决定符号，避免接口语义混乱。

## 3. 过程日志

控制器新增进度回调，UI 将进度写入运行日志。真实硬件测试时重点观察：

- 当前阶段：`phase=LimitedProbe`、`BidirectionalMoveEnable`、`StableShortStrokeMotion` 等。
- 试探方向：`direction=opening/closing`。
- 目标行程：`target_stroke_mm`。
- 螺母速度：`target_nut_speed_mm_s`。
- 等效电机速度：`motor_velocity_rad_s`。
- 电流限制：`current_limit_a`。
- 实际位移：`measured_mm`。
- 平均速度和最大电流/力矩。
- 电机位置：`start_motor_pos_rad`、`end_motor_pos_rad`、`motor_delta_rad`。
- 达妙单帧位置：UI/API/普通操作日志中的 `motor_wrapped_pos_rad`。
- 达妙原始编码器位置：`start_motor_raw_pos_counts`、`end_motor_raw_pos_counts`。
- `direction_ok`、`monotonic`、`velocity_stable`、`current_stable`。
- `limit_suspected`、`jam_suspected`。

Web UI 的 `selfcheck` 动作改为后台执行，避免 HTTP 请求阻塞，页面可以继续轮询日志。

注意：`start_motor_pos_rad`、`end_motor_pos_rad` 和 `motor_delta_rad` 必须是硬件层多圈虚拟编码器输出的连续位置。该连续位置由硬件层根据 `motor.encoder_unwrap_source`、`motor.encoder_wrap_range_rad` 和运行时 `P_MAX` 生成。当前样机默认使用 `protocol_position`；配置 `encoder_wrap_range_rad=0` 表示真实硬件连接时由运行时 `2*P_MAX` 推导，当前实测 `P_MAX=50 rad`，回绕范围为 `100 rad`。达妙状态帧 `q_uint` 仅作为原始诊断量，不得按 `2*pi rad` 直接换算为输出端单圈位置。

v1.8 后，真实硬件 `MotorFeedback.position` 来自后台固定频率反馈服务维护的最新快照。`PreSelfCheck` 仍通过 `MotorInterface::readFeedback()` 获取数据，但该接口不再触发临时清队列、临时请求反馈或局部回绕计算；这可以避免自检流程、UI 轮询和硬件线程竞争同一批反馈帧。

v1.9 后，`runSelfCheckMotionProbe()` 将停止输出后的停稳结果作为样本有效性的前置条件：

- 日志新增 `settled`、`settled_speed_mm_s` 和 `settle_code`，用于区分“命令方向错误”和“上一段残余运动未停稳”。
- 停稳失败时，probe 返回停稳错误，且 sample 的方向匹配、单调性、速度稳定性、电流稳定性均不作为有效结论。
- `BidirectionalMoveEnable` 不接受未停稳样本作为起动样本，也不允许基于未停稳样本触发单向可动降级。
- 稳定短行程、初步限位、回到安全区和多区域往复学习都只把停稳后的样本写入统计。

v1.10 后，停稳判断修正为真实反馈语义：

- 达妙 `disable()` 只发送停止/失能命令并更新输出使能状态，不再把 `last_feedback.velocity/current/torque` 直接改为零。
- 控制器在停稳等待前记录当前反馈时间戳，等待过程中必须看到停机命令之后的新反馈帧；只有新反馈帧速度低于阈值并保持稳定时间，才置 `settled=true`。
- `probe attempt` 日志新增 `target_motor_pos_rad`，用于人工核对目标电机位置方向与 `target_nut_speed_mm_s`、`motor_velocity_rad_s` 是否一致。

v1.11 后，`runSelfCheckMotionProbe()` 内部收尾使用主动保持：

- probe 到达目标、超时或需要结束当前小试探时，先读取当前位置作为固定保持目标并发送低电流 `PositionForce` 保持命令。
- 停稳等待期间不再继续刷新原运动目标，避免换向前还在追逐上一目标。
- 停稳等待期间周期性刷新同一个固定保持目标，不能用最新反馈位置更新目标；否则电机会跟随残余漂移。
- 保持速度限幅使用 `self_check.motion_hold_speed_mm_s`，不使用 0，避免达妙位置力控模式在零速度限幅下无法有效制动。
- 保持电流使用 `self_check.motion_hold_current_a` 与当前 probe 试探电流中的较大值，再由 `safety.self_check_current_limit_a` 截断；这样低电流试探成功后仍有足够余量在换向前停稳。
- 只有 `runPreSelfCheck()` 整体完成、失败、用户停止或其他业务动作结束时才调用 `disableAfterMotion()`。
- 停稳速度阈值使用 `self_check.motion_settle_speed_threshold_mm_s`，默认 `0.03 mm/s`；`motion_settle_timeout_s` 默认调整为 `2.0 s`，给低电流主动保持足够时间消除上一段残余运动。
- 如果主动保持阶段未能停稳，`runSelfCheckMotionProbe()` 会记录 fallback 日志，随后执行零输出/失能停稳等待。fallback 后的样本仍以停机后的真实反馈位置和速度为准；未停稳样本不能被接受。
- 停稳日志输出 `settled_speed_mm_s` 和 `settled_position_delta_speed_mm_s`。前者来自电机反馈速度字段，后者由连续多圈位置相邻反馈差分得到；样本接受以真实位置不再变化为底线。
- `BidirectionalMoveEnable` 中，如果已确认方向的反向 probe 只在低能量下小幅漂回已确认方向，且没有堵转、限位或超流，则输出 `reason=low_energy_opposite_drift` 并按单向低置信边界继续。后续完整开闭关系仍由回零、行程学习和运动健康检查确认。

## 2.7 v1.22 调整

2026-05-21 最新真实硬件日志 `gripper-log-2026-05-21T08-24-21.432Z.txt` 显示，停止链路已经能够取消后台自检，但 `PreSelfCheck` 仍在 `BidirectionalMoveEnable` 中长时间闭合/打开来回扫参。典型样本为：

- 停稳、方向判据日志显示可接受，但单次位移只有约 `0.02~0.04 mm`。
- 这些位移高于静止噪声底，但低于 `motion_start_distance_mm=0.05`，因此没有进入 `breakaway accepted`。
- 部分 probe 的停稳后有符号位移实际朝请求反方向，不能仅凭 `direction_ok=true` 或绝对位移接受。
- 用户停止后的取消 probe 结果曾打印 `end_motor_pos_rad=0`、`end_estimated_mm=0`，这是停稳取消路径没有填充最后反馈导致的诊断噪声。

处理如下：

- 新增配置 `self_check.low_confidence_motion_distance_mm`，默认 `0.02 mm`，只用于 `PreSelfCheck` 第一阶段。
- `BidirectionalMoveEnable` 的运行中起动即停阈值和停稳后接受阈值，在该阶段使用低置信微动阈值；`motion_start_distance_mm` 仍作为正常起动阈值和后续阶段判据。
- `acceptBreakawayCandidate()` 同时检查停稳后有符号位移，只有朝请求方向且超过低置信阈值才可接受。反方向微动进入已有 `early_low_energy_opposite_drift` 或 `low_energy_opposite_drift` 降级路径。
- 低置信微动样本必须满足命令限流不超过 `safety.self_check_current_limit_a`，反馈电流/力矩峰值不超过全局硬保护阈值，并且无反向运动、无硬件故障、无主动停止。达妙反馈电流是由力矩反馈换算而来，第一阶段不能直接用 `self_check_current_limit_a + max_current_ripple_a` 拒绝已经产生同向微动的起动样本。
- 低置信微动样本的过程日志保留真实反馈峰值；写入运行期 profile 和静摩擦样本时，学习值使用命令限流和反馈峰值中的保守较小值，避免下次 `PreSelfCheck` 起点被异常峰值抬高。
- `stopAndWaitForSettledFeedback()` 和 `holdAndWaitForSelfCheckSettledFeedback()` 在进入等待前先填充当前真实反馈，取消或超时时日志仍能显示最后已知电机位置和行程，不再把终点打印为零。

新增软件回归：

- `pre_self_check_accepts_low_confidence_micro_motion`：模拟电机每次位置命令只产生低于正常起动阈值、但高于低置信阈值的微小同向位移，验证 `PreSelfCheck` 能生成保守 profile、设置完成标志并最终失能。

## 4. 当前边界

初步限位搜索第一版会在理论行程范围内进行低速限流分步搜索。若未发现机械限位异常，到理论边界即停止并记录为理论保护边界；该结果用于建立低置信、保守的预自检 profile，不等价于完整机械限位实测标定。

后续真实硬件测试若出现以下现象，需要继续调整配置或实现：

- 电机仍无动作：重点看 `current_limit_a`、`target_nut_speed_mm_s` 和反馈电流/速度是否变化。
- 位移方向不一致：检查 `motor.direction_sign` 和机构安装方向。
- 电流到限但无位移：可能存在机械卡滞、限位、低速迟滞或电流限制过低。
- 速度稳定性不满足：调整 `self_check.min_speed_scan_start/stop/step`。
- 预自检时间过长：调整 `self_check.max_probe_window`、`travel_learning_speed` 或初步限位搜索策略。
- 如果日志中已经出现小位移但最终失败，先区分失败发生在 `BidirectionalMoveEnable` 还是 `StableShortStrokeMotion`。前者代表未能确认双向起动，后者代表起动后仍不能稳定完成规定小行程。

## 5. 验证结果

已执行：

```powershell
.\scripts\build.ps1
.\.venv\Scripts\ctest.exe --test-dir .\build\dev-zig --output-on-failure
```

结果：

- 构建通过。
- CTest 7/7 通过。

另执行 `gripper_app.exe --scripted-demo`，确认日志中已输出完整 `PreSelfCheck` 阶段、试探参数和结果。

真实硬件复测命令：

```powershell
connect
enable
selfcheck
log
quit
```

结果：

- `BidirectionalMoveEnable` 能确认打开/闭合双向起动。
- `StableShortStrokeMotion` 记录低置信样本并降级继续。
- `PreliminaryLimitSearch` 使用理论边界建立低置信安全区。
- `MultiRegionRoundTripLearning` 可降级继续。
- 最终返回 `pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile`。
- 自检完成后电机已失能，日志末尾为 `controller_state=Disabled`、`motor_enabled=false`。

## 6. 单向可动降级更新

2026-05-21 真实硬件日志 `gripper-log-2026-05-21T02-41-31.358Z.txt` 显示：

- `closing` 已在有限电流下起动。
- `opening` 随后无有效位移，并被持续扫描到速度/电流上限。
- 流程最终因“未双向起动”失败。

根据控制设计 v1.8，已调整 `BidirectionalMoveEnable`：

- 一个方向确认低能量起动后，另一个方向只在有限同级窗口内继续验证。
- 如果缺失方向无有效位移、方向未反常、无堵转/接触/ActiveStop/硬件故障，则记录为低置信疑似边界并继续保守流程。
- 降级日志格式为：

```text
PreSelfCheck | phase=BidirectionalMoveEnable | degraded ... reason=no_effective_motion_under_limited_scan
```

- 出现方向相反、主动停止、堵转、接触、硬件故障或通信失败时仍立即失败，不允许降级通过。
- 单向可动降级后跳过双向稳定短行程强验证，避免继续往疑似边界方向反复施加命令；后续零点、行程和机构运动关系仍必须由 `HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 重新确认。

Web UI 同步增加 `self_check_running` 运行态：

- `/api/view` 暴露后台自检运行状态。
- 点击“预自检”后按钮显示“预自检执行中...”并禁用。
- 顶部流程阶段显示“自检中”。
