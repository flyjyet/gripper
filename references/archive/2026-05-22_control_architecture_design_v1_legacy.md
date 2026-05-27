# 夹爪控制系统架构设计

## 版本记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| v1.0 | 2026-05-15 | 建立控制架构、自检、状态机、安全策略和模块边界。 |
| v1.1 | 2026-05-20 | 增加管理员维护恢复模式；补充高电流恢复前的打开方向确认、越卡越紧识别、硬保护不可绕过、恢复后重新自检要求。 |
| v1.2 | 2026-05-20 | 将维护模式拆分为电机空载调试 `MotorBringupMode` 和结构故障恢复 `AdminRecoveryMode`；补充 UI 原型要求。 |
| v1.3 | 2026-05-20 | 补充 `PreSelfCheck` 第一阶段低置信通过和完成后失能收尾要求。 |
| v1.4 | 2026-05-21 | 明确 `PreSelfCheck` 以电机侧低能量试探为基础，允许初步发现限位并建立低置信临时安全区；完整零位、软件限位和夹爪机构运动关系由 `HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 分阶段确认。 |
| v1.5 | 2026-05-21 | 补充 Web UI 顶部分页要求和受限螺母行程拖动入口：`PreSelfCheck` 后仅允许低置信临时安全区移动，`MotionHealthCheck` 后才允许最终软件限位范围移动，所有动作必须经过 controller 安全路径并在运动后失能。 |
| v1.6 | 2026-05-21 | 明确螺母行程拖动是调试定位入口，和最终夹紧力控分离；拖动可使用电机位置/位置速度模式，保留速度、行程、反馈电流、接触/堵转和运动后失能兜底。普通超时未到位不应直接升级为 `ActiveStop`，只有安全风险触发才进入 `ActiveStop`。 |
| v1.7 | 2026-05-21 | 明确 controller 对上层接口的同步语义：运动类调用必须等待动作完成、停稳或明确失败后再返回；自检每段低能量试探必须在换向前停止输出、等待速度进入静止阈值并重新读取停稳反馈，不能用运动中的编码器数据计算终点或判定方向。 |
| v1.8 | 2026-05-21 | 补充 `PreSelfCheck` 单向可动降级策略：如果一个方向已低能量起动确认，另一方向在有限参数窗口内无有效位移且未出现方向反常、堵转或电流危险，应判为低置信疑似边界/受阻方向并继续建立保守 profile，不能把同一不可动方向扫完整个速度电流矩阵。 |
| v1.9 | 2026-05-21 | 明确电机反馈必须先经过多圈虚拟编码器：达妙单帧位置反馈只能作为原始诊断量，控制器、自检、行程估算、手动拖动和业务控制必须使用连续多圈电机位置；后续应把固定频率反馈更新服务作为硬件基础设施。 |
| v1.10 | 2026-05-21 | 将固定频率虚拟编码器反馈服务确定为当前硬件基础设施：达妙连接后后台按 `motor.feedback_poll_period_s` 请求并解析电机状态反馈帧，更新连续多圈 `MotorFeedback.position`；`readFeedback()` 只返回最新快照，不再临时清队列和抢帧。 |
| v1.11 | 2026-05-21 | 明确 `PreSelfCheck` 未停稳样本无效：任何 probe 在停止输出后未达到静止阈值，不得进入起动、低置信边界、摩擦或行程统计；换向前必须再次确认上一段残余运动已停稳，未停稳只能作为未完成/超时处理，不能判为方向反常。 |
| v1.12 | 2026-05-21 | 明确停稳判断必须基于停机命令之后的新硬件反馈帧；硬件层不得在 `disable()` 中伪造零速度、零电流或零力矩反馈，否则会把仍在运动的电机误判为已停稳。 |
| v1.13 | 2026-05-21 | 明确 `PreSelfCheck` probe 间停机策略：自检内部换向前应使用低电流主动保持当前位置并等待真实反馈停稳，不能直接失能释放机构；只有整个流程完成、失败或人工停止时才失能。 |
| v1.14 | 2026-05-21 | 补充 `PreSelfCheck` 主动保持细节：保持目标必须固定为进入保持阶段时的电机位置，等待停稳期间周期性刷新同一个低电流 PositionForce 保持命令；保持速度限幅不得为 0，应使用配置化的极低保持速度上限并受自检电流安全包络约束。 |
| v1.15 | 2026-05-21 | 增加达妙位置命令符号配置 `motor.position_command_sign`：它只修正 Position/PositionForce 命令目标与达妙反馈符号之间的协议差异，不替代机构 `motor.direction_sign`。 |
| v1.16 | 2026-05-21 | 将 `PreSelfCheck` 试探电流和 probe 间保持电流分离：试探仍按扫描电流递增，保持电流使用 `self_check.motion_hold_current_a` 并受 `safety.self_check_current_limit_a` 截断，用于可靠制动已启动的低能量运动。 |
| v1.17 | 2026-05-21 | 明确 `BidirectionalMoveEnable` 是起动识别，不是短行程完成测试；该阶段 probe 一旦达到 `motion_start_distance_mm` 即应立即进入主动保持停稳，避免低电流起动后继续运行到大窗口导致换向困难。 |
| v1.18 | 2026-05-21 | 根据最新硬件日志修正位置类命令符号默认值：当前 DM-J4310P-2EC + DM-USB2FDCAN_Dual 联调配置下 `motor.position_command_sign=1` 才能使 `target_motor_delta_rad` 与反馈 `motor_delta_rad` 同向；该值仍保留为配置项，换设备或固件后必须用低能量 probe 复核。 |
| v1.19 | 2026-05-21 | 将自检停稳速度阈值从堵转速度阈值中拆出为 `self_check.motion_settle_speed_threshold_mm_s`；`PreSelfCheck` 换向前必须按更严格静止阈值等待，避免上一段残余运动被误判为新命令方向错误。 |
| v1.20 | 2026-05-21 | 补充 `PreSelfCheck` probe 收尾 fallback：低电流主动保持若在停稳超时内未收敛，应切换到零输出/失能并等待真实反馈静止；样本仍必须满足停机后静止、方向一致和安全窗口约束才可接受。 |
| v1.21 | 2026-05-21 | 补充自检停稳双判据：除达妙反馈速度外，还应基于连续多圈位置差分计算实际位置变化速度；当反馈速度字段滞后但位置已停止时，可用位置差分速度判定真实停稳。 |
| v1.22 | 2026-05-21 | 扩展 `PreSelfCheck` 单向低置信降级：一个方向已确认起动后，反向低能量 probe 若出现小幅朝已确认方向漂移且电流仍在自检包络内，应标记为疑似边界/残余漂移并停止升流，不作为致命方向错误。 |
| v1.23 | 2026-05-21 | 扩展 `PreSelfCheck` 早期低能量反向漂移处理：尚未确认任一方向时，如果受限 probe 出现小幅反向漂移且电流、速度、位移均在自检安全包络内，应立即收手并生成低置信保守 profile，不继续升流；只有位移超窗、堵转、接触、硬件故障或高能量反向运动才作为致命方向错误。 |
| v1.24 | 2026-05-21 | 修正虚拟编码器定义：达妙协议位置量程 `[-p_max, p_max]`、原始 `q_uint` 计数和物理输出端单圈回绕是三个不同概念。连续多圈位置必须使用配置化的数据源和回绕范围建立，当前样机默认按原始单圈计数、`2*pi rad` 回绕累计，但首帧连续位置基准仍采用协议解析位置，避免位置命令坐标跳变；UI 主夹爪状态只显示控制用虚拟多圈位置，单帧位置和原始计数仅作为诊断/API 字段。 |
| v1.25 | 2026-05-21 | 增加长流程取消语义：`stop()`、`disable()`、断开连接和用户停止必须设置控制器级取消标志，`PreSelfCheck`、回零、行程学习、运动健康检查、夹紧、释放和手动定位等长流程必须在发命令前、等待反馈循环中、probe 收尾后和扫描循环之间检查取消；一旦取消，立即停止刷新命令、失能或零输出、返回 `SafetyActiveStop`，后台 UI 线程不得继续发起新的 probe。 |
| v1.26 | 2026-05-21 | 补充 `BidirectionalMoveEnable` 两级起动判据：`motion_start_distance_mm` 仍作为正常起动阈值；真实硬件低能量试探中，若停稳后有符号位移达到配置化 `low_confidence_motion_distance_mm`、方向正确、电流仍在自检包络内且无堵转/限位，则可记录为低置信微动起动样本并结束起动扫描。反方向微动不得被当作起动样本，应走疑似边界/低能量漂移降级路径；取消或停稳失败日志必须保留最后真实反馈位置，不得把终点打印为零。 |
| v1.27 | 2026-05-21 | 根据滑块实测修正达妙虚拟编码器默认数据源：当前 DM-J4310P-2EC 状态帧 `q_uint` 已确认为协议位置量程编码，不是输出端单圈绝对计数；默认改为 `encoder_unwrap_source=protocol_position`、`encoder_wrap_range_rad=2*motor.max_position_rad`。硬件反馈线程还必须在每轮刷新中消化接收队列到最新状态帧，避免用积压旧帧更新虚拟编码器。 |
| v1.28 | 2026-05-21 | 明确 `ActiveStop` 是停止命令输出的安全状态，不是停止反馈更新；UI 和 controller 在 `ActiveStop` 下仍应读取最新电机反馈并更新夹爪状态。任何电流、堵转、限位等安全超限返回前必须先停止/失能电机输出并刷新停机后的反馈，日志不得出现 `ActiveStop` 但 `motor_enabled=true` 作为正常结果。 |
| v1.29 | 2026-05-21 | 收紧低置信阶段自动放行条件：`PreSelfCheck` 在尚未确认任何方向可控时，不得仅因第一次低能量反向漂移就返回 `Ok` 并放开滑块；低置信螺母拖动在 `MotionHealthCheck` 前只能围绕当前估算位置做小步验证，不能按完整理论安全区一次下发接近一圈的绝对位置目标。 |
| v1.30 | 2026-05-21 | 修正达妙反馈位置量程来源：普通状态反馈帧的 `q_uint` 必须使用连接时从寄存器读取的运行时 `P_MAX/VMAX/TMAX` 解码，当前样机 `P_MAX=50 rad`，协议位置窗口为 `[-50, 50]`；`p_m/xout` 寄存器只作为诊断校验来源，不作为主控制反馈来源。 |
| v1.31 | 2026-05-21 | 修正 `PreSelfCheck` 第一阶段反馈电流判据：`BidirectionalMoveEnable` 的低能量约束以命令限流和速度限幅为主，达妙反馈电流/力矩峰值先作为诊断和硬保护输入；在已停稳、同向微动、无反向运动、无硬件故障且未超过全局硬保护电流时，不得仅因反馈电流峰值超过 `self_check_current_limit` 拒绝起动样本或触发通用堵转失败。 |
| v1.32 | 2026-05-21 | 明确螺母位置滑块的前后端契约：UI 必须使用 controller 快照/API 暴露的当前手动定位允许范围，不得自行用完整低置信安全区推导滑块范围；controller 对越界目标必须在使能电机前拒绝，不能因一次无效滑块命令留下 `motor_enabled=true`。 |
| v1.33 | 2026-05-21 | 重新定义螺母位置滑块的调试目的：该入口主要用于空载/结构未安装阶段验证电机多圈编码器、丝杆导程和螺母行程映射。结构未安装/空载已确认时，`PreSelfCheck` 后可在低置信理论边界内限流限速拖动；未确认空载时仍只允许当前位置附近的小窗口。 |
| v1.34 | 2026-05-21 | 修正螺母位置滑块的执行策略：即使空载确认后允许使用完整低置信理论边界，controller 也不得把完整目标一次性下发为大跨度位置力控目标；必须按速度节拍生成小步递进位置目标，并对达妙反馈电流尖峰做持续时间判据，只有持续超限、堵转/限位或硬件故障才进入 `ActiveStop`。 |

## 1. 设计目标

本夹爪机构除电机自身反馈外，没有外部力传感器、位置传感器或接触传感器。因此控制系统必须基于电机反馈、机构模型、结构参数学习结果和软件保护策略实现可靠控制。

核心设计目标如下：

- 在最底层命令输出前实现限流、限速兜底保护。
- 启动流程分阶段识别关键参数：`PreSelfCheck` 先通过电机侧低能量试探识别电机可控性、基础摩擦、初步限位和低置信临时安全区；`HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 再逐步确认零位、软件限位和夹爪运动关系。
- 基于 `PreSelfCheck` 得到的电机侧保守速度/电流阈值执行低速、低电流堵转靠零。
- 靠零后执行行程学习，找到另一方向极限并设置软件限位。
- 在软件安全范围内执行不同安全速度的运动检查，验证电机运动与机构运动关系的可靠性。
- 支持指定夹爪目标夹紧力。
- 支持指定夹爪恒定夹紧速度，降低接触瞬间冲击。
- 主动检测接触、堵转、机械限位等异常状态，并主动停止。
- 工作完成后电机失能，不持续输出力矩，避免丝杆螺母抱死。
- 硬件访问、电机协议、夹爪控制逻辑、UI 界面彼此解耦。
- 以已跑通的 Python UI 原型行为作为后续重构的硬件联调参考。

## 2. 初始化/自检的新定义

这里的“初始化”或“自检”不是简单检查设备是否连接，而是系统上电后的结构参数识别与健康检查流程。

自检输出一个统一的数据结构：

- `StructureProfile`

该结构保存当前机构在本次启动或本次学习周期内得到的关键结构参数。后续回零、行程学习、限流限速、堵转检测、接触检测、夹紧控制都应使用这些参数。

自检阶段需要识别或确认的参数至少包括：

- 最低稳定运行速度，避免低速爬行、迟滞、stick-slip 对控制判断造成误导。
- 打开方向和闭合方向的静摩擦。
- 打开方向和闭合方向的动摩擦。
- 低速不稳定区间或 Stribeck 影响区间。
- 电流、速度、位置反馈噪声底。
- 初步机械限位位置。
- 与理论行程的偏差。
- 初步安全区。
- 安全区内多区域、多次往复学习结果。
- 靠零后的打开方向零点。
- 行程学习后的闭合方向软件限位。
- 软件安全行程范围。
- 不同安全速度下的运动健康结果。
- 识别时温度和识别结果有效性。

轻度调研结论：

- 伺服系统低速运动容易受静摩擦、库仑摩擦、黏性摩擦、Stribeck 效应和 stick-slip 影响。
- 摩擦识别不能在过低速度下进行，否则容易把爬行、迟滞、速度波动误判为正常摩擦。
- 应先识别最低稳定运行速度，再在高于该速度的区域识别动摩擦。
- 静摩擦应通过起动电流/起动力矩识别，动摩擦应在稳定速度段统计，二者不能混为一个参数。

## 3. 推荐启动流程

```text
Connect
  |
  v
HardwareSanityCheck
  |
  v
PreSelfCheck
  |  电机侧低能量双向试探、初步发现限位、建立低置信临时安全区
  v
HomingOpenStop
  |  使用自检得到的速度/电流阈值进行低速堵转靠零
  v
TravelLearning
  |  从零点向闭合方向运行，寻找另一方向极限，设置软件限位
  v
MotionHealthCheck
  |  在软件安全范围内用不同安全速度运动，检查可靠性
  v
Ready
```

说明：

- `PreSelfCheck` 是回零前的初步结构参数识别，不依赖绝对零点。它的输入和判据以电机侧编码器、速度、电流、力矩和温度反馈为主。
- `PreSelfCheck` 的第一目标不是精确标定，而是让系统知道电机能否双向受控运动、是否存在疑似限位、临时安全区大致在哪里，使后续回零、行程学习和运动健康检查能够继续。
- `PreSelfCheck` 可以建立低置信的初步限位和临时安全区，但该结果不是最终软件限位，也不能直接作为夹紧、释放等业务动作的完整机构模型。
- 如果机构一开始就在机械极限附近，`PreSelfCheck` 应先通过小步尝试和主动停止排除限位风险，再进入限位发现或回零流程，不能强行大行程测试。
- `PreSelfCheck` 的每段小试探必须具备明确的命令完成边界：控制器下发受限命令后持续监控反馈，达到目标、超时、疑似接触/堵转或故障时立即停止输出；随后等待电机速度进入静止阈值并保持短时间，再读取最终反馈用于位移、方向和电流统计。停稳最终反馈必须是停止或失能命令之后的新硬件反馈帧，不得直接使用运动过程中的最后一帧编码器作为终点，也不得在前一方向未停稳时立即启动下一方向试探。
- 如果停止输出后在配置时间内仍未停稳，该段 probe 的最终位置只能用于诊断显示，不得作为有效起动样本、低置信样本、边界样本、摩擦样本或方向一致性结论。后续换向前应先重新等待停稳；若仍无法停稳，应以未完成或不安全运动结束本轮自检，而不是把残余运动判定为新命令方向错误。
- 自检停稳阈值使用独立配置 `self_check.motion_settle_speed_threshold_mm_s`，不复用 `safety.jam_speed_threshold_mm_s`。堵转阈值通常允许更大的低速残余，用作停稳判据会把仍在滑移的机构误判为静止。
- 停稳判断应同时记录达妙反馈速度和连续多圈位置差分速度。若反馈速度字段在失能后滞后或保持非零，但连续位置差分速度已连续低于静止阈值，可以认为机构真实停稳；若位置差分仍超过阈值，则不得接受样本。
- 一个方向已经低能量确认后，另一方向的第一次或有限次低能量 probe 如果出现小幅朝已确认方向漂移，且满足停稳、低电流、无堵转、无限位和位移受限，应降级为低置信疑似边界/残余漂移。此时不应继续向该方向升流扫描，避免把机构推得更紧。
- 硬件实现的 `disable()` 或 stop 命令只能改变输出状态，不能改写最近一次真实反馈中的速度、电流、力矩为零。控制器应依赖反馈时间戳确认已收到停机后的新反馈帧，再使用速度阈值判定停稳。
- `PreSelfCheck` 内部 probe 与 probe 之间不得直接失能电机。由于机构可能在失能后被丝杆/摩擦/外力反推，换向前应先发送“进入保持阶段时的固定位置、极低速度限制、低电流限制”的主动保持命令，等待停机后的新反馈帧速度低于静止阈值并保持稳定时间；完整流程结束、失败或用户主动停止时再失能。
- 主动保持等待期间必须周期性刷新同一个固定保持目标，不能每次使用最新反馈位置更新保持目标，否则会变成跟随漂移；保持速度限幅不应配置为 0，避免位置力控模式因为零速度限幅无法产生有效制动。保持电流不得超过 `safety.self_check_current_limit_a`。
- 主动保持电流和当前 probe 试探电流不是同一个概念。probe 可以从很低电流开始找起动阈值；一旦电机已经启动，换向前的保持/制动应使用配置化的 `self_check.motion_hold_current_a`，并由 `safety.self_check_current_limit_a` 作为硬上限截断。
- 如果主动保持后速度未在 `self_check.motion_settle_timeout_s` 内收敛，控制器应停止继续刷新保持目标，改为零输出/失能等待真实反馈静止。该 fallback 不是正常跳过停稳要求；只有停机后的新反馈帧满足静止阈值、方向一致且位移仍在自检安全窗口内，样本才允许进入统计。
- `HomingOpenStop` 完成后才允许建立系统零位。
- `TravelLearning` 建立闭合方向软件限位。软件限位应比机械限位更保守，不能与机械限位完全重合。
- `MotionHealthCheck` 只允许在软件限位内运行，用于验证电机运动、螺母行程估计、夹爪机构运动关系、摩擦、电流、速度和位置反馈是否稳定。完成该阶段后，后续业务层控制才可以使用完整机构运动关系。

### 3.1 长流程取消与用户停止语义

用户停止、`stop()`、`disable()`、断开连接、通信严重异常和硬件故障属于硬保护入口。控制器必须在本地维护一个控制器级取消标志，且该标志不依赖 UI 线程生命周期。任何运动或自检长流程启动前应清除旧的取消请求；一旦用户停止或失能，该标志立即置位，并禁止长流程继续发新的使能、保持、位置、速度或力控命令。

取消检查必须覆盖以下位置：

- 每个长流程阶段开始前。
- 每次向电机发送命令前，包括 probe 命令、低电流保持命令、位置保持刷新命令和重使能命令。
- 每个等待反馈、等待停稳、等待目标到达的循环内部。
- 每个 probe 结果处理后、扫描参数递增前、换向前和阶段切换前。

取消后的收尾要求：

- 立即停止刷新当前命令。
- 尽可能发送零输出或失能命令。
- 不再为了继续自检而重新使能电机。
- 返回 `SafetyActiveStop`，日志中必须包含取消发生的流程上下文。
- 状态机进入 `ActiveStop`；恢复后不允许直接进入 `Ready`，必须按流程重新确认必要状态。

Web UI 允许把 `PreSelfCheck` 放到后台线程执行，但后台线程不得只依赖 UI 全局运行标志。`self_check_running` 只用于显示和防重复点击，真正的取消必须由 controller 实现。点击停止后，UI 应显示停止请求已经发出，后台线程收到 `SafetyActiveStop` 后清理运行标志；在运行标志清理前，自检按钮保持禁用并显示执行中或停止中。

## 4. 源码目录规划

```text
src/
  app/
    main.cpp
    application.hpp
    application.cpp

  common/
    project_defs.hpp
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
    math_utils.cpp
    geometry_utils.hpp
    geometry_utils.cpp
    filter_utils.hpp
    filter_utils.cpp
    interpolation.hpp
    interpolation.cpp

  hardware_interface/
    motor_interface.hpp
    motor_types.hpp
    virtual_encoder.hpp
    virtual_encoder.cpp
    transport_interface.hpp
    can_frame.hpp
    adapter_types.hpp
    damiao/
      damiao_motor.hpp
      damiao_motor.cpp
      damiao_protocol.hpp
      damiao_protocol.cpp
      dm_usb2fdcan_transport.hpp
      dm_usb2fdcan_transport.cpp
    simulated/
      simulated_motor.hpp
      simulated_motor.cpp

  controller/
    gripper_controller.hpp
    gripper_controller.cpp
    gripper_types.hpp
    self_check/
      structure_profile.hpp
      self_check_manager.hpp
      self_check_manager.cpp
      structure_parameter_identifier.hpp
      structure_parameter_identifier.cpp
      friction_identifier.hpp
      friction_identifier.cpp
      travel_limit_identifier.hpp
      travel_limit_identifier.cpp
      motion_health_checker.hpp
      motion_health_checker.cpp
    state_machine/
      gripper_state_machine.hpp
      gripper_state_machine.cpp
      self_check_state.hpp
      self_check_state.cpp
      homing_state.hpp
      homing_state.cpp
      travel_learning_state.hpp
      travel_learning_state.cpp
      motion_health_check_state.hpp
      motion_health_check_state.cpp
      clamp_state.hpp
      clamp_state.cpp
      release_state.hpp
      release_state.cpp
      active_stop_state.hpp
      active_stop_state.cpp
      fault_state.hpp
      fault_state.cpp
    safety/
      safety_limiter.hpp
      safety_limiter.cpp
      contact_jam_detector.hpp
      contact_jam_detector.cpp
    calibration/
      force_mapper.hpp
      force_mapper.cpp
    mechanism/
      gripper_kinematics.hpp
      gripper_kinematics.cpp

  ui/
    main_window.hpp
    main_window.cpp
    ui_controller.hpp
    ui_controller.cpp
    runtime_log_model.hpp
    runtime_log_model.cpp
    ui_view_model.hpp

  commander/
    command_line.hpp
    command_line.cpp
    command_dispatcher.hpp
    command_dispatcher.cpp

  third_party/
    damiao/
      include/
      lib/
      bin/
```

## 5. 控制架构图

```text
                         +----------------------+
                         |        app           |
                         | 主程序 / 对象装配    |
                         +----------+-----------+
                                    |
                   +----------------+----------------+
                   |                                 |
          +--------v---------+              +--------v---------+
          |        ui        |              |     commander    |
          | PC 测试界面      |              | 命令行/调试入口  |
          +--------+---------+              +--------+---------+
                   |                                 |
                   +----------------+----------------+
                                    |
                         +----------v-----------+
                         |     controller       |
                         | 夹爪控制业务核心    |
                         +----------+-----------+
                                    |
       +----------------------------+-----------------------------+
       |                            |                             |
+------v------+          +----------v---------+        +----------v---------+
| self_check  |          | state_machine      |        | safety             |
| 结构参数学习 |          | 自检/回零/夹紧等   |        | 限流/限速/检测     |
+------+------+          +----------+---------+        +----------+---------+
       |                            |                             |
       +----------------------------+-----------------------------+
                                    |
                         +----------v-----------+
                         | mechanism           |
                         | 运动学/行程/力学映射 |
                         +----------+-----------+
                                    |
                         +----------v-----------+
                         | calibration         |
                         | 夹紧力映射/样机标定 |
                         +----------+-----------+
                                    |
                         +----------v-----------+
                         | hardware_interface  |
                         | 电机与通信抽象      |
                         +----------+-----------+
                                    |
             +----------------------+----------------------+
             |                                             |
   +---------v----------+                       +----------v---------+
   | damiao implementation                      | simulated motor    |
   | 达妙协议 + USB2FDCAN                       | 离线仿真/测试      |
   +---------+----------+                       +--------------------+
             |
   +---------v----------+
   | third_party/damiao |
   | DeviceSDK / DLLs   |
   +--------------------+
```

## 6. 依赖关系原则

推荐依赖方向：

```text
app
  -> ui / commander
  -> controller
  -> hardware_interface 抽象
  -> hardware_interface/damiao 具体实现
  -> third_party
```

公共模块依赖规则：

- `common` 可被所有模块依赖，但只放真正全局通用的基础定义。
- `utils` 可被控制、硬件、UI、命令行等模块依赖，但自身不能依赖业务模块。
- `config` 主要由 `app` 加载，并把配置对象传递给各模块。

强约束：

- `controller` 不直接包含达妙 SDK 头文件。
- `ui` 不直接编码 CAN 帧，也不直接调用 DeviceSDK。
- `third_party` 不放项目自研业务封装逻辑。
- `hardware_interface/damiao` 可以依赖 `third_party/damiao`。
- `simulated` 必须实现和真实电机一致的硬件抽象接口。

## 7. 模块职责说明

### 7.1 app

`app` 是程序装配层，负责加载配置、创建硬件实现、创建夹爪控制器、创建 UI 或命令行入口，并管理程序启动和关闭顺序。

### 7.2 common

`common` 放项目级基础定义，例如错误码、`Result`、时间戳、单位定义、日志基础定义和项目级编译宏。

如果某个类型只属于某个模块，应放回对应模块，而不是放进 `common`。

### 7.3 config

`config` 负责配置结构和配置加载。默认配置文件为：

- `src/config/default_gripper.yaml`

配置项应包含硬件参数、安全限制、自检参数、回零参数、行程学习参数、夹紧参数和 UI 默认参数。

### 7.4 utils

`utils` 只放纯工具代码，例如数学计算、几何计算、滤波、插值、查表和单位换算。

`utils` 不应该知道夹爪状态机、达妙协议或 UI 控件。

### 7.5 hardware_interface

`hardware_interface` 负责硬件抽象，包括电机抽象接口、电机反馈数据、电机命令数据、通信传输接口、CAN/CANFD 帧表示和适配器配置类型。

`controller` 只依赖这些抽象，不依赖具体硬件实现。

电机位置反馈必须在 `hardware_interface` 内先统一成连续多圈位置，再交给 `controller`。原因是当前达妙反馈帧中的位置是有限范围编码量，可能在跨越单圈或协议量程边界时回绕；如果控制器直接用该值计算螺母行程，就会出现“电机实际转了多圈，但行程只变化一点”的错误。

这里必须区分三个概念：

- 达妙协议位置：`q_uint` 按运行时 `[-P_MAX, P_MAX]` 解码得到的单帧位置，`P_MAX` 必须由达妙寄存器读取，不能固定写死为旧默认值。该位置窗口可以覆盖多圈，例如当前样机 `P_MAX=50 rad`，窗口约为 `[-7.96, 7.96]` 圈。
- 原始计数：达妙状态帧中的 16 位 `q_uint`，当前样机中它是协议位置编码的原始量化值，只能作为诊断量。
- 物理输出端单圈回绕：输出端转一圈对应 `2*pi rad`，当前丝杆导程为 2 mm/rev，因此连续多圈电机位置每变化约 `2*pi rad`，螺母行程应变化约 2 mm。

连续多圈虚拟编码器的数据源和回绕范围必须配置化。当前样机默认使用达妙协议解析位置作为回绕检测数据源，`motor.encoder_unwrap_source = protocol_position`，`motor.encoder_wrap_range_rad = 0` 表示真实硬件连接时由运行时 `2 * P_MAX` 推导；当前实测 `P_MAX=50 rad`，因此回绕范围为 `100 rad`。`q_uint` 继续贯通到日志和 API，但不参与默认多圈累计。若后续确认某批硬件存在独立的输出端单圈绝对计数来源，才允许切换为 `raw_position_counts` 并把回绕范围设为 `2*pi rad`。

硬件抽象层需要维护以下语义：

- `MotorFeedback.position`：控制使用的连续多圈电机位置，单位 rad。
- `MotorFeedback.wrapped_position`：供应商单帧解析出的有限范围位置，仅用于诊断。
- `MotorFeedback.raw_position_counts`：供应商状态帧中的 `q_uint` 原始值，仅用于诊断和问题定位。
- `MultiTurnVirtualEncoder`：把单帧有限范围位置映射为连续多圈位置，识别正向和反向回绕。
- `MotorFeedbackService` / `MotorEncoderService` 语义：真实硬件连接后由硬件实现内部后台线程按固定周期读取电机状态反馈、更新多圈虚拟编码器、保存最新反馈快照。

固定频率反馈更新服务是硬件基础设施，不属于 UI、controller 或自检流程的局部逻辑。controller、自检流程、UI 和日志都只能读取 `MotorFeedback` 最新快照，不应各自临时解析 CAN 帧或各自维护回绕状态。该服务必须在连接后初始化，在断开、重新连接、刷零或严重反馈异常后重置虚拟编码器状态。

达妙当前 DeviceSDK 只提供 CAN/CAN-FD 发送接口和接收回调，没有发现单独读取编码器的同步 API。因此虚拟编码器的数据源定义为“已通过 `DamiaoProtocol::parseFeedback()` 校验的达妙电机状态反馈帧”。寄存器读写回包、控制模式写回、命令发送回显和非本电机/非本 host ID 帧不得进入虚拟编码器。连接阶段必须先通过寄存器读取 `P_MAX/VMAX/TMAX`，再启动反馈线程；如果读取失败，不能用旧默认量程继续解析反馈。`p_m` 与 `xout` 寄存器可用于通信诊断和位置校验，其中当前实测 `p_m` 与普通状态帧按运行时 `P_MAX` 解码结果更一致，`xout` 存在轻微滞后/偏差，不作为主控制位置来源。

### 7.6 hardware_interface/damiao

`hardware_interface/damiao` 负责达妙电机和达妙 USB2FDCAN 设备的具体实现，包括：

- DM-J4310P-2EC 命令序列。
- 控制模式切换。
- 电机使能和失能命令。
- 电机反馈解析。
- 单帧反馈位置到连续多圈电机位置的转换。
- POS_FORCE 模式相关命令。
- 首次位置力控命令前写入 `CTRL_MODE=4`。
- DM-USB2FDCAN_Dual DeviceSDK 接入。
- CANFD/BRS 配置。

### 7.7 hardware_interface/simulated

`hardware_interface/simulated` 提供仿真电机实现，用于控制器单元测试、无硬件 UI 开发、状态机验证和安全逻辑验证。

### 7.8 controller

`controller` 是夹爪控制业务核心，负责：

- 高层控制命令入口。
- 周期性更新。
- 夹爪状态维护。
- 电机命令生成。
- 电机反馈到夹爪状态的转换。
- 协调 `self_check`、`state_machine`、`safety`、`mechanism`、`calibration` 和硬件接口。

`controller` 不应依赖 UI，也不应依赖具体达妙 SDK。

controller 对 UI、CLI 或其他上层调用方提供同步语义：

- 对于 `PreSelfCheck`、回零、行程学习、运动健康检查、夹紧、释放、螺母行程拖动等运动类接口，返回 `Ok` 代表动作已达到该接口定义的完成条件，并且必要的停止/失能收尾已经完成。
- 返回 `OperationTimedOut` 代表在规定时间内没有达到目标或反馈条件，但未发现必须进入 `ActiveStop` 的安全风险。
- 返回安全类错误代表已执行主动停止或失能，状态机应进入 `ActiveStop` 或对应故障状态。
- 上层不得把“命令已发送”理解为“运动已完成”；命令发送只存在于 controller 内部和 hardware_interface 边界。
- `ActiveStop` 只禁止新的业务运动命令和继续输出力矩，不禁止反馈刷新。UI、CLI 或其他上层刷新状态时，controller 应继续读取最新 `MotorFeedback`，更新虚拟多圈位置、螺母估算行程、夹爪角度、电流、速度和故障标志，便于人工判断是否仍在运动或是否已经安全静止。
- 任何安全超限路径在返回上层前必须先执行停止/失能收尾，并尝试读取停机后的新反馈帧。若停稳等待失败，仍必须保证不再持续发送原运动命令，并在错误消息中说明停机/反馈结果。

### 7.9 controller/self_check

`self_check` 是初始化阶段的结构参数学习与健康检查模块，负责：

- 最低稳定运行速度识别。
- 低速不稳定区间识别。
- 静摩擦识别。
- 动摩擦识别。
- 反馈噪声底识别。
- 行程极限识别。
- 软件限位生成。
- 不同安全速度下的运动健康检查。
- 生成并更新 `StructureProfile`。

该模块输出的 `StructureProfile` 是后续控制的关键输入。

### 7.10 controller/state_machine

`state_machine` 负责显式控制阶段。状态机采用两层结构：

- `GripperStateMachine`：顶层任务状态机，管理连接、自检、回零、行程学习、健康检查、夹紧、释放、主动停止和故障。
- `PreSelfCheckStateMachine`：`PreSelfCheck` 内部子状态机，管理预自检的分阶段流程。

顶层状态包括：

- 未连接。
- 已连接。
- 硬件基础检查。
- 模式切换。
- 已使能。
- 预自检。
- 堵转靠零。
- 行程学习。
- 运动健康检查。
- 就绪。
- 夹紧。
- 卸载或释放。
- 已失能。
- 主动安全停止。
- 故障。

主要状态机包括：

- 顶层夹爪状态机。
- 预自检子状态机。
- 回零状态。
- 行程学习状态。
- 运动健康检查状态。
- 夹紧状态。
- 释放/卸载状态。
- 主动停止状态。
- 故障状态。

`ActiveStop` 和 `Fault` 应区分：

- `ActiveStop` 是主动安全停止，通常由限流、限速、接触/堵转检测、用户停止或通信超时触发，可能可以恢复。
- `Fault` 是故障状态，通常表示需要重新初始化、重新回零或人工确认。
- 任意状态均可转入 `ActiveStop` 或 `Fault`，但恢复路径必须由故障严重等级决定。

### 7.11 controller/safety

`safety` 是底层保护模块，在电机命令输出前进行兜底约束，包括：

- 电流限制。
- 速度限制。
- 加速度限制。
- 行程限制。
- 接触检测。
- 堵转检测。
- 机械限位保护。
- 主动停止判断。

即使上层控制器生成了不安全命令，`safety` 也必须能够限制或拒绝该命令。

### 7.12 controller/calibration

`calibration` 不负责初始化自检。它主要负责夹紧力相关映射和样机标定，包括：

- 电机电流/力矩到丝杆推力的映射。
- 丝杆推力到单侧夹爪夹紧力的映射。
- 目标夹紧力到电机命令的映射。
- 使用样机实测数据修正理论模型。

摩擦识别属于 `self_check`，不再放在 `calibration`。

### 7.13 controller/mechanism

`mechanism` 负责机构模型，包括：

- 螺母行程到夹爪角度的映射。
- 螺母速度到夹爪角速度的映射。
- 螺母加速度到夹爪角加速度的映射。
- 接触行程估算。
- 机械行程限制。

已知机构参数：

- 丝杆导程：2 mm。
- 实际可用行程：约 16 mm。
- 线缆直径范围：14-28 mm。
- 0-12 mm 区间可能夹其他物体，因此保护逻辑不能只针对线缆区间。
- 用户修正后的接触行程：
  - 14 mm 线缆：约 15.66 mm。
  - 28 mm 线缆：约 12.98 mm。

### 7.14 ui

`ui` 负责 PC 测试界面，包括连接参数配置、使能/失能、初始化自检、回零、行程学习、目标力夹紧、恒速夹紧、状态显示、日志暂停查看和硬件诊断。

UI 只调用 `controller` 的接口，不直接生成 CAN 帧。

### 7.15 commander

`commander` 负责命令行和调试命令，例如连接、使能、失能、执行自检、回零、行程学习、夹紧、释放、读取状态和导出日志。

命令行工具应复用和 UI 相同的 `controller` 接口。

### 7.16 third_party

`third_party` 只放第三方依赖，例如达妙 SDK 头文件、导入库、运行时 DLL 和必要的第三方源码。

项目自己的封装逻辑应放在 `hardware_interface`，不要放进 `third_party`。

## 8. 数据结构放置原则

推荐原则：

- 数据结构跟随拥有该语义的模块。
- 只有真正跨模块的基础类型才放入 `common`。
- 不把所有数据类型集中到一个 `data` 目录，避免所有模块都依赖一个中心杂物目录。

建议放置方式：

```text
common/
  result.hpp
  error_code.hpp
  timestamp.hpp
  units.hpp

hardware_interface/
  motor_types.hpp
  can_frame.hpp
  adapter_types.hpp

controller/
  gripper_types.hpp

controller/self_check/
  structure_profile.hpp

config/
  gripper_config.hpp

ui/
  ui_view_model.hpp
```

## 9. 关键数据结构规划

### 9.1 StructureProfile

归属文件：

- `controller/self_check/structure_profile.hpp`

建议内容：

- `valid_level`
  - 未识别。
  - 初步识别。
  - 已回零。
  - 已学习行程。
  - 已完成运动健康检查。
- `min_stable_speed_open`
  - 打开方向最低稳定运行速度。
- `min_stable_speed_close`
  - 闭合方向最低稳定运行速度。
- `low_speed_unstable_band`
  - 低速不稳定区间。
- `precheck_limit_open`
  - 预自检阶段基于电机侧受限试探初步发现的打开方向疑似限位，低置信，只用于继续自检和后续流程的临时保护。
- `precheck_limit_close`
  - 预自检阶段基于电机侧受限试探初步发现的闭合方向疑似限位，低置信，只用于继续自检和后续流程的临时保护。
- `theoretical_travel_deviation`
  - 初步限位与理论行程之间的偏差。
- `precheck_safe_zone_min`
  - 预自检建立的低置信临时安全区下限。
- `precheck_safe_zone_max`
  - 预自检建立的低置信临时安全区上限。
- `learned_region_count`
  - 安全区内完成有效学习的区域数量。
- `valid_sample_count`
  - 参数学习有效样本数量。
- `parameter_aggregation_policy`
  - 参数聚合策略，例如最大值、高分位值、平均值或中位数。
- `static_friction_open`
  - 打开方向静摩擦等效电流/力矩。
- `static_friction_close`
  - 闭合方向静摩擦等效电流/力矩。
- `dynamic_friction_open`
  - 打开方向动摩擦等效电流/力矩。
- `dynamic_friction_close`
  - 闭合方向动摩擦等效电流/力矩。
- `viscous_friction_coefficient`
  - 可选黏性摩擦系数。
- `stribeck_velocity_estimate`
  - 可选 Stribeck 影响速度估计。
- `current_noise_floor`
  - 静止电流噪声底。
- `velocity_noise_floor`
  - 静止速度噪声底。
- `position_noise_floor`
  - 静止位置噪声底。
- `open_zero_position`
  - 靠零后的打开方向零位。
- `close_hard_stop_position`
  - 行程学习中识别到的闭合方向极限位置。
- `open_soft_limit`
  - 打开方向软件限位。
- `close_soft_limit`
  - 闭合方向软件限位。
- `safe_stroke_min`
  - 安全行程下限。
- `safe_stroke_max`
  - 安全行程上限。
- `safe_motion_speed_set`
  - 已验证的安全运动速度集合。
- `temperature_at_identification`
  - 识别时温度。
- `repeatability_score`
  - 重复性评价。
- `health_flags`
  - 结构健康检查标志。
- `timestamp`
  - 参数生成时间。

### 9.2 电机反馈 Motor Feedback

归属文件：

- `hardware_interface/motor_types.hpp`

建议内容：

- 电机位置。
- 电机速度。
- 电机电流。
- 电机力矩。
- 电机温度。
- 使能状态。
- 故障状态。
- 原始电机 ID。
- 反馈时间戳。

### 9.3 电机命令 Motor Command

归属文件：

- `hardware_interface/motor_types.hpp`

建议内容：

- 请求控制模式。
- 目标位置。
- 目标速度。
- 目标电流或力矩。
- 使能/失能请求。
- 命令有效标志。
- 命令超时时间。

### 9.4 适配器配置 Adapter Configuration

归属文件：

- `hardware_interface/adapter_types.hpp`

当前已验证的默认值：

- 设备类型：`1`。
- 仲裁域波特率：1 Mbps。
- 数据域波特率：5 Mbps。
- 电机 ID：`0x08`。
- 主机 ID：`0x18`。
- CANFD/BRS：开启。

### 9.5 夹爪状态 Gripper State

归属文件：

- `controller/gripper_types.hpp`

建议内容：

- 控制状态。
- 自检状态。
- `StructureProfile` 有效等级。
- 螺母行程。
- 夹爪角度。
- 估算夹紧力。
- 是否检测到接触。
- 是否已回零。
- 是否已学习软件限位。
- 故障码。
- 最新电机反馈。
- 当前安全限制状态。

### 9.6 夹爪命令 Gripper Command

归属文件：

- `controller/gripper_types.hpp`

建议命令：

- 连接。
- 使能。
- 失能。
- 执行初始化/自检。
- 低电流堵转靠零。
- 行程学习。
- 运动健康检查。
- 目标力夹紧。
- 恒速夹紧。
- 释放。
- 停止。
- 清除故障。

### 9.7 安全限制 Safety Limits

归属文件：

- `controller/gripper_types.hpp`
- 或 `controller/safety/safety_limiter.hpp`

建议内容：

- 最大电流。
- 回零电流限制。
- 自检电流限制。
- 行程学习电流限制。
- 夹紧电流限制。
- 释放电流限制。
- 最大电机速度。
- 最大螺母速度。
- 最低稳定速度下限。
- 最大加速度。
- 行程软限位。
- 堵转检测阈值。
- 接触检测阈值。
- 基于 `StructureProfile` 的阈值补偿量。

## 10. 自检识别策略

### 10.1 PreSelfCheck 总流程

`PreSelfCheck` 的推荐流程如下：

```text
LimitedProbe
  |
  v
BidirectionalMoveEnable
  |
  v
StableShortStrokeMotion
  |
  v
PreliminaryLimitSearch
  |
  v
TheoryTravelCheck
  |
  v
SafeZoneBuild
  |
  v
MultiRegionRoundTripLearning
  |
  v
StructureProfileUpdate
```

阶段说明：

- `LimitedProbe`
  - 在极小行程窗口、低电流、低速度下尝试两个方向。
  - 目标是基于电机侧编码器、电流、力矩和速度反馈，**判断是否存在疑似限位**，以及哪个方向在低能量条件下可继续试探。
  - 该阶段不得假设已经存在可靠螺母零点或最终软件限位；所有行程判断均为相对电机位移换算得到的低置信估计。
- `BidirectionalMoveEnable`
  - 逐步调整速度、电流或力矩限制，让电机在两个方向都能动起来。
  - 如果某个方向无法运动，应先判断是否疑似机械限位，而不是继续加大命令。
  - 试探顺序必须采用同一参数点下的闭合/打开成对试探，而不是先把一个方向全部扫完再扫另一个方向，避免机构被连续推向一侧后影响后续判断。当然如何某个方向有疑似机械限位，那就没必要成对试探。
  - 成对试探的两次动作之间必须停止输出并等待停稳；停稳后的最终反馈才可用于本次位移和方向判定。如果刚换向时出现残余运动，应先按“未停稳/未完成”处理并重试或继续扫描，不应直接把运动中的编码器数据判为方向错误。
  - 未停稳 probe 的 `settled=false` 样本不得被 `breakaway accepted` 接受，也不得触发单向可动降级。单向可动降级只能基于已经停稳、方向一致、无有效位移且电流仍在自检包络内的样本。
  - 参数扫描推荐外层按速度从低到高，内层按电流从小到大。速度扫描范围、电流扫描起点/终点/步长均来自 `self_check` 配置；命令电流仍受 `safety.self_check_current_limit` 截断。
  - 起动识别与稳定短行程识别分开：起动识别只要求方向正确、位移超过起动阈值且无堵转/限位；稳定短行程阶段才要求完成规定行程、速度稳定和电流稳定。
  - 起动识别采用两级阈值：`motion_start_distance_mm` 是正常起动阈值；`low_confidence_motion_distance_mm` 是仅用于 `PreSelfCheck` 第一阶段的低置信微动阈值。低置信微动样本必须满足停稳、停稳后有符号位移朝请求方向、位移超过噪声底、无反向运动、无硬件故障、无主动停止，且反馈电流/力矩未超过全局硬保护阈值。达妙反馈电流/力矩峰值在该阶段用于诊断和硬保护，不得直接用 `self_check_current_limit` 作为样本拒绝条件；`self_check_current_limit` 约束的是控制器下发的试探命令能量。该阶段识别结果只能用于建立低置信保守 profile，不能替代后续稳定短行程、回零、行程学习和运动健康检查。
  - 反方向微动不能因为小于全局行程误差而被当作方向正确。若停稳后有符号位移朝请求反方向并超过低置信微动阈值，应进入低能量漂移/疑似边界降级或安全失败，而不是记录 `breakaway accepted`。
  - 尚未确认任一方向可控时，低能量反向漂移只能记录疑似边界并改试另一个方向，不能直接判定 `PreSelfCheck` 完成。只有至少一个方向已确认可控，或者两个方向均被可靠识别为受阻/疑似边界且符合安全策略时，才允许生成低置信保守 profile；否则应失败并保持滑块禁用。
  - 每次识别到有效起动参数后，应写入运行期 `StructureProfile`，并持久化轻量自检种子，作为下次 `PreSelfCheck` 的扫描起点参考。持久化种子不能等同完整标定文件，仍需重新验证。
  - 如果一个方向已经确认低能量起动，另一个方向在有限的同级电流/速度窗口内没有有效位移，且没有出现相反方向位移、堵转、接触或电流危险，应停止继续扫描该不可动方向，将其记录为低置信疑似边界或临时受阻方向，并继续后续保守 `PreSelfCheck` 流程。
  - 上述单向可动降级只能用于 `PreSelfCheck` 第一阶段建立低置信临时安全区，不代表夹爪已完成双向可靠运动验证；后续 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck` 仍必须重新确认零点、行程和机构运动关系。
- `StableShortStrokeMotion`
  - 确认机构能连续稳定移动规定的小行程。
  - 规定行程应明显大于位置噪声底，但小于自检最大安全窗口。
  - 不能在刚完成起动识别后直接跳到较高速度或较大行程。该阶段也应按配置的短行程距离、速度扫描范围和自检限流进行闭合/打开成对验证。
  - 稳定短行程默认目标可小于后续完整学习行程，例如先使用 `0.3 mm` 验证连续运动，再进入限位搜索和多区域学习。
  - 短行程目标到达容差必须随目标距离收敛，不能直接复用与短行程目标同量级的全局行程误差，否则会把微小抖动误判为到达。
  - 如果稳定短行程失败，日志必须给出目标距离、实际距离、平均速度、最大电流和电流稳定性，便于区分电流上限不足、速度过高、低速爬行或机械卡滞。
  - 第一阶段硬件联调允许低置信通过：如果双向起动已确认，且稳定短行程只得到明显小于目标但方向正确、单调、无堵转的短距离样本，应记录为低置信运动样本并继续建立保守 profile，而不是直接阻断 `PreSelfCheck`。
- `PreliminaryLimitSearch`
  - 在严格限流限速下，初步寻找两个方向的机械限位或疑似限位。
  - 限位发现过程要主动排除结构限位风险：一旦出现速度下降、电流上升、位置不再按预期变化，应立即停止或回退。
  - 第一阶段硬件联调中，如果稳定短行程未达到完整目标，允许使用理论开闭限位建立低置信安全区；真实机械限位必须在后续 `HomingOpenStop` 和 `TravelLearning` 中再次验证。
- `TheoryTravelCheck`
  - 将初步发现的相对电机位移换算行程与理论值比较。
  - 偏差不应过大；如果超过阈值，应进入故障或人工确认，而不是继续自动学习。
- `SafeZoneBuild`
  - 在初步限位基础上扣除安全裕量，建立预自检低置信临时安全区。
  - 后续预自检阶段的多次往复学习只允许在该临时安全区内进行；该安全区不能替代 `TravelLearning` 后的软件限位。
- `MultiRegionRoundTripLearning`
  - 将安全区划分为多个区域，在不同区域进行多次往复运动。
  - 只有运动距离与指令距离匹配、速度稳定、电流无异常的样本才记录为有效样本。
- `StructureProfileUpdate`
  - 根据有效样本生成 `StructureProfile`。
  - 用于安全阈值的参数取保守值；用于诊断和模型拟合的参数保留统计量。
  - `PreSelfCheck` 完成后 `StructureProfile` 只能达到预自检有效等级；完整软件限位和机构运动关系必须等待 `HomingOpenStop`、`TravelLearning`、`MotionHealthCheck` 提升有效等级。
  - 完成 `StructureProfile` 更新后必须失能电机，不持续输出力矩；控制器可保留 `PreSelfCheck` 完成标志，后续 `HomingOpenStop` 由控制器重新使能并进入受限动作。

### 10.2 有效样本筛选与参数聚合

只有满足以下条件的往复运动样本才能进入结构参数统计：

- 实际运动方向与指令方向一致。
- 实际运动距离与指令距离匹配，误差低于配置阈值。
- 运动过程中没有触发限位、堵转或主动停止。
- 匀速段速度波动低于配置阈值。
- 匀速段电流没有异常尖峰。
- 起停阶段没有明显卡滞。
- 样本发生在预自检安全区内。

参数取值建议：

- 用于安全保护、回零、堵转检测、接触检测的阈值，优先取有效样本中的最大值或高分位值，再叠加安全裕量。
- 用于模型估计、状态显示、健康评估的参数，应同时保存平均值、中位数、最大值、最小值和方差。
- 静摩擦、起动电流这类“克服最坏情况”的参数，用最大值更合理。
- 动摩擦这类稳定运动参数，控制保守阈值可用最大值或高分位值，模型显示值可用平均值或中位数。
- 如果样本数量不足，不应给出高置信度参数，应降级为保守默认值并提示重新自检。

结论：

- 不是所有参数都简单取最大值。
- 控制安全相关参数取最大值或高分位值更合理。
- 诊断和模型相关参数应保存统计分布，不能只保留一个最大值。

### 10.3 最低稳定运行速度识别

目标：

- 找到机构能连续、单调、可重复运动的最低速度。
- 避免后续摩擦识别和回零控制落入低速迟滞区。

建议方法：

- 从保守小速度开始，按阶梯逐步增加速度。
- 每个速度保持短时间窗口。
- 检查位置是否单调变化。
- 检查实际速度是否持续接近目标速度。
- 检查电流波动是否不过大。
- 满足稳定性条件的最低速度记入 `StructureProfile`。

识别结果应分方向保存，因为打开方向和闭合方向的摩擦可能不同。

### 10.4 静摩擦识别

目标：

- 识别机构从静止到开始运动所需的起动电流或起动力矩。

建议方法：

- 在严格行程窗口和电流限制下执行。
- 从配置的起始电流开始缓慢提升。当前样机默认起始电流建议为 `0.1 A`，避免过低电流造成长时间无效试探；实际起点仍可由上一次自检种子向上修正，并受安全限流约束。
- 一旦位置变化超过噪声底和最小位移阈值，记录起动电流/力矩。
- `BidirectionalMoveEnable` 阶段只验证“是否起动”。位置变化超过 `motion_start_distance_mm` 后应立即结束当前 probe 并进入主动保持停稳；如果真实硬件在低电流下只能产生超过 `low_confidence_motion_distance_mm` 的可重复同向微动，也可记录为低置信微动起动并停止继续扫参。该阶段不能继续追逐 `max_probe_window` 或 `min_measured_distance` 目标。稳定短行程能力由后续 `StableShortStrokeMotion` 阶段验证。
- 打开方向、闭合方向应成对试探，取稳定统计值。

注意：

- 静摩擦识别用于判断起动门槛。
- 不能直接把静摩擦当作稳定运行摩擦。
- 如果某次试探已经发生小位移但没有走完整个目标窗口，应作为起动候选记录，而不是直接按稳定运动失败丢弃。

### 10.5 动摩擦识别

目标：

- 识别机构稳定运动时的摩擦电流或摩擦力矩。

建议方法：

- 运动速度应高于最低稳定运行速度。
- 排除加速段和减速段，只统计匀速段。
- 打开方向和闭合方向分别识别。
- 至少使用一个安全速度；更推荐使用两个或多个安全速度估计库仑摩擦和黏性摩擦趋势。

注意：

- 速度不能太低，否则会受到低速爬行和 Stribeck 效应影响。
- 如果多速度测试时间过长，第一版可以先记录指定识别速度下的平均动摩擦。

### 10.6 反馈噪声底识别

目标：

- 得到电流、速度、位置在静止状态下的噪声水平。

用途：

- 接触检测阈值。
- 堵转检测阈值。
- 运动开始判定。
- 速度稳定判定。

### 10.7 方向切换响应观察

说明：

- 当前系统只有电机侧传感器，没有夹爪侧、连杆侧或螺母侧独立传感器。
- 因此无法可靠分离真实机构反向间隙、销轴间隙、丝杆螺母间隙和电机侧响应。
- 该项不作为 `StructureProfile` 的核心结构参数。

可保留的诊断信息：

- 方向切换后电机侧速度建立时间。
- 方向切换后电流峰值。
- 方向切换后是否出现异常卡滞或短时堵转。
- 方向切换后的接触检测短暂屏蔽时间可以使用保守配置值，而不是在线学习值。

### 10.8 行程学习

目标：

- 在打开方向靠零后，向闭合方向运行，找到另一方向极限。
- 建立闭合方向软件限位。

要求：

- 行程学习使用比正常夹紧更保守的电流和速度。
- 识别到极限后，应回退安全裕量，设置软件限位。
- 软件限位不能与机械限位完全重合。

### 10.9 运动健康检查

目标：

- 在软件安全范围内，以多个安全速度运行，检查机构运动可靠性。

检查内容：

- 速度跟踪稳定性。
- 电流波动。
- 位置单调性。
- 双向重复性。
- 温度变化。
- 是否出现异常堵转、卡滞或电流突增。

## 11. 运行状态流转

顶层状态流转：

```text
Disconnected
   |
   v
Connected
   |
   v
HardwareSanityCheck
   |
   v
ModeSwitching
   |
   v
Enabled
   |
   v
PreSelfCheck
   |
   v
HomingOpenStop
   |
   v
TravelLearning
   |
   v
MotionHealthCheck
   |
   v
Ready
   |
   +------------+-------------+
   |                          |
   v                          v
Clamping                  Releasing
   |                          |
   v                          v
Unloading --------------> Ready
   |
   v
Disabled

任意状态下，如果触发安全规则或硬件异常，均可进入 Fault 或 ActiveStop。
```

`PreSelfCheck` 内部子状态流转：

```text
Idle
  |
  v
LimitedProbe
  |
  v
BidirectionalMoveEnable
  |
  v
StableShortStrokeMotion
  |
  v
PreliminaryLimitSearch
  |
  v
TheoryTravelCheck
  |
  v
SafeZoneBuild
  |
  v
MultiRegionRoundTripLearning
  |
  v
StructureProfileUpdate
  |
  v
Completed

任意子状态均可进入 Failed，并由顶层状态机决定进入 ActiveStop、Fault 或保守回零。
```

状态机头文件对应关系：

```text
gripper_state_machine.hpp
  - GripperTopState
  - GripperEvent
  - GripperStateSnapshot
  - GripperStateMachine

self_check_state.hpp
  - PreSelfCheckPhase
  - PreSelfCheckEvent
  - PreSelfCheckSnapshot
  - PreSelfCheckStateMachine

homing_state.hpp
  - HomingPhase
  - HomingStateSnapshot

travel_learning_state.hpp
  - TravelLearningPhase
  - TravelLearningSnapshot

motion_health_check_state.hpp
  - MotionHealthCheckPhase
  - MotionHealthCheckSnapshot

clamp_state.hpp
  - ClampPhase
  - ClampStateSnapshot

release_state.hpp
  - ReleasePhase
  - ReleaseStateSnapshot

active_stop_state.hpp
  - ActiveStopReason
  - ActiveStopSnapshot

fault_state.hpp
  - FaultSeverity
  - FaultSource
  - FaultSnapshot
```

## 12. 达妙硬件命令规则

达妙硬件实现必须保留前期联调经验：

- 设备连接成功不等于电机控制成功。
- 必须在传输边界记录 TX/RX 日志。
- 位置力控命令路径首次执行前必须显式写入 `CTRL_MODE=4`。
- 初始电机模式应视为未知，不能默认认为已经在目标模式。
- 电机使能前必须先切换到正确控制模式。
- 不允许在第一次收到反馈时自动置零。
- 只有回零流程完成后，才能建立零点参考。
- 回零前的速度运动应使用当前位置附近的小相对目标，不能假设全行程终点。
- 当前 DM-USB2FDCAN_Dual 配置下，默认电机命令帧应使用 CANFD 和 BRS。
- 达妙单帧 `q_uint -> [-P_MAX, P_MAX]` 解析值不能由 controller 直接作为螺母行程换算依据；必须先经过硬件层多圈虚拟编码器后，才能作为 `MotorFeedback.position` 交给控制器。`P_MAX` 必须来自连接阶段读取的运行时寄存器。
- 达妙硬件实现必须输出连续多圈 `MotorFeedback.position`。回绕判断使用配置项 `motor.encoder_unwrap_source` 和 `motor.encoder_wrap_range_rad`。当前样机默认 `encoder_unwrap_source = protocol_position`、`encoder_wrap_range_rad = 0`，表示真实硬件连接后由 `2 * runtime P_MAX` 推导；当前实测为 `100 rad`。
- 当前样机状态帧中的 `q_uint` 不是物理单圈计数，而是协议量程位置编码的原始 16 位值；它仅用于日志诊断，不得按 `2*pi rad` 直接累计成多圈位置。
- 达妙硬件实现连接成功后必须启动后台反馈更新服务。默认周期为 `motor.feedback_poll_period_s = 0.02 s`，即 50 Hz。按当前实测 `VMAX=20 rad/s` 计算，相邻更新的理论最大位移约 `0.4 rad`，远小于当前协议位置半量程 `50 rad`，满足当前协议位置虚拟编码器回绕判定条件。
- 每轮反馈刷新不能只取接收队列中的第一帧。硬件实现应在短时间窗口内连续读取并解析到最新状态帧，再更新 `last_feedback_`，避免命令期间 CAN 回包积压导致 UI 和 controller 使用旧位置。
- 反馈更新频率必须足够高，使相邻两次虚拟编码器更新之间的实际电机位移小于半个回绕范围；否则多圈映射存在歧义，应进入反馈异常或要求降低速度/提高采样频率。
- `MotorInterface::readFeedback()` 在真实硬件上只返回后台服务维护的最新快照。如果反馈超过 `motor.feedback_stale_timeout_s` 未更新，应返回反馈超时或硬件反馈异常，控制器不得用过期位置继续计算行程。

## 13. 安全策略

安全策略采用分层保护：

```text
用户命令
  -> UI/CLI 参数检查
  -> controller 状态检查
  -> 状态机生成命令
  -> safety 限制和裁剪
  -> motor command
  -> hardware transport
```

`safety` 是强制路径，正常 UI 和 CLI 命令不能绕过。

重点安全场景：

- 自检必须使用受限小行程、低电流、低速度。
- 最低稳定速度识别不能突破自检速度上限。
- 摩擦识别速度必须高于最低稳定运行速度。
- 回零必须使用自检得到的保守速度和电流阈值。
- 行程学习必须比正常夹紧更保守。
- 夹紧接近阶段使用受控恒速。
- 接触检测结合电流/力矩上升、速度下降和当前行程上下文。
- 机械限位检测结合行程和堵转行为。
- 夹紧完成后，根据测试结果执行小幅卸载或降电流，然后电机失能。
- 故障或主动停止时，根据已验证的电机行为执行安全零输出或失能。

## 14. 管理员维护恢复模式

维护功能需要分成两个子模式：

```text
Maintenance / Admin
  ├─ MotorBringupMode       电机空载/结构未安装调试
  └─ AdminRecoveryMode      机构抱死/故障恢复
```

`MotorBringupMode` 用于调试初期结构未安装时确认电机通信、模式切换、使能、正反转点动和反馈方向。它属于维护权限，但不属于机构故障恢复，不更新 `StructureProfile`、零位或软件限位。

`AdminRecoveryMode` 用于结构已安装且发生丝杆螺母抱死、夹爪无法打开、普通释放失败等现场问题。

维护模式是独立的维护控制通道，不是普通 `release()`、`clampByForce()` 或 `clampBySpeed()` 的 bypass 参数。

核心约束：

- 不能无限制放开电流、速度或行程。
- 管理员模式可以绕过部分普通业务限制，但不能绕过硬保护。
- 硬保护包括电机温度、通信超时、命令看门狗、最大单次点动时间、最大连续使能时间、工程最大电流、工程最大速度、电机硬件故障、用户停止和断开连接。
- 高电流打开前必须确认打开方向可信。
- 如果打开方向未知或低可信，只允许低能量方向试探，不允许升流恢复。
- 如果期望打开但实际位移方向相反，或出现电流/力矩快速上升但无释放位移，应立即停止、失能并进入管理员恢复阻止状态。
- 管理员恢复后不能直接进入 `Ready`，必须重新执行 `PreSelfCheck`、`HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck`。

建议状态机新增：

```text
MotorBringupArmed
MotorBringupJogging
AdminRecoveryArmed
AdminDirectionProbe
AdminRecoveryJogging
AdminRecoveryBlocked
```

建议安全策略分层：

```text
HardSafetyPolicy
AdminRecoverySafetyPolicy
NormalSafetyPolicy
```

其中 `HardSafetyPolicy` 不允许被管理员模式绕过。管理员恢复模式的第一版实现应优先支持低能量方向试探、短时点动、方向一致性检测、越卡越紧检测、失能收尾和审计记录，不应实现 raw motor command 或无限持续力矩输出。

### 14.1 允许绕过与禁止绕过

`MotorBringupMode` 只能在结构未安装或已明确确认空载时使用。它可以直接使用电机侧正反向点动命令验证通信和方向，但必须保留硬保护、短时命令、低默认电流、低默认速度和命令后失能。

`MotorBringupMode` 禁止：

- 更新夹爪零位。
- 更新软件限位。
- 更新 `StructureProfile`。
- 进入 `Ready`。
- 作为结构已安装后的故障恢复通道。

`MotorBringupMode` 只能确认电机命令正反方向和编码器反馈方向，不能确认夹爪真实打开/闭合方向。夹爪打开方向仍必须在结构安装后通过 `PreSelfCheck`、回零、行程学习或管理员恢复低能量方向试探确认。

`AdminRecoveryMode` 的允许绕过与禁止绕过如下。

管理员模式可以在授权条件下绕过：

- 普通夹紧限流。
- 普通夹紧限速。
- 普通软件行程限位。
- 普通接触/堵转检测阈值。
- `Ready` 状态前置条件。

管理员模式禁止绕过：

- 最高工程电流。
- 最高工程速度。
- 电机温度上限。
- 通信超时。
- 命令看门狗。
- 单次命令最长持续时间。
- 连续使能最长时间。
- 电机硬件故障。
- 用户停止或急停。
- 打开方向未确认时的高电流恢复禁止规则。

### 14.2 打开方向主动识别

夹爪抱死时，管理员可能希望用更大电流尝试打开。但如果电机方向配置、坐标映射或用户判断错误，实际动作可能继续闭合，使机构卡得更紧。因此必须在高电流恢复前设置打开方向保护。

打开方向可信度来自：

- 上一次成功的 `PreSelfCheck`。
- 上一次成功的 `HomingOpenStop`。
- 上一次成功的 `TravelLearning`。
- 管理员恢复模式内的低能量方向试探。

如果方向仅来自配置文件，可信度应标记为低；如果方向未知或低可信，禁止直接进入高电流恢复。

建议方向可信等级：

```cpp
enum class MotionDirectionConfidence {
  Unknown,
  ConfigOnly,
  ProbeConfirmed,
  SelfCheckConfirmed,
  HomingConfirmed,
  TravelLearningConfirmed,
};
```

高电流恢复至少要求 `ProbeConfirmed`。如果现场风险较高，可以要求 `SelfCheckConfirmed` 或更高。

方向不可信时，管理员只能执行 `AdminDirectionProbe`。试探要求：

- 极低电流。
- 极低速度。
- 极短时间。
- 极小目标位移。
- 每次试探后立即失能。
- 只用来判断反馈位移方向、电流变化和是否存在立即卡死迹象。

试探成功条件：

- 编码器位置变化方向与期望打开方向一致。
- 电流或力矩没有异常快速上升。
- 速度反馈与命令方向一致。
- 位移大于噪声阈值。

试探失败时，不能升流。

### 14.3 越卡越紧识别

管理员请求打开时，如果出现以下情况之一，应认为存在方向错误或机构进一步锁紧风险：

- 期望打开，但螺母估算行程朝闭合方向变化。
- 期望打开，但电机编码器位置变化方向与打开方向相反。
- 期望打开，位移接近零，但电流或力矩快速上升。
- 期望打开，闭合方向行程继续增加。
- 低能量试探阶段已经触发堵转或高阻力迹象。

处理策略：

```text
立即停止命令
立即失能
记录故障和审计日志
进入 AdminRecoveryBlocked
UI 明确提示方向异常或疑似越卡越紧
禁止继续升流恢复
```

建议阻止原因：

```cpp
enum class AdminRecoveryBlockReason {
  None,
  OpenDirectionUnknown,
  ActualMotionOpposesCommand,
  CurrentRiseWithoutReleaseMotion,
  SuspectedFurtherTightening,
  TemperatureLimit,
  CommandTimeout,
  ContinuousEnableTimeout,
  HardwareFault,
  UserStop,
};
```

### 14.4 状态机流转

建议流转：

```text
ActiveStop/Fault/Disabled/Enabled
  -> AdminRecoveryArmed
  -> AdminDirectionProbe
  -> AdminRecoveryJogging
  -> Disabled

AdminDirectionProbe/AdminRecoveryJogging
  -> AdminRecoveryBlocked
  -> Disabled
```

退出管理员恢复后不允许直接进入 `Ready`。必须重新执行：

```text
PreSelfCheck
  -> HomingOpenStop
  -> TravelLearning
  -> MotionHealthCheck
  -> Ready
```

### 14.5 控制器接口建议

建议新增管理员维护接口，不污染普通业务接口。

```cpp
common::Result enterMotorBringupMode(const MotorBringupRequest& request);
common::Result motorBringupJog(const MotorBringupJogCommand& command);
common::Result exitMotorBringupMode();

common::Result enterAdminRecovery(const AdminRecoveryRequest& request);
common::Result adminProbeOpenDirection(const AdminDirectionProbeCommand& command);
common::Result adminJog(const AdminJogCommand& command);
common::Result adminPulseRelease(const AdminPulseReleaseCommand& command);
common::Result adminClearMotorFault();
common::Result exitAdminRecovery();
```

接口约束：

- 电机空载调试接口必须要求结构未安装或空载确认。
- 电机空载调试接口只能确认电机命令方向和编码器反馈方向，不能确认夹爪打开/闭合方向。
- 所有管理员命令必须有权限等级和确认信息。
- 每个动作必须有超时。
- 每个动作结束后默认失能。
- 每个动作必须写入审计记录。
- `adminJog()` 不能使用无限持续命令。
- `adminPulseRelease()` 应使用短脉冲、分级升流和方向一致性检测。

### 14.6 数据结构建议

```cpp
enum class AdminRecoveryPrivilege {
  None,
  Service,
  Engineering,
  Factory,
};

struct MotorBringupRequest {
  AdminRecoveryPrivilege privilege;
  bool unloaded_or_structure_removed_confirmed;
};

struct MotorBringupJogCommand {
  common::RadPerS motor_velocity;
  common::A max_current;
  common::S duration;
};

struct MotorFeedback {
  common::Rad position;             // 连续多圈电机位置，控制和行程估算使用
  common::Rad wrapped_position;     // 单帧有限范围位置，仅诊断
  bool wrapped_position_valid;
  common::RadPerS velocity;
  common::A current;
  common::Nm torque;
  common::DegC temperature;
  std::uint32_t raw_position_counts; // 达妙 q_uint，仅诊断
  bool raw_position_counts_valid;
  bool enabled;
  bool fault;
};

struct VirtualEncoderConfig {
  common::Rad wrap_range;
  bool enabled;
};

enum class EncoderUnwrapSource {
  ProtocolPosition,
  RawPositionCounts,
};

struct AdminRecoveryRequest {
  AdminRecoveryPrivilege privilege;
  bool user_confirmed_risk;
  bool require_direction_probe;
};

struct AdminDirectionProbeCommand {
  common::A probe_current;
  common::MmPerS probe_nut_speed;
  common::S probe_time;
  common::Mm expected_min_motion;
};

struct AdminJogCommand {
  MotionDirection direction;
  common::A max_current;
  common::MmPerS max_nut_speed;
  common::S duration;
  bool bypass_software_stroke_limit;
};

struct AdminRecoveryAuditRecord {
  AdminRecoveryPrivilege privilege;
  MotionDirection requested_direction;
  MotionDirectionConfidence direction_confidence_before;
  common::A command_current;
  common::MmPerS command_nut_speed;
  common::S command_duration;
  common::Mm stroke_before;
  common::Mm stroke_after;
  common::A peak_current;
  common::Nm peak_torque;
  common::DegC peak_temperature;
  AdminRecoveryBlockReason block_reason;
};
```

### 14.7 UI 要求

UI 应单独提供维护页面，不应把管理员功能放在普通夹紧、释放按钮旁边。

维护页面应拆分为：

- 电机空载调试：用于结构未安装时验证通信、使能、正反转点动和反馈。
- 管理员恢复：用于结构已安装后的卡滞和抱死恢复。

UI 必须显示：

- 当前是否处于管理员恢复模式。
- 权限等级。
- 打开方向可信度。
- 是否允许升流。
- 当前电流、速度、力矩、温度。
- 当前阻止原因。
- 最近一次管理员命令的结果。

UI 必须提供：

- 明确的风险确认。
- 急停/停止按钮。
- 点动命令持续时间限制。
- 日志暂停查看。
- 恢复后“必须重新自检”的提示。

当打开方向不可信时，UI 不应显示或不应允许点击高电流打开按钮，只允许执行方向试探。

电机空载调试页面必须显示“结构未安装/空载确认”状态，并明确提示该模式不会确认夹爪打开/闭合方向。

Web UI 顶部应提供可切换分页，至少包括：

- 普通控制：目标力夹紧、释放和普通业务参数。
- 自检流程：`PreSelfCheck`、回零、行程学习和运动健康检查。
- 维护模式：`MotorBringupMode` 和 `AdminRecoveryMode`。
- 日志与诊断：运行日志、通信诊断、ActiveStop/Fault 恢复提示。

分页只负责界面组织，不改变控制器状态机和权限边界。隐藏页面中的动作仍必须由前端门禁和后端 controller 门禁共同限制。

### 14.8 受限螺母行程拖动

夹爪状态区可以提供“螺母位置拖动”滑块，用于调试过程中验证电机多圈编码器、丝杆导程和螺母行程映射是否正确。例如电机输出端反馈转一圈时，螺母估算行程应接近丝杆导程 `2 mm/rev`。该功能不是管理员 bypass，也不是原始电机命令入口，必须调用 controller 的受限螺母行程移动接口。

启用条件：

- 未完成 `PreSelfCheck` 时禁用。
- 完成 `PreSelfCheck` 但未完成 `MotionHealthCheck` 时，只允许在 `StructureProfile.travel_limits.safe_zone_open_limit` 到 `safe_zone_closed_limit` 的低置信临时安全区内移动。
- 完成 `MotionHealthCheck` 后，只允许在 `software_open_limit` 到 `software_closed_limit` 的最终置信范围内移动。
- `ActiveStop`、`Fault`、`MotorBringupMode` 或管理员恢复阻止状态下禁用。
- UI 滑块显示范围必须来自 controller 快照或 `/api/view` 中的当前手动定位允许范围字段。
- `PreSelfCheck` 后但 `MotionHealthCheck` 前，如果未确认结构未安装/空载，该字段应是“当前估算螺母位置附近的小窗口”和低置信安全区的交集，而不是完整低置信安全区。
- `PreSelfCheck` 后但 `MotionHealthCheck` 前，如果 UI 和命令明确确认结构未安装/空载，可使用完整低置信边界作为滑块范围，用于验证编码器-丝杆行程映射。

控制要求：

- UI 只提交目标螺母行程、期望螺母速度等业务参数，不直接生成 CAN 帧或电机私有命令。
- controller 必须在运动前检查目标是否落在当前允许范围内，不允许静默裁剪到范围内。
- controller 必须先检查连接状态、流程状态、低置信/最终置信允许范围和目标合法性，再执行电机使能。越界、未自检、终态或维护模式拒绝时不得发送使能命令。
- 该入口是调试定位功能，不是最终夹紧控制。实现上应优先使用带命令级限流的电机位置力控/位置限矩模式；若只能使用普通位置模式，则必须使用更严格的反馈监控和更低速度。
- `PreSelfCheck` 完成但 `MotionHealthCheck` 未完成时，低置信安全区只表示理论保护边界，不代表绝对行程已经标定；滑块单次命令还必须限制在当前估算螺母位置附近的小窗口内。该窗口应来自配置或自检最大安全窗口，并由 controller 后端强制执行，防止 UI 一次下发接近整圈或多圈的目标。
- 上一条小窗口限制只适用于结构已安装或空载未确认场景。结构未安装/空载已确认时，允许在低置信理论边界内做限流、限速、运动后失能的验证运动。
- 结构未安装/空载已确认时，完整低置信理论边界只是用户可选择的目标范围，不等于底层电机单次命令跨度。controller 必须在内部按配置速度、反馈刷新周期和保守最大前瞻距离，把目标拆成连续小步位置目标，避免一次性位置误差过大导致电机短促冲击。
- 低置信范围内移动使用自检/行程学习级别的保守速度和反馈电流阈值；最终置信范围内移动也不得超过全局限速、限加速度和电流反馈阈值。
- 当前默认配置将受限螺母滑块/手动定位命令限流设置为 `safety.manual_positioning_current_limit_a = 1.5 A`，全局反馈硬保护设置为 `safety.max_motor_current_a = 2.0 A`。前者用于空载/低置信滑块调试的命令限流和持续反馈超限判据，后者用于所有流程不可绕过的硬保护。
- 运动日志必须输出 `start_motor_pos_rad`、`end_motor_pos_rad`、`motor_delta_rad`、按 `2*pi rad/rev` 换算的 `motor_delta_rev`、`measured_delta_mm` 和 `mm_per_rev_estimate`，便于人工判断电机转一圈螺母是否约走 `2 mm`。
- 如果底层位置模式没有命令级限流能力，controller 必须明确将电流限制作为反馈监控阈值；反馈电流尖峰不应按单帧直接判定为故障，应结合持续时间、速度下降、行程停滞和限位上下文判断。持续超限、堵转、限位或硬件故障超限时立即停止并进入 `ActiveStop`。
- 手动拖动触发电流、堵转、限位或硬件故障超限时，controller 进入 `ActiveStop` 前必须先停止/失能电机输出并刷新最终反馈；UI 中的夹爪状态仍应继续更新，方便人工确认电机是否仍在运动。
- 普通超时或未到达目标但没有电流/堵转/限位风险时，应停止并失能，返回“未到位/超时”结果，不应直接升级为 `ActiveStop`。
- 运动过程中仍使用速度、行程、反馈电流、接触/堵转检测和通信超时检测。
- 每次拖动运动完成、失败或超时后都必须失能电机，避免丝杆螺母持续受力抱死。
- 运动结果和拒绝原因必须写入运行日志，便于人工确认目标、实际行程、电流、速度和状态变化。

## 15. 配置策略

配置由 `app` 加载，再传入各个模块。

计划配置分组如下：

- `adapter`
  - 设备类型。
  - 通道。
  - FDCAN/BRS 开关。
  - 仲裁域波特率。
  - 数据域波特率。
- `motor`
  - 电机 ID。
  - 主机 ID。
  - 控制模式。
  - 方向符号。
  - 位置命令符号。
  - 编码器比例。
  - 虚拟编码器使能。
  - 虚拟编码器回绕数据源。
  - 虚拟编码器物理回绕范围。
  - 原始编码器计数范围。
  - 固定频率反馈更新周期。
  - 反馈过期超时。
  - `direction_sign` 表示电机反馈位置与螺母行程估算之间的机构方向关系。
- `position_command_sign` 表示达妙位置类命令目标相对反馈位置的协议符号修正，只在硬件实现生成 Position/PositionForce 帧时使用。
- 当前样机硬件联调默认 `position_command_sign = 1`。该值不是机构开闭方向定义，不能替代 `direction_sign`；换电机、固件、控制模式或 SDK 后，应先通过低能量位置类命令确认 `target_motor_delta_rad` 与 `motor_delta_rad` 是否同向。
- `mechanism`
  - 丝杆导程。
  - 可用行程。
  - 理论行程软限位。
  - 运动学表格或几何参数。
- `self_check`
  - 自检最大行程窗口。
  - 最低速度扫描范围。
  - 最低速度扫描步长。
  - 静摩擦识别电流斜坡。
  - 动摩擦识别速度集合。
  - 反馈噪声采样时间。
  - 行程学习速度。
  - 软件限位安全裕量。
  - 运动健康检查速度集合。
- `safety`
  - 电流限制。
  - 速度限制。
  - 加速度限制。
  - 堵转阈值。
  - 超时时间。
- `homing`
  - 回零方向。
  - 回零速度策略。
  - 回零电流策略。
  - 堵转确认时间。
- `clamp`
  - 目标夹紧力。
  - 目标夹紧速度。
  - 接近阶段限制。
  - 卸载距离。
- `ui`
  - 刷新频率。
  - 日志容量。
  - 默认控件参数。
- `admin_recovery`
  - 管理员恢复功能开关。
  - 权限等级限制。
  - 是否要求二次确认。
  - 是否要求高电流恢复前方向试探。
  - 维护级和工程级电流限制。
  - 维护级和工程级速度限制。
  - 单次点动最长时间。
  - 连续使能最长时间。
  - 会话超时。
  - 方向试探电流、速度、时间和最小有效位移。

## 16. 测试策略

前期应优先支持无硬件测试：

- `StructureProfile` 状态有效性测试。
- 最低稳定速度识别逻辑测试。
- 摩擦识别逻辑测试。
- 行程学习逻辑测试。
- 机构运动学查表测试。
- 安全限制器测试。
- 状态机流转测试。
- 仿真电机回零测试。
- 仿真电机夹紧测试。
- 配置解析测试。
- 管理员恢复状态机测试。
- 管理员恢复权限和二次确认测试。
- 打开方向未知时禁止高电流恢复测试。
- 低能量方向试探测试。
- 实际位移方向与打开方向相反时阻止恢复测试。
- 电流/力矩上升但无释放位移时阻止恢复测试。
- 管理员恢复退出后禁止直接进入 Ready 测试。

硬件联调测试：

- 适配器打开和关闭。
- 通道配置。
- TX/RX 帧日志。
- 模式切换。
- 使能和失能。
- 最低稳定速度识别。
- 静摩擦识别。
- 动摩擦识别。
- 低电流堵转靠零。
- 行程学习和软件限位设置。
- 软件安全范围内的多速度运动健康检查。
- 恒速夹紧。
- 目标力夹紧。
- 夹紧后释放。
- 多次夹紧/释放循环，验证丝杆螺母不会抱死。
- 管理员恢复低能量方向试探。
- 错误方向配置下禁止升流。
- 管理员短时点动后自动失能。
- 管理员恢复阻止状态和 UI 提示验证。

## 17. 当前骨架状态

当前 `src/` 下文件均为空占位文件，只用于明确后续重构的模块边界。

后续实现建议顺序：

1. `common` 基础类型。
2. `config` 配置加载。
3. `hardware_interface` 抽象接口。
4. `hardware_interface/simulated` 仿真电机。
5. `controller/self_check` 结构参数数据结构和识别流程。
6. `controller/safety` 安全限制。
7. `controller/state_machine` 自检、回零、行程学习和夹紧状态机。
8. `controller/mechanism` 机构运动学和行程映射。
9. `controller/calibration` 夹紧力映射。
10. `controller/admin_recovery` 管理员维护恢复模式。
11. `hardware_interface/damiao` 达妙硬件实现。
12. `ui` 测试界面。
13. `commander` 命令行调试工具。

## 18. 调研参考

本版自检设计参考了伺服摩擦和低速 stick-slip 问题的通用结论：

- Canudas de Wit、Olsson、Astrom、Lischinsky 的 LuGre 摩擦模型论文说明摩擦建模需要考虑动态摩擦行为和控制补偿问题。
- Karnopp 的 stick-slip 仿真工作强调零速附近强非线性会导致仿真和控制困难。
- 伺服 stick-slip 研究表明低速连续运动时容易出现由摩擦引起的周期性误差，速度反馈对抑制该问题很关键。

参考链接：

- https://www.lu.se/lup/publication/0c411ed4-a01c-41e2-852c-6586fa9295e7
- https://www.deepdyve.com/lp/crossref/computer-simulation-of-stick-slip-friction-in-mechanical-dynamic-IEpnhlLpF0
- https://www.mdpi.com/1424-8220/22/1/383
