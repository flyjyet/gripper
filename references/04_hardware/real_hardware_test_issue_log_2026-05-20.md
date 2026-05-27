# 2026-05-20 真实硬件测试问题盘点

本文记录 C++ 重构版接入 `DM-USB2FDCAN_Dual` 和 `DM-J4310P-2EC` 过程中出现的问题、判断依据、处理方式和当前状态。

## 测试范围

已执行：

- 自动化构建和 CTest 回归。
- 模拟电机 `MotorBringup` 命令路径冒烟。
- 真实硬件 `--damiao` 路径的无运动测试：
  - `connect`
  - `bringup_enter_confirm_unloaded`
  - `bringup_feedback`
  - `disconnect`
  - `quit`

未执行：

- `bringup_enable`
- `bringup_jog_pos`
- `bringup_jog_neg`
- `selfcheck/home/learn/clamp`

原因：当前还没有收到任何电机反馈帧，不能进入使能或点动测试。

## 问题 1：`libdm_device.dll` 加载失败

现象：

```text
connect: HardwareUnavailable
failed to load src/third_party/damiao/bin/libdm_device.dll
```

进一步诊断后显示：

```text
Win32 error 126
```

判断：

- 主 DLL 文件存在。
- Win32 error 126 通常表示 DLL 的依赖项缺失。
- 通过 PE 导入表检查发现 `libusb-1.0.dll` 依赖 `VCRUNTIME140.dll`。

处理：

- 将以下 DLL 放入 `src/third_party/damiao/bin/`：
  - `VCRUNTIME140.dll`
  - `VCRUNTIME140_1.dll`
- C++ transport 改为预加载同目录依赖 DLL，再加载 `libdm_device.dll`。

当前状态：

- 已解决。
- `connect` 已可成功打开 `DM-USB2FDCAN_Dual`。

## 问题 2：`AddDllDirectory` 方案触发应用控制策略拦截

现象：

```text
应用程序控制策略已阻止此文件
An Application Control policy has blocked this file
```

出现阶段：

- 曾尝试在 C++ 中使用 `AddDllDirectory` +
  `LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR | LOAD_LIBRARY_SEARCH_DEFAULT_DIRS`。
- 该版本生成后，`gripper_app.exe` 无法启动。

处理：

- 移除 `AddDllDirectory` 方案。
- 改为按顺序显式 `LoadLibraryA()` 预加载 SDK 同目录依赖 DLL：
  - `VCRUNTIME140.dll`
  - `VCRUNTIME140_1.dll`
  - `libwinpthread-1.dll`
  - `libgcc_s_seh-1.dll`
  - `libstdc++-6.dll`
  - `libusb-1.0.dll`
  - `libdm_device.dll`

当前状态：

- 已解决。
- `gripper_app.exe` 可正常启动。

备注：

- 后续 `test_motor_bringup.exe` 曾出现一次同类应用控制策略拦截。
- 删除该 exe 并重新链接后恢复，判断为本机策略/新生成二进制的瞬态拦截，不是测试逻辑失败。

## 问题 3：连接成功后 `disconnect/quit` 卡住

现象：

- `connect` 成功后执行 `disconnect` 或 `quit`，程序无法正常返回。
- 控制台输出：

```text
libusb_transfer_cancelled or error
```

判断：

- 问题发生在 SDK 关闭链路，不在连接或控制器状态机。
- 对照旧版已验证 Python UI，旧版也规避了部分达妙 Windows SDK 释放函数。
- 当前 PC 上调用 SDK `device_close` / `device_disable_channel` 后可能卡在 USB transfer cancellation。

处理：

- C++ transport 当前采用调试期非阻塞关闭策略：
  - 清理本进程状态和接收队列。
  - 不调用会卡住的 SDK `device_close` / `device_disable_channel`。
  - 由进程退出和 OS 回收 USB 资源。

当前状态：

- 已解决控制台卡死问题。
- `connect -> disconnect -> quit` 可正常返回。

风险：

- 这是调试期折中方案，不是最终严格资源释放方案。
- 后续应单独做 SDK 关闭兼容性验证，确认是否需要使用新版 SDK、旧版 API、后台关闭线程或独立硬件进程隔离。

## 问题 4：`bringup_feedback` 超时，无任何 RX 帧

现象：

```text
bringup_feedback: FeedbackTimedOut
no DM CAN frame received before timeout
tx_count=2 last_tx_id=0x7ff
rx_count=0 last_rx_id=0x0 last_rx_len=0
```

判断：

- `tx_count=2` 说明刷新帧已经通过 SDK 发送。
- `last_tx_id=0x7ff` 说明当前发送的是达妙刷新/寄存器类命令 ID。
- `rx_count=0` 说明 USB2FDCAN 回调没有收到任何 CAN/CANFD 帧。
- 通道 0 和通道 1 都测试过，结果相同。
- 2026-05-20 官方上位机截图显示 FDCAN、仲裁域 `1M`、数据域 `5M`、采样点 `75.0`。
- C++ 新增 SDK 通道配置读回后，确认当前程序实际配置为：

```text
channel_readback=ok rb_ch=0 rb_canfd=1 rb_nominal_bps=1000000 rb_data_bps=5000000 rb_can_sp=0.75 rb_canfd_sp=0.75
```

- 因此当前 `RX none` 不应优先判断为 FDCAN 波特率或采样点不一致。
- 官方上位机中的 `device:1,3` 判断为官方工具内部的 libusb 设备端口绑定信息；当前 DeviceSDK C 接口按 `device_index` / `channel_index` 使用，当前 `sdk_count=1` 时 `device_index=0` 应对应已插入的唯一适配器。
- 2026-05-20 用户补充官方上位机通信参数截图：
  - `CAN ID=0x01`
  - `Master ID=0x11`
  - `CAN Baud=5M`
- 项目此前仍使用旧参数 `motor_id=0x08`、`host_id=0x18`。

当前状态：

- 已解决。
- 根因是项目默认电机 ID / 反馈 ID 与当前电机参数不一致。
- 已将 `src/config/default_gripper.yaml` 和 `src/config/gripper_config.hpp`
  的默认值更新为：
  - `motor_id: 0x01`
  - `host_id: 0x11`

修复后验证：

```text
bringup_can_probe | TX refresh FD id=0x7FF dlc=8 brs=1 data=01 00 cc 00 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 79 b2 7f f7 ea 21 1f
bringup_can_probe | TX query_master_id FD id=0x7FF dlc=8 brs=1 data=01 00 33 08 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 00 33 08 01 00 00 00
bringup_can_probe: Ok
```

```text
bringup_feedback: Ok
stroke_mm=-0.195923
motor_pos_rad=-0.615511
motor_vel_rad_s=-0.00732601
motor_current_a=-0.019536
motor_torque_nm=-0.01221
temp_c=33.000
enabled=false
```

下一步现场测试：

- 仍不要直接进入夹爪机构高风险动作。
- 先在结构未安装或机构机械安全状态下测试 `bringup_enable` /
  `bringup_disable`。
- 再以低电流、短脉冲测试 `bringup_jog_pos` / `bringup_jog_neg`，确认实际
  方向和电流/力矩反馈。

## 当前测试结论

已确认：

- C++ 程序可构建。
- 6 个自动化测试通过。
- 达妙 SDK DLL 及依赖可加载。
- `DM-USB2FDCAN_Dual` 可被打开。
- 通道 0/1 均可配置并进入连接成功状态。
- 通道 0 已读回 FDCAN `1M/5M`、采样点 `0.75/0.75`。
- 使用当前电机 ID `0x01`、反馈 ID `0x11` 后，`bringup_can_probe` 和
  `bringup_feedback` 已成功。
- 控制台退出不再卡住。

未确认：

- 电机使能。
- 电机正反点动。
- 真实方向。
- 电流/力矩缩放。
- 后续 PreSelfCheck、回零、行程学习和夹紧闭环。

## 问题 5：Bring-up 使能后出现异常高速/大电流反馈，正反点动方向疑似相同

现象：

```text
bringup_enable: Ok | controller_state=Connected |
stroke_mm=-3.96855 | motor_pos_rad=-12.4676 |
motor_vel_rad_s=-27.6557 | motor_current_a=-7.99805 |
motor_torque_nm=-4.99878 | motor_enabled=true

bringup_jog: Ok | controller_state=Connected |
stroke_mm=-0.168723 | motor_pos_rad=-0.53006 |
motor_vel_rad_s=-0.00732601 | motor_current_a=-0.0039072 |
motor_torque_nm=-0.002442 | motor_enabled=false
```

判断：

- `bringup_jog` 后自动失能是当前 Bring-up 安全策略，属于预期行为。
  点动命令会短时使能、发送低能量位置脉冲、读取反馈，然后立即失能。
- `bringup_enable` 后的 `motor_pos_rad=-12.4676`、
  `motor_vel_rad_s=-27.6557`、`motor_current_a=-7.99805`
  不像真实静止反馈，更像协议解析错误。
- 结合前面的 CAN 探测日志，`CTRL_MODE` 或 `MASTER_ID` 等寄存器响应帧也使用
  `host_id=0x11` 返回。如果上层把寄存器响应当成电机状态反馈解析，就可能得到
  接近负限位、高速、大电流的假反馈。
- 点动目标以最新反馈位置为基准。如果基准位置被假反馈污染，正向/反向点动目标
  都可能落在真实当前位置的同一侧，现场就会表现为正反方向一致。

处理：

- `DamiaoProtocol::parseFeedback()` 已增加寄存器响应识别：
  - `data[2] == 0x33` 或 `0x55`；
  - 电机 ID 字节匹配；
  - 已知寄存器号如 `MASTER_ID=0x08`、`CTRL_MODE=0x0A`；
  - 满足条件时拒绝作为电机状态帧解析。
- `DmUsb2FdcanTransport` 增加 `clearRxQueue()`，`DamiaoMotor::refreshFeedback()`
  发送刷新帧前先清空旧接收队列，避免上一次寄存器响应污染本次反馈。
- `DamiaoMotor::enable()` 现在会在使能后立即用当前位置发送一次零速度、零电流的
  位置保持命令，避免位置-力控模式沿用电机内部旧目标。
- UI 点动日志增加 signed `rel_rad`，用于确认正向/反向按钮实际下发的相对位置符号。
- Web UI 允许在进入 Bring-up 后直接点动；点动内部会自动使能并自动失能，不再要求
  先手动点击 `Bring-up 使能`。

后续复测建议：

1. 结构仍保持未安装或机械安全状态。
2. `connect -> bringup_enter_confirm_unloaded -> bringup_feedback`。
3. 确认静止反馈速度、电流、力矩接近 0。
4. 优先直接点击“电机正向点动”和“电机反向点动”，不要先手动点击
   `Bring-up 使能`。
5. 对比日志中的 `rel_rad=+0.05` 和 `rel_rad=-0.05`，以及点动前后的
   `motor_pos_rad` 变化方向。
6. 若方向仍相同，记录两次点动的完整 TX/RX 或运行日志，再继续查位置-力控模式
   命令帧和电机固件模式。

## 问题 6：Bring-up 点动返回 Ok 但电机无可见位移

现象：

```text
bringup_jog rel_rad=0.05 max_vel_rad_s=0.5 max_current_a=0.2 pulse_s=0.3: Ok
motor_pos_rad=-0.53006
motor_current_a=0.425885
motor_torque_nm=0.266178
motor_enabled=false
```

正向和反向点动均显示命令完成，但反馈位置基本不变，电机无可见运动。

判断：

- 通信和反馈已经恢复，`bringup_jog` 能读到真实反馈。
- 点动后自动失能仍是预期行为。
- 当前 0.05 rad 相对位置约为 2.9 度，电流限制 0.2 A 在达妙 `POS_FORCE`
  命令里会换算成很小的 per-unit 限矩。结合反馈电流/力矩有上升但位置无变化，
  更像点动幅度/限矩不足以克服电机、减速器或机构静摩擦。
- 也不排除 `POS_FORCE` 电流限制标定仍需现场校准。

处理：

- Web UI 增加 `相对位置 rad` 输入，不再固定写死 `0.05`。
- Bring-up 默认值调整为：
  - `default_relative_motor_position_rad: 0.2`
  - `max_relative_motor_position_rad: 1.0`
  - `default_motor_current_a: 0.5`
  - `max_motor_current_a: 1.0`
- Bring-up 最大电流仍受全局 `safety.max_motor_current_a` 约束。
- 控制器在点动后会比较起始位置、目标位置和结束位置。如果目标已发出但实际位移
  低于最小可测阈值，会返回 `ControlSaturation`，并在日志中打印：
  `start_rad`、`target_rad`、`end_rad`、`delta_rad`、`rel_rad`、
  `max_vel_rad_s`、`max_current_a`。

后续复测建议：

1. 保持结构未安装或机械安全。
2. 使用 `rel_rad=0.2`、`current=0.5A`、`velocity=0.5rad/s`、
   `pulse=0.3s` 测试一次正向点动。
3. 如果返回 `ControlSaturation`，逐步提高到 `rel_rad=0.5`、
   `current=0.8A`；不要超过当前 UI/配置上限。
4. 若 1.0A 仍无位移但反馈力矩明显上升，停止测试，优先检查电机是否处于正确控制
   模式、抱闸/机械阻滞、以及达妙 `POS_FORCE` 限矩字段是否应按 per-unit 直接传入。

## 问题 7：正反点动最终方向正确，但连续同向点动幅度逐渐减小

现象：

- 正向点动初期有位移，连续点动后单次位移逐渐减小。
- 反向点动能向反方向运动，但连续反向后同样出现单次位移变小。
- 日志示例：

```text
rel_rad=0.2 start_rad=0.0215534 target_rad=0.221553 end_rad=0.035668 delta_rad=0.0141146
rel_rad=0.2 start_rad=0.0364309 target_rad=0.236431 end_rad=0.0459678 delta_rad=0.00953689
rel_rad=-0.2 start_rad=-0.0246052 target_rad=-0.224605 end_rad=-0.0379568 delta_rad=-0.0133516
```

判断：

- 这组测试仍处于 `controller_state=Connected` 的 MotorBringup 路径，没有进入
  `PreSelfCheck`、`Homing`、`TravelLearning` 或正常夹紧流程。
- Bring-up 点动使用电机侧 `last_feedback.position + rel_rad` 生成绝对位置目标，
  不调用 `sendLimitedCommand()`，也不使用机构行程软件限位。因此这不是安装结构后的
  正常控制限位逻辑在约束。
- 达妙 `POS_FORCE` 是绝对位置目标 + 速度限制 + 限矩。当前每次点动只给 0.3s，
  结束后立即失能；低限矩、静摩擦、减速器/丝杆阻力和反复使能/失能会导致实际位置
  只跟随目标的一小段。
- 之前诊断阈值按 `min(目标位移 20%, 0.02rad)` 判定，过于严格，导致
  `delta_rad=0.009~0.014` 被标成 `ControlSaturation`。已调整为
  `min(目标位移 5%, 0.005rad)`，避免把小但真实的运动误判为完全不动。

后续复测建议：

1. 继续保持结构未安装或机械安全。
2. 使用 `rel_rad=0.2~0.5`、`current=0.5~0.8A` 做正反方向确认。
3. 如果要评估“每次点动能否到达目标”，需要延长 `pulse_s` 或提高限矩；当前
   Bring-up 逻辑的目标是低风险确认方向，不是精确位置跟踪。
4. 已按该思路将 Bring-up 点动改为 `Velocity` 模式短时点动，不再使用
   `POS_FORCE` 绝对位置目标。UI 中 `方向窗口 rad` 只用于选择正负方向和做配置
   校验，实际运动距离由 `vel_rad_s * pulse_s` 决定。
5. `反馈电流中止 A` 是运行过程中的反馈监控阈值。达妙 `Velocity` 命令帧本身不带
   电流限值，因此控制器在点动过程中周期读取反馈，超过阈值会主动停止并失能。

## 问题 8：Velocity 点动输入 5 rad/s、2 s 后，实机速度和持续时间没有明显提升

现象：

- Bring-up Velocity 点动已经能区分正反方向。
- UI 中把点动速度设为 `5 rad/s`、持续时间设为 `2 s` 后，实机表现仍接近低速短脉冲。

判断：

- 这不是电机失能或控制链路问题，而是 Bring-up 配置上限把 UI 输入静默裁剪了。
- 修改前默认配置为：
  - `motor_bringup.max_motor_velocity_rad_s: 1.0`
  - `motor_bringup.max_pulse_duration_s: 0.3`
- 因此 UI 输入 `5 rad/s`、`2 s` 实际生效只有 `1 rad/s`、`0.3 s`。

处理：

- 将空载 Bring-up 默认上限调整为：
  - `motor_bringup.max_motor_velocity_rad_s: 5.0`
  - `motor_bringup.max_pulse_duration_s: 2.0`
- 控制器 `jogMotorBringup()` 成功返回时增加诊断信息：
  - `requested_vel_rad_s`
  - `effective_vel_rad_s`
  - `requested_pulse_s`
  - `effective_pulse_s`
  - `measured_delta_rad`
- 后续 UI 日志中可以直接判断输入参数是否被配置上限裁剪。

验证：

- `scripts/build.ps1` 构建通过。
- `test_motor_bringup.exe` 通过，并新增覆盖 `5 rad/s`、`2 s` 空载 Velocity 点动配置。
- `test_state_machine.exe` 通过。
- `test_damiao_protocol.exe` 通过。

## 问题 9：PreSelfCheck 中先发生小位移，后续升流反而不动，最终失败

现象：

- 日志文件：`log/gripper-log-2026-05-20T10-56-56.040Z.txt`。
- 现场能看到预自检过程中正向转动一下、反向转动一下。
- 日志中也能看到小位移，例如闭合方向最大约 `0.182 mm`，打开方向最大约
  `0.167 mm`。
- 但最终返回：

```text
PreSelfCheck | phase=LimitedProbe | failed | pre-self-check could not start stable motion in either direction
```

原因判断：

- 旧实现把 `BidirectionalMoveEnable` 写成先连续扫描闭合方向，再连续扫描打开方向。
  这会把机构逐步推向一侧，后续即使升高电流，也可能因为当前位置、静摩擦、接近限位
  或控制目标变化而表现为位移越来越小。
- 旧判定把“起动”和“稳定完成规定小行程”混在一起。即使已经发生小位移，只要没有跑完
  目标窗口，就会以 `OperationTimedOut` 跳过，不能作为起动参数记录。
- 旧配置从 `0.05 A` 开始试探，样机上过低，产生了大量无效探测。

处理：

- `BidirectionalMoveEnable` 改为同一速度/电流参数点下执行闭合一步、打开一步的成对
  试探，避免连续推向单侧。
- 速度扫描仍由 `self_check.min_speed_scan_start_mm_s`、
  `self_check.min_speed_scan_stop_mm_s`、`self_check.min_speed_scan_step_mm_s`
  配置控制。
- 电流扫描新增：
  - `self_check.static_friction_current_start_a`
  - `self_check.static_friction_current_stop_a`
  - `self_check.static_friction_current_step_a`
- 默认起始电流改为 `0.1 A`，仍受 `safety.self_check_current_limit_a` 截断。
- 新增 `self_check.motion_start_distance_mm`，用于识别“已经起动”的最小位移。
- 起动样本会写入运行期 `StructureProfile`，并保存轻量种子到
  `self_check.learned_profile_path`，作为下一次预自检的扫描起点参考。

剩余注意：

- 轻量种子只用于下一次自检的起点参考，不等同完整标定文件。
- `StableShortStrokeMotion` 仍会严格要求完成规定小行程、速度稳定、电流稳定；如果后续
  失败，应按稳定运动能力继续排查，而不是再归因于“完全没动”。

复测补充：

- 新日志显示 `BidirectionalMoveEnable` 已出现双向 `breakaway accepted`，说明闭合和
  打开方向均已起动成功。
- 后续失败发生在 `StableShortStrokeMotion`：

```text
target_nut_speed_mm_s=1 current_limit_a=0.4 measured_mm=0.161013
PreSelfCheck | phase=StableShortStrokeMotion | failed | pre-self-check probe did not reach target before timeout
```

进一步处理：

- 修正扫描起点来源：保守默认 `StructureProfile` 中的摩擦电流不能作为下一次自检的学习
  起点，只有真实起动样本或持久化种子才能抬高扫描起始电流。否则会出现配置为 `0.1 A`，
  日志却从 `0.35 A` 起扫的问题。
- `runSelfCheckMotionProbe()` 在等待目标到达期间周期性重发受限位置力控命令，避免只发送
  一帧后表现成短脉冲。
- `PreSelfCheck` 失败路径增加失能动作，防止失败后日志中仍显示 `motor_enabled=true`。

第三轮复测：

- 失败后 `motor_enabled=false`，失能路径已经生效。
- 双向起动仍已通过，但 `StableShortStrokeMotion` 在 `1 mm/s`、`0.5 mm` 下闭合方向
  仅运动约 `0.10 mm`，且最大电流约 `0.535 A`，超过 `0.4 A` 指令上限后的稳定性判定
  失败。

处理：

- 稳定短行程目标新增 `self_check.stable_short_stroke_distance_mm`，默认先使用 `0.2 mm`。
- 稳定短行程也按低速起步、闭合/打开成对扫描，不再直接跳到 `travel_learning_speed`。
- `monotonic` 日志字段修正为只表达单调性，位移是否足够由稳定短行程阶段单独判定。
- 稳定短行程失败摘要会输出闭合/打开两侧的目标距离、实际距离、平均速度、最大电流和电流稳定性。

第四轮复测：

- 现象：电机在稳定短行程阶段持续来回小幅运动，最终失败。
- 日志显示目标 `0.2 mm`，但每次实际位移只有 `0.006~0.009 mm`，仍被单次 probe 标记
  为 `code=Ok`。
- 原因：自检 probe 的目标到达判断复用了全局 `max_distance_error=0.2 mm`，与短行程目标
  相同，导致几微米位移也满足“到达目标”条件。
- 另一个原因：电流循环终止条件加入了 `max_current_ripple`，当 `current_start` 与
  `current_stop` 都是 `0.4 A` 时，会重复执行多个相同上限参数，看起来像一直来回小幅运动。

处理：

- 自检 probe 使用与目标距离相关的独立到达容差。
- 稳定短行程默认距离改为 `0.3 mm`。
- 稳定短行程样本判定使用目标距离相关的误差阈值。
- 电流扫描不再用电流纹波扩展循环终止条件，避免重复上限参数。

第五轮复测：

- 日志文件：`log/gripper-log-2026-05-20T11-45-06.342Z.txt`。
- `BidirectionalMoveEnable` 已经双向起动成功。
- `StableShortStrokeMotion` 在 `0.4 A` 自检限流下仍只能得到约 `0.06~0.10 mm` 的短距离样本，达不到 `0.3 mm` 目标。
- 日志中未出现限位或堵转特征，方向一致、单调性正常，说明该问题更接近当前限流和低速条件下的短距跟随能力不足，而不是完全无法运动。

处理：

- 稳定短行程失败不再直接阻断第一阶段 `PreSelfCheck`。
- 闭合/打开方向如果都得到方向正确、位移单调、无堵转、无疑似限位且超过起动阈值的样本，则记录为低置信样本。
- `PreliminaryLimitSearch` 在第一阶段硬件联调中使用理论开闭边界建立低置信安全区，真实机械限位留给后续 `HomingOpenStop` 和 `TravelLearning` 验证。
- `MultiRegionRoundTripLearning` 允许降级，不再阻断保守 profile 生成。

第六轮复测：

- 真实硬件命令序列：`connect -> enable -> selfcheck -> log -> quit`。
- 结果：

```text
PreSelfCheck | phase=StableShortStrokeMotion | low_confidence_samples_accepted ...
PreSelfCheck | phase=StableShortStrokeMotion | degraded | stable short-stroke failed ...
PreSelfCheck | phase=PreliminaryLimitSearch | limits accepted ... note=theoretical_limits_used_for_low_confidence_precheck
PreSelfCheck | phase=MultiRegionRoundTripLearning | degraded | pre-self-check probe did not reach target before timeout
pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile | controller_state=Disabled | ... | motor_enabled=false
```

当前结论：

- `PreSelfCheck` 第一阶段真实硬件流程已可完成。
- 本次完成的是低置信、保守 profile，不代表完整机械限位和运动健康标定完成。
- 自检成功后电机已自动失能，避免持续输出力矩。
- 后续应继续验证 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck`，并根据真实运动能力再决定是否调整 `self_check.stable_short_stroke_distance_mm`、自检限流或短行程通过标准。

## 问题 10：PreSelfCheck 中实物电机朝同一方向累积旋转

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T00-33-42.874Z.txt`

现象：

- 实物观察到电机朝一个方向持续累积旋转。
- 日志中 `opening` 试探下发负向速度，但估算行程仍增大，例如：

```text
direction=opening target_nut_speed_mm_s=-0.2 ...
start_mm=8.15482 end_mm=8.29689 measured_mm=0.14207
```

初步原因：

- 预自检阶段依赖电机侧反馈进行低置信行程估算，尚未完成回零、行程学习和运动健康检查，不能把该估算当作最终机构行程。
- 原 `direction_ok` 判定允许相反方向的小位移被全局 `max_distance_error=0.2 mm` 掩盖，导致 `opening` 实际朝闭合方向偏移约 `0.14 mm` 时仍可能被记录为方向可接受。
- 起动确认后仍可能继续重复试探已确认方向，会把机构继续推向同一侧。

处理：

- 设计文件 `references/03_control/control_architecture_design.md` 升级到 v1.4，明确 `PreSelfCheck` 的初步限位和安全区是低置信临时结果，只用于让自检和后续流程继续，不等价最终软件限位。
- 达妙反馈中的原始编码器位置 `q_uint` 已贯通到 `MotorFeedback`、控制器快照、Web `/api/view` 和 UI 操作日志。
- `PreSelfCheck` 详细日志新增 `start_motor_pos_rad`、`end_motor_pos_rad`、`motor_delta_rad`、`start_motor_raw_pos_counts`、`end_motor_raw_pos_counts`。
- 如果反馈位移朝试探相反方向超过收敛后的方向阈值，立即返回 `SelfCheckInconsistentFeedback`，不再继续升电流或进入后续学习。
- `BidirectionalMoveEnable` 中某方向已确认起动后，不再重复试探该方向，降低单侧累积位移风险。

复测关注：

- 对比 `motor_delta_rad` 与 `start/end_motor_raw_pos_counts`，确认达妙原始编码器和换算后的 rad 方向一致。
- 若 `opening` 仍导致估算行程增大，日志应出现 `SelfCheckInconsistentFeedback`，此时应检查 `motor.direction_sign`、丝杆安装方向和夹爪开闭方向定义。

## 问题 11：PreSelfCheck 换向后使用运动中反馈导致方向误判

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T02-09-38.489Z.txt`

现象：

- `0.15 A` 闭合试探已经产生约 `0.143 mm` 正向位移。
- 紧接着打开试探下发负向速度，但日志记录的最终反馈仍为正向位移约 `0.056 mm`。
- 控制器据此返回 `SelfCheckInconsistentFeedback`，`PreSelfCheck` 失败并进入 `ActiveStop`。

分析：

- 这次现象与“方向配置错误导致持续向同一方向累积旋转”不同，更像闭合命令结束后电机/机构尚未停稳，打开试探读取到了前一段动作的残余运动或过渡反馈。
- 运动类 controller 接口不能只表示“命令已发送”。对 UI/CLI 等上层调用方，接口返回时必须已经完成动作、停稳或明确失败。
- 自检、调试定位和后续业务动作都不能用运动过程中的最后一帧编码器直接作为终点位置。

处理：

- 控制设计文档升级到 `v1.7`，明确运动类调用的同步语义，以及 `PreSelfCheck` 每段小试探必须停止输出、等待停稳并重新读取最终反馈。
- 新增配置：
  - `self_check.motion_settle_timeout_s`
  - `self_check.motion_settle_stable_time_s`
- 控制器新增运动后停稳采样路径：先停止/失能，再等待螺母速度进入静止阈值并保持一段时间，最后用停稳后的反馈计算终点、位移和方向。
- `PreSelfCheck` 换向前会经过停稳采样；方向一致性基于停稳后的最终位移判定。
- 通用运动等待和手动螺母行程拖动也复用该语义：返回 `Ok` 表示动作完成且完成必要收尾，普通超时返回 `OperationTimedOut`，安全风险才进入 `ActiveStop`。
- 为避免多段流程被上一段的失能打断，底层受限命令发送前会确认电机输出已使能。

验证：

- 已执行 `.\scripts\test.ps1`，7/7 项通过。
- 需要真实硬件复测 `PreSelfCheck`，重点观察每个 `probe result` 的 `start/end_motor_pos_rad`、原始编码器计数和 `direction_ok` 是否与实物运动一致。

## 问题 12：PreSelfCheck 单向起动后持续扫描 opening，最终失败

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T02-41-31.358Z.txt`

现象：

- `closing` 方向在 `0.2 A`、`0.2 mm/s` 下已出现有效位移，日志记录：

```text
PreSelfCheck | breakaway accepted direction=closing ... measured_mm=0.156399 ...
```

- 随后 `opening` 方向从低速低流一直扫描到配置上限，绝大多数样本 `motor_delta_rad=0`、`measured_mm=0`。
- 流程最终返回：

```text
SelfCheckUnsafeMotion | pre-self-check requires paired breakaway motion in both directions
```

分析：

- 当前硬件位置可能靠近 opening 侧边界，或者 opening 方向在低能量窗口内暂时不可动。
- `PreSelfCheck` 的第一目标是初步发现限位并建立低置信临时安全区，不应在一个方向已经确认低能量起动后，把另一个不可动方向扫完整个速度/电流矩阵。
- 只要不可动方向没有出现相反方向位移、堵转、接触或电流危险，应判为低置信疑似边界/受阻方向并继续保守流程。

处理：

- 控制设计文档升级到 `v1.8`，补充单向可动降级策略。
- `BidirectionalMoveEnable` 在一个方向起动确认后，只允许对缺失方向做有限同级窗口验证；若缺失方向无有效位移且无危险，记录：

```text
PreSelfCheck | phase=BidirectionalMoveEnable | degraded ... reason=no_effective_motion_under_limited_scan
```

- 降级后跳过 `StableShortStrokeMotion` 的双向短行程强验证，由后续初步限位和保守 profile 建立继续推进。
- 方向反常、ActiveStop、堵转、接触、硬件故障和通信故障仍立即失败，不参与降级。
- Web UI `/api/view` 增加 `state.self_check_running`；预自检后台执行期间按钮显示“预自检执行中...”并禁用，顶部流程显示“自检中”。

复测关注：

- 预自检日志中应出现 `degraded` 记录，而不是长时间重复 `opening` 扫描。
- 最终应能进入 `PreliminaryLimitSearch` 并建立低置信临时安全区。
- 如果实物确实在 opening 侧限位附近，后续 `HomingOpenStop` 和 `TravelLearning` 必须重新确认真实零点和软件限位。

## 问题 13：电机多圈旋转但螺母行程估算只变化很小

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T03-28-05.854Z.txt`

现象：

- 用户观察到实物电机已经运动好几圈。
- UI/日志中的 `stroke_mm` 只改变很小一段。
- 典型日志：

```text
move_nut_stroke target_mm=14.9 speed_mm_s=1
start_mm=6.40031 end_mm=8.67052 measured_delta_mm=2.27021
motor_pos_rad=-5.54952 -> 1.58255
raw_counts=18220 -> 36916
```

分析：

- 达妙反馈帧中的位置由 16 位 `q_uint` 按 `[-p_max, p_max]` 解析得到。
- 当前 `p_max=12.5 rad`，单帧位置是有限范围值，不是可直接累计的多圈位置。
- 旧实现把 `DamiaoFeedback.position` 直接写入 `MotorFeedback.position`，控制器再用该值换算螺母行程。
- 当电机跨过反馈量程边界时，单帧位置会回绕，整圈数丢失，因此会出现“电机实际多圈运动，但估算行程只变化一点”的问题。

处理：

- 新增 `src/hardware_interface/virtual_encoder.hpp/.cpp`，提供 `MultiTurnVirtualEncoder`。
- 达妙硬件层在解析反馈后先执行单帧位置到连续多圈位置的映射：
  - `MotorFeedback.position`：连续多圈电机位置，供控制、自检、行程估算、滑块拖动和业务控制使用。
  - `MotorFeedback.wrapped_position`：达妙单帧有限范围位置，仅用于诊断。
  - `MotorFeedback.raw_position_counts`：达妙原始 `q_uint`，仅用于诊断。
- 回绕判断使用反馈量程宽度 `2 * motor.max_position_rad`，相邻反馈差值超过半量程时补偿一个量程宽度。
- UI/API/运行日志增加 `motor_wrapped_pos_rad`，便于同时对比连续位置、单帧位置和原始计数。
- 控制设计文档升级到 v1.9，明确后续应形成固定频率 `MotorFeedbackService/MotorEncoderService`，后台持续更新虚拟编码器。
- 2026-05-21 继续处理：控制设计文档升级到 v1.10，固定频率反馈更新服务从“后续待办”改为“当前硬件基础设施”。
- 达妙硬件层连接成功后启动后台反馈线程，默认 `motor.feedback_poll_period_s = 0.02 s`，即 50 Hz；`readFeedback()` 只返回最新快照，不再临时清空接收队列和抢帧。
- 达妙 DeviceSDK 只发现 CAN/CAN-FD 发送接口和接收回调，未发现独立读取编码器的 API。因此虚拟编码器唯一数据源定义为 `DamiaoProtocol::parseFeedback()` 校验通过的电机状态反馈帧；寄存器读写回包和其他 CAN 帧不会进入虚拟编码器。
- 新增配置项：
  - `motor.continuous_encoder_enabled`
  - `motor.feedback_poll_period_s`
  - `motor.feedback_stale_timeout_s`
- 2026-05-21 再次修正：不能把“达妙协议位置量程”和“物理输出端单圈回绕”混为一个概念。新增配置：
  - `motor.encoder_unwrap_source`
  - `motor.encoder_wrap_range_rad`
  - `motor.encoder_raw_count_range`
- 当前样机默认使用原始 `q_uint` 计数做回绕增量，按 `2*pi rad` 累计连续多圈位置；首帧连续位置基准仍使用达妙协议解析位置，避免位置命令坐标突然跳变。
- Web UI 夹爪状态区改为只显示 `motor_virtual_pos_rad`（虚拟多圈电机位置）。`motor_wrapped_pos_rad` 和 `motor_raw_pos_counts` 继续保留在 API/硬件诊断中，但不再拼接到主夹爪状态显示里。

剩余风险：

- 多圈虚拟编码器要求相邻两次反馈之间的真实电机位移小于半个回绕范围。当前原始计数方案下回绕范围为 `2*pi rad`，50 Hz 配置和 30 rad/s 电机速度上限下，理论相邻位移约 0.6 rad，小于半圈 `pi rad`。
- 如果未来允许更高速度、调低反馈频率或反馈线程长时间读不到有效状态帧，必须提高反馈采样频率或降低运动速度，否则多圈映射会存在歧义。
- 如果后续实测证明 `q_uint` 并不是物理单圈编码器计数，而是达妙协议量程位置编码，应把 `motor.encoder_unwrap_source` 切回 `protocol_position`，并将 `encoder_wrap_range_rad` 设置为 `2 * motor.max_position_rad` 后复测。

复测关注：

- 运动多圈时，`motor_virtual_pos_rad` 应连续增加或减少，不应在单圈边界附近跳回。
- `motor_wrapped_pos_rad` 可以回绕，`motor_raw_pos_counts` 可以跨过边界，但 `stroke_mm` 应按连续电机位置累计。
- 以丝杆导程 2 mm/rev 校核：电机输出端每转一圈，螺母行程应约变化 2 mm。

## 问题 14：PreSelfCheck 未停稳样本被接受导致换向误判

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T04-30-57.100Z.txt`

现象：

- `opening` 试探在 `0.15 A` 时已经产生约 `0.14 mm` 位移，但日志 message 为：

```text
motion did not settle before final feedback sampling timeout_s=1 speed_mm_s=-0.30082
```

- 随后控制器仍输出：

```text
PreSelfCheck | breakaway accepted direction=opening ...
```

- 后续 `closing` 试探读到的连续多圈电机位置仍在 opening 方向变化，最终返回：

```text
SelfCheckInconsistentFeedback | pre-self-check settled feedback moved opposite to requested direction
```

分析：

- `motor_pos_rad`、`motor_raw_pos_counts` 和 `stroke_mm` 的变化方向一致，当前现象不是多圈虚拟编码器丢圈。
- 根因是上一段 opening 试探未停稳，但 `runSelfCheckMotionProbe()` 仍生成了可被 `acceptBreakawayCandidate()` 接受的起动样本。
- 换向后的 closing 试探读到了上一段未衰减完的 opening 残余运动，因而被误判为“新命令方向相反”。

处理：

- 控制设计文档升级到 v1.11，明确未停稳样本无效，不能进入起动、低置信边界、摩擦或行程统计。
- `PreSelfCheck` 实施记录升级到 v1.9，明确日志和样本筛选要求。
- 代码层面将 `settled` 纳入 probe 结果和日志；未停稳时 probe 返回停稳错误，且样本不再被起动接受、单向降级或后续统计使用。

复测关注：

- 每条 `probe result` 应输出 `settled=true/false`、`settled_speed_mm_s` 和 `settle_code`。
- 若仍出现未停稳，应看到该 probe 不会紧接着输出 `breakaway accepted`。
- 如果出现真正的方向配置错误，应是已经停稳的样本产生明确相反方向位移后才返回 `SelfCheckInconsistentFeedback`。

## 问题 15：PreSelfCheck 动一下后不再继续，停稳判断依赖了伪造零速反馈

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T05-14-27.582Z.txt`

现象：

- 低电流阶段多次 `settled=true`，但 `measured_mm` 为零或很小。
- `0.3 A` 的 `closing` probe 出现明显位移，但方向为 opening，且最终：

```text
settled=false ... settle_code=OperationTimedOut ... speed_mm_s=-0.296156
```

- 用户实物观察为“动了一下，后面没动了”。

分析：

- 新日志证明上一轮“未停稳样本被接受”的问题已经被拦住，未再输出 `breakaway accepted`。
- 但达妙 `disable()` 曾在本地缓存中直接把速度、电流、力矩清零，可能让控制器在部分阶段误以为停机后已经静止。
- 停稳判断必须使用停止/失能命令之后的新硬件反馈帧，不能用硬件层写入的本地零速值。
- 日志还需要输出 `target_motor_pos_rad`，便于核对 PositionForce 目标位置方向。如果目标方向正确而实物反向运动，再检查 `motor.direction_sign` 或达妙 PositionForce 模式的目标解释。

处理：

- 控制设计文档升级到 v1.12，明确停稳最终反馈必须来自停机后的新硬件帧。
- PreSelfCheck 实施记录升级到 v1.10，明确达妙 `disable()` 不得伪造零速反馈。
- 代码层面将修正 `disable()` 缓存写入，并让停稳等待跳过停机命令前的旧反馈帧。

复测关注：

- `probe attempt` 应新增 `target_motor_pos_rad`。
- 停机后如果电机仍在动，应稳定输出 `settled=false`，而不是因为本地零速缓存显示 `settled=true`。
- 如果目标电机位置和实际 `motor_delta_rad` 方向相反，再调整 `motor.direction_sign` 或检查 PositionForce 命令模式。

## 问题 16：PreSelfCheck 第二次 closing 目标为正向但实际反向漂移

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T05-22-00.087Z.txt`

现象：

- 第二次 `closing` 的目标和速度均为正向：

```text
target_motor_pos_rad=2.05889
start_motor_pos_rad=0.488098
motor_velocity_rad_s=0.628319
```

- 但实际反馈为反向：

```text
end_motor_pos_rad=0.103571
motor_delta_rad=-0.384527
settled=false
```

分析：

- 这说明控制器计算出的目标方向本身是正向的，不能简单归因为 `target_motor_pos_rad` 未打印或行程换算错误。
- 反向漂移发生在上一条 `opening` 目标之后，且自检 probe 间收尾使用直接失能。失能后电机不再主动保持，机构可能继续朝上一目标或被机械负载反推。
- 在无外部位置传感器的机构上，`PreSelfCheck` probe 间不能靠失能作为停止方式；失能只适合完整动作结束后的防抱死收尾。

处理：

- 控制设计文档升级到 v1.13，明确 probe 间使用低电流主动保持停稳。
- PreSelfCheck 实施记录升级到 v1.11。
- 代码层面将 `runSelfCheckMotionProbe()` 的收尾切换为“当前位置低电流保持 + 新硬件反馈停稳”，自检结束/失败时再失能。

复测关注：

- 同类场景下，第二次 `closing` 若仍出现 `motor_delta_rad < 0`，需要继续检查达妙 PositionForce 模式在低电流下是否被机械负载拖向上一目标。
- 如果主动保持后仍不能停稳，应适当提高保持电流上限或延长 `motion_settle_timeout_s`，但不得超出自检电流安全包络。

## 问题 17：PreSelfCheck 停稳后位置力控目标与反馈方向相反

日期：2026-05-21。

测试日志：

- 硬件复测通过 Web API 执行 `connect -> enable -> selfcheck`。

现象：

- 主动保持后 probe 已能出现 `settled=true`，说明上一轮残余运动问题已经被隔离。
- 但在 `closing` probe 中，目标电机差值为正，实际电机位置和 signed 电流/力矩为负；在 `opening` probe 中，目标电机差值为负，实际位置和 signed 电流/力矩为正。

判断：

- 该现象不再是未停稳样本导致的换向误判。
- `motor.direction_sign` 用于电机反馈位置到螺母行程的机构换算，不宜用它修正达妙位置命令协议符号。
- 当前样机的达妙 Position/PositionForce 位置目标相对反馈位置表现为符号相反，因此需要在硬件层增加独立的位置命令符号修正。

处理：

- 新增配置 `motor.position_command_sign`，初始按当时测试现象设置为 `-1`。
- 达妙硬件实现只在生成 Position/PositionForce 位置目标帧时应用该符号；反馈解析、连续多圈虚拟编码器和 controller 内部行程估算保持原符号。
- 控制设计文档升级到 v1.15。

复测关注：

- `probe result` 中 `target_motor_delta_rad` 和 `motor_delta_rad` 应同向，或至少不再系统性反向。
- 如果仍反向，优先检查 `position_command_sign` 配置是否生效，再检查达妙固件模式与控制帧 ID。

## 问题 18：最新构建日志显示 `position_command_sign=-1` 反而导致实际反向

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T05-22-00.087Z.txt`

现象：

- `closing` 试探的控制器目标电机位置方向为正：

```text
start_motor_pos_rad=0.488098
target_motor_pos_rad=2.05889
```

- 实际反馈位置方向为负，并且停稳失败：

```text
end_motor_pos_rad=0.103571
motor_delta_rad=-0.384527
settled=false
```

分析：

- 该日志中的目标方向和反馈方向相反，说明当前硬件、固件和 PositionForce 模式组合下，默认 `position_command_sign=-1` 不再适配。
- `motor.direction_sign=1` 下，控制器内部行程估算公式是自洽的；本问题应优先通过 `position_command_sign` 修正，而不是更改机构方向定义。
- 该符号属于硬件命令协议适配参数，必须以低能量位置类命令日志中的 `target_motor_delta_rad` 和 `motor_delta_rad` 同向为判据。

处理：

- 将 `src/config/default_gripper.yaml` 中 `motor.position_command_sign` 修正为 `1`。
- 将配置结构和达妙硬件配置默认值同步修正为 `1`。
- 控制设计文档升级到 v1.18，明确该值是当前样机实测默认，不作为所有达妙设备的永久假设。

复测关注：

- `probe result` 应打印 `target_motor_delta_rad`、`motor_delta_rad`、`last_current_a` 和 `last_torque_nm`，确认使用的是当前构建。
- 起动后 `target_motor_delta_rad` 与 `motor_delta_rad` 应同向。
- 若仍只动一次就失败，优先检查主动保持停稳日志，而不是再次修改 `direction_sign`。

## 问题 19：自检停稳阈值过宽导致残余运动被误判为静止

日期：2026-05-21。

测试方式：

- 使用当前构建通过 Web API 执行 `connect -> enable -> selfcheck`。

现象：

- `opening` 低电流 probe 的 `settled_speed_mm_s=-0.0909457`，但日志显示 `settled=true`。
- 随后 `closing` probe 的目标方向为正，反馈仍继续朝负方向变化，并最终停稳失败。

分析：

- 代码此前将停稳速度阈值取为 `max(safety.jam_speed_threshold_mm_s, noise/dt)`。
- 当前 `safety.jam_speed_threshold_mm_s=0.1` 是堵转/接触检测阈值，不适合作为换向前的静止阈值。
- 因此 `0.09 mm/s` 的残余运动被误判为停稳，下一段 probe 实际叠加了上一段残余运动，造成“目标正向、反馈负向”的假象。

处理：

- 新增配置 `self_check.motion_settle_speed_threshold_mm_s`，默认 `0.03 mm/s`。
- `PreSelfCheck` 的直接停机等待和主动保持等待均使用该阈值。
- `self_check.motion_settle_timeout_s` 默认从 `1.0 s` 增加到 `2.0 s`。
- 超时日志新增 `still_speed_threshold_mm_s`，用于判断到底是残余速度未降下来，还是命令方向/模式仍异常。

复测关注：

- 下一轮日志中 `settled=true` 时，`settled_speed_mm_s` 应小于 `0.03 mm/s`。
- 如果严格停稳后 `closing` 的 `target_motor_delta_rad` 与 `motor_delta_rad` 仍反向，再继续检查 `position_command_sign`、PositionForce 模式和实际发送帧。

## 问题 20：失能后反馈速度字段非零但位置基本不变

日期：2026-05-21。

现象：

- 主动保持 fallback 到失能停稳后，多圈位置变化已经很小，但达妙反馈速度字段仍显示约 `0.93 rad/s`，折算约 `0.296 mm/s`。
- 单纯依赖反馈速度会导致停稳等待超时。

分析：

- 反馈速度字段可能存在低速量化、滤波滞后或失能后的短时间非零保持。
- 控制器已经拥有连续多圈位置，因此停稳判断应同时看相邻反馈位置差分速度。

处理：

- `MotionSettleResult` 新增 `settled_position_delta_speed`。
- 直接停机等待和主动保持等待均计算位置差分速度。
- probe 结果日志新增 `settled_position_delta_speed_mm_s`。

复测关注：

- 如果 `settled_speed_mm_s` 偏大但 `settled_position_delta_speed_mm_s` 连续低于阈值，自检应能继续。
- 如果两者都偏大，说明机构仍在真实运动，不能接受样本。

## 问题 21：反向低能量 probe 小幅漂回已确认方向

日期：2026-05-21。

现象：

- `direction_sign=-1` 后，`closing` 低电流 probe 能确认起动：

```text
direction=closing measured_mm=0.0540352 direction_ok=true
```

- 紧接着 `opening` 低电流 probe 的目标电机方向为正，但实际电机仍负向小幅运动，按机构映射表现为继续 closing：

```text
direction=opening target_motor_delta_rad=1.5708 motor_delta_rad=-0.168231 measured_mm=0.0535495
```

分析：

- 该现象发生在反向低电流第一次试探，电流仍在自检包络内，且位移只有约 `0.05 mm`。
- 对无夹爪传感器机构，这更适合解释为反向起动未确认、疑似边界/残余漂移，而不是立即判定硬件方向配置错误。
- 如果继续提高 opening 电流，可能在未知边界上施力，不符合低能量预自检目的。

处理：

- `BidirectionalMoveEnable` 增加 `reason=low_energy_opposite_drift` 降级路径。
- 一个方向已确认后，反向小幅低能量漂移且无安全风险时，停止继续反向升流，建立低置信 profile 继续后续流程。

复测关注：

- 日志应出现 `PreSelfCheck | phase=BidirectionalMoveEnable | degraded ... reason=low_energy_opposite_drift`。
- 后续应进入 `StableShortStrokeMotion`，若单向边界标记存在，该阶段应跳过强双向短行程验证。

## 问题 22：尚未确认方向时的低能量反向漂移导致 PreSelfCheck 失败

日期：2026-05-21。

现象：
- 用户日志 `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T05-22-00.087Z.txt` 显示自检只明显运动一次后失败。
- 使用当前构建和真实硬件复测时，`closing` 低能量 probe 的目标电机增量为负向，但反馈电机增量为正向小幅变化：

```text
direction=closing target_motor_delta_rad=-1.5708 motor_delta_rad=0.339895 measured_mm=0.108192
```

分析：
- 多圈虚拟编码器工作正常，`motor_pos_rad`、`motor_wrapped_pos_rad` 和原始计数方向一致，不是丢圈问题。
- PositionForce 帧格式与 `openarm_can_ref` 一致：位置 float、速度限幅、per-unit 电流限幅。
- 反向位移发生在低电流、低速度、小位移范围内，且未伴随堵转、限位或硬件故障。该阶段的目标是建立低置信安全区，不应继续升流，也不应把小幅反向漂移直接判成致命方向错误。

处理：
- `runSelfCheckMotionProbe()` 在运行中一旦检测到反向位移超过阈值，立即结束当前 probe 并进入停稳流程，避免继续向风险方向施力。
- `BidirectionalMoveEnable` 新增 `reason=early_low_energy_opposite_drift` 降级路径：尚未确认任一方向时，如出现低能量小幅反向漂移，则标记疑似边界并生成保守 profile，不继续扫完整个电流/速度矩阵。
- 达妙 `disable()` 增加基础 CAN ID 失能帧补发，兼容不同 SDK/固件对使能/失能帧 ID 的解释。
- controller 快照中的 `motor.enabled` 改为反映本地输出使能状态 `motor_->isEnabled()`，避免把达妙反馈状态位误当作当前输出使能确认。

验证：

```powershell
$env:ZIG_GLOBAL_CACHE_DIR='D:\jyt_ws_ai\TS2000_human_robot\gripper\build\dev-zig\zig-cache'
.\build\dev-zig\test_hardware_selfcheck.exe --config src\config\default_gripper.yaml
```

结果：

```text
PreSelfCheck | phase=BidirectionalMoveEnable | degraded ... reason=early_low_energy_opposite_drift
PreSelfCheck | phase=StructureProfileUpdate | profile updated
pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile
final_state ... motor_enabled=false
```

软件回归：

```powershell
.\scripts\test.ps1
```

结果：8/8 通过。

剩余风险：
- 当前通过结果是低置信保守 profile，不代表已经完成机构零位、真实软件限位和运动健康验证。
- 达妙反馈速度字段在低速/失能后仍可能显示非零，停稳判断仍需同时看位置差分速度。

## 问题 23：停止并失能后后台 PreSelfCheck 仍继续发 probe

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T07-36-54.510Z.txt`

现象：

- 用户观察到电机一直来回动，`PreSelfCheck` 无法完成。
- 点击“停止并失能”后，电机仍继续来回动作。
- 日志中多次出现：

```text
stop: SafetyActiveStop | active stop requested | controller_state=ActiveStop | ... motor_enabled=false
PreSelfCheck | probe attempt ...
```

- 还出现 `ControlNotReady | Damiao motor is not enabled` 后仍继续下一轮 `probe attempt` 的情况。

分析：

- Web UI 把 `runPreSelfCheck()` 放入后台线程执行，线程只用 `g_self_check_running` 防重复点击，没有取消句柄。
- `stop()` 只发送失能/停止并切换主状态为 `ActiveStop`，但没有通知正在运行的 `PreSelfCheck` 线程退出。
- `runPreSelfCheck()`、`runSelfCheckMotionProbe()` 和各类扫描循环没有统一检查用户停止/ActiveStop，因此后台线程在某次 probe 失败或失能后仍会进入下一次 probe。
- 这不是单纯虚拟编码器问题。虚拟编码器异常会影响行程估计，但“停止后继续发 probe”的直接根因是长流程取消机制缺失。

处理：

- 控制设计文档升级到 `v1.25`，新增“长流程取消与用户停止语义”。
- controller 新增控制器级取消标志：
  - `stop()`、`disable()`、`disconnect()` 设置取消。
  - `PreSelfCheck`、回零、行程学习、运动健康检查、夹紧、释放和手动定位等长流程启动前清除旧取消。
  - `ensureMotorOutputEnabled()`、`sendLimitedCommand()`、`sendNutMotionCommand()`、`runMotionUntil()`、`runSelfCheckMotionProbe()`、停稳等待和自检扫描循环均检查取消。
- `PreSelfCheck` 取消后返回 `SafetyActiveStop`，立即停止刷新命令并失能，不再为了继续自检重新使能。
- `clearFault()` 不再清除取消标志；避免后台自检尚未退出时，用户恢复 ActiveStop 把旧线程重新放行。
- Web UI 在 `self_check_running=true` 时禁用恢复、使能和其他业务动作，只保留停止入口；停止按钮不再被后台自检 busy 状态锁死。
- 新增软件回归测试 `stop_cancels_running_pre_self_check`：后台运行 `PreSelfCheck`，主线程调用 `stop()`，验证自检返回 `SafetyActiveStop` 且最终电机输出失能。

验证：

```powershell
.\scripts\test.ps1
```

结果：8/8 通过。

测试备注：

- 本轮测试中两次遇到 Windows 应用控制策略临时拦截新生成的测试 exe；删除对应构建产物并重新链接后恢复，现象与历史问题 2 类似，不是本次控制逻辑回归失败。

复测关注：

- 真实硬件复测前先确认没有旧 `gripper_app.exe` 后台进程。
- 点击预自检后再点“停止并失能”，日志应出现 `controller cancel requested`、`PreSelfCheck | cancelled` 或 `pre-self-check probe cancelled by user stop`。
- 停止后不应再出现新的 `PreSelfCheck | probe attempt`。
- 停止后如需恢复，必须等后台自检运行标志清除后再点击“恢复 ActiveStop”。

## 问题 24：PreSelfCheck 停止功能正常后仍在 BidirectionalMoveEnable 中持续来回扫参

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T08-24-21.432Z.txt`

现象：

- 用户确认“停止并失能”已经能够正常终止后台自检。
- 但不停止时，`PreSelfCheck` 仍一直停留在 `BidirectionalMoveEnable`，持续执行闭合/打开成对 probe，没有进入后续阶段。
- 日志中多次出现小幅位移，例如 `measured_mm=0.020~0.038`，且已 `settled=true`、无堵转、无疑似限位。
- 这些位移低于原 `motion_start_distance_mm=0.05`，因此没有触发 `breakaway accepted`。
- 用户停止时最后一个 probe 曾打印 `end_estimated_mm=0`、`end_motor_pos_rad=0`，实际是取消路径未填充最后反馈导致的日志误导。

分析：

- 设计上 `BidirectionalMoveEnable` 只应识别“能否起动”，不应追逐 `0.5 mm` 或完整短行程目标。
- 当前实现虽然已经有 `motion_start_distance_mm`，但真实低能量硬件只产生了高于噪声底、低于 `0.05 mm` 的可重复微动，导致扫描无法收敛。
- 不能简单降低 `motion_start_distance_mm`，否则会放松后续稳定短行程、行程学习和业务控制的正常起动判据。
- 部分 probe 的停稳后有符号位移朝请求反方向，说明低置信接受必须检查“有符号位移方向”，不能只看绝对 `measured_mm` 或旧 `direction_ok`。

处理：

- 控制设计文档升级到 `v1.26`，明确 `BidirectionalMoveEnable` 两级起动判据。
- 新增配置 `self_check.low_confidence_motion_distance_mm`，默认 `0.02 mm`，只用于 `PreSelfCheck` 第一阶段低置信微动起动识别。
- `runSelfCheckMotionProbe()` 在 `stop_after_motion_start=true` 的起动识别阶段使用低置信阈值作为运行中停止边界，达到后立即进入主动保持停稳。
- `acceptBreakawayCandidate()` 增加停稳后有符号位移检查：只有位移朝请求方向并超过低置信阈值，且电流在 `safety.self_check_current_limit_a + max_current_ripple_a` 内，才接受为低置信微动起动样本。
- 反方向微动不再被当作起动样本，继续走已有 `early_low_energy_opposite_drift` / `low_energy_opposite_drift` 疑似边界降级逻辑。
- 低置信微动样本的日志保留真实反馈峰值；持久化学习值使用命令限流和反馈峰值中的保守较小值，避免把反馈尖峰抬高为下次扫描起点。
- `stopAndWaitForSettledFeedback()` 和 `holdAndWaitForSelfCheckSettledFeedback()` 进入等待前先记录当前真实反馈，取消或超时时不再把日志终点打印为零。
- 新增软件回归测试 `pre_self_check_accepts_low_confidence_micro_motion`，覆盖低置信微动通过路径。

验证：

```powershell
.\scripts\test.ps1
```

结果：8/8 通过。

测试备注：

- 回归过程中 Windows 应用控制策略临时拦截过新生成的 `test_safety_limiter.exe` 和 `test_virtual_encoder.exe`，删除对应构建产物并重新链接后恢复；最终完整测试 8/8 通过。

复测关注：

- 下一轮真实硬件日志中，若出现同向 `measured_mm >= low_confidence_motion_distance_mm` 且电流未超自检包络，应看到：

```text
PreSelfCheck | breakaway accepted ... confidence=low_micro_motion
```

- 双向微动接受后应进入 `StableShortStrokeMotion`，该阶段若仍不能完成目标短行程，应输出 `degraded` 并继续建立保守 profile。
- 若停稳后位移朝请求反方向，应看到 `reason=early_low_energy_opposite_drift` 或 `reason=low_energy_opposite_drift`，而不是 `breakaway accepted`。
- 点击停止后，取消 probe 的 `end_motor_pos_rad`、`end_estimated_mm` 应保留最后真实反馈附近的值，不应再显示为零。

## 问题 25：滑块拖动时实物电机约转一圈，但螺母行程和虚拟编码器变化很小

日期：2026-05-21。

测试日志：
- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T09-03-26.532Z.txt`

现象：
- 最后几次 `move_nut_stroke` 滑块拖动时，用户观察实物电机运动接近一圈。
- 日志中的 `motor_virtual_pos_rad` 只在约 `-0.53 ~ -0.02 rad` 范围内变化，`stroke_mm` 也只变化约 `0.1 mm` 量级。
- 典型日志：

```text
move_nut_stroke target_mm=10.03 ... start_mm=8.00061 end_mm=8.00061 measured_delta_mm=0 ... motor_virtual_pos_rad=-0.185216
move_nut_stroke target_mm=9.92 ... start_mm=8.10974 end_mm=8.10974 measured_delta_mm=0 ... motor_virtual_pos_rad=-0.528061
```

判断：
- 当前达妙状态帧里的 `q_uint` 不是输出端物理单圈绝对编码器计数，而是协议位置量程 `[-P_MAX, P_MAX]` 的 16 位编码值。
- 旧实现把 `q_uint` 按 `0..65535 -> 2*pi rad` 解包为物理单圈位置，导致虚拟多圈位置增量被错误缩放。
- 反馈线程每轮只取第一帧，也可能在命令期间使用接收队列积压的旧状态帧，让 UI 和 controller 短时间看到滞后的电机位置。

处理：
- 默认配置改为：
  - `motor.encoder_unwrap_source: protocol_position`
  - `motor.encoder_wrap_range_rad: 25.0`
- `q_uint` 继续保留为 `motor_raw_pos_counts` 诊断字段，不再作为当前样机默认多圈累计源。
- 达妙硬件反馈刷新改为在短时间窗口内持续读取接收队列，消化到最新一帧有效电机状态后再更新 `last_feedback_`。
- UI 操作日志增加：
  - `motor_wrapped_pos_rad`
  - `motor_raw_pos_counts`
- Web 夹爪状态区增加单帧协议位置和 `q_uint` 诊断显示，便于下一轮实测对比。
- 控制设计文档升级到 `v1.27`，同步修正虚拟编码器默认数据源定义。

复测关注：
- 滑块拖动电机约一圈时，`motor_virtual_pos_rad` 应变化接近 `6.28 rad`，`stroke_mm` 应按 2 mm/rev 接近变化 `2 mm`。
- `motor_wrapped_pos_rad` 仍可能在 `[-12.5, 12.5]` 内回绕，`motor_raw_pos_counts` 只用于辅助判断反馈帧是否确实在变化。
- 若仍出现实物运动明显大于 `motor_virtual_pos_rad`，应进一步确认达妙是否存在独立输出端绝对编码器反馈接口，或者当前状态帧是否来自电机内部侧而非输出端。

复核备注：

- 对 `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T09-03-26.532Z.txt` 复核后确认，该日志中的 `move_nut_stroke` 行尚未包含 `motor_wrapped_pos_rad` 和 `motor_raw_pos_counts`，因此它是本问题修正前旧程序格式输出，不应再作为修正后行为判据。
- 当前 `build\dev-zig\gripper_app.exe` 已重新构建，二进制中已包含新日志字段和 `protocol_position` 配置解析字符串。
- 本轮本地回归重新编译成功，但 Windows 应用程序控制策略拦截了新生成的 `gripper_app.exe` 和 `test_virtual_encoder.exe`，导致 `ctest` 中相关进程启动失败。失败原因为系统策略阻止启动，不是测试断言失败。
- 下一轮真实硬件复测前，应先确认运行的是最新构建产物，并确认新日志行中已经出现 `motor_wrapped_pos_rad` 和 `motor_raw_pos_counts`；若没有出现，说明仍在运行旧 exe 或旧日志。
- UI 配置阶段增加 `config_summary` 运行日志。复测日志开头应看到 `encoder_unwrap_source=protocol_position`、`encoder_wrap_range_rad=25`、`lead_screw_pitch_mm_per_rev=2`；若缺少该行或数值不一致，应先检查运行程序和配置文件，而不是继续分析运动结果。

## 问题 26：新配置下滑块拖动仍显得行程不跟随，且 ActiveStop 后 UI 状态停止更新

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T09-55-24.973Z.txt`

现象：

- 日志开头已经出现：

```text
config_summary | encoder_unwrap_source=protocol_position | encoder_wrap_range_rad=25 | lead_screw_pitch_mm_per_rev=2
```

- 说明问题 25 的新配置已经加载，当前日志不是旧程序格式。
- 手动滑块动作多次触发反馈电流超限，典型记录为：

```text
move_nut_stroke ... SafetyActiveStop | manual positioning feedback current exceeded limit current_a=-1.16044 limit_a=0.6 | controller_state=ActiveStop ... motor_vel_rad_s=-2.99634 ... motor_enabled=true
```

- 用户观察到电机动了很多，但 UI 中螺母行程变化不明显；进入 `ActiveStop` 后，UI 夹爪状态不再继续刷新。

分析：

- 该日志中 `motor_virtual_pos_rad` 与 `stroke_mm` 的换算关系已经一致：按 2 mm/rev 计算，`motor_virtual_pos_rad` 的变化量对应日志中的 `stroke_mm` 变化量。
- 当前仍不能排除达妙状态帧位置不是用户观察到的输出端完整真实位移，下一轮需要看滑块动作日志中的 `target_motor_pos_rad`、`start_motor_pos_rad`、`trigger_motor_pos_rad`、`end_motor_pos_rad` 和 `motor_delta_rad`。
- 更直接的软件缺陷是：手动定位电流超限时，controller 先把状态机切到 `ActiveStop` 并返回，但没有在返回前完成停机/失能收尾，因此日志出现 `ActiveStop` 但 `motor_enabled=true`，且反馈速度仍非零。
- Web UI 的 `/api/view` 只读取 controller 快照，没有主动触发 `update()`，所以在 `ActiveStop` 后如果没有其他动作，页面会显示旧快照，看起来像夹爪状态停止更新。
- 当前 `PreSelfCheck` 完成后仍只是低置信保守 profile，尚未完成 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck`。滑块在低置信阶段只能作为小范围验证入口，不能把 `stroke_mm` 当作完整标定后的真实绝对行程。

处理：

- 控制设计文档升级到 `v1.28`，明确：
  - `ActiveStop` 只停止命令输出，不停止反馈刷新。
  - 安全超限返回前必须先停止/失能电机输出并刷新停机后的反馈。
- 手动定位电流超限和堵转/限位疑似分支改为：
  - 记录触发瞬间电流、速度、虚拟编码器位置。
  - 调用停机/失能并等待停稳反馈。
  - 再进入 `ActiveStop` 并返回安全错误。
- `/api/view` 在没有后台 `PreSelfCheck` 运行时会调用一次 `controller.update()`，使 `ActiveStop` 下 UI 仍持续更新夹爪状态、电机位置、速度、电流和使能状态。
- `move_nut_stroke` 日志增加手动定位诊断字段：
  - `target_motor_pos_rad`
  - `start_motor_pos_rad`
  - `trigger_motor_pos_rad`
  - `end_motor_pos_rad`
  - `motor_delta_rad`
  - `motor_disabled`

复测关注：

- 触发滑块电流超限后，日志不应再出现 `controller_state=ActiveStop` 且 `motor_enabled=true` 作为最终结果；正常应看到 `motor_disabled=1`，UI 中 `motor_enabled=false`。
- 进入 `ActiveStop` 后，不点击恢复也应能看到 UI 夹爪状态区继续刷新电机位置、速度、电流等反馈。
- 若实物电机仍明显转动很多，而 `motor_delta_rad` 很小，需要进一步确认达妙状态帧位置是否来自输出端，或是否存在独立编码器/寄存器反馈接口。

## 问题 27：PreSelfCheck 单次反向漂移后即完成，低置信滑块目标过大导致电机抖动超流

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T10-15-31.516Z.txt`

现象：

- `PreSelfCheck` 只执行了一次 `closing` probe，反馈停稳后朝相反方向漂移约 `0.105 mm`，随后直接进入 `StructureProfileUpdate` 并返回 `Ok`。
- 典型日志：

```text
PreSelfCheck | phase=BidirectionalMoveEnable | degraded requested_direction=closing observed_drift_direction=opening ... reason=early_low_energy_opposite_drift
pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile
```

- 随后拖动滑块，电机快速抖动并触发 `ActiveStop`。新增诊断显示目标跨度过大：

```text
target_motor_pos_rad=-6.75093 start_motor_pos_rad=-0.702869 trigger_motor_pos_rad=-0.707065 end_motor_pos_rad=-0.836385
```

分析：

- 安全收尾已经生效：日志显示 `motor_disabled=1` 且 `motor_enabled=false`，问题 26 的收尾缺陷已修正。
- 新问题是自检过早通过：尚未确认任何方向可控时，第一次低能量反向漂移被当成足以生成低置信 profile 的条件，这会过早放开滑块。
- 低置信滑块使用理论 `safe_zone` 的完整范围，导致 UI 一次下发跨越约一圈的目标电机位置。对于尚未完成回零、行程学习和运动健康检查的状态，这个目标过大，容易造成位置模式瞬间拉动并超流。

处理计划：

- 控制设计文档升级到 `v1.29`。
- `PreSelfCheck` 在尚未确认任一方向可控时，不再因第一次 `early_low_energy_opposite_drift` 直接返回 `Ok`；应继续尝试另一方向，若仍不能确认可控方向则失败并保持滑块不可用。
- `MotionHealthCheck` 前的滑块单次目标限制在当前估算位置附近的小窗口内，由 controller 后端强制检查；完整理论低置信安全区仍作为硬边界，但不再作为单次可拖动全范围。

复测关注：

- 如果第一段 probe 仍出现 `reason=early_low_energy_opposite_drift`，后续应继续试探另一方向，不能马上 `pre_self_check: Ok`。
- 若两个方向都无法确认可控，`PreSelfCheck` 应失败，滑块保持禁用。
- 低置信阶段滑块目标若离当前估算位置太远，应返回 `OutOfRange` 或类似拒绝原因，不应触发电机大幅位置目标。

## 问题 28：达妙普通反馈帧位置解码使用了旧 P_MAX，导致多圈位置和螺母行程缩放错误

日期：2026-05-21。

背景：

- 用户使用官方上位机确认位置显示是多圈位置，且当前位置范围由参数 `PMAX` 决定。
- 上位机配置显示当前电机 `PMAX=50`，不是早期默认假设的 `12.5`。
- 需要验证两条路径：
  - 直接读寄存器 `p_m` / `xout`。
  - 解析普通状态反馈帧。

测试结论：

- 寄存器读取确认：
  - `P_MAX=50`
  - `V_MAX=20`
  - `T_MAX=28`
  - `p_m≈-5.000000 rad`
  - `xout≈-4.99047 rad`
- 同一普通反馈帧中 `q_uint=29490`：
  - 若按旧 `P_MAX=12.5` 解码，得到约 `-1.250286 rad`。
  - 若按运行时 `P_MAX=50` 解码，得到约 `-5.001144 rad`，与 `p_m` 基本一致。

判断：

- 普通状态反馈帧仍是主控制反馈来源，因为它一次包含位置、速度、力矩/电流、温度和状态，且适合固定频率后台更新。
- `p_m` 与 `xout` 寄存器读取适合作为诊断和校验；其中 `p_m` 与普通反馈按运行时 `P_MAX` 解码结果更一致，`xout` 略有偏差/滞后，不作为主控制位置源。
- 根因不是虚拟编码器算法本身，而是状态帧 `q_uint` 解码时使用了旧的 `P_MAX=12.5` 和旧的 `encoder_wrap_range_rad=25`。

处理：

- `DamiaoMotor::connect()` 在真实硬件连接阶段先读取 `P_MAX/VMAX/TMAX` 寄存器。
- 读取成功后用运行时 `P_MAX/VMAX/TMAX` 覆盖 `DamiaoMotorLimits`，再启动后台反馈线程。
- 如果运行时映射参数读取失败，连接失败并关闭 transport，不允许用旧默认量程继续解析反馈。
- 默认配置改为：
  - `motor.max_position_rad: 50.0`
  - `motor.max_velocity_rad_s: 20.0`
  - `motor.max_torque_nm: 28.0`
  - `motor.encoder_wrap_range_rad: 0.0`
- `encoder_wrap_range_rad=0.0` 表示真实硬件连接后由运行时 `2*P_MAX` 推导。当前样机运行时回绕范围为 `100 rad`。
- `bringup_can_probe` / 通信探针保留 `query_pmax`、`query_vmax`、`query_tmax`、`query_p_m`、`query_xout` 输出，便于现场对比。
- `DamiaoProtocol` 增加寄存器响应解析测试和运行时 `P_MAX=50` 下的反馈解码测试。

复测关注：

- 连接日志或通信探针应能看到 `P_MAX=50`、`VMAX=20`、`TMAX=28`。
- 同一位置下，普通反馈解码位置应与 `p_m` 接近。
- 滑块或自检运动时，`motor_virtual_pos_rad` 的变化应与实际电机输出端运动量同量级；按丝杆导程 2 mm/rev，`2*pi rad` 对应约 `2 mm` 螺母行程。
- 如果后续仍出现实物转动明显大于 `motor_virtual_pos_rad`，优先检查反馈线程是否收到最新状态帧、是否存在接收队列滞后、以及电机上位机显示位置是否来自其他寄存器/内部状态。

## 问题 29：PreSelfCheck 双向已微动但被反馈电流峰值和通用堵转判据误判失败

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T11-15-16.322Z.txt`

现象：

- `BidirectionalMoveEnable` 阶段已经在 0.15 A、0.2 A、0.25 A 指令电流下产生双向同向微动。
- 典型位移：
  - closing：约 `0.052~0.081 mm`
  - opening：约 `0.073~0.080 mm`
- 但这些样本的达妙反馈电流峰值约 `0.73~1.02 A`，高于 `safety.self_check_current_limit_a + max_current_ripple_a`。
- 旧逻辑因此没有接受这些有效起动样本，继续升到 0.3 A 后被通用堵转检测判为：

```text
SafetyJamDetected | pre-self-check probe stopped by jam or limit feedback
```

分析：

- 这不是电机未动作，也不是方向错误。
- 达妙反馈电流来自力矩反馈换算，低速位置力控下会有明显峰值，不等同于控制器下发的 `PositionForce` 命令限流值。
- `BidirectionalMoveEnable` 的目标是判断电机在低能量命令下是否能产生可控微动。该阶段的“低能量”应以命令电流和速度限幅为主，反馈电流/力矩峰值用于诊断和硬保护。
- 通用堵转检测以“速度低 + 电流上升”为条件，在极低速、短距离起动识别阶段容易把有效微动误判为堵转。

处理：

- 控制设计升级到 `v1.31`。
- `BidirectionalMoveEnable` 接受起动样本时：
  - 保留停稳、同向位移、单调、无反向运动、无硬件故障、无主动停止等要求。
  - 命令电流仍受 `safety.self_check_current_limit_a` 截断。
  - 达妙反馈电流峰值不再要求低于 `self_check_current_limit_a + max_current_ripple_a`。
  - 反馈电流峰值仍不得超过全局硬保护 `safety.max_motor_current_a`。
- `BidirectionalMoveEnable` 使用专门的第一阶段风险判断：
  - 已出现同向微动后不再用通用速度跌落堵转判据误杀当前 probe。
  - 未出现同向微动时，仍保留理论行程边界附近的限位风险和全局硬保护电流检查。
- 静摩擦学习值仍使用命令限流和反馈峰值中的保守较小值，避免把反馈尖峰写成下次扫描起点。

验证：

```powershell
.\scripts\test.ps1
.\build\dev-zig\test_hardware_selfcheck.exe --config src\config\default_gripper.yaml
```

结果：

- 软件回归：8/8 通过。
- 真实硬件自检通过：

```text
PreSelfCheck | breakaway accepted direction=closing confidence=normal ...
PreSelfCheck | breakaway accepted direction=opening confidence=normal ...
PreSelfCheck | phase=StableShortStrokeMotion | low_confidence_samples_accepted ...
PreSelfCheck | phase=StructureProfileUpdate | profile updated
pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile
final_state ... motor_enabled=false
```

剩余风险：

- 本次仍是低置信保守 profile。`StableShortStrokeMotion` 和 `MultiRegionRoundTripLearning` 存在降级日志，说明当前自检只证明了低能量可控微动和保守安全区，不代表完整行程、零位和运动健康已经建立。
- 后续仍需继续执行并调试 `HomingOpenStop`、`TravelLearning` 和 `MotionHealthCheck`，才能进入完整业务控制。

## 问题 30：PreSelfCheck 后 UI 滑块显示完整低置信安全区，导致拖动全部被后端拒绝

日期：2026-05-21。

测试日志：

- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T11-26-51.926Z.txt`

现象：

- `PreSelfCheck` 已经成功完成，并生成保守低置信 profile：

```text
pre_self_check: Ok | pre-self-check completed with conservative feedback-derived profile | controller_state=Disabled | stroke_mm=8.25985 ... motor_enabled=false
```

- 后续滑块命令全部被后端拒绝：

```text
move_nut_stroke target_mm=10.28 speed_mm_s=1: OutOfRange | target nut stroke is outside allowed range target_mm=10.28 min_mm=7.75985 max_mm=8.75985 confidence=pre_self_check_safe_zone | controller_state=Enabled ... motor_enabled=true
```

分析：

- controller 在 `MotionHealthCheck` 前只允许围绕当前估算螺母位置的小窗口移动，本次允许范围约为 `7.75985~8.75985 mm`。
- UI 仍按完整低置信安全区显示滑块范围，用户拖动到 `9~10 mm` 等目标时必然被后端 `OutOfRange` 拒绝。
- 旧实现还存在顺序问题：`moveToNutStroke()` 在目标范围校验前调用了使能准备逻辑，导致越界拒绝后日志出现 `controller_state=Enabled`、`motor_enabled=true`。

处理：

- 控制设计升级到 `v1.32`，明确 UI 必须使用 controller/API 暴露的当前手动定位允许范围，不得自行用完整低置信安全区推导滑块范围。
- `GripperStateSnapshot` 增加 `manual_nut_stroke_range`：
  - `open_limit`
  - `closed_limit`
  - `valid`
  - `use_software_limits`
  - `low_confidence_window`
  - `confidence`
- `/api/view` 增加字段：
  - `manual_stroke_range_valid`
  - `manual_stroke_min_mm`
  - `manual_stroke_max_mm`
  - `manual_stroke_uses_software_limits`
  - `manual_stroke_low_confidence_window`
  - `manual_stroke_confidence`
- Web UI 滑块改为只使用上述后端字段；`PreSelfCheck` 后显示“低置信小步验证范围”，不再显示完整低置信安全区作为可拖动范围。
- `moveToNutStroke()` 调整为先做状态、范围和参数校验，目标合法后才使能电机并进入 `ManualPositioning`。

验证：

```powershell
.\scripts\test.ps1
```

结果：

- 完整构建成功，`gripper_app.exe` 已重新链接。
- 自动化测试 `8/8` 通过。
- 新增回归确认：低置信阶段越界 `moveToNutStroke()` 返回 `OutOfRange`，且不会使能电机输出。
- Web UI 冒烟确认：
  - `/` 返回 `200`。
  - `/api/view` 返回 `manual_stroke_range_valid`。
  - 页面包含“低置信小步验证范围”提示。

复测关注：

- 真实硬件上完成 `PreSelfCheck` 后，滑块最小/最大值应接近后端日志中的允许范围，而不是完整理论安全区。
- 若拖动超出当前小窗口，后端仍应拒绝，但日志不应再出现越界拒绝后 `motor_enabled=true`。
- 每次有效滑块运动结束、超时或失败后仍应失能电机。

补充修正：

- 2026-05-21 后续澄清：螺母位置滑块的主要用途不是夹爪业务控制，而是在调试阶段确认电机多圈编码器、丝杆导程和螺母行程映射是否正确，即验证电机输出端反馈转一圈时螺母行程是否约为 `2 mm`。
- 在结构未安装/空载明确确认的前提下，`PreSelfCheck` 已经建立的低置信理论边界 `0~16 mm` 可以作为滑块验证范围；此前只允许当前位置附近小窗口的策略只适用于结构已安装或空载未确认场景。
- `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T11-41-45.684Z.txt` 中已能看到：

```text
PreliminaryLimitSearch | limits accepted open_mm=0 close_mm=16 travel_mm=16 travel_error_mm=0 note=theoretical_limits_used_for_low_confidence_precheck
```

- 因此该日志不是“自检没有识别出 0~16 mm”，而是 UI/控制策略没有区分“低置信边界”和“当前普通滑块范围”。
- 对 UI 增加“结构未安装/空载确认，允许完整低置信边界内限流拖动”选项；勾选后 `move_stroke` 请求带 `unloaded=1`，controller 允许使用完整低置信边界；未勾选时仍保持当前位置附近小窗口。
- 手动定位命令改为带命令级限流的 `PositionVelocityTorque` / PositionForce 路径，不再使用普通无命令限流的位置模式。
- 手动定位成功日志增加：
  - `motor_delta_rev`
  - `mm_per_rev_estimate`
  便于人工核对 `2 mm/rev`。

验证：

```powershell
.\scripts\test.ps1
```

结果：

- 自动化测试 `8/8` 通过。
- 新增回归确认：未确认空载时仍拒绝完整低置信边界外的小窗口目标；确认空载后可在完整低置信边界内执行受限拖动。
- Web UI 冒烟确认页面包含 `sliderUnloadedConfirm` 和 `2mm/rev` 说明。

## 问题 31：空载确认后滑块任意距离拖动仍快速抖动并进入 ActiveStop

现象：

- 最新日志 `C:\Users\jyt22\Downloads\gripper-log-2026-05-21T12-04-48.757Z.txt` 中，`PreSelfCheck` 已经接受低置信理论边界：

```text
PreliminaryLimitSearch | limits accepted open_mm=0 close_mm=16 travel_mm=16 travel_error_mm=0 note=theoretical_limits_used_for_low_confidence_precheck
```

- 滑块请求已带 `unloaded_confirmed=1`，说明 UI 到 controller 的空载确认链路有效。
- 但手动定位开始后很快触发反馈电流超限并进入 `ActiveStop`：

```text
move_nut_stroke target_mm=5.77 speed_mm_s=1 unloaded_confirmed=1:
SafetyActiveStop | manual positioning feedback current exceeded limit
trigger_current_a=2.33026 limit_a=0.6 ...

move_nut_stroke target_mm=6.54 speed_mm_s=1 unloaded_confirmed=1:
SafetyActiveStop | manual positioning feedback current exceeded limit
trigger_current_a=2.65846 limit_a=0.6 ...
```

分析：

- 这不是低置信边界没有识别出来；边界已经是 `0~16 mm`。
- 直接把限流从 `0.6A` 提到 `1.5A` 只能缓解一部分问题，因为本次触发峰值已经超过 `2A`。
- 根因是手动滑块定位一次性向位置力控下发完整目标，目标与当前位置相差数 rad。位置误差过大时，电机会短促快速纠偏，产生反馈电流尖峰，表现为“快速抖一下停止”。
- 达妙反馈电流/力矩峰值在位置力控下不等同于命令限流本身，仍需保留硬保护，但不能把单帧尖峰和持续堵转混为一谈。

处理：

- 控制设计升级到 `v1.34`。
- 新增独立配置 `safety.manual_positioning_current_limit_a`，默认 `1.5A`，用于滑块/手动定位受限调试，不再复用 `travel_learning_current_limit_a=0.6A`。
- 全局不可绕过硬保护 `safety.max_motor_current_a` 默认调整为 `2.0A`。
- `moveToNutStroke()` 的执行策略改为按反馈周期和目标速度生成小步递进位置目标，不再一次性下发完整大跨度目标。
- 手动定位反馈电流超限改为持续时间判据；持续超过 `manual_positioning_current_limit_a` 或超过全局硬保护时才进入 `ActiveStop`。
- 手动定位日志增加 `command_steps` 和 `max_command_step_mm`，用于确认底层是否按小步执行。

复测关注：

- 空载确认后拖动滑块时，电机应按设定速度连续运动，而不是短促抖动一下。
- 日志中 `command_steps` 应大于 1，`max_command_step_mm` 应接近 `speed_mm_s * feedback_poll_period_s`。
- 对于 2mm 导程，若电机输出端反馈变化约 `6.283 rad`，`stroke_mm` 应变化约 `2mm`，成功日志中的 `mm_per_rev_estimate` 应接近 `2`。
- 仍需关注是否出现持续反馈电流超过 `1.5A` 或硬保护超过 `2.0A`。如果空载仍持续超限，应继续查达妙位置力控限流参数或机械/电机模式配置，而不是继续抬高阈值。

## 问题 32：Bring-up 精确转动 2 圈时多数只运动约半圈，接近特定位置时只微动

现象：

- 最新日志 `C:\Users\jyt22\Downloads\gripper-log-2026-05-22T03-49-34.847Z.txt` 中，在 `MotorBringupMode` 执行 `relative_rev=2`。
- 多数记录只运动约 `0.48 rev`：

```text
requested_rev=2 effective_rev=2 start_motor_pos_rad=35.4017
target_motor_pos_rad=47.9681 end_motor_pos_rad=38.4306
motor_delta_rad=3.02892 measured_rev=0.482067 timeout_s=3
```

- 当起点接近 `P_MAX=50rad` 时，正向 2 圈目标超过达妙位置窗口：

```text
start_motor_pos_rad=41.49 target_motor_pos_rad=54.0564
```

分析：

- `max_vel_rad_s=1`、`timeout_s=3` 与 2 圈目标不匹配。2 圈对应约 `12.57rad`，在 `1rad/s` 下理论运动时间至少 `12.57s`，因此 3 秒只走约 `3rad`，即约 `0.48rev`，与日志一致。
- `mm_per_rev_estimate=-2` 稳定，说明电机多圈反馈到虚拟螺母行程的比例关系本身是正确的；负号来自当前 `direction_sign=-1`。
- 达妙位置模式命令受运行时 `P_MAX` 窗口约束。若目标超过 `[-P_MAX, P_MAX]`，应在控制器下发前拒绝并提示，而不是让电机尝试。

处理：

- V2 设计升级到 `v2.2`，增量计划升级到 `v2.6`。
- 达妙硬件层将连接阶段读取到的运行时 `P_MAX/VMAX/TMAX` 透传到 `MotorFeedback`。
- 控制器快照和 Web API 增加：
  - `motor_runtime_limits_valid`
  - `motor_runtime_pmax_rad`
  - `motor_runtime_vmax_rad_s`
  - `motor_runtime_tmax_nm`
- UI 夹爪/电机状态区域增加 `P_MAX`、`VMAX`、`TMAX` 显示。
- `MotorBringupMode` 相对圈数位置命令在发送前检查目标是否位于 `[-P_MAX, P_MAX]`。越界时返回 `OutOfRange`，日志包含 `pmax_rad` 和建议动作 `move_opposite_direction_or_increase_motor_pmax`。
- `motor_bringup.default_position_move_timeout_s` 改为 `0`，表示按 `abs(rev * 2*pi) / velocity` 自动估算到位超时并叠加裕量。
- `motor_bringup.max_position_move_timeout_s` 调整为 `20s`，避免 2 圈低速验证默认必然超时。

验证：

```powershell
.\.venv\Scripts\cmake.exe --build --preset dev-zig --target gripper_app
.\.venv\Scripts\ctest.exe --preset dev-zig
```

结果：

- 完整自动化测试 `10/10` 通过。
- Web UI 冒烟通过，页面包含 `runtimePmax`，`/api/view` 包含 `motor_runtime_pmax_rad`。

复测关注：

- 执行 `+2rev` 前先看 UI 中 `P_MAX` 和当前电机位置。如果 `current + 12.57rad > P_MAX`，UI/日志应拒绝该命令，需先反向运动或在上位机中调大 `P_MAX`。
- 若 `P_MAX=50rad`，当前 `41rad` 附近只能正向再走约 `8.5rad`，不足 2 圈。
- `2rev`、`1rad/s` 时自动超时应接近 `15~16s`，不再因默认 3s 只走半圈。

补充修正（2026-05-22）：
- 新日志 `C:\Users\jyt22\Downloads\gripper-log-2026-05-22T04-23-02.186Z.txt` 仍显示 `timeout_s=3`，说明前一轮只改了控制器和部分配置，UI/API/commander 入口仍有固定 `3s` 默认值残留。
- 该日志中多次 `-2rev` 的 `motor_delta_rad` 约为 `-3.03rad`，与 `1rad/s * 3s` 完全一致，因此“多数只转半圈”的直接原因仍是超时参数链路错误，而不是虚拟螺母编码器比例错误。
- 同一日志中 `mm_per_rev_estimate=-2` 稳定，说明电机多圈反馈到虚拟螺母行程的比例关系正确；负号来自当前 `motor.direction_sign=-1`。
- 当前位置 `40.9972rad` 执行 `+2rev` 时目标 `53.5635rad` 超出 `P_MAX=50rad`，控制器返回 `OutOfRange` 是正确行为，应先反向移动或在达妙上位机中调大 `P_MAX`。
- 个别位置只微动后 `SafetyActiveStop` 的原因是反馈电流尖峰超过旧的 `1.0A` 中止阈值。位置模式启动/制动瞬时尖峰不应等同于持续堵转，因此圈数到位的普通反馈电流超限改为持续超限判据，全局硬保护仍立即停机。

本次处理：
- `src/ui/web_server.cpp`、`src/ui/prototype/admin_recovery_ui_preview.html`、`src/ui/main_window.cpp` 中圈数到位的缺省超时统一改为 `0`，由控制器自动按 `abs(rev * 2*pi) / velocity` 加裕量估算。
- 圈数到位的反馈电流中止默认值改为 `1.5A`，`motor_bringup.max_motor_current_a` 改为 `2.0A`，仍受 `safety.max_motor_current_a=2.0A` 硬保护约束。
- 控制器日志增加 `current_limit_confirm_time_s`，并将普通反馈电流超限区分为 `sustained_feedback_current_limit`，全局硬保护区分为 `hard_feedback_current_limit`。
- 新增回归测试：`2rev`、`1rad/s`、`timeout=0` 时有效 `timeout_s` 应约为 `15.708s`，不得回退到旧的 `3s`。

## 2026-05-26 PreB/TravelLearning 旧 16mm 理论行程截断

现象：

- 用户复测发现 `PreSelfCheck` 和 `TravelLearning` 虽然能运动，但闭合方向仍离真实结构限位很远，疑似被旧 `0-16mm` 行程假设截断。
- 现场判断实际丝杆可用行程可能达到 `18-20mm`，旧 `0-16mm` 只能作为理论参考和日志对照。
- 达妙上位机已将 `P_MAX` 调整到约 `65rad`，避免 `20mm` 级搜索在电机位置窗口内过早越界。

处理：

- 默认 `motor.max_position_rad` fallback 同步到 `65.0`；真实硬件仍以连接阶段读取的运行时 `P_MAX` 为准。
- 新增并启用 `self_check.travel_learning_search_distance_mm: 20.0`，`PreB` 单向扩展默认同步为 `20.0mm`。
- `TravelLearning` 闭合搜索目标改为使用配置搜索距离、PreB 低置信覆盖宽度、旧理论参考行程和最小行程中的最大值，并在发送前按运行时/配置 `P_MAX` 动态裁剪。
- `TravelLearning` 的临时 stroke guard、学习后回退 guard 和摩擦异常 map 重基准不再使用 `theoretical_close_limit_mm=16` 夹断。
- 默认 `safety.travel_learning_current_limit_a` 从 `1.2A` 提高到 `1.5A`，仍低于 PreSelfCheck `1.9A` 和全局 `2.0A` 硬保护。

复测关注：

- 行程学习日志应出现 `search_distance_mm`、`requested_search_distance_mm`、`reference_theoretical_close_mm=16`、`target_reached` 和 `pmax_rad`。
- 若闭合方向到达 `target_reached=true` 但未出现结构端部，说明真实行程可能仍超过当前 `travel_learning_search_distance_mm`，需要继续增大该配置并确认 `P_MAX` 余量。
- 若因 `pmax_travel_search_limited=true` 被裁剪，应优先检查 UI 中当前电机位置和运行时 `P_MAX`，再决定是否进一步增大上位机 `P_MAX` 或改变起始位置。

## 2026-05-26 PreB emergency 电流、端点 hold 收尾和 ActiveStop

现象：

- 日志 `C:\Users\jyt22\Downloads\gripper-log-2026-05-26T07-38-09.586Z.txt` 中，PreB opening 连续探边使用 `current_limit_a=1.9`、`hard_current_limit_a=2.5`、`emergency_current_limit_a=4`。
- opening 探边从约 `7.97mm` 到 `-9.35mm` 后触发：

```text
max_current_a=5.63419 emergency_current=true
message=pre-self-check probe stopped by emergency feedback current limit
```

- 随后的 `BoundaryRelease` 仅移动约 `0.011mm`，再次触发：

```text
max_current_a=4.56205 emergency_current=true
message=pre-self-check probe stopped by emergency feedback current limit
```

- 最终 `PreSelfCheck` 失败并进入 `ActiveStop`，流程不能继续。

分析：

- 控制器已经检测到 emergency 反馈电流，不是“没有检测到”。
- 高电流的直接场景是物理端点/端点附近抱死后，位置力控仍对端点施加输出。达妙 `POS_FORCE` 的命令限流字段按 `current_limit / motor.max_phase_current_a` 转成 per-unit 限幅；反馈电流则由反馈力矩按 `torque_per_amp` 换算。因此命令 `1.9A` 与反馈峰值 `5A+` 不一定一一相等，仍需核对 `max_phase_current_a`、运行时 `TMAX` 和 `torque_per_amp` 标定。
- 软件侧缺陷是 probe 发现持续硬电流/emergency 后仍进入 `holdAndWaitForSelfCheckSettledFeedback(command_current_limit)`。端点顶死时当前位置 hold 可能继续压住机械端，增加释放失败概率。
- 另一个缺陷是 PreB 内部的 emergency 反馈停机没有作为探边诊断降级，`BoundaryRelease` 中同类失败被传播到顶层，导致 `PreSelfCheck` 进入全局 `ActiveStop`。

处理：

- 设计升级到 `v2.43`，实施计划升级到 `v2.47`。
- PreSelfCheck probe 若因持续硬反馈电流、emergency 电流或持续无推进停止，收尾改为立即失能/卸载并等待反馈停稳，不再发送当前位置 hold。
- PreB 内部的持续硬反馈电流、emergency 电流和持续无推进结果作为探边/释放诊断降级处理，保留已有 low-confidence bounds 或 PreA 保守窗口，除反馈方向矛盾、用户停止、硬件/通信/配置错误外，不直接升级为全局 `ActiveStop`。
- `BoundaryRelease` 如果自身遇到同类端点反馈停止，记录 `release_failed_degraded`，上层继续后续阶段；后续多区域学习仍要避开端点/异常区域，区域不足时按机制异常或阶段降级处理。

复测关注：

- 同一场景下，日志应能看到 terminal feedback stop 后的 settle action 为 disable/unload，而不是 hold。
- opening 端触发 emergency 后，PreB 应继续尝试 closing 方向或后续低置信流程，不应立刻 `pre_self_check: SafetyJamDetected ... controller_state=ActiveStop`。
- 若仍出现 `5A+` 峰值，需要继续核对达妙 `POS_FORCE` 电流字段标定、`motor.max_phase_current_a`、运行时 `TMAX` 和实际反馈力矩到电流的换算。
