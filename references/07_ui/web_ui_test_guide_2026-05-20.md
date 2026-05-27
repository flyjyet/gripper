# Web UI 测试说明

日期：2026-05-20

## 启动方式

模拟硬件：

```powershell
.\build\dev-zig\gripper_app.exe --web-ui --config src\config\default_gripper.yaml
```

真实达妙硬件：

```powershell
.\build\dev-zig\gripper_app.exe --web-ui --damiao --config src\config\default_gripper.yaml
```

默认地址：

```text
http://127.0.0.1:8765/
```

如果端口被占用，程序会从 `8765` 起向后尝试 32 个端口。也可以指定端口：

```powershell
.\build\dev-zig\gripper_app.exe --web-ui --web-port 8876 --damiao --config src\config\default_gripper.yaml
```

## 当前界面功能

运行时首页直接加载：

```text
src/ui/prototype/admin_recovery_ui_preview.html
```

该文件是当前 Web UI 的单一页面模板。后续调整页面布局和视觉风格时，应优先修改
这个 prototype 文件；`WebServer` 只负责把它作为首页返回，并提供 `/api/view`
和 `/api/action` 后端接口。

- 连接 / 断开。
- 主动停止 / 清除故障。
- Bring-up 模式：
  - 机构安全确认。
  - 进入 Bring-up。
  - CAN 探测。
  - 读取反馈。
  - Bring-up 使能 / 失能。
  - 正向 / 反向短脉冲点动。
  - 按 `+1rev/-1rev/+2rev/-2rev` 或输入圈数执行空载位置到位验证。
  - 在 Bring-up 操作区显示达妙运行时 `P_MAX/VMAX/TMAX` 和当前电机多圈位置，便于判断圈数目标是否会越过 `[-P_MAX, P_MAX]`。
  - 退出 Bring-up。
- 常规流程按钮：
  - 常规使能 / 失能。
  - PreSelfCheck。
  - 回零。
  - 行程学习。
  - 健康检查。
  - 目标力夹紧。
  - 释放。
- 顶栏控制器状态带：
  - 控制器顶层状态。
  - 通信状态。
  - 电机输出使能状态。
  - 当前流程阶段。
- 自检 / 自学习参数显示：
  - 结构参数有效性。
  - 打开/闭合可动电流。
  - 打开/闭合最低稳定运行速度。
  - 静摩擦和动摩擦估计。
  - 软件安全区。
  - PreB 电流-行程曲线，用于显示预探索和多区域学习中的反馈电流随临时行程变化。
    曲线按后端 `segment_id` 连续采样段断线显示，不连接左右极限、回程跳变或不同扫描段；横纵坐标显示 mm/A 网格和刻度。
    从 closing 限位连续回 opening 软限位的学习准备动作会仅显示在曲线上，不进入 anomaly map 或结构模型学习样本。
    曲线下方可设置打开、闭合和未知/诊断曲线颜色，颜色只保存在浏览器本地，用于人工判读，不下发到控制器。
- 夹爪状态显示：
  - 螺母行程。
  - 夹爪角度。
  - 电机位置、速度、电流、力矩、温度。
  - 估算夹紧力。
  - 达妙运行时 `P_MAX/VMAX/TMAX`。
  - 螺母位置拖动滑块：
    - 未完成 `PreSelfCheck` 时禁用。
    - `PreSelfCheck` 完成后显示后端给出的低置信小步验证范围。
    - `MotionHealthCheck` 完成后显示最终置信运动范围。
    - 滑块拖动过程中只预览目标位置，松开后调用 `/api/action?name=move_stroke`。
    - 后端通过 `UiController -> GripperController::moveToNutStroke()` 执行受限螺母行程移动，不直接发送电机原始命令。
    - 该入口属于调试定位，使用电机位置/位置速度模式；最终夹紧仍走限速下的力控/力代理路径。
    - 调试定位普通超时未到位只返回超时并失能，不直接进入 `ActiveStop`；反馈电流、堵转、限位或硬件故障超限才进入 `ActiveStop`。
- 运行日志显示。
- 日志支持“暂停滚动”，便于人工查看某条日志。
- “配置参数”分页：
  - 只读显示 `/api/view` 返回的当前加载配置快照。
  - 按 `adapter/motor/mechanism/self_check/safety/motor_bringup/homing/clamp/ui` 分组显示。
  - `common` 只定义单位、Result、错误码等基础类型，不作为运行参数组展示。
  - 达妙运行时 `P_MAX/VMAX/TMAX` 以硬件读取值优先，配置页中的 `P_MAX_config/VMAX_config/TMAX_config` 仅表示配置 fallback。
- 浏览器按钮“关闭 Web UI”可请求后端退出。

## 交互门禁

当前 UI 已按状态做前端门禁：

- 未连接时，只允许“连接”。
- 连接后，才允许断开、停止、释放、进入 Bring-up、预自检等连接相关动作。
- 未进入 Bring-up 时，CAN 探测、读取反馈、Bring-up 使能和点动不可用。
- 正向/反向短脉冲点动进入 Bring-up 后即可使用；点动动作内部会自动短时使能，
  并在脉冲结束后自动失能。
- 普通流程按状态逐步解锁：
  - 预自检：连接后可用。
  - 回零：PreSelfCheck 后可用。
  - 行程学习：回零后可用。
  - 健康检查：行程学习后可用。
  - 目标力夹紧：健康检查/Ready 后可用。
- `ActiveStop` 是可恢复主动停止状态。进入该状态时，页面顶部会解锁
  “恢复 ActiveStop”按钮，并在提示区要求先查看日志、确认没有机械风险。
  该按钮调用后端 `clearFault()`，状态机恢复路径为
  `ActiveStop + FaultCleared -> Disabled`。
- 点击可用按钮时，按钮会显示“执行中...”，结束后有成功/失败轮廓反馈，并在提示区显示结果或拒绝原因。
- 顶部页签和左侧导航已实现切页：
  - 普通控制。
  - 自检流程。
  - 维护模式。
  - 配置参数。
  - 日志与诊断。
  分页只组织界面，不改变 controller 状态机和安全门禁。
- 螺母位置拖动滑块启用条件：
  - 后端 `controls.can_move_nut_stroke=true`。
  - 已完成 `PreSelfCheck`。
  - 未处于 `MotorBringupMode`、管理员模式、`ActiveStop` 或 `Fault`。
  - 当前显示范围来自后端 `/api/view` 中的手动定位允许范围字段。
  - `PreSelfCheck` 后但 `MotionHealthCheck` 前，未确认结构未安装/空载时，显示范围是当前估算位置附近的小窗口，不是完整低置信安全区。
  - 勾选“结构未安装/空载确认”后，滑块显示完整低置信边界，用于验证电机多圈编码器、丝杆导程和螺母行程映射。
  - 目标值落在当前显示范围内。后端会再次校验范围，超出时返回 `OutOfRange`，且拒绝前不应使能电机。

## ActiveStop 恢复说明

`ActiveStop` 通常由主动停止、限流、限速、接触/堵转、行程越界等安全路径触发。
触发后程序应停止电机输出，操作者需要先看运行日志确认触发原因。

恢复步骤：

1. 确认电机输出已经关闭，机构没有继续夹紧或卡滞加剧风险。
2. 在 UI 顶部点击“恢复 ActiveStop”。
3. 日志中应出现 `clear_fault: Ok`，控制器状态应从 `ActiveStop` 恢复到
   `Disabled`，电机仍应保持失能。
4. 恢复后不要直接夹紧。需要根据现场原因重新执行必要的 Bring-up、预自检、
   回零、行程学习或健康检查。

日志中 `controller_state` 表示夹爪控制器顶层业务状态，`motor_enabled`
表示电机驱动输出是否使能。两者不是同一个概念。恢复 `ActiveStop` 时不应自动
重新使能电机，因此期望结果是 `controller_state=Disabled` 且
`motor_enabled=false`。

注意：`Bring-up 失能` 是电机空载调试的专用失能动作，只关闭电机输出，
不应进入 `ActiveStop`。若普通“停止并失能”被点击，则仍会进入 `ActiveStop`，
需要按上述流程恢复。

## 管理员恢复区

当前管理员恢复功能只有 UI 门禁，没有完整后端安全策略：

- “进入恢复模式”需要已连接。
- 进入后只改变 UI 管理员会话状态，不发送电机命令。
- “低能量方向试探”“高电流打开”“保存本次参数”“恢复默认限制”均标记为后端未实现并保持禁用。
- 这样处理是为了避免把未落地的高权限恢复动作伪装成可用功能。
- “恢复后要求”显示在管理员恢复区域下方，避免和普通夹爪状态混在一起。
- 后续需要实现 `enterAdminRecovery`、方向试探、高权限点动/释放、错误方向主动阻断、审计记录和恢复后强制重新自检等后端接口后，再逐项解锁。

## 建议人工测试顺序

真实硬件首次测试只建议执行低能量 bring-up 路径：

1. 启动 Web UI。
2. 浏览器打开 `http://127.0.0.1:8765/`。
3. 点击“连接硬件”。
4. 勾选“机构已安全/空载”。
5. 点击“进入 Bring-up”。
6. 点击“CAN 探测”。
7. 确认日志中出现：

```text
TX refresh ... data=01 00 cc ...
RX FD id=0x11 ...
TX query_master_id ... data=01 00 33 08 ...
RX FD id=0x11 ...
```

8. 点击“读取反馈”，确认位置、速度、电流、力矩、温度刷新。
9. 需要确认方向时，优先使用默认低电流、短时间正反向速度点动。
   点动日志会记录 signed `direction_window_rad`，例如正向为正值、反向为负值。
   实际点动距离由 `vel_rad_s * pulse_s` 决定。
10. 手动 `Bring-up 使能 / 失能` 只用于验证使能链路。若使能后反馈出现接近限位、
    高速或大电流异常值，先停止并记录日志，不要继续点动。

在方向、电流、限速、失能路径没有人工确认前，不建议直接执行常规夹紧、
PreSelfCheck、回零或行程学习。

## 当前验证

已执行：

```powershell
.\scripts\build.ps1
.\scripts\test.ps1
```

结果：`6/6` 自动化测试通过。

已执行本地模拟 Web UI 冒烟：

- `/` 返回 `200`。
- 首页内容确认来自 `src/ui/prototype/admin_recovery_ui_preview.html`。
- `/api/view` 返回 `200`。
- `/api/action?name=shutdown` 返回 `200`。
- 首页确认包含状态门禁标记 `data-requires`、未实现标记
  `data-unimplemented`、交互提示区和按钮忙碌反馈文案。
- 首页确认包含“恢复 ActiveStop”按钮、`active_stop` 门禁和
  `can_recover_active_stop` 后端控制字段。
- 首页确认包含“控制器状态”“自检 / 自学习参数”“夹爪状态”三块显示内容。
- 首页确认夹爪状态下包含“螺母位置拖动”滑块，管理员恢复区下包含“恢复后要求”。
- 首页确认包含 `data-tab-target` 和 `data-tab-panel`，顶部分页不再禁用。
- `/api/view` 确认返回 `controls.can_move_nut_stroke`。
- `/api/view` 确认返回当前手动定位范围：
  - `manual_stroke_range_valid`
  - `manual_stroke_min_mm`
  - `manual_stroke_max_mm`
  - `manual_stroke_confidence`
  - `manual_stroke_low_confidence_window`
- `/api/action?name=move_stroke&stroke=...&speed=...` 已接入受限螺母行程移动控制器入口。
- `/api/view` 确认返回：
  - `structure_profile_validity_text`
  - `opening_breakaway_current_a`
  - `closing_breakaway_current_a`
  - `opening_minimum_stable_speed_mm_s`
  - `closing_minimum_stable_speed_mm_s`
  - `static_friction_summary`
  - `dynamic_friction_summary`
  - `safe_zone_summary`
