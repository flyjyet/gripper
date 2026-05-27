# C++ Bring-Up CAN Probe Update

日期：2026-05-20

## 背景

重构前 Python UI 已经遇到过电机使能、反馈和无 TX/RX 的问题，并通过以下策略解决：

- 使能前先写 `CTRL_MODE=4`，进入 `POS_FORCE`。
- 初始 `current_mode` 不假定已经是 `POS_FORCE`。
- 默认启用 CANFD/BRS。
- 位置力控和使能帧使用 `0x300 + motor_id`。
- 先执行 `CAN probe`，确认总线有 RX，再进行 `Enable`、回零或点动。

对应归档：

- `references/01_requirements/夹爪控制系统联调归档.md`
- `references/00_context/2026-05-15_context_handoff.md`
- `references/03_control/verified_ui/gripper_control_ui.py`

## 本次修改

C++ 重构版增加了被动通信探测命令：

```text
bringup_can_probe
```

推荐测试顺序：

```text
connect
bringup_enter_confirm_unloaded
bringup_can_probe
bringup_feedback
bringup_enable
```

## 行为边界

`bringup_can_probe` 只做通信诊断：

- 发送 `0x7FF` refresh 帧。
- 发送 `0x7FF` 读取 `MASTER_ID` / RID `0x08` 帧。
- 记录原始 TX/RX 行。
- 不使能电机。
- 不点动。
- 不回零、不置零、不学习行程、不更新结构参数。

## 结果判断

如果日志中出现 `RX` 行：

- USB2FDCAN 接收回调和总线响应基本成立。
- 后续重点检查反馈 ID、反馈解析、模式切换、使能帧 ID。

如果两次请求均显示 `RX none`：

- 不要继续执行 `bringup_enable` 或 `bringup_jog_*`。
- 优先检查电机控制电源、CANH/CANL、终端电阻、通道号、FDCAN/BRS、ID、固件协议和是否有 DMTool 占用设备。

## 验证

已执行：

```powershell
.\scripts\test.ps1
```

结果：`6/6` 自动化测试通过。

## SDK 同步检查

用户确认 `references/04_hardware/DM_DeviceSDK` 是当前最新版 SDK。

检查结果：

- 当前运行时 DLL：
  - `src/third_party/damiao/bin/libdm_device.dll`
  - SHA256: `DB7FC43D0D60B847479DCB58CFBE13E524B0CA43C5DFA847D14E760BC0E2A8A7`
- 最新 SDK v1.1.0 Windows mingw DLL：
  - `references/04_hardware/DM_DeviceSDK/C&C++/lib/v1.1.0/windows/mingw/libdm_device.dll`
  - SHA256: `DB7FC43D0D60B847479DCB58CFBE13E524B0CA43C5DFA847D14E760BC0E2A8A7`

结论：当前程序实际加载的 DLL 与最新版 SDK v1.1.0 DLL 一致。

本次同步：

- 用最新版 SDK 的完整 `dmcan.h` 覆盖 `src/third_party/damiao/include/dmcan.h`。
- C++ 动态加载新增：
  - `dmcan_get_sdk_version`
  - `dmcan_device_get_version`
- `bringup_can_probe` 日志新增 SDK/设备版本、设备数量和发现方式。

实测日志示例：

```text
sdk_count=1 discovery=all-devices sdk_version_raw=0x1010000 device_version=dual_app v1.0.0.7
```

后续用户补充官方上位机通信参数截图后确认，当前电机实际使用
`CAN ID=0x01`、`Master ID=0x11`。项目此前沿用了旧 ID `0x08/0x18`，
这是当时 TX 成功但 RX none 的根因。

## 旧版配置复核

用户提供的旧版可运行 UI 配置为：

- Mode: `Damiao USB2FDCAN`
- DM device index: `0`
- DM device type: `1`
- CAN channel: `0`
- Bitrate: `1000000`
- FDCAN data bitrate: `5000000`
- Motor ID: `0x08`
- Master ID: `0x18`
- Max phase A: `20.0`
- Torque/A Nm: `0.625`
- Stroke sign: `1`
- Auto mode switch: enabled
- Adapter FDCAN/BRS: enabled
- Motor frames CANFD: enabled
- Command ID + mode: enabled

该配置与当前 `src/config/default_gripper.yaml` 中的达妙默认配置一致。
`CAN interface=slcan` 是旧 UI 的通用字段；在 `Damiao USB2FDCAN` 模式下，
实际访问路径仍然是达妙 `DM_DeviceSDK`，不是 `python-can slcan`。

本次同时修正了 C++ DeviceSDK 设备发现路径：旧归档已经记录
`dmcan_find_devices_with_type()` 不稳定，旧 UI 使用 `dmcan_find_devices()`。
C++ 当前也改为优先全设备发现，缺少该符号时才回退到 type-filtered discovery。

复核测试：

- C++ `bringup_can_probe`，channel `0`：TX 成功，RX none。
- 旧版 Python 后端按截图配置，channel `0`：TX 成功，RX none。
- 旧版 Python 后端，channel `1`：TX 成功，RX none。

结论：当前无 RX 现象不是 C++ 参数与旧 UI 配置不一致导致的。下一步应优先检查当前硬件现场状态：

- 电机控制电源是否上电。
- CANH/CANL 是否接在当前 USB2FDCAN 物理通道。
- 终端电阻是否正确。
- 电机 ID 是否仍为 `0x08`。
- 当前固件是否响应 `0x7FF` refresh 和 register-read 帧。
- DMTool 或官方上位机是否占用设备。
- 官方上位机在同一接线、同一通道、同一 ID 下是否能看到反馈。

## 官方上位机设备配置复核

用户在 2026-05-20 提供的官方上位机设备配置截图显示：

- USB2CAN 波特率：`1000 Kbps`。
- USB2FDCAN 设备端口：`device:1,3`。
- 工作模式：`FDCAN`。
- 仲裁域波特率：自定义 `1M`。
- 数据域波特率：自定义 `5M`。
- 采样点：`75.0`。

项目当前默认配置与该截图中的 CAN/FDCAN 参数一致：

- `channel_index: 0`
- `fdcan_enabled: true`
- `brs_enabled: true`
- `nominal_bitrate_bps: 1000000`
- `data_bitrate_bps: 5000000`

本次 C++ `bringup_can_probe` 增加了 SDK 通道配置读回，实测日志：

```text
channel_readback=ok rb_ch=0 rb_canfd=1 rb_nominal_bps=1000000 rb_data_bps=5000000 rb_can_sp=0.75 rb_canfd_sp=0.75 rb_fd_details=11/4/4/1
```

结论：

- 当前 C++ 程序已通过 DeviceSDK v1.1.0 成功写入并读回 FDCAN `1M/5M`、采样点 `0.75/0.75`。
- 当时的 `RX none` 不应归因于 FDCAN 波特率、采样点或 SDK DLL 版本不一致。
- 官方工具中的 `device:1,3` 更像是官方工具内部的 libusb 设备端口绑定信息；当前公开 DeviceSDK C 接口仍以 `device_index` 和 `channel_index` 选择设备和 CAN 通道。当前 SDK 枚举结果只有 `sdk_count=1`，因此 `device_index=0` 应对应这一个已插入适配器。
- 若官方上位机在同一接线、同一 ID、同一物理通道下可以收到电机反馈，而 C++ 仍 `RX none`，下一步应重点比较官方上位机发送的实际帧格式/通道绑定，或增加设备枚举详情诊断。

## 电机 ID 修正后验证

用户补充的官方上位机通信参数截图显示：

- `CAN ID=0x01`
- `Master ID=0x11`
- `CAN Timeout=0`
- `CAN Baud=5M`

已更新：

- `src/config/default_gripper.yaml`
- `src/config/gripper_config.hpp`

当前默认值：

```yaml
motor:
  motor_id: 0x01
  host_id: 0x11
```

修正后被动探测成功：

```text
bringup_can_probe | DM_DeviceSDK probe motor_id=0x1 host_id=0x11 channel=0 canfd=1 brs=1 nominal_bps=1000000 data_bps=5000000 sdk_count=1 discovery=all-devices sdk_version_raw=0x1010000 device_version=dual_app v1.0.0.7 tx_count=1 rx_count=1 channel_readback=ok rb_ch=0 rb_canfd=1 rb_nominal_bps=1000000 rb_data_bps=5000000 rb_can_sp=0.75 rb_canfd_sp=0.75 rb_fd_details=11/4/4/1
bringup_can_probe | TX refresh FD id=0x7FF dlc=8 brs=1 data=01 00 cc 00 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 79 b2 7f f7 ea 21 1f
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 79 b2 7f f7 e2 21 1f
bringup_can_probe | TX query_master_id FD id=0x7FF dlc=8 brs=1 data=01 00 33 08 00 00 00 00
bringup_can_probe | RX FD id=0x11 dlc=8 brs=1 data=01 00 33 08 01 00 00 00
bringup_can_probe: Ok
```

`bringup_feedback` 也已成功：

```text
bringup_feedback: Ok | stroke_mm=-0.195923 | motor_pos_rad=-0.615511 | motor_vel_rad_s=-0.00732601 | motor_current_a=-0.019536 | motor_torque_nm=-0.01221 | temp_c=33.000 | enabled=false
```

结论：当前 CAN/FDCAN 接收链路已跑通。进入使能或点动前，仍应保持低能量
bring-up 流程，并确保机构处于机械安全状态。
