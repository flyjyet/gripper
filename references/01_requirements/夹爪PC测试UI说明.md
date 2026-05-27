# 夹爪 PC 测试 UI 说明

日期：2026-05-14

## 1. 目的

`gripper_control_ui.py` 用于 PC 调试夹爪控制流程。当前版本支持三类后端：

- `Simulation`：仿真电机，用于无硬件验证状态机、限流限速、日志和参数。
- `Damiao CAN`：基于 DM-J4310P-2EC CAN 协议和 `python-can` 的真实电机测试后端。
- `Damiao USB2FDCAN`：基于 `assets\dm-tools\DM_DeviceSDK` 最新 DeviceSDK C 接口的真实电机测试后端，支持单通道 `DM-USB2FDCAN` 和双通道 `DM-USB2FDCAN_Dual`。

控制重点仍是全行程限速限流、低速低电流回零、摩擦识别、目标力夹紧、接触/堵转检测、夹紧后卸载并失能、打开前低电流解锁。

## 2. 运行方式

仿真后端不需要额外依赖：

```powershell
.\.venv\Scripts\python 需求讨论\gripper_control_ui.py
```

真实 CAN 后端需要安装 `python-can`，并安装对应 USB-CAN 适配器驱动：

```powershell
.\.venv\Scripts\python -m pip install python-can
```

当前 `.venv` 已安装 `python-can 4.6.1`。不同 CAN 适配器的 `interface/channel` 填法不同，需要按实际硬件驱动确认。

`Damiao USB2FDCAN` 后端不走 `python-can`，而是直接加载达妙最新 DeviceSDK DLL。需要保证运行目录下存在以下 DLL：

```powershell
Copy-Item "assets\dm-tools\DM_DeviceSDK\C&C++\lib\mingw\libdm_device.dll" "dlls\libdm_device.dll"
```

当前工程已经把 `libdm_device.dll`、`libusb-1.0.dll`、`libstdc++-6.dll`、`libgcc_s_seh-1.dll`、`libwinpthread-1.dll` 放在 `dlls` 目录，打包时也会复制到可执行程序目录。设备驱动仍需按达妙工具说明安装。

如果 UI 报 `Found DM-USB2FDCAN, but failed to open it`，说明系统已经能枚举到适配器，但 SDK 没有拿到设备句柄。优先检查：关闭 DMTool 等占用设备的软件；确认设备仍为 DeviceSDK/WinUSB 驱动可访问的固件和驱动；按达妙说明重新安装 WinUSB/UsbDk 类驱动。

## 3. 后端参数

`Backend` 面板用于选择后端和 CAN 参数：

| 参数 | 说明 |
|---|---|
| Mode | `Simulation`、`Damiao CAN` 或 `Damiao USB2FDCAN` |
| DM device index | 达妙 USB2FDCAN 设备序号，通常填 `0` |
| DM device type | `0` 表示单通道 `DEV_USB2CANFD`，`1` 表示双通道 `DEV_USB2CANFD_DUAL` |
| CAN interface | `python-can` 接口名，例如 `pcan`、`slcan`、`socketcan` |
| CAN channel | `Damiao CAN` 时为 `python-can` 通道；`Damiao USB2FDCAN` 时填 `0` 或 `1` |
| Bitrate | 默认 `1000000` |
| FDCAN data bitrate | FDCAN 数据域波特率，当前填 `5000000` |
| SLCAN tty baud | `slcan` 串口层波特率，DM-USB2FDCAN 当前填 `921600` |
| Motor ID | 电机接收 ID，默认 `1` |
| Master ID | 电机反馈帧 ID，默认 `0` |
| Max phase A | 驱动最大相电流，用于 PVT 限流归一化 |
| Torque/A Nm | 输出端力矩/相电流估算，用于 MIT 力矩命令 |
| Stroke sign | 行程方向，闭合方向为正时填 `1`，反向填 `-1` |
| Auto mode switch | 自动写 `0x0A` 控制模式寄存器 |
| Use CANFD/BRS | 使用 FDCAN 帧和数据域加速，当前设备配置打开 |

当前这只 `DM-USB2FDCAN` 在 PC 上位机已验证可用的配置为：

| UI 参数 | 填写值 |
|---|---|
| Mode | 优先用 `Damiao USB2FDCAN`；若设备刷为 SLCAN 固件，再用 `Damiao CAN` |
| DM device type | `0` |
| CAN channel | `0`；SLCAN 备用路径为实际串口号，例如 `COM4` |
| Bitrate | `1000000` |
| FDCAN data bitrate | `5000000` |
| Use CANFD/BRS | 勾选 |
| SLCAN tty baud | `921600` |

设备自身配置为 FDCAN 模式，仲裁域 `1Mbps`，数据域 `5Mbps`。使用 DeviceSDK 后端时，关键参数是 `device type=0`、`channel=0`、FDCAN `1Mbps/5Mbps`。只有走 SLCAN 备用路径时才需要填写 `CAN interface=slcan`、`CAN channel=COMx`、`SLCAN tty baud=921600`。

协议层已参考 `assets\openarm_can` 中可用的达妙电机控制包对齐：

- DM4310 MIT/反馈映射范围使用 `p=12.5 rad`、`v=30 rad/s`、`t=10 Nm`。
- 控制模式 `4` 按 `POS_FORCE` 处理，对应发送 ID `0x300 + Motor ID`。
- FDCAN 路径发送帧启用 CANFD/BRS，与 `openarm_can` 的 SocketCAN 发送行为一致。

`assets\openarm_can` 的设备层是 Linux SocketCAN 实现；当前 Windows UI 仍使用 `DM_DeviceSDK` 或 `python-can/slcan` 打开硬件，只复用其达妙协议编码规则。

## 4. 安全使用顺序

1. 先用 `Simulation` 熟悉状态机和参数。
2. 真实电机接入后，先把 `Max current` 设为 `0.35 A` 到 `0.5 A`，`Close speed` 设为 `0.1-0.2 mm/s`。
3. 点击 `Connect backend`，再执行 `Home low-current`。
4. 空载确认行程方向正确；如果行程反向，修改 `Stroke sign`。
5. 做低电流夹紧测试，确认接触检测和卸载失能。
6. 逐步提高目标力和电流，建议真实样机初始不超过 `2 A`。

真实 CAN 后端在 `Max current > 2 A` 时会弹窗二次确认，避免误操作导致丝杆螺母卡死。

## 5. 当前限制

- 目标力到电流的映射仍需要样机标定。
- `Damiao CAN` 适配器依赖 `python-can` 支持；`Damiao USB2FDCAN` 使用达妙最新 DeviceSDK DLL，不依赖 `dmcan-sdk`。
- UI 里的 `reset_zero()` 是软件零点，不写入电机 flash，不会发送“保存位置零点”命令。
- 该 UI 面向测试调参，不作为最终产品界面。
