# 源码注释补强任务清单

## 1. 目的

当前第一轮代码已经能构建和运行，但源码实现层注释仍不足以支撑轻松的人工审查。后续应按本清单补充关键注释，避免审查人员只能通过逐行读代码理解控制意图。

## 2. 优先级 P0

以下文件必须优先补注释：

- `src/controller/gripper_controller.cpp`
  - 补充默认控制器职责说明。
  - 补充 `runPreSelfCheck()` 流程阶段注释。
  - 补充 `homeOpenStop()` 回零判据注释。
  - 补充 `learnTravelLimits()` 行程学习边界注释。
  - 补充 `runMotionHealthCheck()` 健康检查反馈判据注释。
  - 补充 `clampByForce()` 和 `clampBySpeed()` 的接触/力代理说明。
  - 补充动作完成后失能的原因。
- `src/controller/state_machine/gripper_state_machine.hpp`
  - 为每个顶层状态补业务含义。
  - 为关键事件补触发条件。
- `src/controller/safety/safety_limiter.hpp`
  - 说明限流、限速、限加速度、行程限制各自是裁剪还是主动停止。
- `src/controller/safety/contact_jam_detector.hpp`
  - 说明无外部传感器时如何使用电机反馈作为接触/堵转代理。
- `src/hardware_interface/damiao/damiao_motor.hpp`
  - 明确当前是占位实现，禁止用于真实硬件联调。
- `src/hardware_interface/damiao/dm_usb2fdcan_transport.hpp`
  - 明确真实 SDK 接入前不会打开设备。

P0 当前状态：已完成首轮补强，并已通过 `gripper_app` 构建和 CTest 验证。后续如修改上述模块，应继续按本清单检查注释是否仍能支撑人工审查。

## 3. 优先级 P1

- `src/controller/self_check/*`
  - 为各识别器补输入样本含义和质量等级说明。
- `src/controller/mechanism/gripper_kinematics.hpp`
  - 说明当前是一阶近似，不是完整非线性机构模型。
- `src/controller/calibration/force_mapper.hpp`
  - 说明目标力到电机电流/力矩映射需要后续样机标定。
- `src/config/gripper_config.hpp`
  - 补充关键配置字段来源和保守默认值含义。
- `src/hardware_interface/motor_interface.hpp`
  - 补充电机抽象接口的安全约束。

## 4. 注释验收标准

补注释后，人工审查者应能不打开实现细节就回答：

- 这个模块负责什么？
- 这个模块不负责什么？
- 关键输入输出单位是什么？
- 参数来自配置、学习结果还是硬件反馈？
- 失败后进入什么状态？
- 是否会继续输出力矩或速度？
- 是否可用于真实硬件联调？
