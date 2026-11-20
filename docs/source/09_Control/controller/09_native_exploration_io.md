# 探索局部控制的原生输入与命令校验

`ControllerServer` 新增接收现有共享 Costmap 与 `NativeControllerIO` 的
构造接口，供 RGB-D 探索平台装配已有 PolicyController、NMPC/MPPI 和
FollowPath/恢复动作服务。它没有增加新的消息定义、优化器或样条算法。

## 状态与消息

原生平台通过 `read_odometry` 提供实际 Atlas 健康状态对应的校准底盘
Odometry。ControllerServer 校验顶层/嵌套 Header、地图帧、底盘帧、原始
采集时间和有限位姿/速度。失效输入返回无反馈，不回退到全局 TF，不
使用零填充速度来伪装测量。原生控制器 I/O 的状态年龄上限由平台反馈窗口
配置并限制在 1 秒内。短暂缺失新位姿时 ControllerServer 向同一原生出口
发送零速度并等待下一条采集证据；超过有界窗口才终止 FollowPath。

原生模式不订阅全局 `/odom`、`/map` 或默认未配置的相机/点云/BEV
通道。Policy 输入只订阅显式配置的通道。完整深度消息可经过平台的
独立光学帧别名转发，该别名使用实际相机安装链；它不改变深度数值、
分辨率或内参。几何建图的射线步长不改变控制器获得的完整深度。

原生 `publish_command` 接收已有 TwistStamped，并负责独立校验与指定
基座通道的实际写入。拒绝或异常不会回退到全局 cmd_vel；非零提议
被拒绝时通过同一个出口请求零速度，并报告控制失败。零速度写入成功
是传输结果，不是物理停止回执。

## 地图命令校验

NativeRgbdPlatform 使用 `VerifyNativeCommand` 作为命令出口的独立地图
校验。它检查命令/反馈/地图的坐标身份和年龄、差分驱动平面速度范围、
有效姿态，以及实际地图提供的高度证据。地图或反馈缺失时不能输出
非零命令。

校验复用 GeometrySafety 的完整圆形包络、地面台阶和垂直净空检查。
水平包络增加以下保守停止余量：

```text
d = speed × (actuator_watchdog + reaction_time)
    + speed² / (2 × minimum_braking_deceleration)
```

speed 取实际水平反馈速度和提议前向速度的较大值。停止余量沿当前命令
方向展开：先校验当前位姿的完整圆形足迹，再校验到停止终点的连续线段。
线段检查包含路径上的每个中间足迹，未知格子及地图外部仍拒绝运动；
同时允许机器人从侧向近墙位置沿安全方向离开，不会把制动余量错误扩张
到所有方向而永久冻结。watchdog、反应时间和制动能力需要来自真实平台
配置与测量；代码本身不能证明硬件达到了所填的参数。

PolicyController 输入的两点或多点路径按弧长均匀采样到模型要求的固定
点数，不再把首尾点各复制成一段。探索视点采用“按行进方向到达、再原地
旋转到观察航向”两段式执行；原地旋转复用 `/spin`，并校验累计转角和最终
航向。这样避免差分底盘在路径起点或终点执行不可跟踪的大角度掉头。

这是保守的 2.5D 地图模型校验。它不替代 SanD 的严格三维参考验证、
执行 lease/停止 authority，也不证明感知不到的动态或亚体素障碍不存在。
这些独立验证仍由现有 execution_esdf/local_ground/trajectory_tracker
及实际基座接入承担，完整探索验收需要对应的真实运行证据。

## 现有 ControllerOptions 配置

`native_rgbd_config` 可增加 `controller_options_file`，读取真实已有
ControllerOptions 的 JSON 表示。请求的 controller id 必须出现在该
消息的 controller_plugins 中。没有该文件时平台不创建控制服务；
插件或模型失败时也不会静默切换其他控制器。

```yaml
controller_options_file: actual-controller-options.json
command_topic: /validation/exploration/run/cmd_vel
controller_image_topic: /validation/exploration/run/native/control_depth
controller_camera_frame: native_exploration_camera
maximum_linear_speed: 0.5
maximum_yaw_rate: 1.0
actuator_watchdog: 0.5
reaction_time: 0.1
minimum_braking_deceleration: 1.2
```

SanD 侧继续使用现有 PolicyConfig、完整控制点和公共 common/math
B 样条求值。`sand_tracker: "MPPIController"` 或 `"NMPCController"`
调用远端原有控制实现。MPPI 的 model_dt 必须匹配实际控制周期；例如
controller_frequency 为 10 Hz 时，rollout model_dt 应设为 0.1 秒，
不能使用与此周期冲突的 0.05 秒配置。

当前已存在的 SanD ONNX 权重以原始四帧深度为输入；它不能通过重命名
或 reshape 变成 DeFM token 条件模型。原始深度版本的控制接入验证与
DeFM 条件版本的权重/接口验证是独立结果。

## 生命周期与验证

平台先创建定位读者、命令/完整深度出口和控制服务，再发布原生任务
配置，防止任务 owner 的 mapper 回调与资源初始化竞争。关停撤销非零
命令许可，由独立 ReaderRetirement 清理本次提供者和深度转发 Reader，
再关闭原有 ControllerServer。

v2.70/v2.71 的原生测试覆盖实际状态年龄与身份、无全局回退、命令
拒绝、原生注册，以及扩大停止包络中的障碍/未知高度拒绝。SpaceHero
原生回归 38/38 通过。真实控制运行的模型配置、原始日志与整体探索
状态见 Tasks 下的 native exploration 验证文档；这些接口测试不能单独
证明机器人已运动或探索完成。
