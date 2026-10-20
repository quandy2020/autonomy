(policy-controller)=
# 7. Policy Controller

源码：`autonomy/control/controller/policy_controller/`。学习型局部控制器：加载满足 I/O 契约的 ONNX / TensorRT 策略图，在线推理 `Twist`（默认差速 2DOF，可配 6DOF）。

不绑定具体 RL 框架；推理走 `autonomy::common::network::Engine`。导出脚手架：`tools/policy/`。

### 最小观测 / 动作

| 方向 | 默认名 | 形状含义 |
|------|--------|----------|
| 输入 | `observation_kinematic` | `[1,6]` 位姿速度 |
| 输入 | `observation_path` | `[1,40]` 机器人系路径点 |
| 输出 | `action` | `[vx, wz]`（可扩展） |

还可编码激光、costmap、图像、点云（见 `observation/`）。

### 配置

1. 写 `PolicyConfig` sidecar（例：`conf/policy_config_example.pb.txt`）
2. `ControllerOptions`：

```text
controller_plugins: "policy_controller"
policy_controller_options {
    config_path: "/path/to/policy_config.pb.txt"
    path_points: 20
    collision_check: true
    hold_on_failure: true
}
```

安全：`action/safety_filter` 限幅与碰撞 hold。

Proto：`autonomy/control/proto/policy_controller.proto`。
