(policy-controller)=
# 7. Policy Controller

加载符合 I/O 契约的 **ONNX / TensorRT** 策略，在线推理 `Twist`（默认差速 2DOF，可配 6DOF）。不绑定特定 RL 框架；引擎 `autonomy::common::network::Engine`。源码：`policy_controller/`。

加载符合 sidecar 契约的 ONNX/TensorRT，把观测张量映射为 `Twist`（默认差速 2DOF）。已有自训策略、维度和归一化与 `PolicyConfig` 一致时用；比 NeuPAN 更通用但不绑定特定 RL 框架。

## 何时用

已有自训策略、观测与动作维度和 sidecar 一致；需要比 NeuPAN 更通用的「多模态 obs → action」。无 `PolicyConfig` 或未导出图时不要启用。

## 默认 I/O 契约

| 方向 | 名 | 形状（示例） |
|------|-----|--------------|
| 输入 | `observation_kinematic` | `[1,6]` 位姿速度 |
| 输入 | `observation_path` | `[1,40]` 机体系路径点 |
| 输出 | `action` | `[vx, wz]` |

还可配置激光、costmap、图像、点云等（见 `observation/`）。

## 配置步骤

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

`action/safety_filter` 限幅与碰撞 hold。

## 与 Server 的交互

Policy 推理与 costmap crop 可能 tens of ms；Server **不** 在整个推理期间持 costmap 全局锁（与其它控制器不同）。

## 常见坑

| 现象 | 处理 |
|------|------|
| hold 不动 | `hold_on_failure` + 碰撞检查失败 |
| 观测维错 | sidecar 与导出图不一致 |
| 诊断无输出 | `PublishPolicyDiagnostics` 话题订阅 |

Proto：`autonomy/control/proto/policy_controller.proto`。导出脚手架：`tools/policy/`。

归属 [04](../04_controller_algorithms.md)。
