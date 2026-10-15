# Policy Controller

通用 **学习型局部控制器**：加载满足 I/O 契约的深度策略导出图（DRL / IL / Diffusion 等），在线推理 `geometry_msgs/Twist`（默认 2DOF 差速 `linear_x, angular_z`，也可配置完整 6DOF）。

- 不绑定 PPO / SAC / TD3 / 具体 diffusion 训练框架
- 推理仅通过 [`autonomy::common::network::Engine`](../../../common/network/README.md)（ONNX / TensorRT）
- 导出 / 契约校验脚手架：[`tools/policy/`](../../../../tools/policy/)

Proto 定义：[`autonomy/control/proto/policy_controller.proto`](../../proto/policy_controller.proto)

---

## 快速开始

### 1. 准备模型

导出与 sidecar 中 tensor 名一致的 ONNX（或 TensorRT `.engine`）。最小 Twist 策略示例：

| 方向 | 默认名 | 含义 |
|------|--------|------|
| 输入 | `observation_kinematic` | `[1, 6]` → `x,y,yaw,vx,vy,wz` |
| 输入 | `observation_path` | `[1, 40]` → 20 个机器人系 `(x,y)` |
| 输出 | `action` | 默认 `[linear_velocity_x, angular_velocity_z]`；可配 6DOF |

### 2. 写策略 sidecar（`PolicyConfig`）

最小可用示例见 [`conf/policy_config_example.pb.txt`](../../conf/policy_config_example.pb.txt)：

```text
name: "example_twist_policy"
version: "0.1"

observation {
  kinematic: true
  path: true
}

action {
  limits {
    max_vx: 0.5
    max_wz: 1.0
    max_ax: 1.0
    max_alpha_z: 2.0
  }
}

engine {
  model_path: "/path/to/policy.onnx"
}
```

默认（可省略）：`action.type = TWIST`，`name = "action"`，layout = `linear_velocity_x` / `angular_velocity_z`，`backend = "onnx"`。6DOF 示例见 [`conf/policy_config_twist6_example.pb.txt`](../../conf/policy_config_twist6_example.pb.txt)。

### 3. 在 ControllerOptions 中启用插件

```text
controller_plugins: "policy_controller"

controller_frequency: 20.0

policy_controller_options {
  config_path: "/path/to/policy_config.pb.txt"

  path_points: 20
  laser_bins: 180
  costmap_cells: 64

  collision_check: true
  collision_distance: 0.5
  hold_on_failure: true

  goal_xy_tolerance: 0.25
  goal_yaw_tolerance: 0.25
}
```

启动 `ControllerServer` 后，控制循环会调用 `ComputeVelocityCommands`；`SetPlan` 写入路径。Server 默认转发：

| Topic（可被 `*_topic` 覆盖） | 方法 |
|------------------------------|------|
| `/scan` | `SetLaserScan` |
| `/camera/image_raw` | `SetImage` |
| `/points` | `SetPointCloud` |
| `/perception/lidar_bev` | `SetLidarBevArray`（Float32MultiArray） |
| `/perception/policy_raw` | `SetRawTensorArray` |
| `/policy_controller/reload_config` | `LoadConfigAsync`（`std_msgs/String` 路径） |
| `/policy_controller/diagnostics` | `DiagnosticArray` 导出 |

热切换：`LoadConfig` / `LoadConfigAsync`；运维可向 reload topic 发 sidecar 路径。可观测性：内存 `diagnostics()` + 上述 diagnostics topic。

---

## 架构与数据流

```
SetPlan / SetLaserScan / SetImage
pose / velocity / costmap (每拍)
        │
        ▼
 ObservationBuffer  ──Snapshot()──► ObservationSnapshot
        │
        ▼
 PredictTwist（EncodeObservations → infer → Decode）
        │
        ▼
 CommandSafetyFilter（限幅 / 斜率 / 可选碰撞）
        │
        ▼
 TwistStamped
```

| 类型 | 职责 |
|------|------|
| `PolicyController` | `ControllerInterface` 插件入口 |
| `PolicyPipeline` | buffer 更新 → `PredictTwist` → safety |
| `PredictTwist` | 可离线复用的 encode → infer → decode |
| `PolicyConfig` | 策略 sidecar（观测 / 动作 / 模型） |
| `PolicyManager` / `PolicyModel` | 加载、校验、双槽热切换 |
| `CommandSafetyFilter` | 指令后处理与失败降级 |

---

## 配置分层

两层配置，职责分开：

| 层级 | 消息 | 作用 |
|------|------|------|
| **策略 sidecar** | `PolicyConfig` | 与权重绑定：观测组合、动作头、模型路径、diffusion |
| **插件运行参数** | `PolicyControllerOptions` | 控制器侧：config 路径、path 点数、碰撞、目标容差 |

- sidecar 可放文件（`config_path`），或嵌在 `config` 里
- **二者勿混用观测开关语义**：模态开关写在 sidecar 的 `observation {}`；裁剪/采样长度写在 `policy_controller_options`

---

## PolicyConfig（策略 sidecar）详解

### 元信息

| 字段 | 说明 |
|------|------|
| `name` / `version` | 策略标识（日志 / 热切换） |
| `mode` | `INFERENCE_SINGLE_SHOT`（默认）或 `INFERENCE_DIFFUSION` |

### observation：模态开关

```text
observation {
  kinematic: true      # 启用
  path: true
  costmap: false       # 关闭（也可省略，proto3 默认 false）
  laser: false
  image: false
}
```

加载时展开为内部 `inputs[]`。**至少启用一个模态**，否则加载失败。

#### 默认 tensor 名与 shape

| 开关 | 默认 `name` | 默认 shape | 默认内容 |
|------|-------------|------------|----------|
| `kinematic` | `observation_kinematic` | `[1, 6]` | `x,y,yaw,vx,vy,wz` |
| `path` | `observation_path` | `[1, 40]` | 20×`(x,y)` 机器人系 |
| `costmap` | `observation_costmap` | `[1, 1, 64, 64]` | 局部 crop，`[0,1]` |
| `laser` | `observation_laser` | `[1, 180]` | range / range_max |
| `image` | `observation_image` | `[1, 3, 224, 224]` | NCHW，默认 letterbox |
| `point_cloud` | `observation_point_cloud` | `[1, 2048, 3]` | PointCloud2 → xyz 轻量打包 |
| `lidar_bev` | `observation_lidar_bev` | `[1, 1, 200, 200]` | 预计算 BEV（`SetLidarBev`） |
| `raw` | `observation_raw` | `[1, 1]` | 预计算张量（`SetRawTensor`，可多路用 `inputs[]`） |

`path` 实际采样点数由运行参数 `path_points` 决定（默认 20 → 特征长度 40）。`costmap` / `laser` / `point_cloud` 同理由 `costmap_cells` / `laser_bins` / `point_cloud_points` 影响编码。

**原则**：体素 / 完整 BEVFusion 等重预处理放进 ONNX；C++ 只做轻量规整与转发。

#### 覆盖默认（`*_input`）

仅在对应开关为 `true` 时生效：

```text
observation {
  kinematic: true
  path: true
  image: true

  image_input {
    name: "obs_rgb"
    preprocess: "letterbox"   # letterbox | stretch | center_crop
    required: false
    # shape: 1
    # shape: 3
    # shape: 224
    # shape: 224
    # norm { mean: ... stddev: ... }
  }

  path_input {
    required: true
  }
}
```

| `ObservationInput` 字段 | 说明 |
|----------------------|------|
| `name` | 与 ONNX 输入名一致；空则用上表默认 |
| `shape` | 期望 shape；`<=0` 表示一个动态轴 |
| `required` | 未写默认 `true`；缺数据时本拍失败并走安全降级 |
| `norm` | 可选 mean / stddev |
| `preprocess` | 仅 image |

#### 高级：直接写 `inputs[]`

与 `observation {}` **二选一**，不可混用：

```text
inputs {
  name: "observation_kinematic"
  modality: OBSERVATION_KINEMATIC
  shape: 1
  shape: 6
  required: true
}
```

### action：动作头（已统一，无需再写 `outputs`）

默认 **2DOF 差速**（可省略 layout）：

```text
action {
  limits {
    max_vx: 0.5
    max_wz: 1.0
    max_ax: 1.0
    max_alpha_z: 2.0
  }
}
```

完整 **6DOF Twist**（全向 / 飞行器等）：

```text
action {
  layout: "linear_velocity_x"
  layout: "linear_velocity_y"
  layout: "linear_velocity_z"
  layout: "angular_velocity_x"
  layout: "angular_velocity_y"
  layout: "angular_velocity_z"

  limits {
    max_vx: 0.5
    max_vy: 0.5
    max_vz: 0.5
    max_wx: 1.0
    max_wy: 1.0
    max_wz: 1.0
    max_ax: 1.0
    max_ay: 1.0
    max_az: 1.0
    max_alpha_x: 2.0
    max_alpha_y: 2.0
    max_alpha_z: 2.0
  }
}
```

也可只声明需要的子集（例如平面全向 `lx, ly, az`）。

| 字段 | 默认 | 说明 |
|------|------|------|
| `type` | `ACTION_SPACE_TWIST` | 另有 `DIFFUSION_TWIST` |
| `name` | `"action"` | 引擎输出名 |
| `layout` | `lx, az`（2DOF） | 见下表槽位名 / 别名 |
| `scale` | `1.0` | 解码前乘到对应槽 |
| `limits` | `lx`/`az` 有默认上限 | 其它轴 `max<=0` 表示不限幅 |

**layout 槽位名（及别名）**

| Twist 分量 | 推荐名 | 别名 |
|------------|--------|------|
| `linear.x` | `linear_velocity_x` | `linear_x`, `vx`, `v` |
| `linear.y` | `linear_velocity_y` | `linear_y`, `vy` |
| `linear.z` | `linear_velocity_z` | `linear_z`, `vz` |
| `angular.x` | `angular_velocity_x` | `angular_x`, `wx`, `roll_rate` |
| `angular.y` | `angular_velocity_y` | `angular_y`, `wy`, `pitch_rate` |
| `angular.z` | `angular_velocity_z` | `angular_z`, `wz`, `w`, `omega`, `yaw_rate` |

多头输出高级用法仍可用 `outputs[]` + `action.output_index`（一般不需要）。

### engine：后端与模型路径

```text
engine {
  backend: "onnx"      # 或 "tensorrt"；空 → onnx
  model_path: "/path/to/policy.onnx"
  provider: "cpu"      # "" | "cpu" | "cuda"
  device: 0
  io_binding: false
}
```

### diffusion（可选）

仅当 `mode: INFERENCE_DIFFUSION`（或等价检测）时使用。示例：[`conf/policy_config_diffusion_example.pb.txt`](../../conf/policy_config_diffusion_example.pb.txt)

```text
mode: INFERENCE_DIFFUSION

diffusion {
  steps: 8
  horizon: 8
  dim: 2
  sample: "sample"                 # 默认 sample
  timestep: "timestep"             # 默认 timestep
  prediction: "action"             # 空则用 action.name
  prediction_type: DIFFUSION_PRED_SAMPLE  # 或 EPSILON
  execute_index: 0
  ddim: true
  ddim_eta: 0.0
  timestep_scale: 1.0    # 训练 T=1000 时可设 1000
}
```

导出 **单步 denoiser**（每去噪步一次 `Engine::Run`）：

| 方向 | Tensor | 含义 |
|------|--------|------|
| 输入 | `observation_*` | 与 single-shot 相同条件 |
| 输入 | `sample` | 当前噪声 / 细化中的动作序列 |
| 输入 | `timestep` | 标量时间步 |
| 输出 | `action` 或 `noise` | SAMPLE(x0) 或 EPSILON |

每控制拍循环 `steps` 次，再取 `execute_index` 映射为 Twist。

---

## PolicyControllerOptions（插件运行参数）

写在 `ControllerOptions.policy_controller_options`：

| 字段 | 默认/建议 | 说明 |
|------|-----------|------|
| `config_path` | — | sidecar 文件路径；空则用 `config` |
| `config` | — | 内嵌完整 `PolicyConfig` |
| `path_points` | 20 | 路径采样点数（→ path 特征长度 ×2） |
| `laser_bins` | 180 | 激光降采样长度 |
| `point_cloud_points` | 2048 | 点云 xyz 打包上限 |
| `costmap_cells` | 64 | 局部地图边长（格） |
| `max_obs_age` | 0 | >0 时校验相对主 stamp 的新鲜度 |
| `max_obs_skew` | 0 | >0 时校验 \|模态 stamp − pose stamp\| |
| `hold_on_failure` | true | 失败时保持上次指令；false 则零速 |
| `infer_timeout` | 0 | >0 时 encode+infer 墙钟超时强制零速（单次 forward 用 `wait_for`；不 hold） |
| `transform_tolerance` | 0.1 | path TF 对齐超时 |
| `robot_frame` | base_link | 传感器期望帧 |
| `require_frame_match` | false | true 时传感器 `frame_id` 必须非空且等于 `robot_frame`（C++ 不做激光/图/点云 TF） |
| `reload_config_allow_prefix` | [] | 热加载路径前缀白名单；空=不限制 |
| `require_tensor_stamp` | false | BEV/RAW 接收时写入 wall-clock stamp；true 时要求 stamp>0 |
| `require_model` | false | Activate 时无模型则抛异常 |
| `max_consecutive_failures` | 0 | >0 连续失败达阈值后强制零速 |
| `collision_check` | false | 沿运动方向碰撞硬刹 |
| `use_footprint_collision` | false | true 时用 costmap footprint；否则单点 |
| `collision_cost_threshold` | 253 | footprint/点代价阈值 |
| `collision_distance` | 0.5 | 探测距离 (m) |
| `relative_kinematic` | false | kinematic 相对 path 终点 |
| `point_cloud_stride_sample` | false | 点云按 stride 下采样 |
| `max_diffusion_steps` | 0 | >0 时运行时截断 denoising 步数 |
| `missing_modality_grace` | 0 | >0 时短暂丢模态走 `missing_modality_policy` |
| `missing_modality_policy` | FAIL | `FAIL` / `ZERO_FILL` / `HOLD` |
| `near_goal_distance` | 0 | >0 时近目标额外乘 `near_goal_speed_scale` |
| `near_goal_speed_scale` | 0.3 | 近目标速度比例 |
| `goal_xy_tolerance` | 0.25 | `IsGoalReached` 位置容差 |
| `goal_yaw_tolerance` | 0.25 | `IsGoalReached` 航向容差 |

### 帧契约（训练 / 部署）

1. Path 与 ego pose 经 TF 对齐到同一 frame，再编码为机器人系相对 `(x,y)`。
2. Laser / Image / PointCloud：`require_frame_match=true` 时 `frame_id` 必须非空且等于 `robot_frame`（发布端已变换；C++ 不对该模态做 TF）。
3. Kinematic 使用 pose-frame 绝对 `x,y,yaw` + twist；训练地图原点需稳定。
4. BEV/RAW `Float32MultiArray` 无 header：接收时写入 wall-clock `stamp_sec`，以便 `max_obs_age` / `max_obs_skew` 生效。

### 内嵌 sidecar（无单独文件）

```text
policy_controller_options {
  config {
    name: "inline_policy"
    observation { kinematic: true path: true }
    action {
      limits {
        max_vx: 0.5
        max_wz: 1.0
      }
    }
    engine { model_path: "/path/to/policy.onnx" }
  }
  path_points: 20
}
```

---

## 使用方式

### 观测从哪里来

| 模态 | 注入方式 |
|------|----------|
| pose / velocity | 每拍 `ComputeVelocityCommands` 参数 |
| path | `SetPlan`（仅规划更新时写入，热路径不重复拷贝） |
| costmap | 每拍从 `Costmap2DWrapper` 裁剪 |
| laser | `ControllerServer` 转发 `/scan` → `SetLaserScan` |
| image | `SetImage` |
| point_cloud | `SetPointCloud`（xyz 打包） |
| lidar_bev | `SetLidarBev`（预计算 `TensorFrame`） |
| raw | `SetRawTensor`（`name` 对齐 `ObservationInput.name`） |

离线 / 单测可直接调用 `PredictTwist`（见 `runtime.hpp`），无需走 `ControllerInterface`。

### 完整单步配置示例

```text
# ControllerOptions 片段
controller_plugins: "policy_controller"
controller_frequency: 20.0

policy_controller_options {
  config_path: "/opt/autonomy/conf/policy_config.pb.txt"
  path_points: 20
  laser_bins: 180
  costmap_cells: 64
  collision_check: true
  collision_distance: 0.5
  hold_on_failure: true
}
```

```text
# policy_config.pb.txt
name: "nav_twist_v1"
version: "1.0"

observation {
  kinematic: true
  path: true
  costmap: true
  laser: true
  image: false
}

action {
  limits {
    max_vx: 0.6
    max_wz: 1.2
    max_ax: 1.0
    max_alpha_z: 2.5
  }
}

engine {
  backend: "onnx"
  model_path: "/opt/models/nav_twist_v1.onnx"
  provider: "cuda"
  device: 0
}
```

### 运行时热切换

```cpp
std::string error;
if (!policy_controller->LoadConfig("/path/to/other_config.pb.txt", &error)) {
  // 加载失败：当前策略保持不变
  AERROR << error;
}
// 成功：PolicyManager 双槽切换，下一拍使用新模型
```

也可传入内存中的 `proto::PolicyConfig`：

```cpp
policy_controller->LoadConfig(proto_config, &error);
```

### 限速

`SetSpeedLimit` 会换算为 safety filter 的 `speed_scale`：

- `percentage = true`：`speed_limit` 为相对最大线速度的百分比（0–100）
- `percentage = false`：绝对线速度上限，按 `max(max_vx, max_vy, max_vz)` 归一化（6DOF 友好）

近目标：`near_goal_distance > 0` 时再乘 `near_goal_speed_scale`。

---

## 契约校验

### 加载时（C++）

`PolicyModel::Create` 会对引擎做硬校验：

1. 每个 `required` 输入名存在于 `Engine::GetInputInfos()`，shape 兼容（允许一个动态轴）
2. 动作 / 预测输出存在，且长度 ≥ layout（或 `horizon × dim`）
3. Diffusion：引擎须含 `sample` / `timestep`（或自定义名）及预测输出

### 部署前（Python）

```bash
python3 tools/policy/validate_policy_contract.py --checklist
python3 tools/policy/validate_policy_contract.py \
  --config autonomy/control/conf/policy_config_example.pb.txt \
  --onnx /path/to/policy.onnx
```

### 导出契约清单（给算法同学）

| 项 | 要求 |
|----|------|
| dtype | float32 |
| 布局 | Image 为 NCHW；其余按 sidecar `shape` |
| 输入名 | 与 `observation_*` / `inputs[].name` 一致 |
| 输出名 | 默认 `action`；Diffusion 可为 `prediction` |
| Twist layout | 默认 `linear_velocity_x, angular_velocity_z`；6DOF 列出全部轴 |
| 归一化 | 训练侧与 `TensorNorm` 一致，或导出图内完成 |
| sidecar | `engine.model_path` 指向导出文件 |

Toy 冒烟：`tools/policy/export_policy_onnx.py --toy` → `conf/models/policy_toy.onnx`。

「支持任意模型」= **任意满足上述契约的导出图**，运行时不会猜测 head。

---

## 常见问题

| 现象 | 排查 |
|------|------|
| 配置后无指令 / 一直零速 | 看日志：required 模态缺失？模型路径无效？失败降级策略？ |
| `observation has no enabled modality` | 至少设一个 `kinematic/path/...: true` |
| `Use either observation or inputs` | 不要同时写 `observation {}` 与 `inputs {}` |
| shape mismatch | sidecar shape 与 ONNX 不符；或 `path_points` 与训练不一致 |
| 激光无效 | 确认 server 转发 `/scan`，且 `laser: true` |
| Diffusion 慢 | 降低 `steps`；确认导出的是单步 denoiser |

---

## 目录

```text
policy_controller/
  controller.*                 # ControllerInterface 插件
  pipeline.hpp                 # buffer → PredictTwist → safety
  runtime.*                    # encode → infer → decode（可离线复用）
  config.*                     # PolicyConfig 解析 / 校验
  model.*                      # Engine + config
  manager.*                    # 双槽热切换
  diffusion.*                  # 多步 denoising
  types.hpp
  observation/
    buffer.*
    encoder.*
    kinematic_encoder.*
    path_encoder.*
    costmap_encoder.*
    laser_scan_encoder.*
    image_encoder.*
    point_cloud_encoder.*
    tensor_frame_encoder.*     # RAW / lidar BEV
  action/
    space.hpp
    schema.cpp
    safety_filter.*
```

示例配置：

- 单步 2DOF：[`conf/policy_config_example.pb.txt`](../../conf/policy_config_example.pb.txt)
- 单步 6DOF：[`conf/policy_config_twist6_example.pb.txt`](../../conf/policy_config_twist6_example.pb.txt)
- 多模态：[`conf/policy_config_multimodal_example.pb.txt`](../../conf/policy_config_multimodal_example.pb.txt)
- Diffusion：[`conf/policy_config_diffusion_example.pb.txt`](../../conf/policy_config_diffusion_example.pb.txt)
- Toy 冒烟：[`conf/policy_config_toy_example.pb.txt`](../../conf/policy_config_toy_example.pb.txt)
- 量产 overlay：[`conf/policy_controller_production.pb.txt`](../../conf/policy_controller_production.pb.txt)
- 主控片段：[`conf/controller.pb.txt`](../../conf/controller.pb.txt)（注释掉的 `policy_controller_options`）
