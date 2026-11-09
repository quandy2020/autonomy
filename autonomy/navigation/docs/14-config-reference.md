# 14 · 配置参数参考与调试

> 本文按当前代码解释每个配置字段的含义、单位、消费方和调试方法。
> “消费方”为 ⛔ 的字段目前**保留但未生效**，改它不会改变行为。

## 1. 文件与加载

```text
conf/
├── navigation.pb.txt
├── safety.pb.txt
├── guardian.pb.txt
├── platform/<name>.pb.txt
└── models/{defm,vntpa}.pb.txt
```

- 路径相对 `NAVIGATION_CONF_DIR` 解析；未设置时相对进程工作目录。
- 启动时一次性加载；**修改后必须重启进程**，没有热重载。
- 运行前校验：

```sh
./build/navigation_configuration_validator conf
echo $NAVIGATION_CONF_DIR
```

## 2. navigation.pb.txt

### 2.1 顶层

| 字段 | 含义 | 单位/取值 | 消费方 |
| --- | --- | --- | --- |
| `platform_profile` | 平台 preset 短名 | `differential_drive` / `omnidirectional` / `ackermann` / `legged` | ✅ `LoadPlatformProfile` |

### 2.2 `scenario`

| 字段 | 含义 | 取值 | 消费方 |
| --- | --- | --- | --- |
| `default_scenario` | 默认场景名 | `destination` / `exploration` / `following` / `coverage` | 🟡 仅 `PlanningOptions::Validate` 非空检查 |
| `available_scenarios` | 可用场景列表 | 同上 | ⛔ 运行时不读；场景由 `GoalSpec.type` 映射 |

> 实际选择：`TYPE_POSE→destination`、`TYPE_EXPLORATION→exploration`、
> `TYPE_TRACKING→following`、`TYPE_REGION→coverage`。

### 2.3 `policy`

| 字段 | 含义 | 取值 | 消费方 |
| --- | --- | --- | --- |
| `name` | 策略插件名 | `simulation` / `vntpa` / `vint` / `nomad` | ✅ `PluginManager` |
| `vntpa.model_path` | VNTPA ONNX 路径 | 文件路径 | ✅ Engine policy（`vntpa` 时） |
| `vntpa.action_chunk_length` | 每次输出的动作数 | 1–32，默认 8 | ✅ Blender/Postprocessor |
| `vntpa.execute_length` | 每次执行的动作数 | ≤ chunk，默认 2 | ✅ Blender |
| `vntpa.decision_rate_hz` | 策略决策频率 | >0，默认 5 | ✅ Runner worker |
| `vntpa.action_dt_s` | 单个动作时长 | >0，默认 0.5 | ✅ Blender/Postprocessor |
| `vntpa.context_size_k` | 空间上下文条目数 | 默认 64 | ✅ VNTPA preprocessor |
| `vntpa.kf_tokens_input_name` | patch tokens 输入名 | 默认 `kf_tokens` | ✅ Engine |
| `vntpa.kf_pose_input_name` | 关键帧 pose 输入名 | 默认 `kf_pose` | ✅ Engine |
| `vntpa.valid_input_name` | keyframe mask 输入名 | 默认 `kf_valid` | ✅ Engine |
| `vntpa.query_pose_input_name` | query pose 输入名 | 默认 `query_pose` | ✅ Engine |
| `vntpa.goal_input_name` | goal 输入名 | 默认 `goal` | ✅ Engine |
| `vntpa.forward_step` | 单步前进距离 | m，默认 0.25 | ✅ 离散动作 |
| `vntpa.turn_step_rad` | 单步转角 | rad，默认 30° | ✅ 离散动作 |
| `vntpa.context_mask_input_name` | mask 输入名 | 与模型一致 | ✅ Engine |
| `vntpa.output_name` | 动作输出名 | 与模型一致 | ✅ Engine |
| `vntpa.pose_input_dim` | pose 特征维度 | 默认 64 | 🟡 预处理器实际使用 64 |
| `vntpa.execution_provider` | 推理后端 | `onnx` / `tensorrt` / `cuda` | ✅ Engine |
| `vntpa.device_id` | GPU id | 0..N | ✅ Engine |
| `vntpa.precision` | 精度 | `fp32` / `fp16` | ✅ Engine |
| `vntpa.enable_reachability_adjuster` | Docking 可达性微调 | bool | 🟡 参数存在，默认关闭 |
| `vntpa.reach_min` | 机械臂可达内半径 | m | 🟡 adjuster 使用 |
| `vntpa.reach_max` | 机械臂可达外半径 | m | 🟡 adjuster 使用 |


#### VINT（`policy.vint`）

| 字段 | 含义 | 单位/默认 |
| --- | --- | --- |
| `model_path` | VINT ONNX 路径 | 文件 |
| `tokenizer_path` | 文本 tokenizer | 文件 |
| `max_instruction_tokens` | 指令最大 token 数 | 默认 64 |
| `image_input_name` / `text_input_name` / `output_name` | ONNX I/O 名 | 与模型一致 |
| `action_chunk_length` | 动作块长度 | 默认 8 |
| `action_dt_s` | 动作间隔 | s，默认 0.5 |

#### NOMAD（`policy.nomad`）

| 字段 | 含义 | 单位/默认 |
| --- | --- | --- |
| `model_path` | NOMAD ONNX 路径 | 文件 |
| `diffusion_steps` | 扩散步数 | 默认 10 |
| `num_samples` | 采样数 | 默认 4 |
| `horizon` | 预测 horizon | 默认 16 |
| `temperature` | 采样温度 | 默认 1.0 |
| `action_chunk_length` | 动作块长度 | 默认 8 |
| `action_dt_s` | 动作间隔 | s，默认 0.5 |

#### 其它插件参数

| 字段 | 含义 | 单位/默认 |
| --- | --- | --- |
| `reflex.velocity_sampler.horizon_s` | 采样时域 | s，⛔ 当前未消费 |
| `reflex.velocity_sampler.sample_count` | 采样数 | — |
| `reflex.velocity_sampler.max_acceleration_mps2` | 采样加速度上限 | m/s² |
| `reflex.velocity_sampler.safety_margin` | 采样安全余量 | m |
| `memory.frame_queue.capacity` | 队列容量 | 默认 16 |
| `memory.frame_queue.stride` | 抽帧步长 | ⛔ 当前未消费 |
| `memory.single_frame` | 单帧，无参数 | — |

> 仿真 `policy.name: "simulation"` 时，`vntpa` 块只用于 Blender 的
> `action_chunk_length/execute_length/action_dt_s`，其余字段不参与。

### 2.4 `reflex`（navigation 进程）

| 字段 | 含义 | 单位 | 消费方 |
| --- | --- | --- | --- |
| `name` | 反射插件名 | — | ⛔ navigation 进程当前不读；safety 用自己的 `safety.pb.txt` |
| `reactive_avoider.near_field_range` | 近场范围 | m | 仅安全进程 |
| `reactive_avoider.reaction_time_ms` | 反应时间 | ms | 仅安全进程 |
| `reactive_avoider.safety_margin` | 安全余量 | m | 仅安全进程 |
| `reactive_avoider.fov_rad` | 前向视场 | rad | 仅安全进程 |
| `reactive_avoider.candidate_count` | 候选数 | — | 仅安全进程 |

### 2.5 `memory`

| 字段 | 含义 | 单位/取值 | 消费方 |
| --- | --- | --- | --- |
| `name` | 记忆插件 | `pose_indexed` / `frame_queue` / `single_frame` | ✅ |
| `pose_indexed.max_entries` | 最大关键帧数 | 默认 5000 | ✅ |
| `novelty_threshold_eta` | 新帧阈值 | 0..1，默认 0.08 | ✅ |
| `voxel_resolution` | 体素分辨率 | m，默认 0.1 | ✅ |
| `voxel_direction_bins` | 方向分箱 | 默认 16 | ✅ |
| `query_radius` | 检索半径 | m，默认 2.0 | ✅ |
| `min_insert_interval_s` | 最小插入间隔 | s，默认 0.2 | ✅ |

### 2.6 `rule`

| 字段 | 含义 | 取值 | 消费方 |
| --- | --- | --- | --- |
| `enabled_rules` | 启用的规则链 | `speed_limit` / `safety_chain` / `forbidden_zone` / `dynamic_obstacle` / `pedestrian` | ✅ |

### 2.7 `behavior`

`enabled` 启用默认 XML 与 Runner 无进度恢复；心跳仍由 BehaviorComponent 发布。
`tree` 默认为 `navigate_with_recovery.xml`，相对文件名从
`NAVIGATION_CONF_DIR/behavior/trees/` 解析，也可给绝对路径。
`tick_rate_hz` 为树最大频率（默认配置 10 Hz），受组件 Timer 周期限制。
`max_recovery_attempts` 默认配置 3，按当前目标已接受的恢复次数限制，重启
Behavior 不重置预算。

`no_progress_timeout_s` 未指定/非正时用 8 s，正常 pose 导航与恢复阶段均使用；
`recovery_lateral_distance_m`、`recovery_forward_distance_m` 未指定/非正时各 2 m；
`recovery_phase_timeout_s` 未指定/非正时用 25 s。
Runner 以实测位置/朝向推进动作，命令经过 Safety，有限尝试耗尽后 Action 失败。
真实差速场景结果与更多场景限制见 [19](19-behavior-recovery-validation.md)。

### 2.8 `limits`

| 字段 | 含义 | 单位 | 消费方 |
| --- | --- | --- | --- |
| `max_pose_jump` | 位姿跳变阈值 | m | ⛔ 未消费 |
| `max_yaw_jump_rad` | 航向跳变阈值 | rad | ⛔ |
| `max_consecutive_rejects` | 连续拒绝上限 | — | ⛔ |
| `max_hold_ms` | 最大保持 | ms | ⛔ |
| `intent_timeout_ms` | 意图超时 | ms | ⛔ |

### 2.9 `debug`

| 字段 | 含义 | 消费方 |
| --- | --- | --- |
| `log_level` | 日志级别 | ⛔ 当前未接组件日志 |
| `publish_debug_markers` | 调试 marker | ⛔ |
| `synchronous_policy_for_test` | 同步策略（测试） | ⛔ |

### 2.10 `perception`

| 字段 | 含义 | 单位/默认 | 消费方 |
| --- | --- | --- | --- |
| `min_valid_ratio` | 有效深度比例阈值 | 0..1，默认 0.15 | ✅ Engine encoder |
| `max_range` | 最大深度范围 | m，默认 8 | ⛔ 当前未消费 |
| `uncertainty_slowdown_threshold` | 不确定性减速阈值 | 0..1，默认 0.6 | ⛔ |
| `uncertainty_reject_threshold` | 不确定性拒绝阈值 | 0..1，默认 0.85 | ⛔ |
| `enable_attention_debug` | attention 调试 | bool | ⛔ |
| `model_path` | DeFM ONNX 路径 | 文件 | ✅ 覆盖 manifest.path |
| `manifest_path` | 模型 manifest | 文件 | ✅ DeFM/embedding |
| `input_name` | 输入名 | 默认 `depth` | ⛔ Engine 用 manifest 输入名 |

### 2.11 `initial_goal`

| 字段 | 含义 | 消费方 |
| --- | --- | --- |
| `type` | 初始 Goal 类型 | ✅ 启动时 `Runner.SetGoal` |
| `pose` | 点到点目标 | ✅ |
| `goal_id/sequence` | 幂等/排序 | ✅ |

## 3. safety.pb.txt

| 字段 | 含义 | 单位/默认 | 消费方 |
| --- | --- | --- | --- |
| `platform_profile` | 平台 preset | — | ✅ |
| `reflex.name` | 反射插件 | `reactive_avoider` | ✅ |
| `reflex.reactive_avoider.*` | 近场反射参数 | 见上 | ✅ |
| `chain.reflex_priority_ratio` | 反射权重 | 0..1，默认 0.2 | ✅ |
| `chain.max_acceleration_mps2` | 安全链加速度上限 | m/s²，默认 1.0 | ✅ |
| `chain.max_jerk_mps3` | jerk 上限 | m/s³ | ⛔ 当前未消费 |
| `chain.ttc_reaction_time_s` | AEB 反应时间 | s，默认 0.15 | ✅ BrakingModel |
| `chain.ttc_safety_margin_s` | TTC 安全余量 | s | ⛔ 当前未消费 |
| `chain.cbf_alpha` | CBF 衰减率 | 1/s，默认 2.0 | ✅ |
| `chain.watchdog_deadline_ms` | 看门狗硬截止 | ms，默认 50 | ✅ |
| `chain.aeb_sample_count` | AEB 采样数 | 默认 45 | ✅ |
| `chain.min_clearance` | 最小间隙 | m，默认 0.05 | ✅ |
| `chain.max_hold_ms` | 最大保持 | ms | ⛔ |

## 4. guardian.pb.txt

| 字段 | 含义 | 单位/默认 | 消费方 |
| --- | --- | --- | --- |
| `heartbeat_timeout_ms` | 健康超时 | ms，默认 200 | ✅ |
| `heartbeat_fatal_ms` | 致命超时 | ms，默认 400 | ✅ |
| `diagnostics_rate_hz` | 诊断频率 | Hz，默认 2 | ✅ |
| `allow_reflex_only` | 允许反射模式 | bool | ✅ |

## 5. platform/*.pb.txt

| 字段 | 含义 | 单位 | 消费方 |
| --- | --- | --- | --- |
| `kinematic_type` | 底盘类型 | enum | ✅ |
| `max_linear_mps` | 最大前进速度 | m/s | ✅ |
| `max_lateral_mps` | 最大侧向速度 | m/s | ✅ |
| `max_angular_radps` | 最大角速度 | rad/s | ✅ |
| `max_acceleration_mps2` | 最大加速度 | m/s² | ✅ |
| `max_jerk_mps3` | 最大 jerk | m/s³ | ✅ |
| `max_lateral_acceleration_mps2` | 最大侧向加速度 | m/s² | ✅ |
| `footprint_radius` | 足迹半径 | m | ✅ |
| `wheelbase` | 轴距 | m | ✅ 差速/阿克曼 |
| `max_steering_rad` | 最大转角 | rad | ✅ 阿克曼 |
| `camera_height` | 相机高度 | m | 🟡 已转换，当前算法未用 |
| `camera_pitch_rad` | 相机俯仰 | rad | 🟡 |
| `camera_to_base_x` | 相机 x 偏移 | m | 🟡 |
| `camera_to_base_y` | 相机 y 偏移 | m | 🟡 |

## 6. 模型 manifest

| 字段 | 含义 |
| --- | --- |
| `name` | 模型名 |
| `path` | ONNX 文件 |
| `sha256` | 文件完整性；**唯一哈希来源** |
| `opset` | ONNX opset |
| `precision` | fp32/fp16 |
| `inputs[]` | 名称/形状/dtype |
| `outputs[]` | 名称/形状/dtype |
| `mean/std` | 归一化 |
| `embedding_dim` | 特征维度 |
| `token_count` | token 数 |

## 7. 调试流程

### 7.1 配置校验

```sh
./build/navigation_configuration_validator conf
```

失败时打印文件与字段，不会启动半配置。

### 7.2 观察链路

```sh
autolink channel hz /camera/depth/image_raw
autolink channel echo /navigation/observation --once
autolink channel echo /navigation/status -n 5
autolink channel echo /navigation/cmd_vel_raw -n 5
autolink channel echo /cmd_vel -n 5
autolink channel hz /cmd_vel
```

### 7.3 目标

```sh
navigation_goal_sender 2.0 1.0 0.0
navigation_goal_sender                 # 探索
```

或 Action：

```sh
autolink action list
autolink action send_goal /navigate_to_pose \
  '{"pose":{"header":{"frame_id":"map"},"pose":{"position":{"x":2,"y":1},"orientation":{"w":1}}}}'
```

### 7.4 心跳/降级

```sh
autolink channel echo /navigation/heartbeat -n 3
autolink channel echo /safety/heartbeat -n 3
autolink channel echo /behavior/heartbeat -n 3
autolink channel echo /guardian/degradation -n 3
```

### 7.5 配置到行为的快速判断

| 现象 | 先查 |
| --- | --- |
| 进程起不来 | `configuration_validator`；`platform_profile` 文件 |
| 不动 | `/navigation/observation` 是否有；`policy.name` 是否注册；`Runner::Create` |
| 立即零速 | depth stamp 与 safety 时间；`/cmd_vel` 类型；reflex 参数 |
| 不探索 | `/map` 是否发布；`initial_goal.type` |
| guardian 降级 | 三路 heartbeat |
| VNTPA 不加载 | `policy.name=vntpa`、`RegisterEnginePlugins`、`model_path` |
| 模型校验失败 | manifest `sha256` / `inputs`/`outputs` |

## 8. 调参示例

### 8.1 保守慢速

```textproto
# platform/differential_drive.pb.txt
max_linear_mps: 0.25
max_acceleration_mps2: 0.4
max_jerk_mps3: 1.0

# safety.pb.txt
reflex.near_field_range: 2.5
reflex.safety_margin: 0.4
chain.max_acceleration_mps2: 0.6
```

### 8.2 窄通道

```textproto
footprint_radius: 0.22
reflex.fov_rad: 1.9
reflex.candidate_count: 48
max_linear_mps: 0.2
```

### 8.3 探索

```textproto
initial_goal { type: TYPE_EXPLORATION }
policy { name: "simulation" }
```

`policy.exploration` 的零值使用默认：`clearance_m=0.4`、`max_range_m=6`、
`no_progress_timeout_s=8`、`target_timeout_s=45`、`max_history_entries=64`、
`history_ttl_s=120`。容量硬上限 1024，`log_transitions` 输出目标选择/扫描/放弃日志。
Runner 将地图净空限制为至少 `platform.footprint_radius + 0.1 m`，不能用此配置缩小机器人。
地图最多 1,000,000 格；细栅格所需膨胀半径超过 128 格时不规划。

地图优先于 FrontierNet；无地图才使用模型候选。无 frontier 时先按 odom 累计扫描，
有地图则等待地图更新，无地图且中心深度 ≥1 m 才缓慢前进。
地图帧和策略输入pose必须对齐；当前组件可按直接map→odom将odom合成到map。
该基础路径不等于完整TF缓存查询，时效性及真实SLAM漂移/重定位尚待验收。

### 8.4 切换到 VNTPA

```textproto
policy {
  name: "vntpa"
  vntpa {
    model_path: "conf/models/vntpa.onnx"
    ...
  }
}
```

前提：

1. Engine 适配器随导航库编译并链接 `autonomy::common`；
2. 部署入口调用 `RegisterEnginePlugins()`；
3. `conf/models/vntpa.onnx` 存在；
4. manifest I/O 与模型一致。

## 9. 已知未消费字段

以下字段当前**改了不会生效**，后续接线前不要用于调参：

- `navigation.scenario.available_scenarios`
- `navigation.reflex.*`
- `navigation.behavior.*`
- `navigation.limits.*`
- `navigation.debug.*`
- `navigation.perception.max_range`
- `navigation.perception.uncertainty_*`
- `navigation.perception.enable_attention_debug`
- `navigation.perception.input_name`
- `safety.chain.max_jerk_mps3`
- `safety.chain.ttc_safety_margin_s`
- `safety.chain.max_hold_ms`
- platform `camera_*`（已解析，算法未用）

这些是 TODO，不是“可调参数”；见 [13-todo.md](13-todo.md)。

地图frontier的实现边界、真实深度栅格输入、600 s探索阶段与最终合并回归证据见
[20-exploration-validation.md](20-exploration-validation.md)。
