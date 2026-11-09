# 04 · Navigation 最终方案 v2


> 2026-11-09 实现补充：simulation 探索策略消费有效 OccupancyGrid，选择连通自由
> frontier 成员，并用带 footprint 净空的四邻接 BFS 路径跟随；pose goal seek 和 VNTPA
> 的记忆契约保持各自范围。探索目标有进展/绝对时限和容量受控的访问记忆。
> 无 frontier 扫描后，已知地图等待更新，无地图才允许前向深度门控巡航。
> 这里的栅格路径仅服务 simulation 探索，不宣称 VNTPA 学习策略已完成。

> 状态：设计冻结候选（Design Freeze Candidate）
> 版本：2.0.0
> 工程根：`autonomy/navigation/`
> 基线：v1「Component + Runner / VNTPA / 4 场景 / 3 DAG」
> 本版说明：只收敛设计，不改实现、不要求构建。所有相对 v1 的修改均标注 **v2 变更**，
> 汇总见附录 A。
> 2026-11-09 实现补充：默认恢复树由 BehaviorComponent 执行，可独立进程部署；
> 会话/目标版本与有限恢复契约以第 9 节及 [19](19-behavior-recovery-validation.md) 为准。

---

## 0. 修订摘要

v1 的架构方向成立：薄 Component + 业务 Runner、无显式地图、位姿索引记忆、三级安全链、
独立安全/守护进程。v2 修正的是**不可实现、不可部署或语义冲突**的细节，共 16 项。

| 编号 | v1 问题 | v2 修正 | 影响面 |
| --- | --- | --- | --- |
| C1 | “顶层 11 项”与目录树不一致 | 顶层 16 项：2 个构建/说明文件 + 2 个入口单元 + 7 个功能模块 + 5 个支撑目录（本设计文档另计） | 文档 |
| C2 | DAG 不符合 autolink schema：`module_config` 没有 `timer_interval`；`ComponentConfig` 没有 `writers` | 按 `ModuleConfig`/`ComponentInfo`/`TimerComponentInfo` 重写 3 份 DAG；写端在代码里 `CreateWriter` | DAG、Component |
| C3 | `ComponentBase` 没有 `Stop()`，`Init()` 是 protected 纯虚 | 用 `Clear()` + 析构收尾；补 `AUTOLINK_REGISTER_COMPONENT`；明确 `Proc` 为 private override | Component |
| C4 | 一个 `Component<Observation>` 无法同时读 depth / camera_info / odom / imu / tf | 新增 `ObservationBuilderComponent`，`NavigationComponent` 只消费 `/navigation/observation` | perception、DAG |
| C5 | `navigation.proto` 包名 `navigation.navigation` 与 `navigation::GoalSpec` 用法冲突 | 包名改为 `navigation`，其余四个 proto 保持子包 | proto、全部代码 |
| C6 | DAG 引用 `SafetyComponent`/`GuardianComponent`，目录树里没有入口 | 补 `safety/safety_component.*`、`guardian/guardian_component.*` | safety、guardian |
| C7 | Runner 持有 `chunk_blender_`，目录树没有该文件 | 补 `planning/chunk_blender.*`，定义时间集成与加速度连续 | planning、runner |
| C8 | 场景表引用 `coverage_planner`，tasks 里没有 | 补 `planning/tasks/coverage_planner.*` | planning |
| C9 | 15 Hz tick 与 5 Hz、百毫秒级推理同步调用矛盾；编码结果也没有传给策略 | 策略异步 worker + SPSC + 双缓冲；`DepthFeature` 作为 Query 入参复用编码；Tick 永不阻塞 | runner、planning |
| C10 | `Twist2D{v, ω}` 无法表达全向底盘 | 改为 3-DOF `{vx, vy, ωz}`；差速/阿克曼适配器强制 `vy=0` | common、safety、planning |
| C11 | `ContextView&` 跨线程生命周期未定义 | `ContextView` 是不可变快照句柄，持 `shared_ptr<const KeyframeBatch>` | memory、policy |
| C12 | `platform_profile` 内联与 `conf/platform/*` 重复；`safety_chain` 两处配置 | 平台配置单一来源（preset 文件），安全链归 `safety.pb.txt` | conf |
| C13 | 模型二进制不入库（`conf/models/*.onnx`） | 只保留 manifest（sha256/shape/opset）；权重本地导出/制品库 | conf、tools |
| C14 | 验收指标无量值、无参照平台 | 给出参照平台、分位指标和数值门槛 | 验收 |
| C15 | guardian 读 `/behavior/heartbeat`，没有生产者 | BehaviorComponent 加载 XML 并发心跳；默认同 DAG，可拆成独立进程 | behavior、runner |
| C16 | 单位、术语、失败语义不统一 | 统一 SI 后缀、失败关闭（fail-closed）、状态机命名 | 全部 |

**结论**：v1 可以直接进入实现的前提是先落 C2/C3/C5/C9 四项，否则代码无法编译或无法部署。

**v2 保持不变**（避免误读为推倒重来）：

- 不需要地图/costmap，VNTPA 以位姿索引关键帧替代显式地图。
- Component + Runner 双层；Component 薄、Runner 可单测。
- 4 个场景（destination / exploration / following / coverage），无优先级栈。
- 3 个进程、3 份 DAG；安全层独立、反射层独立。
- 模型推理统一走 `autonomy::common::network::Engine`；公共宏用 `autonomy/common/macros.hpp`。
- VNTPA 插件按职责拆文件，文件名去插件前缀，靠命名空间隔离。

**阅读指引**

- 决策/评审：§0 修订、§12 性能预算、§13.3 数值验收、§15 风险与待决。
- 实现：§2 契约、§3 架构、§4 消息与接口、§8 安全、§11 配置与部署。
- 算法/训练：§5 感知、§6 记忆、§7.5 VNTPA、§7.7 ChunkBlender。
- 编码：§16 编码规范与工程约束。

**约定**：文中“必须/禁止”为不可协商项；“建议/默认”为可配置项，默认值写在各节与附录 B。

---

## 1. 定位、边界与术语

### 1.1 定位

`autonomy/navigation` 是 autonomy 生态里的**移动机器人端到端导航子模块**：

- 输入：目标（位姿 / 探索 / 跟踪 / 区域）、深度图、相机内参、里程计、IMU、TF、语言指令。
- 输出：一条经过安全链校核的底盘速度 `/cmd_vel` 候选（navigation 只写 `/navigation/cmd_vel_raw`）。
- 核心策略：VNTPA（Vision-and-Pose Navigation Transformer with Pose-indexed Attention），以位姿索引
  关键帧记忆替代显式地图/costmap。
- 部署形态：3 个进程（navigation / safety / guardian），3 份 DAG。

### 1.2 非目标

| 非目标 | 归属 |
| --- | --- |
| SLAM、建图、回环检测的底层实现 | `autonomy/localization`、`autonomy/map` |
| 全局 costmap / 栅格路径规划 | 本模块不引入；由 VNTPA + 记忆替代 |
| 机械臂运动规划与抓取 | `automanip` |
| 底盘电流环、轮速闭环 | `autonomy/control`、`autodriver` |
| 仿真器、数据回放器本体 | `autosim`、`autonomy/tools` |

### 1.3 上下游契约

```
 sensor/transform ──┐
 localization ──────┼──► ObservationBuilder ──► NavigationComponent ──► /navigation/cmd_vel_raw
 task/intent ───────┘          │                                             │
                               └── /tf, /odom, /imu                          ▼
                                                                    SafetyComponent ──► /cmd_vel
                                                                           ▲
                                                                    /safety/heartbeat
                                                                           │
                                                                     GuardianComponent
```

- 上游只保证“消息存在”，不保证“消息新鲜/自洽”；导航必须自己校验（见 §2.2）。
- `navigation` 绝不直接写 `/cmd_vel`；`safety` 是唯一出口。
- `guardian` 不进入控制通路，只发布降级意图；最终执行由 `safety` 与底盘看门狗兜底。

### 1.4 术语

| 术语 | 含义 |
| --- | --- |
| Keyframe | 带位姿与 DeFM 嵌入的关键帧，记忆的最小单位 |
| Context | 由 K 个关键帧组成的不可变快照，供策略查询 |
| Chunk | 一次策略推理输出的定长动作块（T×3） |
| Intent | 策略输出的语义动作：chunk / waypoint / path / velocity |
| Stage | 场景内的一级处理阶段（可组合） |
| Task | Stage 内可复用的算法单元（优化器、检测器、恢复执行器） |
| Rule | 对候选速度的硬约束（限速、禁区、行人） |
| Degradation | guardian 发布的运行模式降级：Normal → ReflexOnly → SafeStop → Manual |

### 1.5 设计决策总表

| 决策点 | 选择 | 理由 |
| --- | --- | --- |
| 地图/costmap | 不需要 | VNTPA 以位姿索引关键帧替代显式地图 |
| 安全层 | 独立进程 | 故障域隔离，决策层崩溃不影响制动 |
| 安全链 | 三级（另设入口校验） | 单点 AEB 存在动力学可行性间隙 |
| 反射层 | 独立 | 近场毫秒级 vs 策略推理延迟峰值 |
| 部署形态 | Component + Runner | Component 绑定通信，Runner 可独立单测 |
| 模型推理 | `autonomy::common::network::Engine` | 后端可切换（ONNX Runtime / TensorRT） |
| 公共宏 | `autonomy/common/macros.hpp` | 与 autonomy 生态统一，不自定义 |
| 场景数 | 4（destination / exploration / following / coverage） | 窄通道由 CBF 连续约束，门/电梯下沉为任务原语 |
| 默认场景 | `destination` | 单单词，与其他场景在“任务目的”维度对齐 |
| 插件目录 | `vntpa/`、`vint/`、`nomad/` 平行 | 目录表达身份，文件名只表达职责 |
| 插件命名空间 | `navigation::planning::vntpa` | 隔离三个插件的同名类 |
| proto 数量 | 5 | 按语义域合并（observation/navigation/lifecycle/config/plugins） |
| 头文件保护 | `#pragma once` | 对齐 context.hpp |
| 缩进/括号 | 4 空格；函数左括号独占一行 | 对齐 context.hpp |
| 异常策略 | 决策层不穿越边界；安全层 `-fno-exceptions` | 异常展开时间不可预测 |
| 策略调用（v2） | 异步 worker + 动作块 | 推理延迟峰值远大于 66 ms tick |
| 平面速度（v2） | 3-DOF `Twist2D` | 全向底盘需要侧向分量；差速强制 `vy=0` |
| 行为层（当前） | BehaviorComponent Timer | 保持 3 份 DAG，BehaviorComponent 自身 tick/发心跳，可拆进程 |
| 模型制品（v2） | manifest + 外置权重 | 避免二进制入库，支持 sha256 校验 |

---

## 2. 全局不变量（契约）

这一节是 v2 新增的“不可协商项”。任何模块违反，单测必须失败。

### 2.1 坐标与时间

| 项 | 规定 |
| --- | --- |
| 位姿坐标系 | `map`（会话坐标系）；局部平滑在 `odom`；底盘在 `base_link` |
| 相机坐标系 | `camera_optical_frame`（x 右、y 下、z 前），外参由 TF 给出 |
| TF 链 | `map → odom → base_link → camera_optical_frame`，任一环缺失即无效观测 |
| 时间戳 | `int64 stamp_ns`，会话单调时钟（`steady_clock` 对齐），禁止墙钟进入控制 |
| `Header.stamp` | 传感器采样时刻；不是接收时刻 |
| 序号 | 每个通道单调递增 `sequence`，用于丢帧/乱序检测 |
| Epoch | 重定位或回环校正后 `epoch++`；旧 epoch 的记忆条目失效 |
| 楼层 | `floor_id` 变化视为新会话：`epoch++`、清空记忆与动作块，禁止跨楼层复用位姿索引 |

**时间预算**（超时即无效，不允许“继续用上一帧”）：

| 输入 | 最大年龄 | 超时动作 |
| --- | --- | --- |
| depth | 150 ms | 本次 Tick 无效，输出保持 + 诊断 `kStaleDepth` |
| pose / odom | 100 ms | 同上，`kStalePose` |
| TF（map→camera） | 100 ms | 主动降级到 ReflexOnly，`kTFDisconnected` |
| IMU（若使用） | 50 ms | 仅降低速度上限，不阻断 |
| 控制指令输入（cmd_vel_raw） | 100 ms | safety 进入零速，`kStaleCommand` |

### 2.2 位姿连续性

- 单 Tick 位移 > `max_pose_jump`（默认 0.5 m）或航向 > `max_yaw_jump_rad`（默认 0.52 rad）→ 丢弃该帧。
- 连续 3 次丢弃 → 触发 `kPoseDiscontinuity`，Runner 清空动作块并保持零速，guardian 降级。
- `epoch` 变化时，允许一次大跳变，但必须同时重建姿态索引并清空上下文。

### 2.3 速度与动力学

- 单位一律 SI，字段名带单位后缀（`_m`、`_mps`、`_radps`、`_ms`、`_ns`）。
- `Twist2D` 复用 automsgs `geometry_msgs/Twist2D`，字段为 `x`(vx)、`y`(vy)、`theta`(ωz)，仍为 3-DOF。
- 差速/阿克曼平台在适配层强制 `linear_y_mps = 0`；全向平台允许保留侧向分量。
- 任何输出必须满足：`|v| ≤ profile.max_linear_mps`、`|ω| ≤ profile.max_angular_radps`、
  `|a| ≤ profile.max_acceleration_mps2`、`|j| ≤ profile.max_jerk_mps3`。
- 速度方向与航向的夹角超过 `max_lateral_ratio` 时，差速平台拒绝该指令。

### 2.4 线程与所有权

| 资源 | 所有者 | 访问方式 |
| --- | --- | --- |
| `Observation` | autolink 消息线程 | 只读；仅 Tick 调用期间有效，不跨帧保存 |
| 记忆（插入/索引） | navigation 线程 | 独占；策略线程只拿不可变快照 |
| 策略推理 | policy worker 线程 | 单线程串行，不并发 `Engine::Run` |
| 动作块 / 混合器 | navigation 线程 | 独占 |
| 安全链 | safety 进程 | 独占；无跨线程共享 |
| 心跳 / 诊断 | 各进程 writer | 线程安全，允许丢弃旧消息 |

### 2.5 优先级

从高到低，低优先级不得覆盖高优先级：

```
SafetyStop > Reflex > CBF/AEB > Feasibility > Rule > Task/Stage > Policy > ChunkBlender
```

### 2.6 失败关闭（fail-closed）

- 输入缺失、超时、NaN、越界、插件异常、模型错误、记忆损坏 → **零速 + 诊断**。
- 决策层（perception/memory/planning/behavior）允许异常，但**不得穿越模块边界**：
  在插件/进程边界统一转 `StatusOr`，在 Component 边界统一转诊断。
- 安全层（safety）`-fno-exceptions -fno-rtti`，所有错误路径返回 `std::optional`/`Status`。
- 禁止“降级为上一帧速度”超过 `max_hold_ms`（默认 200 ms）；超过即零速。

---

## 3. 架构

### 3.1 Component + Runner 双层

```
┌──────────────────────────────────────────────────────────────┐
│ autolink 进程（navigation）                                   │
│                                                              │
│  ObservationBuilderComponent ──► /navigation/observation      │
│          ▲ depth / camera_info / odom / imu / tf              │
│                                                              │
│  NavigationComponent（薄入口）                                │
│    Init(): 读取 conf → Runner::Create → Runner::Start         │
│    Proc(): Runner::Tick(observation) → 写 cmd_vel_raw          │
│    Clear(): Runner::Stop                                     │
│          │                                                   │
│          ▼                                                   │
│  Runner（业务核心，不依赖 autolink）                          │
│    encoder / memory / policy / chunk_blender / recovery      │
└──────────────────────────────────────────────────────────────┘
```

**原则**

1. Component 不知道 VNTPA、DeFM、CBF 的存在；只做消息转换、配置加载、端点注册。
2. Runner 不知道 autolink 的存在；只依赖抽象接口，可纯单测。
3. Component 的 `Proc` 必须在预算内返回：普通帧 8 ms、编码帧 35 ms（周期 66 ms）；
   推理等重活全部在 policy worker 线程。

**生命周期修正（C3）**：`autolink::ComponentBase` 没有 `Stop()`，只有 protected 的 `Init()`
与 `Clear()`；`Proc` 是 private 纯虚。正确写法：

```cpp
class NavigationComponent final
    : public autolink::Component<observation::Observation>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(NavigationComponent)
    NavigationComponent();
    ~NavigationComponent() override;

protected:
    bool Init() override;                 // 加载配置、构造 Runner、注册端点
    void Clear() override;                // ~Runner::Stop()，可重复调用

private:
    bool Proc(const std::shared_ptr<observation::Observation>& observation) override;

    std::unique_ptr<Runner> runner_{nullptr};
};

// navigation_component.cpp
AUTOLINK_REGISTER_COMPONENT(navigation::NavigationComponent)
```

> 不要写 `void Stop() override`：基类没有该虚函数，编译期即失败。
> 不要在 `Proc` 里加载配置或创建 writer；这些只在 `Init` 做一次。

### 3.2 进程拓扑

| 进程 | 组件 | 驱动方式 | 周期 | 职责 |
| --- | --- | --- | --- | --- |
| navigation | ObservationBuilderComponent | 消息驱动（depth） | ~15 Hz | 多传感器融合成 `Observation` |
| navigation | NavigationComponent | 消息驱动（observation） | ~15 Hz | Runner 业务循环 |
| navigation | BehaviorComponent | 定时器 | 100 ms（10 Hz） | 默认 XML、恢复/失败请求、自身心跳 |
| safety | SafetyComponent | 定时器 | 20 ms（50 Hz） | 三级安全链、唯一 `/cmd_vel` 出口 |
| guardian | GuardianComponent | 定时器 | 500 ms（2 Hz） | 心跳、故障分类、降级 |

> v2 变更 C4：融合节点独立成组件，避免一个 `Component<Observation>` 绑 5 个通道的伪设计。
> 当前 C15：BehaviorComponent 在默认 navigation DAG 中自身 tick/发心跳；验收可将其拆成独立 mainboard，
> 因此仍然只有 3 份 DAG。

### 3.3 线程模型与调度

| 线程 | 归属 | 周期/触发 | 预算 | 阻塞策略 |
| --- | --- | --- | --- | --- |
| autolink 调度线程 | navigation | Observation 到达 | 普通帧 8 ms；编码帧 35 ms（周期 66 ms） | 永不等待推理 |
| policy worker | navigation | 条件变量 + 最新帧 | 200 ms | 单飞（single-flight） |
| autolink 定时器 | safety | 20 ms | 硬截止 50 ms | 固定节拍，允许丢帧不允许超时 |
| autolink 定时器 | guardian | 500 ms | 20 ms | 可丢帧 |

**异步策略（v2 变更 C9）**

```
Tick (15 Hz, 66 ms):
  1. 校验 Observation / pose / TF / epoch
  2. encoder（5 Hz 降频，缓存复用；P99 ≤ 15 ms）
  3. 若本帧产生新特征：memory.TryInsert(keyframe)（P99 < 2 ms）
  4. 若到决策周期且 worker 空闲：提交 (KeyframeBatch, GoalSpec, depth_feature, pose)
     （depth_feature 来自第 2 步，策略不再重复编码）
  5. 非阻塞取回 Intent；失败/超时保留上一 chunk
  6. 发布状态供独立 BehaviorComponent tick（耗时门限待测）
  7. scenario pipeline → chunk_blender.Consume(dt) → Twist2D
  8. 发布 cmd_vel_raw / status / heartbeat
```

- 决策周期 `decision_rate_hz = 5`，动作块 `T = 8`、`dt = 0.1 s` → 单块覆盖 0.8 s，
  执行前 `execute_length = 2` 步（0.2 s）后即可重规划，留 600 ms 余量吸收推理抖动。
- Worker 结果带 `sequence` 与 `produced_at_ns`；超过 `intent_timeout_ms`（默认 400 ms）即丢弃，
  按当前 chunk 剩余动作继续执行；chunk 耗尽 → 平稳减速到零，并上报 `kPolicyStarved`。
- 不允许在 Tick 内调用 `Engine::Run`；单测用同步模式（`policy.synchronous_for_test = true`）覆盖。
- 编码在 Tick 内完成（5 Hz 帧），因此编码帧预算放宽到 35 ms，但仍远小于 66 ms 周期；
  若目标平台编码 P99 超过 35 ms，则把编码移入 policy worker，接口不变。

### 3.4 数据流

```
Observation
  → DepthPreprocessor → DepthValidity → DeFMEncoder → DepthFeature
  → KeyframeFilter → (accept) → PoseIndexedMemory.TryInsert
  → ContextBuilder.QueryNearest(pose, K) → ContextView（不可变快照）
  → Policy.Query(obs, depth_feature, goal, context) → NavigationIntent
  → ScenarioManager（按 GoalSpec.type）
      → Stage 链（policy → trajectory_optimizer → obstacle_avoider → speed_optimizer）
      → Rule 链（speed_limit / safety_chain / forbidden_zone / dynamic_obstacle / pedestrian）
  → ChunkBlender（时间集成 + 加速度连续）
  → Twist2D（candidate）
  → /navigation/cmd_vel_raw
```

### 3.5 依赖分层与构建目标

依赖方向只能向下：

```
autolink ─► component ─► runner ─► behavior ─► planning ─► {perception, memory, common}
                                                   │
                                                   ├─► proto
                                                   └─► platform (common)

safety ─► {common, proto, platform}          # 不依赖 planning/behavior/autolink
guardian ─► {common, proto, lifecycle}
```

| CMake 目标 | 类型 | 内容 | 关键约束 |
| --- | --- | --- | --- |
| `navigation_proto` | 共享 | 5 个 proto | protoc ≥ 3.15，proto3 optional |
| `navigation` | 共享 | 导航核心：common / perception / memory / planning / vntpa / safety / guardian / behavior / runner + Engine adapters | 单链接单元；根工程自动提供 `autonomy::navigation` 别名 |
| `navigation_observation_component` | 共享 | ObservationBuilderComponent | 链接 autolink + navigation |
| `navigation_component` | 共享 | NavigationComponent | 链接 autolink + navigation |
| `navigation_behavior_component` | 共享 | BehaviorComponent | 链接 autolink + navigation |
| `navigation_safety_component` | 共享 | SafetyComponent | 链接 autolink + navigation |
| `navigation_guardian_component` | 共享 | GuardianComponent | 链接 autolink + navigation |

> v2 设计曾按模块拆成多个静态库；当前实现参考 `autonomy/planning`、`autonomy/control`
> 的模块写法，收敛为一个共享库 `navigation`，减少链接组合与别名。
> 早期 `navigation_safety` 的 `-fno-exceptions` 约束由代码审查保证；若后续需要强隔离，
> 再拆出独立 safety target。

### 3.6 完整目录树（v2 修正）

新增文件以 `+` 标注，删除/改名以 `~` 标注。

```
navigation/
├── CMakeLists.txt
├── README.md
├── docs/04-design.md                                  # 本文件
├── navigation_component.hpp / .cpp / _test.cpp
├── runner.hpp / .cpp / _test.cpp
│
├── common/
│   ├── pose2d.hpp / pose2d_test.cpp
│   ├── twist2d.hpp / twist2d_test.cpp
│   ├── pose2d_convert.hpp / .cpp / _test.cpp
│   ├── twist2d_convert.hpp / .cpp / _test.cpp
│   ├── status.hpp / status_test.cpp
│   ├── status_or.hpp / status_or_test.cpp
│   ├── array_view.hpp / array_view_test.cpp
│   ├── spsc_ring_buffer.hpp / spsc_ring_buffer_test.cpp
│   ├── configuration_loader.hpp / .cpp / _test.cpp   # 路径复用 autonomy/common/file.hpp
│   ├── （时间戳转换）                                 # 复用 automsgs/msgs/time_utils.hpp
│   ├── joinable_thread.hpp / joinable_thread_test.cpp
│   ├── （时钟）                                      # std::chrono + autonomy::common::ToSeconds
│   ├── platform_profile.hpp
│   ├── platform_registry.hpp / .cpp / _test.cpp
│   ├── differential_drive.hpp / .cpp / _test.cpp
│   ├── omnidirectional.hpp / .cpp / _test.cpp
│   ├── ackermann.hpp / .cpp / _test.cpp
│   └── legged.hpp / .cpp / _test.cpp
│
├── perception/
│   ├── observation_encoder.hpp
│   ├── observation_builder.hpp / .cpp / _test.cpp          # + 融合组件（C4）
│   ├── defm_encoder.hpp / .cpp / _test.cpp
│   ├── depth_preprocessor.hpp / .cpp / _test.cpp
│   ├── defm_model.hpp / .cpp / _test.cpp
│   ├── attention_pooler.hpp / .cpp / _test.cpp
│   ├── depth_validity.hpp / .cpp / _test.cpp
│   └── perceptual_uncertainty_estimator.hpp / .cpp / _test.cpp
│
├── memory/
│   ├── context_view.hpp
│   ├── context_provider.hpp
│   ├── voxel_grid.hpp / .cpp / _test.cpp
│   ├── spatial_index.hpp / .cpp / _test.cpp
│   ├── keyframe_filter.hpp / .cpp / _test.cpp
│   ├── dynamic_filter.hpp / .cpp / _test.cpp
│   ├── importance_scorer.hpp / .cpp / _test.cpp
│   ├── drift_compensator.hpp / .cpp / _test.cpp
│   ├── loop_aligner.hpp / .cpp / _test.cpp
│   ├── snapshot_serializer.hpp / .cpp / _test.cpp
│   ├── semantic_scene_graph.hpp / .cpp / _test.cpp
│   └── dead_reckoning_bridge.hpp / .cpp / _test.cpp
│
├── planning/
│   ├── interface/
│   │   ├── policy_interface.hpp
│   │   ├── reflex_interface.hpp
│   │   ├── memory_interface.hpp
│   │   ├── navigation_session.hpp                    # + behavior 的窄接口
│   │   ├── scenario_base.hpp
│   │   ├── stage_base.hpp
│   │   ├── task_base.hpp
│   │   ├── rule_base.hpp
│   │   ├── plugin_exception.hpp
│   │   └── configuration_exception.hpp
│   ├── base/
│   │   ├── frame.hpp / .cpp / _test.cpp
│   │   ├── navigation_frame.hpp / .cpp / _test.cpp
│   │   └── waypoint.hpp / .cpp / _test.cpp
│   ├── plugins/
│   │   ├── policy/
│   │   │   ├── vntpa/  (policy/model_loader/preprocessor/postprocessor/
│   │   │   │            query_builder/context_builder/pose_attention/
│   │   │   │            action_decoder/manipulation_reachability_adjuster)
│   │   │   ├── vint/   (policy/model_loader/preprocessor/postprocessor/action_decoder)
│   │   │   └── nomad/  (policy/model_loader/preprocessor/postprocessor/action_decoder)
│   │   ├── reflex/
│   │   │   ├── reactive_avoider.hpp / .cpp / _test.cpp
│   │   │   └── reflex_velocity_sampler.hpp / .cpp / _test.cpp
│   │   └── memory/
│   │       ├── pose_indexed_memory.hpp / .cpp / _test.cpp
│   │       ├── frame_queue_memory.hpp / .cpp / _test.cpp
│   │       └── single_frame_memory.hpp / .cpp / _test.cpp
│   ├── scenarios/
│   │   ├── destination/  (destination_scenario.* / destination_stage.*)
│   │   ├── exploration/  (exploration_scenario.* / exploration_stage.*)
│   │   ├── following/    (following_scenario.* / following_stage.*)
│   │   └── coverage/     (coverage_scenario.* / coverage_stage.*)
│   ├── tasks/
│   │   ├── trajectory_optimizer.hpp / .cpp / _test.cpp
│   │   ├── obstacle_avoider.hpp / .cpp / _test.cpp
│   │   ├── speed_optimizer.hpp / .cpp / _test.cpp
│   │   ├── path_follower.hpp / .cpp / _test.cpp
│   │   ├── coverage_planner.hpp / .cpp / _test.cpp          # +（C8）
│   │   ├── recovery_task.hpp / .cpp / _test.cpp
│   │   ├── recovery_attributor.hpp / .cpp / _test.cpp
│   │   ├── recovery_executor.hpp / .cpp / _test.cpp
│   │   ├── relocalization_task.hpp / .cpp / _test.cpp
│   │   ├── semantic_search_task.hpp / .cpp / _test.cpp
│   │   ├── door_detector_task.hpp / .cpp / _test.cpp
│   │   └── frontier_detector_task.hpp / .cpp / _test.cpp
│   ├── rules/
│   │   ├── speed_limit_rule.hpp / .cpp / _test.cpp
│   │   ├── safety_chain_rule.hpp / .cpp / _test.cpp
│   │   ├── forbidden_zone_rule.hpp / .cpp / _test.cpp
│   │   ├── dynamic_obstacle_rule.hpp / .cpp / _test.cpp
│   │   └── pedestrian_rule.hpp / .cpp / _test.cpp
│   ├── chunk_blender.hpp / .cpp / _test.cpp                 # +（C7）
│   ├── scenario_manager.hpp / .cpp / _test.cpp
│   ├── plugin_manager.hpp / .cpp / _test.cpp
│   ├── policy_arbiter.hpp / .cpp / _test.cpp
│   └── planning_options.hpp / .cpp / _test.cpp
│
├── safety/
│   ├── safety_component.hpp / .cpp                          # +（C6）
│   ├── safety_shield.hpp
│   ├── safety_shield_implementation.hpp / .cpp / _test.cpp
│   ├── reflex_policy_arbiter.hpp / .cpp / _test.cpp
│   ├── cbf_filter.hpp / .cpp / _test.cpp
│   ├── ttc_evaluator.hpp / .cpp / _test.cpp
│   ├── braking_model.hpp / .cpp / _test.cpp
│   ├── aeb_velocity_sampler.hpp / .cpp / _test.cpp
│   ├── velocity_feasibility_filter.hpp / .cpp / _test.cpp
│   ├── degradation_controller.hpp / .cpp / _test.cpp
│   └── watchdog.hpp / .cpp / _test.cpp
│
├── behavior/
│   ├── engine/
│   │   ├── bt_engine.hpp
│   │   ├── bt_engine_implementation.hpp / .cpp / _test.cpp
│   │   └── blackboard_keys.hpp
│   ├── nodes/  (7 个节点，各 hpp/cpp/_test.cpp)
│   ├── tasks/  (task_primitive + docking/coverage/following/waypoint/elevator/door)
│   ├── intent/ (click_to_goal / language_to_goal)
│   └── trees/  (navigate_with_recovery / navigate_with_relocalization / navigate_with_door)
│
├── guardian/
│   ├── guardian_component.hpp / .cpp                        # +（C6）
│   ├── supervisor.hpp
│   ├── supervisor_implementation.hpp / .cpp / _test.cpp
│   ├── heartbeat_monitor.hpp / .cpp / _test.cpp
│   ├── degradation_policy.hpp / .cpp / _test.cpp
│   ├── diagnostics_publisher.hpp / .cpp / _test.cpp
│   ├── fault_classifier.hpp / .cpp / _test.cpp
│   └── fault_injector.hpp / .cpp / _test.cpp
│
├── proto/  (observation / navigation / lifecycle / config / plugins)
├── conf/
│   ├── navigation.pb.txt / safety.pb.txt / guardian.pb.txt
│   ├── platform/  (differential_drive / omnidirectional / ackermann / legged)
│   └── models/    (defm.pb.txt / vntpa.pb.txt / defm_vit_s14.{pth,onnx})
├── dag/  (navigation / safety / guardian)
├── launch/  (navigation / simulation / replay)
└── tools/vntpa/  (requirements / README / train / eval / explore / expert /
                   model / tokens / bank / export_onnx / config/ / checkpoints/)
```

顶层项按“构建与说明文件 2 + 入口单元 2 + 功能模块 7 + 支撑目录 5 = 16”计（C1），
本设计文档 `docs/04-design.md` 另计，不改变工程结构。

### 3.7 插件模型

- **内置注册，不加载外部 `.so`**：`PluginManager` 使用静态工厂表，
  `AUTONOMY_REGISTER_POLICY("vntpa", vntpa::Policy)`；部署确定、ABI 简单、可静态裁剪。
- 插件名来自配置（`policy.name` / `memory.name` / `reflex.name`）；未知名称是配置错误，启动失败。
- 每个插件实现 `Configure` + `Reset`；`Configure` 失败不得进入半初始化状态。
- 多策略（P6）由 `PolicyArbiter` 选择：健康度 → 显式优先级 → 置信度 → 最近成功；
  安全层拥有否决权，仲裁结果不能绕过安全链。

### 3.8 底盘抽象

`kinematic_type` 只决定 `common/platform_registry` 构造哪个 `DriveModel`；planning / safety
只依赖接口，不出现 `if (differential)` 分支。

```cpp
struct ActuatorCommand;  // 平台特有；定义在 common/platform_profile.hpp

class DriveModel
{
public:
    virtual ~DriveModel() = default;

    // 把任意候选速度裁剪到该底盘可行域（含侧向/转向约束）。
    virtual bool Clamp(const common::Twist2D& desired,
                       const common::Twist2D& measured,
                       double dt_s,
                       common::Twist2D* clamped) = 0;

    // 给定当前速度与曲率，返回该底盘的最大可行速度。
    virtual double MaxSpeedAt(double curvature_inv) const = 0;

    // 平台特有的执行器换算（差速轮速 / 阿克曼转角 / 足式步态），
    // 只在 control 边界使用，不进入规划热路径。
    virtual bool ToActuatorCommand(const common::Twist2D& command,
                                   ActuatorCommand* out) const = 0;
};
```

- `common/differential_drive`：`vy=0`，角速度→轮速差；`max_steering` 不适用。
- `common/omnidirectional`：保留 `vy`，裁剪到速度椭圆。
- `common/ackermann`：`vy=0`，曲率 `κ = ω / v` 限制 `|κ| ≤ tan(max_steer)/wheelbase`。
- `common/legged`：允许侧向，速度上限随步态/负载降额。

验收：切换 `kinematic_type` 后，`safety/` 与 `planning/` 的源码零改动，只改
`conf/platform/*.pb.txt`。

---

## 4. 消息与接口

### 4.1 proto 文件与包名

| 文件 | 包名 | C++ 命名空间 | 说明 |
| --- | --- | --- | --- |
| `proto/observation.proto` | `navigation.observation` | `navigation::observation` | 传感器融合结果与深度特征 |
| `proto/navigation.proto` | `navigation` **（v2 变更 C5）** | `navigation` | 目标、意图、动作块、记忆、反射观测 |
| `proto/lifecycle.proto` | `navigation.lifecycle` | `navigation::lifecycle` | 心跳、模块状态、诊断 |
| `proto/config.proto` | `navigation.config` | `navigation::config` | 平台、导航、安全、守护配置 |
| `proto/plugins.proto` | `navigation.plugins` | `navigation::plugins` | 各插件参数 |

> 为什么改包名：v1 的 `navigation.navigation` 会让所有代码写成 `navigation::navigation::GoalSpec`，
> 而骨架里写的是 `navigation::GoalSpec`。改成包 `navigation` 后两者一致；这 5 个 proto 只在本模块
> 内部使用，不涉及外部 `Any`，重命名无兼容负担。若未来需要跨模块复用，再引入 `navigation.v1`
> 包名并保留别名。

**生成与链接**

```cmake
# 关键点：proto_path 使用“包根”，使 import 名与生成头路径都带 navigation/ 前缀
set(NAVIGATION_PROTO_PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")  # autonomy/
protoc --cpp_out=${generated}
       --proto_path=${NAVIGATION_PROTO_PACKAGE_ROOT}
       --proto_path=${AUTOMSGS_PROTO_ROOT}          # automsgs/proto
       ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto
# 输入名为 navigation/proto/observation.proto
# 输出：generated/navigation/proto/{observation,navigation,lifecycle,config,plugins}.pb.{h,cc}
```

- 生成头路径为 `navigation/proto/xxx.pb.h`，`target_include_directories` 指向 `${generated}`。
- proto 内部 import 统一写 `navigation/proto/observation.proto`，与包根一致；
  不要写 `proto/observation.proto`，否则生成头路径会退化为 `proto/...`。
- automsgs 依赖：优先使用构建树中已有的 `automsgs::core` / `automsgs` 目标；独立构建时
  `find_package(automsgs CONFIG REQUIRED)`，并链接其 proto 库，禁止手工复制生成代码。
- 所有 proto 禁止 `import public`；跨文件依赖显式列出，避免生成头顺序耦合。
- 若工程被单独检出（不在 `autonomy/` 下），用 `NAVIGATION_PROTO_PACKAGE_ROOT` 指向
  “包含 `navigation/` 目录的父目录”，不要改 proto 内的 import 字符串。

### 4.2 关键消息（v2 草案）

**observation.proto**

```proto
syntax = "proto3";
package navigation.observation;

import "automsgs/msgs/std_msgs/header.proto";
import "automsgs/msgs/sensor_msgs/image.proto";
import "automsgs/msgs/sensor_msgs/camera_info.proto";
import "automsgs/msgs/sensor_msgs/imu.proto";
import "automsgs/msgs/nav_msgs/odometry.proto";

enum ObservationFlag {
    OBSERVATION_FLAG_UNSPECIFIED = 0;
    OBSERVATION_FLAG_DEPTH_VALID = 1;
    OBSERVATION_FLAG_CAMERA_INFO_VALID = 2;
    OBSERVATION_FLAG_ODOM_VALID = 4;
    OBSERVATION_FLAG_IMU_VALID = 8;
    OBSERVATION_FLAG_TF_VALID = 16;
    OBSERVATION_FLAG_LANGUAGE_PRESENT = 32;
}

message Language
{
    string text = 1;
    string locale = 2;
    repeated float embedding = 3 [packed = true];
}

// 一次 Tick 的完整输入快照；由 ObservationBuilder 原子构造。
message Observation
{
    automsgs.msgs.std_msgs.Header header = 1;
    automsgs.msgs.sensor_msgs.Image depth = 2;
    automsgs.msgs.sensor_msgs.CameraInfo camera_info = 3;
    automsgs.msgs.nav_msgs.Odometry odom = 4;
    automsgs.msgs.sensor_msgs.Imu imu = 5;
    Language language = 6;
    uint64 sequence = 7;
    uint32 validity = 8;              // ObservationFlag 位或
    string map_frame = 9;             // 会话坐标系，默认 "map"
    string camera_frame = 10;         // 默认 "camera_optical_frame"
}

// DeFM 输出；不进 Observation，避免热路径重复序列化。
message DepthFeature
{
    repeated float tokens = 1 [packed = true];    // [N, D] 行主序
    repeated float pooled = 2 [packed = true];    // [D]
    uint32 token_count = 3;
    uint32 token_dim = 4;
    float uncertainty = 5;                        // [0, 1]
    int64 stamp_ns = 6;
    uint64 sequence = 7;
}
```

**navigation.proto**

```proto
syntax = "proto3";
package navigation;

import "automsgs/msgs/geometry_msgs/point.proto";
import "automsgs/msgs/geometry_msgs/pose2d.proto";
import "automsgs/msgs/geometry_msgs/pose_stamped.proto";
import "automsgs/msgs/geometry_msgs/twist.proto";
import "automsgs/msgs/nav_msgs/path.proto";
import "automsgs/msgs/sensor_msgs/image.proto";
import "navigation/proto/observation.proto";

message Region
{
    repeated automsgs.msgs.geometry_msgs.Point polygon = 1;  // 外环，逆时针
    repeated automsgs.msgs.geometry_msgs.Point holes = 2;    // 内环，顺时针
    string frame_id = 3;
}

message GoalSpec
{
    enum Type
    {
        TYPE_UNSPECIFIED = 0;
        TYPE_POSE = 1;           // destination
        TYPE_EXPLORATION = 2;    // exploration
        TYPE_TRACKING = 3;       // following
        TYPE_REGION = 4;         // coverage
    }

    Type type = 1;
    optional automsgs.msgs.geometry_msgs.PoseStamped pose = 2;
    optional automsgs.msgs.sensor_msgs.Image goal_image = 3;
    optional navigation.observation.Language text = 4;
    optional double relative_bearing_rad = 5;
    optional Region region = 6;
    optional string floor_id = 7;
    string goal_id = 8;                  // 幂等键，重发不重建会话
    double position_tolerance = 9;     // 默认 0.15
    double yaw_tolerance_rad = 10;       // 默认 0.15
    uint64 sequence = 11;                // 目标更新序号，防乱序
}

message ActionChunk
{
    repeated automsgs.msgs.geometry_msgs.Twist2D actions = 1;
    double dt_s = 2;
    uint64 sequence = 3;
    int64 produced_at_ns = 4;
    string frame = 5;                    // 通常是 base_link
}

message NavigationIntent
{
    oneof output
    {
        ActionChunk chunk = 1;
        automsgs.msgs.geometry_msgs.PoseStamped waypoint = 2;
        automsgs.msgs.nav_msgs.Path path = 3;
        automsgs.msgs.geometry_msgs.Twist2D velocity = 4;
    }
    uint64 sequence = 5;
    int64 stamp_ns = 6;
    string frame = 7;
    uint32 epoch = 8;
    float confidence = 9;                // [0, 1]
}

enum RecoveryState
{
    RECOVERY_IDLE = 0;
    RECOVERY_REQUIRED = 1;
    RECOVERY_RUNNING = 2;
    RECOVERY_SUCCEEDED = 3;
    RECOVERY_FAILED = 4;
}

message NavigationStatus
{
    uint64 sequence = 1;
    int64 stamp_ns = 2;
    string scenario = 3;          // destination / exploration / following / coverage
    string stage = 4;             // 当前 Stage 名
    GoalSpec goal = 5;
    bool goal_reached = 6;
    bool degraded = 7;
    string reason = 8;
    string session_id = 9;
    uint64 goal_epoch = 10;
    bool has_active_goal = 11;
    RecoveryState recovery_state = 12;
    uint32 recovery_count = 13;
    bool pose_valid = 14;
    int64 steady_stamp_ns = 15;
}

message BehaviorRequest
{
    enum Kind { RECOVER = 0; FAIL_GOAL = 1; }
    Kind kind = 1;
    string session_id = 2;
    uint64 goal_epoch = 3;
    uint32 expected_recovery_count = 4;
    string reason = 5;
    int64 steady_stamp_ns = 6;
}

message ContextEntry
{
    uint64 id = 1;
    automsgs.msgs.geometry_msgs.Pose2D pose = 2;
    repeated float embedding = 3 [packed = true];
    float novelty = 4;
    float importance = 5;
    int64 stamp_ns = 6;
    uint32 epoch = 7;
    uint32 observation_count = 8;
}

message ContextSnapshot
{
    uint32 format_version = 1;
    uint32 epoch = 2;
    uint32 k = 3;                        // 目标条目数，不足时已填充零并置 mask
    repeated ContextEntry entries = 4;
    uint64 checksum = 5;
}
```

> `ContextSnapshot` 只用于持久化、回放和调试；进程内策略查询使用
> `memory::KeyframeBatch`（定长数组、无分配），避免在 5 Hz 热路径上构造 proto。

```proto
enum ConsistencyCode
{
    CONSISTENCY_CODE_OK = 0;
    CONSISTENCY_CODE_EPOCH_MISMATCH = 1;
    CONSISTENCY_CODE_FRAME_MISMATCH = 2;
    CONSISTENCY_CODE_STALE = 3;
    CONSISTENCY_CODE_INDEX_DEGRADED = 4;
}

message ConsistencyReport
{
    bool compatible = 1;
    ConsistencyCode code = 2;
    uint32 epoch = 3;
    string frame = 4;
    string reason = 5;
}

message ObstaclePoint
{
    double x = 1;
    double y = 2;
    double vx_mps = 3;
    double vy_mps = 4;
    double radius = 5;
    float confidence = 6;
}

message ReflexObservation
{
    repeated ObstaclePoint obstacles = 1;
    int64 stamp_ns = 2;
    bool valid = 3;
    string frame = 4;
    double coverage_radius = 5;
}
```

**lifecycle.proto**

```proto
syntax = "proto3";
package navigation.lifecycle;

enum ModuleState
{
    MODULE_STATE_UNSPECIFIED = 0;
    MODULE_STATE_STARTING = 1;
    MODULE_STATE_READY = 2;
    MODULE_STATE_DEGRADED = 3;
    MODULE_STATE_FAILED = 4;
    MODULE_STATE_STOPPED = 5;
}

enum FaultClass
{
    FAULT_CLASS_UNSPECIFIED = 0;
    FAULT_CLASS_TRANSIENT = 1;    // 自恢复，如单帧丢失
    FAULT_CLASS_PERSISTENT = 2;   // 需人工/恢复动作
    FAULT_CLASS_FATAL = 3;        // 必须零速
}

enum DiagnosticLevel
{
    DIAGNOSTIC_LEVEL_INFO = 0;
    DIAGNOSTIC_LEVEL_WARN = 1;
    DIAGNOSTIC_LEVEL_ERROR = 2;
    DIAGNOSTIC_LEVEL_FATAL = 3;
}

message Heartbeat
{
    string module = 1;            // "navigation" / "safety" / "behavior"
    uint64 sequence = 2;
    int64 stamp_ns = 3;
    ModuleState state = 4;
    string mode = 5;              // "normal" / "reflex_only" / "safe_stop"
    string detail = 6;
}

message DiagnosticReport
{
    string module = 1;
    uint32 code = 2;              // 稳定编号，见 §10.4
    DiagnosticLevel level = 3;
    FaultClass fault_class = 4;
    string message = 5;
    map<string, string> attributes = 6;
    int64 stamp_ns = 7;
    uint64 sequence = 8;
}
```

**config.proto**

```proto
syntax = "proto3";
package navigation.config;

import "navigation/proto/plugins.proto";

enum KinematicType
{
    KINEMATIC_TYPE_UNSPECIFIED = 0;
    KINEMATIC_TYPE_DIFFERENTIAL = 1;
    KINEMATIC_TYPE_OMNIDIRECTIONAL = 2;
    KINEMATIC_TYPE_ACKERMANN = 3;
    KINEMATIC_TYPE_LEGGED = 4;
}

message PlatformProfile
{
    KinematicType kinematic_type = 1;
    double max_linear_mps = 2;
    double max_lateral_mps = 3;
    double max_angular_radps = 4;
    double max_acceleration_mps2 = 5;
    double max_jerk_mps3 = 6;
    double max_lateral_acceleration_mps2 = 7;
    double footprint_radius = 8;
    double wheelbase = 9;
    double max_steering_rad = 10;
    double camera_height = 11;
    double camera_pitch_rad = 12;
    double camera_to_base_x = 13;
    double camera_to_base_y = 14;
}

message ScenarioOptions
{
    string default_scenario = 1;
    repeated string available_scenarios = 2;
}

message PolicyOptions
{
    string name = 1;
    oneof options
    {
        navigation.plugins.VntpaPolicyOptions vntpa = 2;
        navigation.plugins.VintPolicyOptions vint = 3;
        navigation.plugins.NomadPolicyOptions nomad = 4;
    }
}

message ReflexOptions
{
    string name = 1;
    oneof options
    {
        navigation.plugins.ReactiveAvoiderOptions reactive_avoider = 2;
        navigation.plugins.VelocitySamplerOptions velocity_sampler = 3;
    }
}

message MemoryOptions
{
    string name = 1;
    oneof options
    {
        navigation.plugins.PoseIndexedMemoryOptions pose_indexed = 2;
        navigation.plugins.FrameQueueMemoryOptions frame_queue = 3;
        navigation.plugins.SingleFrameMemoryOptions single_frame = 4;
    }
}

message RuleOptions
{
    repeated string enabled_rules = 1;
}

message BehaviorOptions
{
    bool enabled = 1;
    string tree = 2;                    // trees/ 下的文件名
    double tick_rate_hz = 3;
    uint32 max_recovery_attempts = 4;
}

message LimitsOptions
{
    double max_pose_jump = 1;
    double max_yaw_jump_rad = 2;
    uint32 max_consecutive_rejects = 3;
    uint32 max_hold_ms = 4;
    uint32 intent_timeout_ms = 5;
}

message DebugOptions
{
    uint32 log_level = 1;
    bool publish_debug_markers = 2;
    bool synchronous_policy_for_test = 3;
}

message PerceptionOptions
{
    double min_valid_ratio = 1;                    // 默认 0.15
    double max_range = 2;                        // 默认 8.0
    double uncertainty_slowdown_threshold = 3;     // 默认 0.6
    double uncertainty_reject_threshold = 4;       // 默认 0.85
    bool enable_attention_debug = 5;
}

message NavigationConfig
{
    string platform_profile = 1;        // preset 名，如 "differential_drive"
    ScenarioOptions scenario = 2;
    PolicyOptions policy = 3;
    ReflexOptions reflex = 4;
    MemoryOptions memory = 5;
    RuleOptions rule = 6;
    BehaviorOptions behavior = 7;
    LimitsOptions limits = 8;
    DebugOptions debug = 9;
    PerceptionOptions perception = 10;
}

message SafetyChainOptions
{
    double reflex_priority_ratio = 1;
    double max_acceleration_mps2 = 2;
    double max_jerk_mps3 = 3;
    double ttc_reaction_time_s = 4;
    double ttc_safety_margin_s = 5;
    double cbf_alpha = 6;
    uint32 watchdog_deadline_ms = 7;
    uint32 aeb_sample_count = 8;
    double min_clearance = 9;
    uint32 max_hold_ms = 10;
}

message SafetyConfig
{
    string platform_profile = 1;
    ReflexOptions reflex = 2;
    SafetyChainOptions chain = 3;
}

message GuardianConfig
{
    uint32 heartbeat_timeout_ms = 1;
    uint32 heartbeat_fatal_ms = 2;
    uint32 diagnostics_rate_hz = 3;
    bool allow_reflex_only = 4;
}

message TensorSpec
{
    string name = 1;
    repeated int64 shape = 2;
    string dtype = 3;                   // "float32" / "float16" / "bool"
}

message ModelManifest
{
    string name = 1;                    // "vntpa" / "defm_vit_s14"
    string path = 2;                    // 相对 conf/ 或制品缓存路径
    string sha256 = 3;
    string opset = 4;                   // 默认 "17"
    string precision = 5;               // "fp32" / "fp16"
    repeated TensorSpec inputs = 6;
    repeated TensorSpec outputs = 7;
    repeated float mean = 8;
    repeated float std = 9;
    uint32 embedding_dim = 10;
    uint32 token_count = 11;
}
```

**plugins.proto**

| 消息 | 关键字段 |
| --- | --- |
| `VntpaPolicyOptions` | `model_path`、`action_chunk_length`、`execute_length`、`decision_rate_hz`、`action_dt_s`、`context_size_k`、`depth_input_name`、`pose_input_name`、`context_input_name`、`context_pose_input_name`、`context_mask_input_name`、`output_name`、`pose_input_dim`、`execution_provider`、`device_id`、`precision`、`enable_reachability_adjuster`、`reach_min`、`reach_max` |
| `VintPolicyOptions` | 视觉-语言模型路径、tokenizer/vocab、`max_instruction_tokens`、`image_input_name`、`text_input_name`、`output_name`、chunk 参数 |
| `NomadPolicyOptions` | 扩散/采样步数、`num_samples`、`horizon`、`temperature`、chunk 参数 |
| `ReactiveAvoiderOptions` | `near_field_range`、`reaction_time_ms`、`safety_margin`、`fov_rad`、`candidate_count` |
| `VelocitySamplerOptions` | `horizon_s`、`sample_count`、`max_acceleration_mps2`、`safety_margin` |
| `PoseIndexedMemoryOptions` | `max_entries`、`novelty_threshold_eta`、`voxel_resolution`、`voxel_direction_bins`、`query_radius`、`min_insert_interval_s` |
| `FrameQueueMemoryOptions` | `capacity`、`stride` |
| `SingleFrameMemoryOptions` | 无 |

约束：

1. 所有数值字段必须带单位后缀；布尔字段以 `enable_` / `allow_` 开头。
2. 插件参数只放在 `plugins.proto`，`config.proto` 只引用；避免同一参数两处定义。
3. 每个 proto 预留 `reserved 100 to 199;`，为后续字段留空间。

### 4.3 热路径值类型

```cpp
namespace navigation {
namespace common {

using Pose2D = automsgs::msgs::geometry_msgs::Pose2D;   // x, y, theta
using Twist2D = automsgs::msgs::geometry_msgs::Twist2D; // x, y, theta

Pose2D Compose(const Pose2D& parent_from_child, const Pose2D& child_from_leaf);
Pose2D Inverse(const Pose2D& pose);
Pose2D Rotate(const Pose2D& pose, const Twist2D& delta, double dt_s);
double Distance(const Pose2D& lhs, const Pose2D& rhs);

}  // namespace common
}  // namespace navigation
```

- `Pose2D`/`Twist2D` 直接复用 automsgs `geometry_msgs/Pose2D`、`Twist2D`；
  四元数/SE(3) 只在 `common/pose2d_convert`、`twist2d_convert` 的边界转换中出现。
- 转换点：ObservationBuilder（proto → Pose2D）、runner（proto → Pose2D/Twist2D）、safety 入口。
  热循环内不得再做 proto 解析。
- 3-DOF **v2 变更 C10**：差速/阿克曼用 `x + theta`，`y` 必须为 0；
  全向平台可用侧向分量；`common/omnidirectional.*` 负责可行性裁剪。
- `navigation.proto` 不再自定义 `Pose2D`/`Twist2D`，字段直接引用 automsgs 类型；
  C++ 业务代码通过 `common::Pose2D` / `common::Twist2D` 别名使用同一消息。

### 4.4 接口（v2 修正签名）

```cpp
namespace navigation {
namespace planning {

class PolicyInterface
{
public:
    virtual ~PolicyInterface() = default;

    virtual bool Configure(const config::PolicyOptions& options) = 0;

    virtual StatusOr<NavigationIntent> Query(
        const observation::Observation& observation,
        const observation::DepthFeature& depth_feature,
        const GoalSpec& goal,
        const memory::ContextView& context) = 0;

    virtual void Reset() = 0;

    virtual std::string_view Name() const = 0;
};

}  // namespace planning
}  // namespace navigation
```

关键变化：

| 接口 | v1 | v2 | 原因 |
| --- | --- | --- | --- |
| `Query` 返回 | `NavigationIntent` | `StatusOr<NavigationIntent>` | 失败可区分“无动作”与“模型错误” |
| `ContextView` | 可变引用 | 不可变句柄（const） | 跨 policy worker 线程安全 |
| `Reset` | 无 | 有 | 目标切换/epoch 变化必须清状态 |
| `Name` | `std::string` | `std::string_view` | 热路径零分配 |

```cpp
// memory/context_view.hpp
namespace navigation {
namespace memory {

// 定长、不可变，供 policy worker 只读；热路径无分配（Keyframe 定义见 §6.1）。
struct KeyframeBatch
{
    static constexpr std::size_t kMaxEntries = 64;
    std::array<Keyframe, kMaxEntries> entries{};
    std::size_t size{0};
    std::uint32_t epoch{0};
    std::uint32_t format_version{1};
};

class ContextView
{
public:
    ContextView() = default;
    explicit ContextView(std::shared_ptr<const KeyframeBatch> batch);

    bool Valid() const;
    std::size_t Size() const;
    const KeyframeBatch& Batch() const;
    std::uint32_t Epoch() const;

private:
    std::shared_ptr<const KeyframeBatch> batch_{nullptr};
};

}  // namespace memory
}  // namespace navigation
```

```cpp
// planning/interface/scenario_base.hpp
class ScenarioBase
{
public:
    virtual ~ScenarioBase() = default;
    virtual bool Configure(const config::ScenarioOptions& options) = 0;
    virtual bool Accepts(GoalSpec::Type type) const = 0;
    virtual Status Update(const GoalSpec& goal) = 0;
    virtual void Reset() = 0;
    virtual StatusOr<common::Twist2D> Plan(
        NavigationFrame& frame) = 0;
};
```

```cpp
// planning/interface/stage_base.hpp
class StageBase
{
public:
    virtual ~StageBase() = default;
    virtual bool Configure(const config::NavigationConfig& configuration) = 0;
    virtual void Reset() = 0;
    virtual Status Process(NavigationFrame& frame) = 0;
};
```

```cpp
// planning/base/navigation_frame.hpp
struct NavigationFrame
{
    const observation::Observation* observation{nullptr};
    const memory::ContextView* context{nullptr};
    GoalSpec goal;
    common::Pose2D pose;
    common::Twist2D velocity;
    common::Twist2D command;
    NavigationIntent intent;
    std::uint64_t now_ns{0};
    std::uint32_t epoch{0};
    double dt_s{0.0};
    bool degraded{false};
};
```

```cpp
// behavior
class NavigationSession
{
public:
    virtual ~NavigationSession() = default;
    virtual bool SetGoal(const GoalSpec& goal) = 0;
    virtual void ClearGoal() = 0;
    virtual bool HasActiveGoal() const = 0;
    virtual bool IsGoalReached() const = 0;
    virtual bool RequestRecovery(std::string_view reason) = 0;
    virtual lifecycle::ModuleState State() const = 0;
};

class BehaviorEngine
{
public:
    virtual ~BehaviorEngine() = default;
    virtual bool Configure(const config::BehaviorOptions& options,
                           NavigationSession* session) = 0;
    virtual Status Tick(std::uint64_t now_ns) = 0;
    virtual void Reset() = 0;
};
```

### 4.4.1 外部 `navigate_to_pose` Action

`NavigationComponent` 提供：

```text
/navigate_to_pose
  type: automsgs.actions.NavigateToPoseAction
  Goal: { pose: geometry_msgs/PoseStamped, behavior_tree: string }
  Feedback: { current_pose, navigation_time, estimated_time_remaining,
              number_of_recoveries, distance_remaining }
  Result: { error_code: status_msgs/StatusCode, error_msg }
```

- Goal 转 `GoalSpec{TYPE_POSE, pose, goal_id="navigate_to_pose"}`。
- Action 与 `/navigation/goal` 都是目标源，**最新目标胜出**：
  - 新 Action goal 抢占旧 Action goal；
  - `/navigation/goal` 抢占 Action goal；
  - Action goal 覆盖旧 topic goal。
- 抢占旧 Goal 返回 `NAVIGATION_CANCELLED`，新 Goal 重新计时/反馈。
- `behavior_tree` 字段保留但当前不执行。

### 4.5 错误模型

| 层 | 机制 | 边界 |
| --- | --- | --- |
| common | `Status` / `StatusOr<T>` / `std::optional` | 不抛异常 |
| perception / memory / planning | 允许内部异常；插件边界捕获 | `PluginException` → `StatusOr` |
| safety | `-fno-exceptions -fno-rtti` | 全部 `std::optional` / `Status` |
| component | 捕获所有异常，转 `DiagnosticReport` | 进程不因单帧崩溃 |

```cpp
namespace navigation {
namespace common {

enum class ErrorCode
{
    kOk = 0,
    kInvalidArgument,
    kUnavailable,
    kStale,
    kTimeout,
    kUnsupported,
    kIo,
    kModelError,
    kConfigurationError,
    kInternal,
};

struct Status
{
    ErrorCode code{ErrorCode::kOk};
    std::string message;

    [[nodiscard]] bool Ok() const { return code == ErrorCode::kOk; }
};

template <typename T>
class StatusOr
{
public:
    StatusOr(T value);
    StatusOr(Status status);

    [[nodiscard]] bool Ok() const;
    T& Value();
    const Status& Error() const;

private:
    std::variant<T, Status> storage_;
};

}  // namespace common
}  // namespace navigation
```

诊断码与 `Status` 码分离：诊断码稳定、可被 guardian 分类。

> `common/status.hpp` 在 `navigation` 根命名空间提供 `using common::Status;` 与
> `using common::StatusOr;`，因此 `planning`/`behavior` 等子命名空间可以直接写 `Status`，
> 不必层层限定。

### 4.6 线程安全标注

每个公共类头文件必须写 `@par Threading`：

- `Runner`：单线程（autolink 调度线程）；`SetGoal`/`ClearGoal` 由行为树在同一线程调用。
- `PolicyInterface`：`Query` 在 policy worker 单线程，入参 `DepthFeature` 由 Runner 编码并缓存；
  `Configure`/`Reset` 在启动线程且不并发。
- `ContextProvider`：写（`TryInsert`）在 navigation 线程；读只通过不可变 `ContextView`。
- `SafetyShield`：safety 进程单线程。
- `ChunkBlender`：navigation 线程；`Consume` 常数时间、无分配。

---

## 5. 感知（perception）

### 5.1 ObservationBuilder

职责：把 autolink 上分散的 depth / camera_info / odom / imu / tf 融合成一次 `Observation`。

| 项 | 规定 |
| --- | --- |
| 触发 | depth 帧到达；以 depth 的 `Header.stamp` 为主时钟 |
| 缓存 | 各通道保留最近 1 帧 + 时间戳；不做队列 |
| 同步窗口 | camera_info/odom 与 depth 的时间差 ≤ 50 ms；超窗置对应 validity 位为 0 |
| TF | 查询 `map → camera_optical_frame`，超时 5 ms；失败置 `TF_VALID=0` |
| 输出 | `/navigation/observation`，`sequence` 单调递增 |
| 背压 | 只发最新帧；旧帧覆盖，不排队 |

失败语义：若 depth 有效但 TF 无效，仍发布 Observation（带 validity 位），由 Runner 决定降级；
不在这里静默补零。

```cpp
class ObservationEncoder
{
public:
    virtual ~ObservationEncoder() = default;

    // 5 Hz 调用；失败返回 Status，不抛异常穿过边界。
    virtual Status Encode(
        const observation::Observation& observation,
        observation::DepthFeature* feature) = 0;

    virtual void Reset() = 0;
};
```

### 5.2 深度预处理

- 支持编码：`32FC1`（米）、`16UC1`（毫米，×0.001）、`mono16` 视为毫米。
- 无效值：0、NaN、Inf、> `max_range`（默认 8 m）→ 无效掩码。
- 有效像素比例 < `min_valid_ratio`（默认 0.15）→ `kDepthInvalid`，不进入策略。
- 几何：按相机内参去畸变（若模型提供 D）、缩放到模型输入（默认 224×224）、保持长宽比
  letterbox，其余填 0 并记录 padding。
- 归一化：DeFM 固定均值/方差（`mean=[0.5,0.5,0.5]`、`std=[0.5,0.5,0.5]`，三通道复制），
  不做逐帧标准化，保证可复现。
- 输出：`float32` NCHW，连续内存；使用 `std::pmr::monotonic_buffer_resource`，稳态零 `new`。

### 5.3 DeFM 编码

| 项 | 值 |
| --- | --- |
| 模型 | DeFM ViT-S/14（冻结） |
| 输入 | `depth` `[1, 3, 224, 224]` float32 |
| 输出 | `tokens` `[1, 256, 384]`、`pooled` `[1, 384]` |
| 推理后端 | `autonomy::common::network::Engine`（ONNX Runtime / TensorRT） |
| 频率 | 5 Hz（与决策同频），缓存最近一次特征 |
| 预算 | P99 ≤ 15 ms（Orin NX，FP16） |
| 失败 | 模型缺失/校验失败 → `Configure` 失败；运行期 `Run` 失败 → 本帧无特征，诊断 `kModelError` |

`AttentionPooler`：对 256 个 token 做单头注意力池化（可学习 query 或均值），输出 `[1, 384]`，
供记忆入库与策略查询共用；池化权重可选发布用于调试。

### 5.4 感知不确定性

`PerceptualUncertaintyEstimator` 输出 `u ∈ [0,1]`：

```
u = clamp(w1 * invalid_ratio
        + w2 * (1 - token_norm / token_norm_ref)
        + w3 * temporal_jitter, 0, 1)
```

- `u > 0.6`：降低速度上限 30%，并在 chunk 上叠加保守偏移。
- `u > 0.85`：本帧不更新记忆，触发 `kPerceptionDegraded`。
- 权重与阈值放 `PerceptionOptions`（在 config.proto 新增，挂在 `NavigationConfig` 下），不硬编码。

### 5.5 单元测试与 Mock

- `MockEngine`：返回固定 `tokens/pooled`，用于 `defm_model_test`。
- `MockPreprocessor`：返回合成张量，用于 `defm_encoder_test`。
- `MockPooler`：验证形状与数值稳定性。
- 边界用例：全零深度、NaN、单像素有效、分辨率变化、时间戳倒退。

---

## 6. 记忆（memory）

### 6.1 数据布局

| 结构 | 布局 | 说明 |
| --- | --- | --- |
| `VoxelGrid` | SoA：`x[]`、`y[]`、`z[]`、`count[]` | 0.1 m 体素；动态点先过滤 |
| `SpatialIndex` | 哈希 `(voxel_key, direction_bin)` → `small_vector<id>` | 方向 16 桶；查询半径可调 |
| `PoseIndexedMemory` | 连续 `std::pmr::vector<Keyframe>` + 空闲链表 | 容量固定，无稳态分配 |
| `SemanticSceneGraph` | 节点表 + 邻接表 | 房间/门/电梯/语义地标 |

```cpp
struct Keyframe
{
    std::uint64_t id{0};
    common::Pose2D pose;
    std::array<float, kMaxEmbeddingDim> embedding{};  // kMaxEmbeddingDim = 512
    std::uint32_t embedding_dim{0};                   // 运行期维度，默认 384
    float novelty{0.0F};
    float importance{0.0F};
    std::int64_t stamp_ns{0};
    std::uint32_t epoch{0};
    std::uint32_t observation_count{0};
};
```

`embedding_dim` 由模型 manifest 给出（默认 384）；编译期上限 `kMaxEmbeddingDim = 512`，
超过上限的配置直接报错。

### 6.2 关键帧接受

`KeyframeFilter::TryAccept`：

```
novelty = 1 - max_{k in N(pose)} cosine(embedding, k.embedding)
accept = novelty > eta
      && dt_since_last > min_interval
      && motion > min_motion
      && memory.size() < max_entries
```

- `eta` 默认 0.08；`min_interval` 默认 0.2 s；`min_motion` 默认 0.05 m / 0.05 rad。
- 容量满时按 `importance` 淘汰：`importance = w_r * recency + w_n * novelty + w_g * goal_proximity`。
- 入库与索引更新在同一临界区；P99 < 2 ms（不含模型推理）。

### 6.3 查询

`PoseIndexedMemory::Query(pose, k, radius)`：

1. 位姿所在体素及邻域（半径 `query_radius`，默认 2.0 m）取候选。
2. 按 `pose_distance` + `-importance` 排序，取前 K（默认 32）。
3. 不足 K 时用零 embedding + mask 补齐；`KeyframeBatch.size` 固定为 K。
4. 跨 epoch 条目直接过滤；若可用条目 < K/4，返回 `ConsistencyReport{CONSISTENCY_CODE_INDEX_DEGRADED}`。

预算：K=32、半径 2 m 时 P99 ≤ 1 ms。

```cpp
class ContextProvider
{
public:
    virtual ~ContextProvider() = default;

    virtual Status Insert(const Keyframe& keyframe) = 0;
    virtual ContextView View(const common::Pose2D& pose) const = 0;
    virtual std::size_t Size() const = 0;
    virtual void Reset(bool keep_pose_index) = 0;
    virtual ConsistencyReport CheckConsistency(
        const common::Pose2D& pose, std::uint32_t epoch) const = 0;
};
```

- `View` 返回不可变快照；构造在 navigation 线程，读取在 policy worker。
- `Insert` 不做模型推理；特征由 `ObservationEncoder` 提供。
- `CheckConsistency` 在每帧校验 epoch/frame/索引健康度，失败按 §2.6 处理。

### 6.4 漂移与回环

- `DriftCompensator`：收到回环校正 `(old_pose, new_pose)` 后，对受影响条目做 SE(2) 变换，
  `epoch++`，增量重建索引；不阻塞 Tick（在 worker 或下一帧前完成，带 deadline）。
- `LoopAligner`：embedding + 位姿先验检索候选，几何校验（相对距离/航向）后接受；
  拒绝率与误匹配率进诊断。
- 校正失败 → 保持旧 epoch，标记 `kDriftUncorrected`，不影响安全（安全层不依赖记忆）。

### 6.5 快照与恢复

- `SnapshotSerializer`：magic `NAVSNAP` + `format_version` + `epoch` + 条目 + CRC32；
  写入临时文件后 `rename` 原子替换。
- 加载时校验版本、CRC、维度、epoch；任一失败 → 拒绝加载（不部分恢复）。
- 快照只在启动/诊断时使用，不进入控制热路径。

### 6.6 DeadReckoningBridge

- odom 短时缺失（≤ 200 ms）时，用 `Twist2D` 积分外推位姿，标记 `extrapolated=true`。
- 超过 200 ms → 位姿无效，停止入库，Runner 进入保持/减速。
- 外推期间不写入新关键帧，避免污染记忆。

### 6.7 测试

- VoxelGrid：插入/查询/边界/负坐标。
- KeyframeFilter：novelty 单调性、容量淘汰、时间门限。
- DriftCompensator：已知变换后索引误差 < 体素分辨率。
- SnapshotSerializer：往返一致、CRC 损坏拒绝、版本不兼容拒绝。
- 增长测试：10 km 仿真轨迹下条目数收敛到 `max_entries`，不随时长线性增长。

---

## 7. 规划（planning）

### 7.1 场景映射

| `GoalSpec.type` | 场景 | 触发条件 | 核心 Stage/Task |
| --- | --- | --- | --- |
| `TYPE_POSE` | destination | 默认；`TYPE_UNSPECIFIED` 且有 pose | policy → trajectory_optimizer → obstacle_avoider → speed_optimizer |
| `TYPE_EXPLORATION` | exploration | 无目标位姿 | frontier_detector_task → trajectory_optimizer → obstacle_avoider |
| `TYPE_TRACKING` | following | 外部持续更新目标位姿 | target_predictor → trajectory_optimizer（高频重查询） |
| `TYPE_REGION` | coverage | 提供 `Region` | coverage_planner → trajectory_optimizer → obstacle_avoider |

```cpp
Status ScenarioManager::Update(const GoalSpec& goal)
{
    ScenarioBase* next = nullptr;
    switch (goal.type())
    {
        case GoalSpec::TYPE_POSE:        next = destination_.get(); break;
        case GoalSpec::TYPE_EXPLORATION: next = exploration_.get(); break;
        case GoalSpec::TYPE_TRACKING:    next = following_.get(); break;
        case GoalSpec::TYPE_REGION:      next = coverage_.get(); break;
        case GoalSpec::TYPE_UNSPECIFIED:
            next = goal.has_pose() ? destination_.get() : nullptr;
            break;
        default:
            return common::Status{common::ErrorCode::kInvalidArgument, "unknown goal type"};
    }
    if (next == nullptr)
    {
        return common::Status{common::ErrorCode::kInvalidArgument, "goal type requires a target"};
    }
    if (next != active_)
    {
        active_ = next;
        active_->Reset();
    }
    return active_->Update(goal);
}
```

约定：

- `TYPE_UNSPECIFIED` 且没有 pose → 不启动场景，保持 IDLE 并返回 `kInvalidArgument`。
- 场景由 `ScenarioManager::Update(goal)` 选择，**无优先级栈**；同一时刻只有一个场景。
- 目标切换：先 `Cancel`（清空 chunk、平滑到零），再切场景；不允许两个场景同时写 command。
- 窄通道不是场景：由 `safety/cbf_filter` 连续约束 + `reflex_policy_arbiter` 权重曲线处理。
- 门/电梯不是场景：几何检测在 `door_detector_task`，语义交互在 `behavior/tasks/door_task`、
  `elevator_task`。

### 7.2 Stage 管线

```cpp
class DestinationScenario final : public ScenarioBase
{
public:
    StatusOr<common::Twist2D> Plan(NavigationFrame& frame) override;

private:
    std::unique_ptr<PolicyStage> policy_stage_{nullptr};
    std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_{nullptr};
    std::unique_ptr<ObstacleAvoider> obstacle_avoider_{nullptr};
    std::unique_ptr<SpeedOptimizer> speed_optimizer_{nullptr};
};
```

每个 Stage 的契约：

1. 只读写 `NavigationFrame`；不得直接发消息。
2. 失败返回 `Status`，由场景决定是恢复、降级还是零速。
3. 任何 Stage 不得把速度改到超过 `frame.command` 的安全上限（规则链随后再校核）。
4. Stage 必须可独立单测（输入 frame，输出 frame）。

当策略输出 `path` / `waypoint` 而不是 chunk 时，由 `path_follower` 把导航意图转换成速度，
再进入规则链；chunk 意图由 `trajectory_optimizer` 直接处理。两种路径共享同一套规则与安全链。

### 7.3 规则链

顺序固定（先硬约束后舒适约束）：

```
speed_limit → safety_chain → forbidden_zone → dynamic_obstacle → pedestrian
```

| 规则 | 语义 | 失败动作 |
| --- | --- | --- |
| `speed_limit` | 平台/场景/区域限速 | 裁剪 |
| `safety_chain` | 把 navigation 侧安全裕度映射为速度上限 | 裁剪 + 诊断 |
| `forbidden_zone` | 禁区多边形（含膨胀半径） | 拒绝并零速 |
| `dynamic_obstacle` | 动态障碍 TTC | 减速/绕行 |
| `pedestrian` | 行人近场权重大于动态障碍 | 停车等待 |

规则只做“收缩”，不做“扩张”；任何规则拒绝 → 本帧零速并上报，不允许其他规则覆盖。

### 7.4 PolicyArbiter（P6）

- 默认单策略：`policy.name` 指定，仲裁器直通。
- 多策略启用时：健康度（最近失败率）→ 显式 `priority` → `confidence` → 最近成功时间。
- 安全层对仲裁结果有否决权；仲裁器不得改变安全链顺序。
- 切换策略必须 `Reset` 新策略并清空 chunk，避免动作块拼接跳变。

### 7.5 VNTPA 插件

#### 7.5.1 数据流

```
DepthFeature（Runner 编码） ──► Preprocessor ──► depth_tokens [1,N,D]
pose + goal ──► QueryBuilder ──► query_token [1,D]
memory ──► ContextBuilder ──► context [1,K,D] + pose_bias [1,K]
                    │
                    ▼
             PoseAttention（相对位姿编码）
                    ▼
             Transformer planner
                    ▼
             action_chunk [1,T,3]
                    ▼
     Postprocessor（反归一化/限幅） → ActionDecoder（可选 GRU）
                    ▼
          ManipulationReachabilityAdjuster（可选）
```

#### 7.5.2 QueryBuilder

- 输入：`Pose2D current`、`GoalSpec`（pose 或相对方位）。
- 特征：`[x, y, sin θ, cos θ]` 转换到目标坐标系 + 傅里叶编码（4 个频带）。
- 目标缺位姿时（探索/覆盖）：用当前子目标；无子目标 → 零向量 + `has_goal=false`。
- 输出维度必须等于 `pose_input_dim`（manifest 校验）。

#### 7.5.3 ContextBuilder

- 调 `PoseIndexedMemory::Query(pose, K, radius)`；按距离升序。
- `depth_feature` 由 Runner 的 `ObservationEncoder` 生成并随查询传入；ContextBuilder 不重新编码。
- 每个条目：`embedding`（D）+ `pose_bias`（相对位姿的傅里叶编码）。
- mask：无效条目置 0，`attention_mask=0`。
- K 默认 32；超过 `max_context_k` 配置报错。

#### 7.5.4 PoseAttention

- 相对位姿 `(dx, dy, dθ)` 经 MLP 生成 attention bias，加到 self-attention logits。
- 位置编码使用固定频率（不学习），保证不同分辨率/地图尺度下可迁移。
- 训练与推理必须使用同一归一化常数（写入 manifest）。

#### 7.5.5 模型 I/O

| 名称 | 形状 | dtype | 说明 |
| --- | --- | --- | --- |
| `depth_feature` | `[1, N, D]` | float32 | DeFM tokens |
| `query_token` | `[1, D]` | float32 | 位姿+目标 |
| `context` | `[1, K, D]` | float32 | 关键帧嵌入 |
| `context_pose` | `[1, K, P]` | float32 | 相对位姿编码 |
| `context_mask` | `[1, K]` | bool | 有效掩码 |
| `action_chunk` | `[1, T, 3]` | float32 | `(vx, vy, ω)` |

- ONNX opset 17；动态轴只允许 batch=1；`T`、`K` 固定。
- `ModelLoader` 用 `Engine::CreateEngine(InferenceOptions, &error)`；加载后校验输入/输出名称、
  形状、dtype，不符 → `Configure` 失败（不降级为随机动作）。
- 模型文件校验：sha256 与 manifest 一致；否则拒绝加载。

#### 7.5.6 后处理

1. 反归一化（训练统计写入 manifest）。
2. 平台限幅（`max_linear_mps`、`max_lateral_mps`、`max_angular_radps`）。
3. 非有限值检测：任一 NaN/Inf → 整块丢弃，`kModelOutputInvalid`。
4. 平滑：与上一块重叠部分做加权平均（权重随预测步长线性衰减）。
5. 置信度：由 `uncertainty` 与输出方差估计；`< 0.3` 时只执行 `execute_length` 步并请求重规划。

#### 7.5.7 ActionDecoder

- 默认恒等（ONNX 已输出动作）。
- 可选 GRU 解码：输入 `[1, T, D]` 隐状态，输出 `[1, T, 3]`；用于低维 token 模型。
- 解码器权重独立于 ONNX，放 manifest 的 `decoder` 段；缺失则禁用。

#### 7.5.8 ManipulationReachabilityAdjuster

- 仅在 `enable_reachability_adjuster=true` 时启用（默认 false）。
- 作用：当目标位姿进入机械臂可达环 `[reach_min, reach_max]` 时，微调最终底盘位姿，
  使目标落在可达域；调整量 ≤ 0.15 m / 0.2 rad。
- 不得改变路径中段；只在最后 `execute_length` 步生效。
- 纯导航任务不启用，避免引入操纵耦合。
- adjuster 的输出仍然经过规则链与安全链；它不能绕过 CBF/可行性检查。

#### 7.5.9 训练闭环（tools/vntpa）

| 文件 | 职责 |
| --- | --- |
| `expert.py` | 仿真专家：A* + DWA/MPC 生成动作标签 |
| `bank.py` | 关键帧库：位姿、深度特征、动作、场景元数据 |
| `tokens.py` | 位姿 tokenizer / 傅里叶编码，与 C++ 实现逐位对齐 |
| `model.py` | Transformer planner 定义 |
| `train.py` / `eval.py` / `explore.py` | 训练、离线评估、在线探索 |
| `export_onnx.py` | 导出 opset 17，校验 I/O 名称/形状，生成 manifest |
| `config/*.yaml` | 数据、模型、训练、评估超参 |

验收：导出后的 ONNX 在 `MockEngine` 与真实 Engine 上输出一致（`atol=1e-4`）。

### 7.6 VINT / NOMAD 适配器

| 插件 | 输入 | 适用 | 备注 |
| --- | --- | --- | --- |
| VNTPA | depth + pose + 记忆 | 默认，长程目标 | 位姿索引记忆是核心 |
| VINT | 图像 + 语言指令 | 语言目标、无记忆 | 指令解析在 `language_to_goal` |
| NOMAD | depth + 局部目标 | 无图、短程 | 扩散/采样解码，延迟较高 |

三者实现同一 `PolicyInterface`；切换只改 `policy.name` 与对应 options。新增插件只允许改
`planning/plugins/policy/` 与 `conf/navigation.pb.txt` 的 policy 段，这是 P6 的验收条件。

### 7.7 ChunkBlender（v2 新增）

```cpp
class ChunkBlender
{
public:
    struct Options
    {
        std::size_t chunk_length{8};
        std::size_t execute_length{2};
        double action_dt_s{0.1};
        double blend_tau{2.0};
    };

    explicit ChunkBlender(Options options);

    void Push(const ActionChunk& chunk, std::uint64_t now_ns);
    bool Consume(double dt_s, common::Twist2D* command);
    void Reset();
    bool HasActions() const;

private:
    struct Slot
    {
        common::Twist2D action;
        double confidence{1.0};
    };

    Options options_;
    std::array<Slot, kMaxChunkLength> slots_{};
    double position_{0.0};          // 以 action 步为单位，可为小数
    std::uint64_t last_chunk_ns_{0};
};
```

算法（时间基，不是“一 tick 一步”）：

1. 控制 tick 15 Hz（66 ms）与动作 `dt=0.1 s` 不同频，`position_` 每 tick 前进
   `dt_tick / action_dt_s`（约 0.66 步），必须做**线性插值**，否则实际速度会放大约 1.5 倍。
2. `Push` 从 `ceil(position_)` 开始覆盖剩余槽位；新动作与旧槽按
   `w_i = exp(-i / τ)` 融合：`action = w_i * new + (1 - w_i) * old`，`confidence = min(...)`。
3. `Consume` 在 `floor(position_)` 与 `ceil(position_)` 之间插值，再过加速度/jerk 限幅
   （与 `velocity_feasibility_filter` 同参数），保证块边界连续。
4. `position_ ≥ T` 且无新块 → 返回 false，Runner 进入“滑行到零”分支（受 `max_hold_ms` 约束）。
5. 单次 `Consume` O(1)，T ≤ 16，P99 < 100 µs；无分配。

相邻块切换的无突变验收：在仿真中随机注入 1000 次块边界，加速度差分
`|a[k] - a[k-1]| < max_jerk * dt`，且速度不出现 1.5× 台阶。

### 7.8 恢复与重定位

`RecoveryAttributor` 归因：

| 归因 | 条件 | 执行器动作 |
| --- | --- | --- |
| `kLocalizationLost` | 位姿连续无效 / epoch 不一致 | 暂停、重定位、请求操作员 |
| `kPathBlocked` | 连续 N 帧速度 ≈ 0 且障碍距离 < 阈值 | 后退/旋转/绕行 |
| `kGoalUnreachable` | 规划器多次失败 / 目标在禁区内 | 上报、请求新目标 |
| `kSensorDegraded` | 深度有效性低 / 模型失败 | 降速、切换 reflex |
| `kStuck` | 位移 < 0.05 m 持续 5 s | 恢复序列 |

`RecoveryExecutor` 序列：`clear chunk → 停车 → 重采样 → 重规划 → 恢复`；每次失败改变失败条件
（例如放宽到达容差、切换拓扑），避免死循环。恢复次数与结果进诊断。

`RelocalizationTask`：触发全局重定位，成功后 `epoch++`、清空 chunk、重建索引；失败则保持零速。

### 7.9 场景切换语义

```
SetGoal(new):
  1. 若 goal_id 相同且 type 相同 → 仅更新位姿/容差，不重置策略
  2. 否则 Cancel()：停止发新动作，chunk 平滑到零（≤ 200 ms）
  3. ScenarioManager.Update(goal) → 新 Scenario::Configure + Reset
  4. 新场景第一帧必须从零速开始（避免跨场景拼接）
```

不允许“边切换边执行旧动作块”；`Cancel` 期间的 command 必须经过安全链。

---

## 8. 安全（safety，独立进程）

### 8.1 三级链定义

安全链不是“三个并联过滤器”，而是**有序、逐级收紧**的三级：

| 级 | 名称 | 输入 | 输出 | 预算 | 失败动作 |
| --- | --- | --- | --- | --- | --- |
| 0 | 输入校验 | cmd、深度、位姿、dt、时间戳 | 合法/非法 | 1 ms | 零速 |
| 1 | 反射覆盖 | 候选 cmd、近场障碍 | 反射修正后的 cmd | 30 ms | 零速 |
| 2 | CBF/AEB | 反射 cmd、障碍集合 | 安全集内 cmd | 1 ms（CBF） | AEB 采样 → 零速 |
| 3 | 可行性 | 安全 cmd、实测速度 | 满足 a/j 的 cmd | 1 ms | 沿安全方向减速 → 零速 |

整体硬截止 50 ms；`SafetyShieldImplementation::Process` 超时率必须为 0。

### 8.2 处理流程

```cpp
SafetyResult SafetyShieldImplementation::Process(
    const common::Twist2D& candidate,
    const ReflexObservation& reflex_observation,
    const common::Twist2D& measured,
    std::int64_t now_ns,
    double dt_s)
{
    // 0) 输入校验：有限值、时间新鲜、dt 合法、位姿连续
    if (!ValidateInputs(...)) {
        return Zero("invalid_input");
    }

    // 1) 反射优先：近场风险时覆盖方向；无风险时透传
    const common::Twist2D reflex =
        reflex_arbiter_.Arbitrate(candidate, reflex_observation);

    // 2) CBF 投影；不可行则 AEB 采样（修正优先于零速）
    std::optional<common::Twist2D> safe =
        cbf_filter_.Project(reflex, reflex_observation);
    if (!safe.has_value()) {
        safe = aeb_sampler_.Sample(reflex, reflex_observation, config_);
    }
    if (!safe.has_value()) {
        return Zero("infeasible");
    }

    // 3) 可行性限幅 + 复核；限幅破坏安全则沿安全方向减速
    common::Twist2D feasible =
        feasibility_filter_.Filter(*safe, measured, dt_s);
    if (!cbf_filter_.IsSafe(feasible, reflex_observation)) {
        feasible = DecelerateAlongSafeDirection(feasible, measured, dt_s);
    }
    if (!cbf_filter_.IsSafe(feasible, reflex_observation)) {
        return Zero("unsafe_after_clamp");
    }

    // 4) 看门狗：超时仍然必须发布零速
    if (watchdog_.Expired()) {
        return Zero("deadline");
    }
    return Result(feasible, ...);
}
```

**不变量**

1. `Zero()` 是任何失败路径的唯一出口；不存在“返回上一帧”。
2. 第 3 级永远执行，即使 AEB 未触发；加速度/jerk 不得超限。
3. AEB 触发时，若存在“减速 + 转向”的可行解，优先修正；只有无解才零速。
4. 处理是确定性的：相同输入序列与相同内部状态产生相同输出；不使用随机采样。
5. 任意一级抛错/超时/返回无效 → 零速。

### 8.3 反射层（Stage 1）

- 输入：近场障碍（`coverage_radius` 内，默认 2.0 m）、候选速度、`reaction_time_ms`。
- `ReactiveAvoider`：在速度空间采样（角度 × 速度），用 TTC + 间隙打分，选最小代价方向。
- `ReflexPolicyArbiter`：按风险连续加权：

```
if min_ttc >= (reaction_time + safety_margin):
    w_reflex = 0                                  # 无风险：透传 policy
else:
    risk = clamp(1 - min_ttc / (reaction_time + safety_margin), 0, 1)
    w_reflex = max(reflex_priority_ratio, risk)   # 有风险：至少 0.2，随风险升到 1
command = (1 - w_reflex) * policy + w_reflex * reflex
```

- `w_reflex ≥ 0.8` 时视为反射接管，发布 `kReflexOverride` 诊断；行为树据此暂停任务。
- 反射层不得产生比候选速度更大的前向速度；只允许减速/转向。

```cpp
struct ReflexCommand
{
    common::Twist2D velocity;
    bool intervened{false};     // true: 反射接管方向
    double risk{0.0};           // [0, 1]
};

class ReflexInterface
{
public:
    virtual ~ReflexInterface() = default;

    // 近场反射：返回建议速度与是否接管；不修改输入。
    virtual ReflexCommand Compute(
        const ReflexObservation& observation,
        const common::Twist2D& desired,
        double dt_s) = 0;
};
```

### 8.4 CBF 与 AEB（Stage 2）

对每个障碍 `i` 定义 barrier：

```
h_i(p) = ||p - p_i|| - (r_robot + r_i + margin)
ḣ_i    = -n_i · (v - v_i),  n_i = (p - p_i) / ||p - p_i||
约束：ḣ_i + α h_i ≥ 0  →  n_i · v ≤ α h_i + n_i · v_i
```

求解：

```
minimize   ||u - u_ref||²
subject to A u ≤ b            (障碍半平面)
           u_min ≤ u ≤ u_max  (平台速度盒)
```

- 2 维变量、≤ 32 约束；用固定 20 次投影梯度（或小规模 active-set 闭式解），
  不引入外部 QP 求解器，保证确定性。
- 不可行 → `AebVelocitySampler`：在 `[0, v_current] × [ω_min, ω_max]` 采样
  （默认 9×5），用 `BrakingModel` 预测制动轨迹，剔除碰撞样本，选“前进最多且安全”的解。
- 仍不可行 → 零速。
- 数值保护：`h_i < 0` 时直接判不安全；`||p - p_i|| < 1e-6` 时用上一帧法向。

### 8.5 可行性（Stage 3）

- 加速度限幅：`|v - v_measured| ≤ a_max * dt`；角加速度同理。
- jerk 限幅：`|a - a_prev| ≤ j_max * dt`；需保存上一周期加速度。
- 限幅后再做一次 CBF 检查；若破坏安全，沿“上一安全方向”以 `a_max` 减速。
- 零速在速度盒内；若单周期从当前速度减到 0 会超过 `a_max`，则按 `a_max` 减速，
  下一周期继续，直至零（不会因“必须立即零速”而伪造不可行解）。

### 8.6 看门狗与实时性

- `Watchdog` 使用 `steady_clock`；`Process` 入口记起点，出口检查；超时返回零速并计数。
- 超时后丢弃已排队的 tick，避免安全进程积压造成“补发旧命令”。
- 安全进程 50 Hz，周期 20 ms；硬截止 50 ms（含发布）。
- 预分配：障碍数组、采样数组、约束矩阵全部定长（≤ 32），稳态零分配。
- `-fno-exceptions -fno-rtti`；禁止 `dynamic_cast`、`std::function` 热路径、`iostream`、`malloc`。
- 若平台提供 `mlockall` / 实时优先级，由 launch 配置，不在代码中假设。

### 8.7 失效矩阵

| 失效 | 检测 | 动作 | 诊断码 |
| --- | --- | --- | --- |
| cmd 超时/NaN | Stage 0 | 零速 | 5000 |
| 深度超时/空洞 | Stage 0 | 零速 | 1001 |
| CBF 不可行 | Stage 2 | AEB → 零速 | 5001 |
| AEB 采样无解 | Stage 2 | 零速 | 5002 |
| 限幅破坏安全 | Stage 3 | 减速 → 零速 | 5001 |
| 处理超时 | Watchdog | 零速 | 5003 |
| 进程崩溃 | 底盘/guardian | 零速 | 6001 |
| 输入帧率不足 | 计数器 | 降速 + 告警 | 1000 |

### 8.8 SafetyComponent

```cpp
class SafetyComponent final : public autolink::TimerComponent
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(SafetyComponent)
    AUTONOMY_DISABLE_COPY(SafetyComponent)

    SafetyComponent();
    ~SafetyComponent() override;

    void Clear() override;

protected:
    bool Init() override;

private:
    bool Proc() override;

    std::unique_ptr<SafetyShield> shield_{nullptr};
    std::shared_ptr<autolink::Reader<automsgs::msgs::geometry_msgs::Twist>> command_reader_{nullptr};
    std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::Twist>> command_writer_{nullptr};
};
```

- 读端：`/navigation/cmd_vel_raw`、`/camera/depth/image_raw`、`/camera/camera_info`、`/odom`。
- `Proc`：取最新 cmd/depth/odom → 构建 `ReflexObservation`（近场点、覆盖半径、时间戳）→
  `SafetyShield::Process` → 发布 `/cmd_vel` 与状态；深度/里程计无效时直接零速。
- 写端：`/cmd_vel`、`/safety/status`、`/safety/heartbeat`。
- 定时器组件没有 `readers` 配置，读端必须在 `Init()` 里创建（v2 变更 C2/C3）。
- 心跳每周期发布；状态包含当前模式、AEB 次数、超时次数。

---

## 9. 行为（behavior）

### 9.1 引擎选型

- 采用 **BehaviorTree.CPP 4.x**；behavior 库链接 `BT::behaviortree_cpp`。
- `BtEngine` 是薄封装：加载 XML、注册节点、tick、黑板（blackboard）；不实现调度器。
- `BtCppEngine` 加载 XML 并注册内建节点；`BtEngineImplementation` 保留为独立测试原语，默认组件加载失败不会使用它回退。
- 树文件使用 BT.CPP v4 格式（`<root BTCPP_format="4">`）。

### 9.2 与 Runner 的关系

BehaviorComponent 持有 BtCppEngine 与 BehaviorSession，默认 Timer 100 ms。
Runner 持有 RecoveryManeuver，由 NavigationComponent 接收目标/行为请求后调用。
默认 DAG 同进程；真实验收将 Behavior 拆成独立 mainboard，所有操作经 IPC。
目标只由 NavigationComponent 接收，BT 的 StartNavigation 验证当前快照目标，
不会重复提交/清空 Runner。Behavior 心跳由组件自己发布，不由 Runner 发布。

### 9.3 黑板与 IPC

`node_context` 为 BtCppEngine 所有的 NodeContext*，由 BehaviorComponent 更新
目标、位姿有效性、状态新鲜度与请求回调。`max_recovery_attempts` 为配置次数预算。
BehaviorSession 读取 NavigationStatus 的会话、目标版本、恢复状态/次数；
恢复/失败请求用 BehaviorRequest 经 `/navigation/behavior_request` 传回 Navigation。
接收方拒绝不活跃目标、旧会话/目标版本、不匹配次数和超过 500 ms 的请求。

### 9.4 默认恢复节点

StartNavigation、CheckPoseHealth、CheckModuleHealth 沿用内建节点；
WaitForGoalReached 根据目标到达返回 SUCCESS、无进度/失败返回 FAILURE，其他情况 RUNNING。
RecoveryNode 有导航/恢复两个子节点，共享 Runner 已接受的次数预算；
PipelineSequence 在下游执行时复查上游，失败时 halt 下游。
RecoveryExecutor 请求并等待异步确认与动作结果，成功后恢复导航、失败可换侧重试。
RequestOperatorAssist 在耗尽后锁存原因并请求终止目标，Action 返回 PATH_BLOCKED。
原有安全/可达/重定位条件节点保留，但其未接入的任务链不在默认树验收范围内。

### 9.5 任务原语

| 原语 | 阶段 | 说明 |
| --- | --- | --- |
| `DockingTask` | Search → Approach → Align → FinalApproach → Verify | 对接；可调用 reachability adjuster |
| `CoverageTask` | Decompose → Order → ExecuteSegment → Verify | 牛耕式覆盖，逐段调度 |
| `FollowingTask` | Acquire → Track → KeepDistance → RecoverLost | 目标丢失进入搜索 |
| `WaypointSequenceTask` | Next → Navigate → Arrive | 多航点串行 |
| `ElevatorTask` | Call → WaitDoor → Enter → SelectFloor → Ride → Exit | 6 阶段，含超时/重试 |
| `DoorTask` | Detect → Approach → Open → Pass → Release | 语义门；几何检测在 planning |

原语必须能脱离完整树单测：输入 mock `NavigationSession` + 时间，输出阶段转移。

### 9.6 树

默认 `navigate_with_recovery.xml` 实际执行如下：

```xml
<root BTCPP_format="4" main_tree_to_execute="NavigateWithRecovery">
  <BehaviorTree ID="NavigateWithRecovery">
    <Fallback>
      <Sequence>
        <CheckPoseHealth/>
        <CheckModuleHealth/>
        <RecoveryNode number_of_retries="{max_recovery_attempts}">
          <PipelineSequence>
            <StartNavigation/>
            <WaitForGoalReached timeout_s="600"/>
          </PipelineSequence>
          <RecoveryExecutor/>
        </RecoveryNode>
      </Sequence>
      <RequestOperatorAssist reason="recovery_exhausted"/>
    </Fallback>
  </BehaviorTree>
</root>
```

Runner 的 steady 8 s 无进度触发 RECOVERY_REQUIRED；实际位姿驱动
face_flank → flank → face_goal → advance，完成后恢复原始目标策略查询。
配置距离默认各 2 m，阶段上限 25 s，恢复次数配置默认 3。
恢复不跳过 Safety；Behavior 心跳过期时暂停恢复命令，取消/抢占清空状态。
具体参数、协议、CBF 缓冲边界与真实验证见 [19](19-behavior-recovery-validation.md)。

`navigate_with_relocalization.xml` 与 `navigate_with_door.xml` 为任务示例，
完整组件联调/验收仍属 T-17，不视为可直接替换默认树的已验收功能。

### 9.7 意图

- `ClickToGoal`：像素 + 深度 + 相机内参 + TF → 射线与地面交点 → `PoseStamped(map)`；
  射线平行/超距/命中无效深度 → 拒绝并给出原因。
- `LanguageToGoal`：语言 → 语义场景图检索（房间/门/电梯/地标）→ `GoalSpec`；
  无匹配 → 交给 VINT 策略或请求澄清，不猜测。
- 意图转换只产生 `GoalSpec`，不直接产生速度。

---

## 10. 守护与诊断（guardian，独立进程）

### 10.1 心跳

| 来源 | 通道 | 周期 | 超时（3×） | 致命（6×） |
| --- | --- | --- | --- | --- |
| navigation | `/navigation/heartbeat` | 15 Hz | 200 ms | 400 ms |
| safety | `/safety/heartbeat` | 50 Hz | 60 ms | 120 ms |
| behavior | `/behavior/heartbeat` | 10 Hz | 300 ms | 600 ms |

- 心跳携带 `sequence`、`stamp_ns`、`state`、`mode`。
- 乱序/回退的 `sequence` 视为丢失；重复旧心跳不刷新超时。
- guardian 自己不产生控制命令。

### 10.2 降级状态机

```
Normal ──(nav 心跳超时)──► ReflexOnly ──(恢复)──► Normal
   │                          │
   │(safety 致命/超时)         │(持续超时)
   ▼                          ▼
SafeStop ◄────────────────────┘
   │
   │(操作员确认 + 全部健康)
   ▼
 Manual ──(操作员释放)──► Normal
```

| 模式 | 导航 | 安全 | 底盘 |
| --- | --- | --- | --- |
| `normal` | 正常 | 三级链 | 正常 |
| `reflex_only` | 停发新动作 | 反射 + 零速兜底 | 仅反射速度 |
| `safe_stop` | 停止 | 零速 | 制动 |
| `manual` | 停止 | 零速 | 人工接管 |

转移必须原子发布（一条 `Degradation` 消息含 `from/to/reason`）；重复消息幂等。

### 10.3 故障分类

| 类别 | 判据 | 例子 | 动作 |
| --- | --- | --- | --- |
| `TRANSIENT` | 单次/短窗口，自动恢复 | 单帧丢深度、一次推理超时 | 记录，不降级 |
| `PERSISTENT` | 连续 N 次或 > 1 s | 心跳丢失、TF 断开、模型持续失败 | ReflexOnly |
| `FATAL` | 安全链故障/看门狗/内存损坏 | 安全进程无心跳、处理超时、快照损坏 | SafeStop |

分类器输入：心跳、诊断、退出码；规则表可配置，默认规则写入 `guardian.pb.txt`。

### 10.4 诊断码

| 段 | 码 | 含义 |
| --- | --- | --- |
| 1000–1099 | 1001/1002/1003/1004/1005 | 深度/位姿/TF/跳变/epoch |
| 2000–2099 | 2001/2002/2003/2004 | 模型加载/推理/输出/深度无效 |
| 3000–3099 | 3001/3002/3003 | 索引退化/快照损坏/漂移未校正 |
| 4000–4099 | 4001/4002/4003/4004 | 策略饥饿/意图超时/目标不可达/恢复失败 |
| 5000–5099 | 5001/5002/5003/5004 | 超时/CBF 不可行/AEB/看门狗 |
| 6000–6099 | 6001/6002 | 心跳超时/致命故障 |

### 10.5 故障注入（6 类）

| 注入 | 方法 | 期望状态 |
| --- | --- | --- |
| 心跳丢失 | 停发 navigation 心跳 | 200 ms → ReflexOnly |
| 深度陈旧 | 冻结 depth 时间戳 | Runner 无效帧 + 诊断 1001；持续 → ReflexOnly |
| TF 断开 | 移除 map→camera 变换 | 主动降级，不静默等待 |
| 模型失败 | 替换为损坏 ONNX / 注入 Run 失败 | 策略不可用 → ReflexOnly |
| 记忆损坏 | 破坏快照 CRC / 注入越界条目 | 拒绝加载，保持旧记忆/空记忆 |
| 安全超时 | 注入 60 ms 阻塞 | 零速 + 诊断 5003，超时率统计 +1 |

注入器只在测试构建启用（`NAVIGATION_ENABLE_FAULT_INJECTION=ON`），生产构建剔除。

### 10.6 GuardianComponent

- 定时器 500 ms；`Init` 创建心跳 reader、状态/降级 writer。
- `Proc`：更新心跳状态 → 分类 → 状态机 → 发布 `Degradation` + `DiagnosticReport`。
- 自身心跳写 `/guardian/heartbeat`（供外部监控）。
- 任何异常 → 进入 SafeStop（fail-closed），不静默退出。

---

## 11. 配置、部署与运维

### 11.1 配置分层

| 文件 | 所有者 | 内容 |
| --- | --- | --- |
| `conf/navigation.pb.txt` | navigation | 引用平台 preset；scenario/policy/reflex/memory/rule/behavior/limits/debug |
| `conf/safety.pb.txt` | safety | 引用平台 preset；reflex；三级链参数 |
| `conf/guardian.pb.txt` | guardian | 心跳窗口、分类规则、诊断频率 |
| `conf/platform/*.pb.txt` | common | `PlatformProfile` 预设（4 种底盘） |
| `conf/models/defm.pb.txt` | perception | DeFM ONNX manifest（sha256/I/O） |
| `conf/models/vntpa.pb.txt` | planning | VNTPA ONNX manifest（占位，待权重） |

**单一来源（C12）**：运行时只读 `navigation.pb.txt` / `safety.pb.txt` / `guardian.pb.txt`；
插件、场景、规则参数全部内联在 `navigation.pb.txt` 的 `policy` / `scenario` / `rule` 段。
不再维护 `conf/plugin/*`、`conf/scenario/*`、`conf/rule/*` 预设，避免同一参数两份来源。

> 当前平台 preset 的查找路径为 `$NAVIGATION_CONF_DIR/conf/platform/<name>.pb.txt`；
> `NAVIGATION_CONF_DIR` 指向 navigation 资源根目录。未设置时相对于工作目录解析，
> 不自动回退安装目录；部署时须显式设置。`platform_profile` 可写短名或配置文件路径。

### 11.2 校验

启动前（以及 CI）执行：

1. 文本格式可解析、字段名拼写正确。
2. 枚举/范围：`execute_length ≤ chunk_length`、`decision_rate_hz ≤ tick_rate`、
   `watchdog_deadline_ms ∈ [period_ms, 3 × period_ms]`、`ttc_safety_margin < ttc_reaction`。
3. 交叉引用：`platform_profile` 存在；`policy.name`/`memory.name`/`reflex.name` 已注册；
   scenario 名在 `available_scenarios` 内；规则名已注册。
4. 模型：文件存在、sha256 匹配、输入输出名与 options 一致、形状非零。
5. 平台：`max_lateral_mps > 0` 仅允许全向/足式；差速/阿克曼必须为 0。

任一失败 → 进程启动失败并打印 `kConfigurationError`；禁止“带病启动”。

### 11.3 DAG（按 autolink schema 重写）

**dag/navigation.dag**

```
module_config {
  module_library: "libnavigation_observation_component.so"
  components {
    class_name: "navigation::perception::ObservationBuilderComponent"
    config {
      name: "observation_builder"
      config_file_path: "navigation/conf/navigation.pb.txt"
      readers { channel: "/camera/depth/image_raw" }
      readers { channel: "/camera/camera_info" }
      readers { channel: "/odom" }
      readers { channel: "/imu" }
    }
  }
}

module_config {
  module_library: "libnavigation_component.so"
  components {
    class_name: "navigation::NavigationComponent"
    config {
      name: "navigation"
      config_file_path: "navigation/conf/navigation.pb.txt"
      readers { channel: "/navigation/observation" }
    }
  }
}
```

**dag/safety.dag**

```
module_config {
  module_library: "libnavigation_safety_component.so"
  timer_components {
    class_name: "navigation::safety::SafetyComponent"
    config {
      name: "safety"
      config_file_path: "navigation/conf/safety.pb.txt"
      interval: 20
    }
  }
}
```

**dag/guardian.dag**

```
module_config {
  module_library: "libnavigation_guardian_component.so"
  timer_components {
    class_name: "navigation::guardian::GuardianComponent"
    config {
      name: "guardian"
      config_file_path: "navigation/conf/guardian.pb.txt"
      interval: 500
    }
  }
}
```

**写端不在 DAG 中声明**（`ComponentConfig` 没有 `writers` 字段），由代码创建：

> `class_name` 必须与 `AUTOLINK_REGISTER_COMPONENT(...)` 的实参字符串完全一致（含命名空间），
> 因为 autolink 的 class_loader 按字符串查找；例如：
> `AUTOLINK_REGISTER_COMPONENT(navigation::NavigationComponent)` 对应
> `class_name: "navigation::NavigationComponent"`。`module_library` 由 `AUTOLINK_LIB_PATH`
> 或进程工作目录解析，部署时必须保证库路径一致。

| 组件 | 写端 |
| --- | --- |
| ObservationBuilder | `/navigation/observation` |
| Navigation | `/navigation/cmd_vel_raw`、`/navigation/status`、`/navigation/diagnostic`、`/navigation/heartbeat` |
| Behavior | `/navigation/behavior_request`、`/behavior/heartbeat` |
| Safety | `/cmd_vel`、`/safety/status`、`/safety/heartbeat` |
| Guardian | `/guardian/status`、`/guardian/degradation`、`/guardian/heartbeat` |

### 11.4 启动

```xml
<!-- launch/navigation.launch -->
<autolink>
    <module>
        <name>navigation</name>
        <dag_conf>navigation/dag/navigation.dag</dag_conf>
        <process_name>navigation</process_name>
    </module>
    <module>
        <name>safety</name>
        <dag_conf>navigation/dag/safety.dag</dag_conf>
        <process_name>safety</process_name>
    </module>
    <module>
        <name>guardian</name>
        <dag_conf>navigation/dag/guardian.dag</dag_conf>
        <process_name>guardian</process_name>
    </module>
</autolink>
```

- `simulation.launch`：在 navigation.launch 基础上追加 autosim 模块与合成观察源。
- `replay.launch`：用 `tools/replay` 注入录制的 `Observation`，navigation/safety 正常启动。
- 三个进程独立崩溃；safety 对 cmd 超时零速；guardian 只发降级意图。

### 11.5 安装布局

```
lib/libnavigation_component.so
lib/libnavigation_observation_component.so
lib/libnavigation_safety_component.so
lib/libnavigation_guardian_component.so
include/navigation/...
share/navigation/conf/{navigation,safety,guardian}.pb.txt
share/navigation/conf/{platform,plugin,scenario,rule,models}/...
share/navigation/dag/*.dag
share/navigation/launch/*.launch
share/navigation/trees/*.xml
```

### 11.6 模型制品

- DeFM ViT-S/14 的 `.pth`/`.onnx` 为本地制品，已 gitignore；VNTPA planner `.onnx` 仍为外部制品，仓库只保留 `vntpa.pb.txt` 与导出脚本。
- manifest 字段：`name`、`path`、`sha256`、`opset`、`precision`、`inputs[]`、`outputs[]`、
  `normalization{mean,std}`、`embedding_dim`、`token_count`。
- 缓存目录：`$NAVIGATION_MODEL_CACHE`（默认 `~/.cache/navigation/models`）；按 sha256 命名。
- TensorRT engine 缓存 key = `sha256 + gpu + trt_version`；升级任一因素即重建。
- 加载顺序：manifest 校验 → 文件存在 → engine 创建 → I/O 校验 → 策略可用。
- `VntpaPolicyOptions.model_path` 只负责定位；完整性只由 manifest 的 `sha256` 校验。
- 配置中不再有 `model_sha256` 字段，避免“配置哈希 + manifest 哈希”双重来源。

### 11.7 可观测性

| 通道 | 类型 | 频率 | 用途 |
| --- | --- | --- | --- |
| `/navigation/cmd_vel_raw` | Twist | 15 Hz | 候选速度 |
| `/cmd_vel` | Twist | 50 Hz | 安全后速度 |
| `/navigation/status` | 自定义 | 15 Hz | 场景/阶段/目标状态 |
| `/navigation/heartbeat` | Heartbeat | 15 Hz | 守护 |
| `/safety/heartbeat` | Heartbeat | 50 Hz | 守护 |
| `/behavior/heartbeat` | Heartbeat | 10 Hz | 守护 |
| `/guardian/degradation` | Degradation | 事件 | 模式切换 |
| `/navigation/diagnostic` | DiagnosticReport | 事件 | 故障 |
| `/navigation/debug/*` | 调试 | 按需 | 仅 debug 构建 |

所有通道必须带 `sequence` 与 `stamp_ns`；禁止无时间戳消息。

---

## 12. 性能预算

参照平台：

- 机器人端：Jetson Orin NX 16GB，JetPack 6，TensorRT 10，FP16。
- 开发端：x86_64 i7-13700 + RTX 4060，ONNX Runtime CUDA。

| 路径 | P50 | P99 | 硬上限 | 说明 |
| --- | --- | --- | --- | --- |
| ObservationBuilder | 1 ms | 3 ms | 8 ms | 含 TF |
| DepthPreprocessor | 3 ms | 8 ms | 15 ms | 224×224 |
| DeFM 编码 | 6 ms | 15 ms | 25 ms | FP16 |
| KeyframeFilter::TryAccept | 0.3 ms | 2 ms | 4 ms | 容量 5000 |
| Memory::Query (K=32) | 0.2 ms | 1 ms | 2 ms | 半径 2 m |
| Policy 推理（VNTPA） | 60 ms | 180 ms | 250 ms | 异步 |
| ChunkBlender::Consume | 10 µs | 100 µs | 200 µs | T ≤ 16 |
| Scenario 管线 | 0.5 ms | 3 ms | 8 ms | 不含推理 |
| 端到端（观察→cmd_raw） | 80 ms | 250 ms | 350 ms | 异步 |
| CbfFilter::Solve | 0.2 ms | 1 ms | 2 ms | ≤ 32 约束 |
| SafetyShield::Process | 1 ms | 5 ms | 50 ms | 50 Hz |
| Guardian 周期 | 2 ms | 10 ms | 20 ms | 2 Hz |

测量方式：`tools/benchmark` 用固定输入回放 10k 次，输出 P50/P99；CI 记录趋势，
超过上表 1.2× 即失败。

> ObservationBuilder / DepthPreprocessor / DeFM / KeyframeFilter / Memory::Query / Scenario
> 都在 navigation Tick 线程；Policy 推理在独立 worker。编码帧的合计预算 ≈ 3 + 8 + 15 + 2 + 1 + 3
> ≈ 32 ms，低于 35 ms 上限与 66 ms 周期。

---

## 13. 测试与验收

### 13.1 单元测试矩阵

| 模块 | 必测类 | 测试文件数 | Mock |
| --- | --- | --- | --- |
| common | Pose2D / Twist2D / Pose2DConvert / StatusOr / SpscRingBuffer / 平台模型 | ≥ 10 | 无 |
| perception | DepthPreprocessor / DeFMEncoder / DepthValidity / Uncertainty / Builder | ≥ 5 | MockEngine / MockPooler |
| memory | VoxelGrid / SpatialIndex / KeyframeFilter / DriftCompensator / Snapshot | ≥ 5 | MockVoxelGrid |
| planning | PluginManager / VNTPA Policy+Preprocessor / ScenarioManager / ChunkBlender | ≥ 6 | MockModelLoader / MockPolicy |
| safety | CbfFilter / Feasibility / Watchdog / AEB / Shield | ≥ 5 | 合成障碍 |
| behavior | 7 节点 + Elevator/Door + BtEngine | ≥ 9 | MockSession |
| guardian | HeartbeatMonitor / FaultClassifier / DegradationPolicy | ≥ 3 | 无 |
| runner/component | Runner / NavigationComponent / ObservationBuilder | ≥ 3 | MockEncoder/MockMemory/MockPolicy |

### 13.2 集成与系统测试

1. **闭环仿真**：destination / exploration / following / coverage 各 200 回合；含静态、动态障碍。
2. **退化场景**：传感器退化、SLAM 失效、目标不可达、窄通道、电梯/门。
3. **故障注入**：§10.5 六类，全部进入预期状态。
4. **确定性**：固定随机种子，两次运行输出逐位一致。
5. **配置矩阵**：4 种底盘 × 3 种策略 × 4 场景，启动成功率 100%，无代码改动。
6. **Sanitizer**：ASan/UBSan/TSan 全量通过；`-Werror -std=c++20`（目标验收项，本轮未运行 sanitizer）。
7. **延迟基准**：§12 表格全项达标。

### 13.3 数值验收

| 指标 | 门槛 |
| --- | --- |
| 静态场景成功率 | ≥ 98% |
| 动态障碍成功率 | ≥ 90% |
| 探索覆盖率（10 min） | ≥ 90% 可通行区域 |
| 覆盖作业面积完成率 | ≥ 95%，路径 ≤ 1.2× 最优 |
| 恢复成功率 | ≥ 90% |
| 安全违规（碰撞/超限） | 5,000 回合为 0 |
| 故障注入降级正确率 | 100% |
| 心跳超时切换 | ≤ 1.5× 超时窗口 |
| 策略饥饿降级 | 400 ms 内进入减速，800 ms 内零速 |

### 13.4 验收口径

- 所有指标在参照平台上测量；仿真与实机分开报告。
- “成功率”定义：到达目标容差内且未触发安全违规，且无人工干预。
- 延迟指标取 P99，样本 ≥ 10,000。
- 任何一项不达标不得进入 P6/发布。

---

## 14. 实施路线

| 阶段 | 内容 | 依赖 | 退出准则 |
| --- | --- | --- | --- |
| P0 | common 契约、5 proto、safety 三级链骨架、guardian 心跳 | — | 位姿连续性、三级链、心跳超时单测通过 |
| P1 | perception DeFM、memory 关键帧/漂移/快照 | P0 | 离线特征验证；记忆收敛；漂移误差 < 体素 |
| P2 | VNTPA 插件、Runner 异步、chunk blender、训练闭环 | P0/P1 | 仿真闭环“目标→动作块→cmd_raw”；块切换无突变 |
| P3 | 安全链完整（CBF/AEB/可行性/降级） | P0/P2 | 修正优先零速；Stage3 始终限幅；故障零速 |
| P4 | behavior 节点/原语/树、恢复、门/电梯 | P2/P3 | 恢复回归；原语独立单测 |
| P5 | guardian 分类/降级/注入、诊断 | P0/P3 | 六类注入全部预期降级 |
| P6 | VINT/NOMAD、PolicyArbiter、配置矩阵 | P2/P4/P5 | 切 policy/底盘/场景零改动 |

顺序约束：P0 必须先落 C2/C3/C5，否则 P2 无法编译；P3 不依赖 P4，可与 P2 并行。

---

## 15. 风险与待决

### 15.1 风险

| 风险 | 影响 | 缓解 |
| --- | --- | --- |
| DeFM 模型不可得/许可不明 | 感知不可用 | 冻结接口 + Mock 模型；训练脚本可复现 |
| ONNX/TensorRT 版本漂移 | 推理失败 | manifest 固定 opset；engine 缓存按版本隔离 |
| BT.CPP 依赖不可引入 | 行为层缺失 | `null` 引擎 + 自研最小执行器（仅顺序/回退） |
| 15 Hz 与推理延迟不匹配 | 抖动/超时 | 异步 + chunk 覆盖 0.8 s；降级路径明确 |
| 深度/位姿时间不同步 | 策略输入错位 | ObservationBuilder 严格同步窗口 + validity 位 |
| 记忆漂移 | 长程退化 | epoch + 回环校正 + 定期重建；安全层不依赖记忆 |
| 配置组合爆炸 | 部署错误 | validator + 矩阵测试 + 单一来源 |
| 安全链实时性 | 超时 | 定长内存、无异常、50 ms 硬截止、看门狗 |

### 15.2 待决决策（建议默认值）

| 编号 | 问题 | 建议 | 备选 |
| --- | --- | --- | --- |
| D1 | `Twist2D` 2-DOF 还是 3-DOF | **3-DOF**（C10） | 保持 2-DOF，全向底盘降级为差速 |
| D2 | BT 引擎 | **BehaviorTree.CPP 4.x** | 自研最小执行器 |
| D3 | 模型制品 | **外部制品库 + sha256**（C13） | Git LFS |
| D4 | navigation.proto 包名 | **`navigation`**（C5） | 保留 `navigation.navigation` + 别名 |
| D5 | behavior 承载 | **BehaviorComponent**，默认同 DAG，可拆独立进程 | Runner 内置 tick |
| D6 | 策略调用 | **异步 worker**（C9） | 同步（仅仿真/单测） |
| D7 | CBF 求解 | **固定迭代投影梯度** | 外部 QP（Osqp/qpOASES） |

---

## 16. 编码规范与工程约束

### 16.1 文件与注释

| 项 | 规定 |
| --- | --- |
| 文件头 | 版权块 + `/** @file @brief @details @par @see */` |
| 头文件保护 | `#pragma once`（对齐 `autolink/context.hpp`） |
| 缩进 | 4 空格，禁止 Tab；列宽 80 |
| 函数体左括号 | 独占一行（`.clang-format` 需 `AfterFunction: true`） |
| 命名空间 | 传统嵌套 `namespace navigation { namespace planning {`，结尾逐层注释 |
| 类注释 | `@brief` + `@details` + `@par Invariants` / `@par Threading` / `@par Ownership` |
| 参数注释 | `@param[in]` / `@param[out]` / `@return` / `@note` / `@warning` |
| 注释语言 | 英文 |
| 析构 | `~Class() override = default;` |

### 16.2 命名

| 类别 | 规则 | 示例 |
| --- | --- | --- |
| 目录 | `snake_case`，单单词 | `perception/`、`memory/` |
| 文件 | `snake_case`，≤ 4 单词 | `pose_indexed_memory.cpp` |
| 类 | `PascalCase` | `PoseIndexedMemory`、`vntpa::Policy` |
| 函数 | `PascalCase`，动词开头 | `TryInsert`、`ComputeReflexCommand` |
| 变量 | `snake_case` + 单位 | `novelty_score`、`max_deceleration_mps2` |
| 成员 | `snake_case_` | `encoder_`、`voxel_grid_` |
| 常量 | `kPascalCase` | `kMaxEntries`、`kDefaultTtcMargin` |
| 枚举 | C++ `kPascalCase`；proto `UPPER_SNAKE` | `kHealthy` / `TYPE_POSE` |
| 宏 | `AUTONOMY_<NAME>` | `AUTONOMY_SMART_PTR_DEFINITIONS` |

- 插件文件名去前缀：`planning/plugins/policy/vntpa/` 下统一 `policy.hpp`、`model_loader.hpp`、
  `preprocessor.hpp`、`postprocessor.hpp`、`query_builder.hpp`、`context_builder.hpp`、
  `pose_attention.hpp`、`action_decoder.hpp`、`manipulation_reachability_adjuster.hpp`，
  由 `navigation::planning::vntpa` 命名空间隔离。
- 成员初始化：`{nullptr}` / `{}` / `{0}`；禁止未初始化 POD。

### 16.3 C++20 工具链与接口约定

- 强制使用：`std::optional`、`std::variant` + `std::visit`、`std::string_view`、
  `std::filesystem`、`if constexpr`、`std::shared_mutex`、`std::scoped_lock`、
  结构化绑定、折叠表达式、`inline constexpr`、`[[nodiscard]]`、`std::pmr`、`std::byte`、
  `std::clamp`、`std::apply`、`std::invoke`、`std::as_const`、
  `std::hardware_destructive_interference_size`。
- 当前 umbrella 构建使用 C++20；已有接口继续使用 `ArrayView`、`JoinableThread`：
  `std::span` → `ArrayView`；`std::jthread` → `JoinableThread`；
  `std::format` → `fmt`；`[[likely]]` → `__builtin_expect` 宏。
- 禁止：`std::regex` 热路径、`std::function` 热路径、RTTI（safety）、异常（safety）、
  `iostream`（库代码）、隐式 `double`/`float` 混算。

### 16.4 异常与边界

- 决策层（perception/memory/planning/behavior）允许异常，但不得穿越模块边界；
  插件/组件边界统一转 `StatusOr` / `DiagnosticReport`。
- 安全层 `-fno-exceptions -fno-rtti`；所有错误路径显式返回 `std::optional` / `Status`。
- 禁止在析构函数中抛异常；禁止在 `noexcept` 函数中调用可能抛出的代码。

### 16.5 性能约束

- 热路径零堆分配：`KeyframeFilter::TryAccept`、`CbfFilter::Solve`、`ChunkBlender::Consume`、
  `SafetyShield::Process` 使用 `std::pmr` 预分配或定长数组。
- SoA 布局：`VoxelGrid` 的 `x[]`、`y[]`、`z[]` 分离；避免 AoS 跨缓存行。
- 热路径避免虚函数：reflex/safety 用 `variant` + `visit` 或 CRTP。
- 跨线程通信用 `SpscRingBuffer`，`alignas(std::hardware_destructive_interference_size)`
  隔离读写索引，避免 false sharing。
- 所有公共类标注 `@par Threading`；所有跨线程数据标注所有权。

### 16.6 构建与静态检查

- `-std=c++17 -Wall -Wextra -Wpedantic -Werror`；CI 另跑 clang-tidy（bugprone/performance）。
- 安全库编译选项：`-fno-exceptions -fno-rtti`，并在 CI 检查链接闭包不含 `libstdc++` 异常符号。
- 禁止在头文件中 `using namespace`；禁止宏污染（使用后 `#undef`）。
- 生成代码（`*.pb.h/cc`）不参与格式检查。

---

## 附录 A：v1 → v2 修订对照

| 文件/主题 | v1 | v2 |
| --- | --- | --- |
| 顶层结构 | 11 项 | 16 项（另加 `docs/04-design.md`） |
| `proto/navigation.proto` | 包 `navigation.navigation` | 包 `navigation` |
| `proto/observation.proto` | 仅 Observation/DepthFeature/Language | 增加 CameraInfo/Odometry/Imu/validity/sequence |
| `navigation_component` | `Stop() override` | `Clear() override` + 注册宏 |
| `safety/safety_component.*` | 缺失 | 新增 |
| `guardian/guardian_component.*` | 缺失 | 新增 |
| `perception/observation_builder.*` | 缺失 | 新增 |
| `planning/chunk_blender.*` | 缺失 | 新增 |
| `planning/tasks/coverage_planner.*` | 缺失 | 新增 |
| `common/twist2d.hpp` | 2-DOF | 3-DOF |
| `memory/context_view.hpp` | 可变引用 | 不可变快照句柄 |
| `dag/*.dag` | 非法字段 | 按 autolink schema 重写 |
| `conf/navigation.pb.txt` | 内联平台 + safety_chain | 平台 preset + 安全链归 safety |
| `conf/models/*.onnx` | 本地权重（gitignore） | manifest + 外部权重 |
| 验收 | 定性 | 量化门槛 + 参照平台 |

## 附录 B：主配置示例（v2）

```textproto
# conf/navigation.pb.txt
platform_profile: "differential_drive"

scenario {
  default_scenario: "destination"
  available_scenarios: ["destination", "exploration", "following", "coverage"]
}

policy {
  name: "vntpa"
  vntpa {
    model_path: "conf/models/vntpa.onnx"
    action_chunk_length: 8
    execute_length: 2
    decision_rate_hz: 5.0
    action_dt_s: 0.1
    context_size_k: 32
    depth_input_name: "depth_feature"
    pose_input_name: "query_token"
    context_input_name: "context"
    output_name: "action_chunk"
    execution_provider: "cuda"
    device_id: 0
    precision: "fp16"
    enable_reachability_adjuster: false
  }
}

reflex {
  name: "reactive_avoider"
  reactive_avoider {
    near_field_range: 2.0
    reaction_time_ms: 30
    safety_margin: 0.3
    fov_rad: 1.57
    candidate_count: 32
  }
}

memory {
  name: "pose_indexed"
  pose_indexed {
    max_entries: 5000
    novelty_threshold_eta: 0.08
    voxel_resolution: 0.1
    voxel_direction_bins: 16
    query_radius: 2.0
    min_insert_interval_s: 0.2
  }
}

rule {
  enabled_rules: ["speed_limit", "safety_chain", "forbidden_zone",
                  "dynamic_obstacle", "pedestrian"]
}

behavior {
  enabled: true
  tree: "navigate_with_recovery.xml"
  tick_rate_hz: 10.0
  max_recovery_attempts: 3
}

limits {
  max_pose_jump: 0.5
  max_yaw_jump_rad: 0.52
  max_consecutive_rejects: 3
  max_hold_ms: 200
  intent_timeout_ms: 400
}

debug {
  log_level: 1
  publish_debug_markers: false
  synchronous_policy_for_test: false
}
```

```textproto
# conf/safety.pb.txt
platform_profile: "differential_drive"

reflex {
  name: "reactive_avoider"
  reactive_avoider {
    near_field_range: 2.0
    reaction_time_ms: 30
    safety_margin: 0.3
    fov_rad: 1.57
    candidate_count: 32
  }
}

chain {
  reflex_priority_ratio: 0.2
  max_acceleration_mps2: 1.0
  max_jerk_mps3: 5.0
  ttc_reaction_time_s: 0.15
  ttc_safety_margin_s: 0.1
  cbf_alpha: 2.0
  watchdog_deadline_ms: 50
  aeb_sample_count: 45
  min_clearance: 0.05
  max_hold_ms: 200
}
```

```textproto
# conf/platform/differential_drive.pb.txt
kinematic_type: KINEMATIC_TYPE_DIFFERENTIAL
max_linear_mps: 0.5
max_lateral_mps: 0.0
max_angular_radps: 1.0
max_acceleration_mps2: 0.8
max_jerk_mps3: 2.0
max_lateral_acceleration_mps2: 0.5
footprint_radius: 0.3
wheelbase: 0.0
max_steering_rad: 0.0
camera_height: 0.18
camera_pitch_rad: 0.0
camera_to_base_x: 0.12
camera_to_base_y: 0.0
```

## 附录 C：通道清单

| 通道 | 方向 | 类型 | 频率 |
| --- | --- | --- | --- |
| `/camera/depth/image_raw` | in | `sensor_msgs/Image` | 15–30 Hz |
| `/camera/camera_info` | in | `sensor_msgs/CameraInfo` | 15–30 Hz |
| `/odom` | in | `nav_msgs/Odometry` | 50 Hz |
| `/imu` | in | `sensor_msgs/Imu` | 100–200 Hz |
| `/tf` | in | `tf2_msgs/TFMessage` | 50 Hz |
| `/navigation/observation` | internal | `navigation.observation.Observation` | 15 Hz |
| `/navigation/cmd_vel_raw` | internal | `geometry_msgs/Twist` | 15 Hz |
| `/cmd_vel` | out | `geometry_msgs/TwistStamped` | 50 Hz |
| `/navigation/status` | out | 自定义 | 15 Hz |
| `/navigation/heartbeat` | out | `lifecycle.Heartbeat` | 15 Hz |
| `/safety/heartbeat` | out | `lifecycle.Heartbeat` | 50 Hz |
| `/behavior/heartbeat` | out | `lifecycle.Heartbeat` | 10 Hz |
| `/guardian/degradation` | out | 自定义 | 事件 |

## 附录 D：验收检查单

- [ ] `-std=c++20` 全量编译通过；全仓 `-Werror` 另行验收。
- [ ] ASan/UBSan/TSan 通过。
- [ ] 5 个 proto 生成、字段号与保留段检查通过。
- [ ] 3 份 DAG 可被 autolink 加载，class_name 注册正确。
- [ ] 4 场景 × 3 策略 × 4 底盘配置矩阵启动成功。
- [ ] 三级链故障注入全部零速。
- [ ] 心跳超时在 1.5× 窗口内完成降级。
- [ ] 块切换 1,000 次无加速度突变。
- [ ] 记忆 10 km 收敛，快照损坏可拒绝。
- [ ] 热路径延迟达标（§12）。
- [ ] 模型 sha256/I/O 校验失败即拒绝启动。

---

**一句话**：v2 在不改变 v1 架构方向的前提下，补齐了进程入口、融合节点、动作混合器，
修正了 proto 命名、DAG schema、Component 生命周期和异步推理模型，并把验收从“定性描述”
收敛为可测量的契约。按 P0 先落 C2/C3/C5/C9，再进入实现。

2026-11-09实现补充：基础`/tf` map→odom缓存与平面位姿合成已接入ObservationBuilder，
尚未实现上文完整map→camera TF链/5 ms查询/时效性契约。动作块融合仅消费仍有效
的重叠轨迹，不能复用已耗尽槽位；探索及构建回归证据见 [20](20-exploration-validation.md)。
