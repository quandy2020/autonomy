# 08 · Navigation v2 验收矩阵

> TODO/闭环：[13-todo.md](13-todo.md)。需求基线：[01-requirements.md](01-requirements.md)。对应 [04-design.md](04-design.md) §13。状态：✅ 已实现并有 CTest；🟡 部分实现；⛔ 外部依赖阻塞。

## 1. 模块与测试

| 模块 | 验收项 | 测试 | 状态 |
| --- | --- | --- | --- |
| common | Pose2D 组合/逆/距离 | `pose2d_test` | ✅ |
| 模型 | DeFM ViT-S/14 pth/ONNX + sha256 | `conf/models/defm.pb.txt` | ✅ |
| common | Twist2D 速度幅值 | `twist2d_test` | ✅ |
| common | Status / StatusOr | `status_test`、`status_or_test` | ✅ |
| common | ArrayView | `array_view_test` | ✅ |
| common | SPSC 环形缓冲（含并发） | `spsc_ring_buffer_test` | ✅ |
| common | JoinableThread | `joinable_thread_test` | ✅ |
| common | 文本 proto 配置加载 | `configuration_loader_test` | ✅ |
| common | automsgs 位姿/速度/时间转换 | `pose2d_convert_test`、`twist2d_convert_test` | ✅ |
| common | 平台模型：差速/全向/阿克曼/足式 | `platform_registry_test` | ✅ |
| perception | 深度有效性/归一化/letterbox | `depth_preprocessor_test` | ✅ |
| perception | 注意力池化 | `attention_pooler_test` | ✅ |
| perception | DeFM encoder 形状/元数据 | `defm_encoder_test` | ✅ |
| perception | 多传感器融合/validity | `observation_builder_test` | ✅ |
| memory | VoxelGrid 查询/清空 | `voxel_grid_test` | ✅ |
| memory | 关键帧 novelty/门限 | `keyframe_filter_test` | ✅ |
| memory | 位姿索引记忆/一致性 | `pose_indexed_memory_test` | ✅ |
| memory | 快照 CRC/版本/文件往返 | `snapshot_serializer_test` | ✅ |
| memory | 漂移校正/epoch | `drift_compensator_test` | ✅ |
| memory | 动态过滤/重要性/航位推算 | `memory_modules_test` | ✅ |
| memory | 语义场景图 | `semantic_scene_graph_test` | ✅ |
| memory | 回环候选/漂移校正 | `loop_aligner_test` | ✅ |
| memory | 位姿哈希索引 | `spatial_index_test` | ✅ |
| memory | SingleFrame / FrameQueue 插件 | `memory_plugins_test` | ✅ |
| planning | ChunkBlender 时间插值/耗尽 | `chunk_blender_test` | ✅ |
| planning | 场景类型映射/规则链 | `scenario_manager_test` | ✅ |
| planning | 覆盖率路径/航点跟随 | `coverage_planner_test`、`path_follower_test` | ✅ |
| planning | 速度/避障/轨迹优化 | `optimizers_test` | ✅ |
| planning | 恢复归因/执行序列 | `recovery_tasks_test` | ✅ |
| planning | 多策略仲裁 | `policy_arbiter_test` | ✅ |
| 仿真 | ObservationBuilderComponent / 仿真策略 / goal sender | 待构建（autolink） | 🟡 |
| 交互 | 外部 `/navigate_to_pose` Action（feedback/result/cancel/抢占） | `navigation_component`（autolink） | 🟡 待构建 |
| planning | reflex 插件 v2 | `reflex_plugin_test`（新增，按用户要求未构建） | 🟡 |
| planning | `RecoveryTask` 包装 | `recovery_task_test`（新增，待构建） | 🟡 |
| planning | PlanningOptions / 语义搜索 / 重定位 | `planning_gaps_test`（新增，待构建） | 🟡 |
| planning | coverage stage | `coverage_stage_test` | ✅ |
| planning | VNTPA query/postprocess/policy | `vntpa_query_builder_test`、`vntpa_postprocessor_test`、`vntpa_policy_test` | ✅ |
| planning | VINT/NOMAD 备选策略（goal-seeking fallback） | `policies_test` | ✅ |
| tools | VNTPA 训练脚手架（Python 语法检查） | `python -m py_compile` | ✅ |
| safety | TTC/制动距离 | `ttc_evaluator_test`、`braking_model_test` | ✅ |
| safety | CBF 投影/安全检查 | `cbf_filter_test` | ✅ |
| safety | 可行性/加速度/零速 | `velocity_feasibility_filter_test` | ✅ |
| safety | AEB 采样 | `aeb_velocity_sampler_test` | ✅ |
| safety | 反射仲裁 | `reflex_policy_arbiter_test` | ✅ |
| safety | 三级链输入/超时/碰撞 | `safety_shield_test` | ✅ |
| safety | 看门狗/降级零速 | `watchdog_test`、`degradation_controller_test` | ✅ |
| behavior | 7 个节点条件/动作 | `behavior_nodes_test` | ✅ |
| behavior | 回退 BT 引擎/恢复重试 | `bt_engine_test` | ✅ |
| behavior | Waypoint/Elevator/Door 状态机 | `waypoint_sequence_task_test`、`elevator_task_test`、`door_task_test` | ✅ |
| behavior | Docking/Coverage 任务 | `docking_coverage_test` | ✅ |
| behavior | 语义搜索/重定位 | `semantic_relocalization_test` | ✅ |
| behavior | 目标跟随 | `following_task_test` | ✅ |
| behavior | 点击/语言目标转换 | `intent_test` | ✅ |
| planning | 几何门检测（深度列剖面） | `door_detector_task_test` | ✅ |
| planning | 2D 占据栅格前沿检测（automsgs OccupancyGrid） | `frontier_detector_task_test` | ✅ |
| guardian | 心跳超时/致命 | `heartbeat_monitor_test` | ✅ |
| guardian | 故障分类 | `fault_classifier_test` | ✅ |
| guardian | 降级策略 | `degradation_policy_test` | ✅ |
| guardian | 故障注入开关 | `fault_injector_test` | ✅ |
| guardian | 多模块聚合 | `supervisor_test` | ✅ |
| runner | 异步 policy/动作块/规则链 | `runner_test` | ✅ |
| runner | PluginManager + `Runner::Create` | `runner_create_test` | ✅ |
| config | 全部 conf/*.pb.txt 解析 | `configuration_validator` | ✅ |

## 2. 系统级验收

| DESIGN 验收项 | 当前状态 | 证据/说明 |
| --- | --- | --- |
| 位姿契约/TF 超时降级 | 🟡 | Runner 校验 pose/TF validity；ObservationBuilder 输出 validity；运行时 component 未接 |
| 安全链 AEB/限幅/零速 | ✅ | 三级链单测；`-fno-exceptions -fno-rtti` 编译 |
| 进程守护/心跳切换 | ✅ 核心 | Supervisor 聚合/降级；guardian component 未编译 |
| 诊断/FaultClassifier | ✅ 核心 | 分类/降级单测；publisher 组件未接 |
| 故障注入 6 类 | 🟡 | `FaultInjector` 开关齐全；自动化注入到进程未接 |
| 任务原语可独立单测 | ✅ | Door/Elevator/Docking/Coverage/Waypoint |
| 切换 policy/底盘/场景零改动 | 🟡 | PluginManager/DriveModel/ScenarioManager 就绪；真实模型未接 |
| 闭环仿真成功率 | ⛔ | 无仿真器闭环与模型 |
| 单测覆盖 | ✅ | 69 个 CTest 全通过 |
| C++17 / -Werror | 🟡 | `-std=c++17`、`-Wall -Wextra -Wpedantic`，0 warning；`-Werror` 未全局开启 |
| ASan | ✅ | `-DNAVIGATION_SANITIZER=address` 下 69/69 通过（leak 检测关闭） |
| TSan | ⛔ | 未运行 |
| 热路径延迟 P99 | 🟡 | `navigation_benchmark` 已输出 P50/P99；未设门限/未在目标平台采集 |
| 安全层 50 ms 硬截止 | ✅ 代码 | Watchdog/零速路径；未做实时平台测量 |
| Vision/DeFM/VNTPA 模型 | 🟡 | 接口与 Engine 适配器可编译；无权重/Engine 运行时 |

## 3. 运行验证命令

```sh
cmake -S . -B build -G Ninja \
  -DProtobuf_DIR=/opt/homebrew/lib/cmake/protobuf \
  -DNAVIGATION_AUTOMSGS_BUILD_DIR=/tmp/automsgs_build
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /opt/navigation
```

结果：69/69 passed，0 warning，install 成功。

## 4. 外部阻塞

- **autolink Component**：autolink 依赖 `sys/epoll.h`，仅 Linux；macOS 无法编译 Navigation/Safety/Guardian Component。三个 Component 已用 stub 做 `-fsyntax-only` 校验，需 Linux CI 实编。
- **autonomy_common / ONNX Runtime**：当前环境未构建 `autonomy_common`，Engine 适配器只做 object 编译；无 ONNX 权重，无法跑闭环。
- **BehaviorTree.CPP**：未安装；当前使用回退状态机，节点与 XML 已就绪。
