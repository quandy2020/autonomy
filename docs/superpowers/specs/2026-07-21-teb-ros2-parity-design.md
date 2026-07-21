# TEB 对齐 ROS2 全功能设计

日期：2026-07-21  
状态：已批准  
范围：`autonomy/control/controller/teb_controller`  
对照：`ros2/ros-planning/teb_local_planner/teb_local_planner`

## 1. 目标

在现有单轨迹 `TebOptimalPlanner` 内核之上，补齐 ROS2 TEB 的核心规划能力，使本仓库可通过配置走与上游一致的「单拓扑 / 多拓扑 + Recovery + 丰富障碍物表示」路径，且不引入 ROS/rclcpp 依赖。

成功标准：

- `enable_homotopy_class_planning=false`：行为与当前单 planner 一致
- `enable_homotopy_class_planning=true`：可生成多拓扑候选并选出一条执行轨迹
- `enable_multithreading` 开/关均可正确优化多候选
- 障碍物转换在合适地图下可输出点 / 线段 / 多边形
- 振荡场景下 `FailureDetector` 可置位并影响 prefer_rotdir / shrink-horizon 逻辑
- `libautonomy` 编译通过

非目标（本次不做）：

- 复刻 `teb_local_planner_ros` 插件壳、pluginlib、`costmap_converter` 插件加载器
- 完整可视化 Marker 发布链路（保留 stub / 可选扩展）
- 把 Boost.Graph 重写为自研图库

## 2. 已确认决策

| 项 | 选择 |
|----|------|
| 范围 | **A**：Homotopy + Recovery + 障碍物转换增强，接到现有 `Optimizer` / `PlannerInterface` |
| 多线程 | **A1**：对齐 ROS2，可配置开关（默认开） |
| 障碍物 | **B1**：增强内置转换器（点/线/多边形），不引入插件系统 |
| 移植策略 | **P1**：从 ROS2 直迁算法；图搜索保留 **Boost.Graph**；智能指针/`optional` 用标准库 |
| 默认 | Homotopy 默认开、多线程默认开（与 ROS2 `TebConfig` 一致），可用 proto/配置关掉 |

## 3. 架构

```text
TEBController
  └─ Optimizer
       ├─ CostmapObstacleConverter   # 点 / 线段 / 多边形
       ├─ FailureDetector            # 振荡检测（Recovery）
       └─ PlannerInterface
            ├─ TebOptimalPlanner           # hcp.enable = false
            └─ HomotopyClassPlanner        # hcp.enable = true
                 ├─ GraphSearchInterface     # Boost.Graph PRM + DFS
                 ├─ HSignature / Equivalence
                 └─ N × TebOptimalPlanner    # 可选 std::thread 并行
```

数据流（控制周期）：

1. `path_handler` 裁剪/变换全局路径 → 局部 plan + goal  
2. `CostmapObstacleConverter::update` → `ObstContainer`  
3. `planner_->plan(...)`  
4. `getVelocityCommand` → cmd  
5. `FailureDetector::update`；若振荡且启用 recovery → 设置 prefer_rotdir / 缩地平线（对齐 ROS2 `computeVelocityCommands` 中相关分支）

## 4. 组件

### 4.1 Homotopy 相关（新增）

| 文件 | 职责 |
|------|------|
| `core/equivalence_relations.hpp` | 拓扑等价关系接口 |
| `core/h_signature.hpp` | H-signature 计算与比较 |
| `core/graph_search.hpp/.cpp` | Homotopy 搜索图（Boost.Graph） |
| `core/homotopy_class_planner.hpp/.cpp` + impl | 多拓扑规划器，实现 `PlannerInterface` |

适配要点：

- 命名空间：`autonomy::control::controller::teb_controller`
- 去 `rclcpp` / `geometry_msgs`（ROS2）；统一走 `teb_core.hpp` 类型
- `boost::optional` → `std::optional`；`boost::shared_ptr` → `std::shared_ptr`
- 多线程：`std::thread` + 互斥，受 `TebConfig::hcp.enable_multithreading` 控制
- 可视化：调用现有 `TebVisualization` stub，不阻塞主路径

### 4.2 Recovery（新增）

| 文件 | 职责 |
|------|------|
| `core/recovery_behaviors.hpp/.cpp` | `FailureDetector` |

适配要点：

- `boost::circular_buffer` → 定长 `std::deque`（或等价环缓）
- Twist 使用 `teb_core::Twist`
- 在 `Optimizer`（或 Controller）中持有实例；buffer 长度 ≈ `oscillation_filter_duration * controller_frequency`

### 4.3 障碍物转换（扩展）

扩展 `tools/CostmapObstacleConverter`：

1. 对 lethal（及可选 inscribed）做连通域聚类  
2. 小簇 → `PointObstacle`  
3. 细长簇 → `LineObstacle` / 线段障碍  
4. 大簇 → 轮廓提取 + 简化 → `PolygonObstacle`  
5. 保留 `costmap_obstacle_sample_resolution`、`costmap_obstacles_behind_robot_dist`

可选模式：`points_only` | `points_lines_polygons`（默认后者），便于回归对比。

### 4.4 Optimizer 接线（修改）

- `planner_` 改为 `PlannerInterface` 智能指针（不再写死 `TebOptimalPlanner`）
- `initialize` 按 `teb_config_.hcp.enable_homotopy_class_planning` 构造 planner
- 每周期接入障碍物更新、plan、速度提取、FailureDetector
- 将 recovery 相关状态（prefer_rotdir、shrink horizon）接到 `HomotopyClassPlanner` / `TebOptimalPlanner` 已有接口（与 ROS2 同名方法对齐）

### 4.5 配置 / Proto（修改）

`TEBControllerOptions` 至少新增：

| 字段 | 默认（对齐 ROS2） |
|------|-------------------|
| `enable_homotopy_class_planning` | true |
| `enable_multithreading` | true |
| `max_number_classes` | 与 `TebConfig::hcp` 一致 |
| `oscillation_recovery` | true |
| `oscillation_filter_duration` | 与 `TebConfig::recovery` 一致 |
| `obstacle_conversion_mode` | `points_lines_polygons`（或 enum） |

其余 `hcp`/`recovery` 细参可继续用 `TebConfig` 代码默认值，按需后续暴露。`teb_options.cpp` 同步解析。

## 5. 依赖

- **允许保留**：Boost.Graph（`adjacency_list`、DFS）、现有 g2o / Eigen
- **禁止新增**：rclcpp、pluginlib、costmap_converter 插件运行时
- **已完成的标准库替换**：智能指针、`optional`、iterator 辅助；本设计延续该约定

## 6. 实现顺序

1. Recovery（`FailureDetector`）+ Optimizer 挂载（可独立测）  
2. `equivalence_relations` + `h_signature`  
3. `graph_search`（Boost.Graph）  
4. `HomotopyClassPlanner` + Optimizer 按配置选型  
5. 障碍物转换增强 + proto  
6. 联调与验收（见 §1）

## 7. 风险与缓解

| 风险 | 缓解 |
|------|------|
| Homotopy 计算量大 | 默认参数对齐 ROS2；可用配置关 Homotopy / 多线程；限制 `max_number_classes` |
| Boost.Graph 与 C++11 defaulted 历史问题 | 沿用上游 `#define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS` 变通 |
| 障碍物聚类与 ROS 插件结果不一致 | 提供 `points_only` 模式保证旧行为可回退 |
| Visualization 不完整 | stub 不阻塞；不作为本阶段成功标准 |

## 8. 验收清单

- [ ] Homotopy 关：与改造前单 planner 路径一致（无回归）
- [ ] Homotopy 开：多候选生成 + 选出一条 + 可输出速度
- [ ] 多线程开/关均通过编译与冒烟
- [ ] 障碍物容器在测试地图上出现线/多边形（或明确退化到点）
- [ ] FailureDetector 在合成振荡速度序列上 `isOscillating()==true`
- [ ] `docker` 内 `cmake --build ... --target autonomy` 成功
