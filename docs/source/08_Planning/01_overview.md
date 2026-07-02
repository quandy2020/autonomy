(planning-overview)=
# 1. 模块概览

### 1.1 定位

| 维度 | 说明 |
|------|------|
| 规划层级 | 全局路径规划（Global Planner） |
| 输入 | 起点/终点位姿、全局代价地图 |
| 输出 | `planning_msgs::Path` |
| 下游 | Navigator、Controller、可视化 |
| 对标 | nav2_planner、nav2_navfn_planner、nav2_theta_star_planner |

### 1.2 核心能力

- 内置 **NavFn / Dijkstra / Theta\*** 三种规划器插件
- 基于 autolink `PluginManager` 加载外部 `.so` 插件
- 共享 `Costmap2DWrapper`（静态层 + 障碍层 + 膨胀层）
- 路径后处理：`PathSimplifier`、`SimpleSmoother`
- 服务：`is_path_valid`；支持 `cancel_checker` 中断规划

### 1.3 源码结构

```
autonomy/planning/
├── planner_server.*          # 服务入口
├── planner_options.*         # Lua → PlannerOptions
├── common/                   # GlobalPlanner / Smoother 接口
├── planner/
│   ├── navfn/                # NavFn + NavfnPlanner
│   ├── dijkstra/             # DijkstraPlanner
│   └── theta_star/           # ThetaStarPlanner
├── utils/                    # 平滑、简化、几何工具
└── proto/                    # 配置 Protobuf
```

### 1.4 相关模块

- `autonomy/map/costmap_2d` — 全局代价地图
- `autonomy/navigator` — 行为树，调用规划
- `autonomy/control` — 局部轨迹跟踪
