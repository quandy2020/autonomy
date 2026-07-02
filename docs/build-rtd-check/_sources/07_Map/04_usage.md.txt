(map-usage)=
# 4. 使用指南

本章按 **Costmap2D** 与 **GridMap** 分节说明。二者职责不同：前者是规划栈的自动融合管线，后者是应用层驱动的浮点多层容器。

---

## 4.1 Costmap2D 使用

### 4.1.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | `config/planner/planner.lua` → `costmap` 块 |
| 加载 API | `map::CreateCostmap2DOptions("config")` |
| 典型集成 | `PlannerServer` 自动构造并 `Start()` |

```lua
-- config/autonomy.lua
AUTONOMY = { planning = AUTONOMY_PLANNER }
```

### 4.1.2 推荐使用路径

| 路径 | 适用 | 你需要做的 |
|------|------|------------|
| **A. 经 PlannerServer** | 标准导航 | 配好 `costmap` 块 + MapServer + 激光 bridge |
| **B. 独立 Wrapper** | 自定义规划/仿真 | 自己 `new Costmap2DWrapper` + `Start()` |

### 4.1.3 端到端集成（路径 A）

```
启动顺序:
  1. MapServer::Start()           → 发布 /map
  2. PlannerServer 构造           → 内部创建 Costmap2DWrapper 并 Start()
  3. static_layer 收到地图        → applyOccupancyGrid 或订阅 /map
  4. ROS bridge 转发 /scan        → feedLaserScan
  5. TF map↔base_link 就绪        → getRobotPose 成功
  6. costmap->isReady() == true   → 可规划
```

### 4.1.4 API 与调用时机

| API | 谁调用 | 何时 |
|-----|--------|------|
| `Start()` | 系统启动 | MapServer/TF 就绪后 |
| `applyOccupancyGrid(grid)` | MapServer 回调 | 静态地图首次到达 |
| `feedLaserScan(scan)` | 传感器 bridge | 每次激光回调 |
| `getCostmap()` | PlannerServer | 每次 `GetPlan()` 前加锁 |
| `isReady()` / `isCurrent()` | 规划前检查 | 避免 `PlannerTimedOut` |
| `Stop()` | 系统关闭 | 析构前 |

### 4.1.5 图层配置要点

**static_layer**

| 参数 | 说明 | 建议 |
|------|------|------|
| `map_topic` | 订阅话题 | `"map"` |
| `subscribe_to_updates` | 动态更新 | 全局图 `false` |

**obstacle_layer**

| 参数 | 说明 | 建议 |
|------|------|------|
| `marking` / `clearing` | 标记/清除 | 激光两者都开 |
| `obstacle_max_range` | 标记距离 | 3.0 m |
| `raytrace_max_range` | 清除距离 | 略大于 marking |
| `footprint_clearing_enabled` | 清除 footprint | `true` |

**inflation_layer**

| 参数 | 说明 | 建议 |
|------|------|------|
| `inflation_radius` | 膨胀半径 [m] | 0.35–0.55 |
| `cost_scaling_factor` | 衰减 $\lambda$ | 3.0–10.0 |

### 4.1.6 与 Planning 的读取约定

```cpp
std::unique_lock lock(costmap->getMutex());
auto* cm = costmap->getCostmap();
memcpy(local, cm->getCharMap(), size_x * size_y);
lock.unlock();
// 在 local 上规划，不阻塞地图更新
```

详见 [Costmap2D §6.3](06_costmap2d.md#63-详细使用指南)。

---

## 4.2 GridMap 使用

### 4.2.1 配置入口

| 项 | 值 |
|----|-----|
| Proto | `autonomy/map/proto/map_grid_option.proto` |
| 加载 API | `map::grid_map::CreateGridMapOptions(dict)` |
| 封装类 | `GridMapWrapper` |

### 4.2.2 与 Costmap2D 的使用差异

| 步骤 | Costmap2D | GridMap |
|------|-----------|---------|
| 构造后 | 调 `Start()` 即有后台更新 | `Start()` 仅标记运行，**不写数据** |
| 传感器 | `feedLaserScan` 自动处理 | **应用层投影点云到 elevation** |
| 查询 | `getCost(mx,my)` 离散代价 | `atPosition` 连续插值 |
| 滑动 | `rolling_window=true` 自动 | 手动 `move(robot_position)` |
| 规划 | 直接给 PlannerServer | 需桥接或自定义逻辑 |

### 4.2.3 推荐使用路径

```cpp
// 1. 构造（setGeometry 在 wrapper 构造时根据 options 完成）
auto wrapper = std::make_shared<GridMapWrapper>(opts);
wrapper->Start();

auto& gm = *wrapper->getGridMap();

// 2. 应用层：点云 → elevation（你的算法）
projectPointCloudToElevation(cloud, gm);

// 3. 派生层：elevation → slope
computeSlopeLayer(gm);

// 4. 机器人移动时滑动
gm.move(grid_map::Position(robot_x, robot_y));

// 5. 查询
double elev, slope;
gm.atPosition("elevation", pos, elev);
gm.atPosition("slope", pos, slope);
```

### 4.2.4 API 速查

| API | 作用 |
|-----|------|
| `getGridMap()` | 获取 `grid_map::GridMap` 共享指针 |
| `Start()` / `Stop()` | 生命周期（无后台线程） |
| `loadMap()` | ⚠️ 未实现 |
| `publishMap()` | ⚠️ 未实现 |

核心操作在 `grid_map::GridMap` 上：`add`、`at`、`atPosition`、`move`、`addDataFrom`。

详见 [GridMap §7.3](07_grid_map.md#73-详细使用指南)。

---

## 4.3 MapServer

| API | 用途 |
|-----|------|
| `Start()` | 加载地图并启动发布线程 |
| `GetStaticMapShared()` | 获取 `OccupancyGrid` |
| `SetMapPublishCallback(fn)` | 注册回调 → 转发给 `applyOccupancyGrid` |

---

## 4.4 通信接口

| 类型 | 名称 | 组件 |
|------|------|------|
| 静态地图 | `/map` | MapServer |
| 全局代价地图 | `/global_costmap` | Costmap2DWrapper |
| 激光 | `/scan` | → `feedLaserScan` |
| GridMap | 按应用配置 | 自行发布 |

---

## 4.5 故障排查

| 现象 | 常见原因 | 处理 |
|------|----------|------|
| 规划报 `PlannerTimedOut` | costmap 未更新 | 检查 `update_frequency`、`isCurrent()` |
| 路径穿墙 | 膨胀不足或 denoise 过强 | 增大 `inflation_radius`，禁用 `denoise_layer` |
| 机器人被标记为障碍 | footprint 未清除 | 开启 `footprint_clearing_enabled` |
| 地图全为未知 (255) | 静态地图未加载 | 检查 MapServer、`static_layer.map_topic` |
| 坐标偏移 | TF / frame_id 错误 | 统一 `frame_id` 与 TF 树 |
| GridMap 查询 NaN | 该位置无数据或越界 | 检查 `move()` 后 buffer 范围 |

**检查清单**：`MapServer::HasStaticMap()` · `costmap->isCurrent()` · `worldToMap` 边界 · 启动日志图层加载 · 可视化 costmap 快照。

### 4.6 性能建议

| 建议 | 说明 |
|------|------|
| 全局图 ≤ 2048² | 控制内存与更新耗时 |
| 更新 5 Hz | `update_frequency` 通常足够 |
| 局部 rolling window | 大地图场景用 `rolling_window=true` |
| 按需复制 | 规划时复制 char map，避免长时间持锁 |
| GridMap 插值 | 实时查询用 `INTER_LINEAR`，离线分析用 `INTER_CUBIC` |

### 4.7 自定义 Costmap 图层

1. 继承 `CostmapLayer` 或 `Layer`，实现 `updateBounds()` 与 `updateCosts()`
2. 编译为 `.so` 插件，在 `plugins` 列表中注册
3. 在 Lua 配置中添加对应图层参数块

参考现有图层：`autonomy/map/costmap_2d/layers/`。
