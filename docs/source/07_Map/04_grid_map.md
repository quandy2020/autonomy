# 4. GridMap 2.5D 栅格地图

ETH `grid_map` 封装：XY 平面上 **多层 float**（如 `elevation`、`slope`）。架构见 [01 §1.3](01_architecture.md)。 GridMap 存 elevation/slope 等多层 float，Follow 等可写层供 MPPI Grid critic 读取。NavFn/Theta* 等 2D 搜索不直接消费 GridMap——需在应用层映射成 cost 或只用读 Grid 的控制器。

## 用途

高程、坡度、多模态层；Perception Follow 等会写 GridMap 供 MPPI Grid critic 使用。**不能** 直接替代 Costmap2D 给 NavFn/Theta* 搜索——要么应用层把坡度映射成 LETHAL，要么只用 MPPI 等读 Grid 的 critic。

## 和 Costmap2D 的差异

| | Costmap2D | GridMap |
|--|-----------|---------|
| 原点 | 左下角 | **地图中心** |
| 无效值 | 255（UNKNOWN） | **NaN** |
| 更新 | 插件 + 后台线程 | **应用手动写** + `move()` |
| 传感器 | `feedLaserScan` 等 | 无内置 feed |

两者 **没有** 仓库内自动桥接。

## 结构

```text
GridMapWrapper → grid_map::GridMap
  data_["elevation"|…]  // Eigen::MatrixXf
  position_ / length_ / resolution_ / startIndex_
```

## 用法

```cpp
proto::GridMapOptions opts;
opts.set_resolution(0.1); opts.set_length_x(10); opts.set_length_y(10);
opts.add_layers("elevation");
auto wrapper = std::make_shared<GridMapWrapper>(opts);
wrapper->Start();
auto& gm = *wrapper->getGridMap();
grid_map::Index idx;
if (gm.getIndex(Position(1,2), idx)) gm.at("elevation", idx) = 0.35f;
gm.move(Position(robot_x, robot_y));
```

| API | 作用 |
|-----|------|
| `at` / `atPosition` | 读写 / 插值 |
| `getIndex` / `move` | 索引与滑动窗口 |
| `add` / `addDataFrom` | 加层 / 融合 |

点云→高程：应用层变换、分桶、写层，可选再算 slope。与 Costmap 联合时按 **同一世界坐标** 查两层，注意原点换算。

## 坐标

$N_i=\mathrm{round}(L_i/\Delta)$；索引相对中心，带 `startIndex_` 循环 buffer。和 Costmap 互转必须显式处理原点差。

## 限制（当前实现）

无后台传感器管线、无 `feedLaserScan`；部分发布/加载能力仍 TODO。跟人不走时，先确认 Follow 是否在写层、MPPI 是否启用 Grid 相关 critic（[Perception · Follow](../10_Perception/11_follow.md)）。

## 常见坑

| 现象 | 先查 |
|------|------|
| 全 NaN | 是否有人写 elevation |
| `move()` 后值错位 | `startIndex_` 与查询 API |
| 规划仍穿坡 | 未映射到 Costmap，规划器看不到 Grid |
