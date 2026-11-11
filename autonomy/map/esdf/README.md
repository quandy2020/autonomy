# ESDF（FIESTA，无 ROS）

固定稠密栅格上的增量欧氏距离场。算法来自 [FIESTA](https://arxiv.org/abs/1903.02144)（IROS 2019）：概率占据 + 最近障碍双向链表 + 24 邻域 BFS。本目录是无 ROS 的移植，编进 `autonomy_map`。

距离是到**最近占据体素中心**的米；占据格为 `0`。未更新过的格是 `kInfinity`（`10000`）。`GetDistanceWithGradient` 为三线性插值。

## 文件

| 文件 | 作用 |
|:---|:---|
| `esdf_map.hpp` / `.cpp` | `EsdfMap`：占据融合、增量 ESDF、查询 |
| `options.hpp` | 原点、尺寸、分辨率、log-odds |
| `raycast.hpp` / `.cpp` | Amanatides–Woo 体素遍历（`IntegrateRay`） |
| `esdf_map_test.cpp` | 占据阈值、距离、删除、射线、梯度 |
| `esdf_visualize_test.cpp` | 多场景 z 切片（占用图 + TURBO 距离图） |

命名空间：`autonomy::map::esdf`。

## 用法

```cpp
#include "autonomy/map/esdf/esdf_map.hpp"

using autonomy::map::esdf::EsdfMap;
using autonomy::map::esdf::Options;

Options options;
options.origin = {-5.0, -5.0, -1.0};
options.map_size = {10.0, 10.0, 2.0};
options.resolution = 0.1;
options.min_ray_length = 0.0;  // 短射线默认会被丢掉

EsdfMap map(options);
map.SetOccupancy(Eigen::Vector3d(0, 0, 0), true);
map.UpdateOccupancy(true);   // log-odds → insert/delete 队列
map.UpdateESDF();            // BFS 距离

double distance = map.GetDistance(Eigen::Vector3d(0.5, 0, 0));
Eigen::Vector3d gradient;
map.GetDistanceWithGradient(Eigen::Vector3d(0.5, 0, 0), &gradient);
```

点云 / 深度：

```cpp
map.IntegrateCloud(sensor_origin, points);  // 忽略 NaN
map.UpdateOccupancy(true);
map.UpdateESDF();
```

`IntegrateRay`：终点标记占据（超长射线只清到 `max_ray_length`），沿线 miss。坐标已是地图 / 世界系。

## 默认参数

| 字段 | 默认 | 含义 |
|:---|:---|:---|
| `origin` | `(-5,-5,-1)` | 体素 `(0,0,0)` 的世界角点 |
| `map_size` | `(10,10,2)` | 从 origin 起的 AABB（米） |
| `resolution` | `0.1` | 立方体素边长 |
| `hit_probability` / `miss_probability` | `0.70` / `0.35` | 占据 / 空闲观测 |
| `min_probability` / `max_probability` | `0.12` / `0.97` | log-odds 夹紧 |
| `occupancy_probability` | `0.80` | 超过则视为占据（通常要 **两次** hit） |
| `min_ray_length` | `0.5` | 更短的射线丢弃 |
| `max_ray_length` | `5.0` | 更长的射线截断为 free |

一次 `SetOccupancy(..., true)` 只累加一轮 logit，默认 `occupancy_probability=0.80` 时单次 hit **不会**占住。测试里对同一格 `Hit` 多轮，每轮都 `UpdateOccupancy`。

删除：对已占据格反复 `SetOccupancy(..., false)`（测试约 8 轮 miss），再 `UpdateOccupancy` + `UpdateESDF`。

## 查询约定

- `GetOccupancy`：`1` 占据，`0` 空闲，越界 `kUndefined`（`-10000`）。
- `GetDistance`：米；从未 BFS 过的格为 `kInfinity`。
- `GetDistanceWithGradient`：越界返回 `-1`。
- `OccupiedPoints()`：当前更新 AABB 内占据体素中心。
- `CheckConsistency()`：最近障碍链表自检。
- `SetUpdateRange`：限制 BFS AABB；`SetOriginalRange` 恢复全图。

## 相对上游 FIESTA

对齐的是 **dense + probabilistic + 24-BFS + 三线性 + raycast** 这条默认路径。

未移植：ROS 节点、`HASH_TABLE`、有符号距离、`SetAway`、多线程。`StoredDistance` 把内部 `kUndefined` 当成 `kInfinity` 参与 BFS，避免邻域停在 `-10000`。

## 测试与切片

```sh
# 在 autonomy 构建目录
cmake --build . --target map.esdf_map_test map.esdf_visualize_test -j8
./bin/map.esdf_map_test
./bin/map.esdf_visualize_test
```

可视化场景：empty、wall、pillar、wall+pillar、corridor、L 角、boxes、rays、removed。PNG 写到 gtest `TempDir()`，不要提交进本目录。
