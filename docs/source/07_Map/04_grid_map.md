# 4. GridMap 2.5D 栅格地图

ETH `grid_map`：XY 上多层 float（高程等）。架构 [§1.3](01_architecture.md)。

### 4.1 适用

| 场景 | — |
|------|--|
| 高程 / 坡度 / 多模态层 | — |
| 直接作规划输入 | → Costmap 或应用桥接 |

无自动传感器管线，应用层写入。

### 4.2 结构

```
GridMapWrapper → GridMap
data_["elevation"|…] MatrixXf
position_ = 地图中心 length_ / resolution_ / startIndex_
```

| — | Costmap2D | GridMap |
|--|-----------|---------|
| 原点 | 左下 | **中心** |
| 无效 | 255 | **NaN** |
| 更新 | 后台插件 | 手动写 + `move()` |

### 4.3 用法

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
| `at` / `atPosition` | 读写 / 插值查询 |
| `getIndex` / `move` | 坐标 ↔ 索引；滑动窗口 |
| `add` / `addDataFrom` | 加层 / 融合 |

点云→高程：应用层变换→分桶→写层→可选算 slope。与 Costmap 联合：按世界坐标查 slope，映射为 LETHAL/中代价。

**限制**：无后台更新、无 `feedLaserScan`、发布/加载 TODO、无官方 Costmap 桥接。

### 4.6 坐标

$N_i=\mathrm{round}(L_i/\Delta)$；索引相对**中心**，含循环 buffer（`startIndex_`）。互转 Costmap 须显式处理原点差。
